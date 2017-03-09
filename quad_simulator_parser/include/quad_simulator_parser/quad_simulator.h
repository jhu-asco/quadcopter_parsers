#ifndef QUAD_SIMULATOR_H
#define QUAD_SIMULATOR_H
/* This header is subclassed from parser.h and will include data from Quadcopter simulator for testing GUI. It will also provide interfaces for commanding the Simulated Quadcopter.
 */
#include <parsernode/parser.h> //main parser base class

// Standard includes
#include <boost/thread.hpp>
#include <chrono>
#include <stdexcept>
#include <queue>

//Gcop System Includes
#include <gcop/qrotoridmodel.h>
#include <gcop/so3.h>

//Messages:
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/Joy.h>

#define C_EARTH (double) 6378137.0
#define C_PI (double) 3.141592653589793
#define DEG2RAD(DEG) ((DEG)*((C_PI)/(180.0)))
#define RAD2DEG(RAD) ((RAD)*((180.0)/(C_PI)))


using namespace Eigen;

using Clock = chrono::high_resolution_clock;
using TimePoint = chrono::high_resolution_clock::time_point;

namespace quad_simulator{
class QuadSimulator : public parsernode::Parser
{
    
protected:
    struct RpytCmdStruct {
      geometry_msgs::Quaternion rpytmsg;
      TimePoint time;
      double dt;
    };
    //Members depicting the state of the quadcopter
    parsernode::common::quaddata data;
    gcop::QRotorIDModel sys_;///< Quadrotor system from GCOP for propagating the system
    gcop::QRotorIDState state_;///< Current state of Quadcopter
    bool enable_qrotor_control_;///< Should be set to true before the quadrotor is controlled
    bool rpyt_ratemode;///< Specifies to use yaw rate mode or yaw angle mode in cmdrpyt
    bool vel_yaw_ratemode;///< State used to switch between rate control vs angle control of y in vel
    bool vel_thread_running;///< Whether thread for moving quad using velocity commands
    TimePoint prev_vel_cmd_time_;///< Previous command time
    int16_t rcin[4];///< Radio channel input
    queue<RpytCmdStruct> rpyt_cmds;///< Command queue
    double delay_send_time_;///< How much delay between commanded and executed
    double global_ref_lat, global_ref_long;///<Lat and Long of Home
    double battery_percent_;///< Percentage of battery remaining for quadrotor 
    double takeoff_altitude_;///< Altitude to reach when taking off
    //so3
    gcop::SO3 &so3;

    //Internal modes:
    bool armed;///< Whether quadrotor is armed or not

protected:
    void setRCInputs(const sensor_msgs::Joy &joy_msg);
    //void ktTimerCallback(const ros::TimerEvent& );
    inline void ned_convert_gps(const float &ned_x, const float &ned_y,
        double &gps_t_lon, double &gps_t_lat)
    {
      gps_t_lat = RAD2DEG((ned_x/C_EARTH));
      gps_t_lon = RAD2DEG((ned_y/(C_EARTH*cos(DEG2RAD(gps_t_lat)))));
    }


public:
    QuadSimulator();
    virtual ~QuadSimulator()
    {
        disarm();//release sdk control
    }
    //Extend functions from Parser:
    virtual bool takeoff();
    virtual bool land();
    virtual bool disarm();
    virtual bool flowControl(bool);
    virtual bool calibrateimubias();
    virtual bool cmdrpythrust(geometry_msgs::Quaternion &rpytmsg, bool sendyaw = false);
    virtual bool cmdvelguided(geometry_msgs::Vector3 &vel_cmd, double &yaw_inp);
    virtual bool cmdwaypoint(geometry_msgs::Vector3 &desired_pos, double desired_yaw = 0);
    virtual void grip(int state);
    virtual void reset_attitude(double roll, double pitch, double yaw);
    virtual void setmode(std::string mode);
    void non_ros_initialize();
    virtual void initialize(ros::NodeHandle &){
        non_ros_initialize();
    }
    virtual void getquaddata(parsernode::common::quaddata &d1);
    virtual void setaltitude(double altitude_)
    {
        std::cerr<<"Setting Altitude not supported"<<std::endl;
        return;
    }

    virtual void setlogdir(string logdir)
    {
      //Not Implemented
    }
    virtual void controllog(bool logswitch)
    {
      //Not Implemented
    }
    /**
     * @brief set the rc channel information stored
     * in the UAV brain
     *
     * @param channels rc channel values (-10000, 10000)
     */
    void setRC(int16_t channels[4]) {
      for (int i = 0; i < 4; i++) {
        rcin[i] = channels[i];
      }
    }
    void set_delay_send_time(double delay_send_time) {
      if(delay_send_time < 0) {
        throw std::logic_error("Delay send time cannot be less than zero");
      }
      delay_send_time_ = delay_send_time < 0.02?0.02:delay_send_time;
    }
    void setBatteryPercent(double battery_percent) {
      battery_percent_ = battery_percent;
    }
    void setTakeoffAltitude(double altitude) {
      takeoff_altitude_ = altitude;
    }
};
}
#endif // QUAD_SIMULATOR_H
