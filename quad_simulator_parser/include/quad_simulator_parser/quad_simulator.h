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


/**
* @brief Clock to get the current time
*/
using Clock = chrono::high_resolution_clock;
/**
* @brief TimePoint to perform arithmetic on time points
*/
using TimePoint = chrono::high_resolution_clock::time_point;

namespace quad_simulator{
/**
* @brief Simulator class for quadrotor
*
* Provides methods to propagate quadrotor dynamics
* using a second order model for rotational dynamics
*/
class QuadSimulator : public parsernode::Parser
{
    
protected:
    /**
    * @brief Command to Quadrotor
    *
    * Stores roll, pitch, yaw, thrust command
    * along with time when the command is sent
    */
    struct RpytCmdStruct {
      geometry_msgs::Quaternion rpytmsg;///< rpyt message
      TimePoint time;///< time when the message is sent
      double dt;///< Time difference from previous message
    };

    /// \brief Members depicting the state of the quadcopter
    parsernode::common::quaddata data;
    gcop::QRotorIDModel sys_;///< Quadrotor system from GCOP for propagating the system
    gcop::QRotorIDState state_;///< Current state of Quadcopter
    bool enable_qrotor_control_;///< Should be set to true before the quadrotor is controlled
    bool vel_thread_running;///< Whether thread for moving quad using velocity commands
    TimePoint prev_vel_cmd_time_;///< Previous command time
    int16_t rcin[4];///< Radio channel input
    queue<RpytCmdStruct> rpyt_cmds;///< Command queue
    double delay_send_time_;///< How much delay between commanded and executed
    double global_ref_lat, global_ref_long;///<Lat and Long of Home
    double battery_percent_;///< Percentage of battery remaining for quadrotor 
    double takeoff_altitude_;///< Altitude to reach when taking off
    ///

    gcop::SO3 &so3;///< SO3 instance to perform arithmetic on SO3 group
    bool armed;///< Whether quadrotor is armed or not
    bool use_perfect_time_; ///< Do not differentiate based on high resolution
                            ///clock

  protected:
    /**
    * @brief map and save joystick input to rc channels
    *
    * @param joy_msg Input joystick messsage
    */
    void setRCInputs(const sensor_msgs::Joy &joy_msg);
    /**
    * @brief Convert NED coordinates to GPS lat long
    *
    * @param ned_x NED X position
    * @param ned_y NED y position
    * @param gps_t_lon Converted GPS longitude
    * @param gps_t_lat Converted GPS latitude
    */
    inline void ned_convert_gps(const float &ned_x, const float &ned_y,
        double &gps_t_lon, double &gps_t_lat)
    {
      gps_t_lat = RAD2DEG((ned_x/C_EARTH));
      gps_t_lon = RAD2DEG((ned_y/(C_EARTH*cos(DEG2RAD(gps_t_lat)))));
    }
    /**
    * @brief send velocity and yaw command to quadrotor
    *
    * @param vel_cmd Velocity command to send
    * @param yaw_inp Yaw input can be yaw angle/ yaw rate.
    * @param vel_yaw_ratemode if set to true, uses yaw input as rate and as yaw angle otherwise
    *
    * @return true if successful
    */
    virtual bool cmdvelguided(geometry_msgs::Vector3 &vel_cmd, double &yaw_inp, bool vel_yaw_ratemode);
    /**
    * @brief Send rpythrust command to quadrotor
    *
    * @param rpytmsg rpyt message to send to quadrotor
    * @param rp_angle_yawrate_mode if set to true, interprets rpytmessage as roll, pitch,
    * yawrate and thrust. If false, interprets input as roll, pitch, yaw and thrust
    *
    * @return true if successful
    */
    virtual bool cmdrpythrustInternal(geometry_msgs::Quaternion &rpytmsg, bool rp_angle_yawrate_mode); 
public:
    /**
    * @brief Constructor
    */
    QuadSimulator();
    /**
    * @brief virtual destructor
    */
    virtual ~QuadSimulator()
    {
        disarm();//release sdk control
    }
    /// \brief Extend functions from Parser:
    /**
    * @brief Takeoff
    *
    * @return true if successful
    */
    virtual bool takeoff();
    /**
    * @brief Land
    *
    * @return true if successful
    */
    virtual bool land();
    /**
    * @brief disable quadrotor control
    *
    * @return  true if successful
    */
    virtual bool disarm();
    /**
    * @brief stop software control
    *
    * @param bool toggle on/off
    *
    * @return true if successful
    */
    virtual bool flowControl(bool);
    /**
    * @brief Recalibrate IMU
    *
    * @return true if successful
    */
    virtual bool calibrateimubias();
    /**
    * @brief Command roll, pitch , yaw and thrust to quadrotor
    *
    * @param rpytmsg rpyt message to send to quadrotor
    *
    * @return true if successful
    */
    virtual bool cmdrpythrust(geometry_msgs::Quaternion &rpytmsg);
    /**
    * @brief Command roll, pitch, yawrate and thrust to quadrotor
    *
    * @param rpytmsg rpyt message to send to quadrotor
    *
    * @return true if successful
    */
    virtual bool cmdrpyawratethrust(geometry_msgs::Quaternion &rpytmsg);
    /**
    * @brief Send velocity and yaw rate to quadrotor
    *
    * @param vel_cmd Velocity command to send
    * @param yaw_rate Yaw rate to be sent in radians/s
    *
    * @return True if successful
    */
    virtual bool cmdvel_yaw_rate_guided(geometry_msgs::Vector3 &vel_cmd, double &yaw_rate);
    /**
    * @brief Send velocity and yaw angle to quadrotor
    *
    * @param vel_cmd Velocity command to send
    * @param yaw_angle Yaw angle to send in radians
    *
    * @return
    */
    virtual bool cmdvel_yaw_angle_guided(geometry_msgs::Vector3 &vel_cmd, double &yaw_angle);
    /**
    * @brief Send position and yaw command to quadrotor
    *
    * @param desired_pos Position command
    * @param desired_yaw desired yaw command
    *
    * @return true if successful
    */
    virtual bool cmdwaypoint(geometry_msgs::Vector3 &desired_pos, double desired_yaw = 0);
    /**
    * @brief Gripper state change command
    *
    * @param state if using tri-state gripper, specifies close/open/neutral
    */
    virtual void grip(int state);
    /**
    * @brief Set attitude of the quadrotor state
    *
    * @param roll roll angle
    * @param pitch pitch angle
    * @param yaw yaw angle
    */
    virtual void reset_attitude(double roll, double pitch, double yaw);
    /**
    * @brief Use for initializing without ROS
    */
    virtual void initialize();
    /**
    * @brief Get sensor data from quadrotor
    *
    * @param d1 data struct to fill the quadrotor data
    */
    virtual void getquaddata(parsernode::common::quaddata &d1);
    /**
    * @brief Set the altitude of the quadrotor
    *
    * @param altitude_ desired altitude
    */
    virtual void setaltitude(double altitude_)
    {
        std::cerr<<"Setting Altitude not supported"<<std::endl;
        return;
    }

    /**
    * @brief Set logging directory for loggins debug info
    *
    * @param logdir directory name
    */
    virtual void setlogdir(string logdir)
    {
      //Not Implemented
    }
    /**
    * @brief Toggle logging
    *
    * @param logswitch switch toggle on/off
    */
    virtual void controllog(bool logswitch)
    {
      //Not Implemented
    }
    ///

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
    /**
    * @brief adjust time offset between commands and the dynamics
    *
    * @param delay_send_time time offset in seconds
    */
    void set_delay_send_time(double delay_send_time) {
      if(delay_send_time < 0) {
        throw std::logic_error("Delay send time cannot be less than zero");
      }
      delay_send_time_ = delay_send_time < 0.02?0.02:delay_send_time;
    }
    /**
    * @brief set battery percentage left
    *
    * @param battery_percent percentage left
    */
    void setBatteryPercent(double battery_percent) {
      battery_percent_ = battery_percent;
    }
    /**
    * @brief set takeoff altitude to reach when issued takeoff command
    *
    * @param altitude height to reach
    */
    void setTakeoffAltitude(double altitude) {
      takeoff_altitude_ = altitude;
    }

    /**
    * @brief Choose not to differentiate commands based on time received.
    *
    * Automatically increment the time on the commands by 0.02
    *
    * @param flag true to not differentiate
    */
    void usePerfectTime(bool flag = true) {
      if (flag) {
        use_perfect_time_ = true;
        // Clear existing rpyt commands if any
        {
          queue<RpytCmdStruct> empty;
          rpyt_cmds.swap(empty);
        }
        // Add default rpyt command with zero rpyt to
        // queue
        RpytCmdStruct rpyt_cmd;
        rpyt_cmd.time = TimePoint();
        rpyt_cmd.dt = 0.02;
        rpyt_cmds.push(rpyt_cmd);
      } else {
        use_perfect_time_ = false;
      }
    }
};
}
#endif // QUAD_SIMULATOR_H
