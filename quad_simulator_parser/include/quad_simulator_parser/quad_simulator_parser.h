#ifndef QUAD_SIMULATOR_PARSER_H
#define QUAD_SIMULATOR_PARSER_H
/* This header is subclassed from parser.h and will include data from Quadcopter simulator for testing GUI. It will also provide interfaces for commanding the Simulated Quadcopter.
 * This class uses internal locking to ensure the quaddata is available to Qt thread without bumping into ros serial. This is enough since the order of working is not important to us. Read internal vs external locking: http://www.boost.org/doc/libs/1_55_0/doc/html/thread/synchronization.html
 */
#include <parsernode/parser.h> //main parser base class

// Standard includes
#include <iostream>
#include <cstdlib>
#include <unistd.h>
#include <cmath>
#include <string>
#include <inttypes.h>
#include <fstream>
#include <sys/time.h>
#include <time.h>
#include <bitset>         // std::bitset
#include <boost/thread.hpp>

//Gcop System Includes
#include <gcop/qrotoridmodel.h>
#include <gcop/so3.h>

//Messages:
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Joy.h>

using namespace std;
using namespace gcop;
using namespace Eigen;
namespace quad_simulator_parser{
class QuadSimParser: public parsernode::Parser
{
private:
    //Members depicting the state of the quadcopter
    parsernode::common::quaddata data;
    QRotorIDModel sys_;///< Quadrotor system from GCOP for propagating the system
    QRotorIDState state_;///< Current state of Quadcopter
    bool enable_qrotor_control_;///< Should be set to true before the quadrotor is controlled
    bool rpyt_ratemode;///< Specifies to use yaw rate mode or yaw angle mode in cmdrpyt
    bool vel_thread_running;///< Whether thread for moving quad using velocity commands
    ros::Time prev_rpy_cmd_time_;///< Previous ros command time
    ros::Time prev_vel_cmd_time_;///< Previous ros command time
    int16_t rcin[4];

    //Subscribers:
    ros::Subscriber joy_sub_;

    //Internal modes:
    bool armed;
protected:
    void setRCInputs(const sensor_msgs::Joy &joy_msg);

public:
    QuadSimParser();
    ~QuadSimParser()
    {
        disarm();//release sdk control
    }
    //Extend functions from Parser:
    bool takeoff();
    bool land();
    bool disarm();
    bool flowControl(bool);
    bool calibrateimubias();
    bool cmdrpythrust(geometry_msgs::Quaternion &rpytmsg, bool sendyaw = false);
    bool cmdvelguided(geometry_msgs::Vector3 &vel_cmd, double &yaw_ang);
    bool cmdwaypoint(geometry_msgs::Vector3 &desired_pos, double desired_yaw = 0);
    void grip(int state);
    void reset_attitude(double roll, double pitch, double yaw);
    void setmode(std::string mode);
    void initialize(ros::NodeHandle &nh_);
    void getquaddata(parsernode::common::quaddata &d1);
    void setaltitude(double altitude_)
    {
        return;
    }

    void setlogdir(string logdir)
    {
      //Not Implemented
    }
    void controllog(bool logswitch)
    {
      //Not Implemented
    }
};
}
#endif // DJI_PARSHER_H