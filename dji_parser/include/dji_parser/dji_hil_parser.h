#ifndef DJI_HIL_PARSER_H
#define DJI_HIL_PARSER_H
/* This is the plugin which is dependent on Parser base class.
 * It provides code implementation for DJI Matrice Quadcopter in
 * Hardware In Loop (HIL) mode.
 * In this mode, the data from Quadrotor is provided. But functions
 * such as arming, takeoff etc only modify the height of the quadrotor
 * and state of the quadrotor without sending those commands
 * to the low-level API. This can be used to test controllers and estimators
 * without actually flying the Quadrotor.
 */
#include <parsernode/parser.h> //main parser base class
#include <dji_parser/dji_parser.h> // Parent DJI Parser class

namespace dji_parser{
class DjiHILParser: public DjiParser
{
private:
//    ros::NodeHandle nh_;///< Internal node handle
public:
    DjiHILParser();
    //Extend functions from Parser:
    bool takeoff();
    bool land();
    bool disarm();
    bool flowControl(bool);
    bool cmdrpythrust(geometry_msgs::Quaternion &rpytmsg);
    bool cmdrpyawratethrust(geometry_msgs::Quaternion &rpytmsg);
    bool cmdvel_yaw_rate_guided(geometry_msgs::Vector3 &vel_cmd, double &yaw_rate);
    bool cmdvel_yaw_angle_guided(geometry_msgs::Vector3 &vel_cmd, double &yaw_angle);
    bool cmdwaypoint(geometry_msgs::Vector3 &desired_pos, double desired_yaw = 0);
    //void getquaddata(parsernode::common::quaddata &d1);
    virtual void get50HzData(Vehicle *vehicle, RecvContainer recvFrame);
    virtual void get10HzData(Vehicle *vehicle, RecvContainer recvFrame);
    virtual void get1HzData(Vehicle *vehicle, RecvContainer recvFrame);
};
}
#endif // DJI_HIL_PARSHER_H
