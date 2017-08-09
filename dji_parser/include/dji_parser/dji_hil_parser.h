#ifndef DJI_HIL_PARSER_H
#define DJI_HIL_PARSER_H
/* This header is subclassed from parser.h and will include data from DJI Matrice Quadcopter. It will also provide interfaces for commanding the DJI Quadcopter. It uses the official dji_sdk drivers to do the commanding and getting the navdata
 * This class uses internal locking to ensure the quaddata is available to Qt thread without bumping into ros serial. This is enough since the order of working is not important to us. Read internal vs external locking: http://www.boost.org/doc/libs/1_55_0/doc/html/thread/synchronization.html
 */
#include <parsernode/parser.h> //main parser base class
#include <dji_parser/dji_parser.h> // Parent DJI Parser class

namespace dji_parser{
class DjiHILParser: public DjiParser
{
private:
    void receiveDJIData();//receive dji data from its lib 
public:
    DjiHILParser();
    //Extend functions from Parser:
    bool takeoff();
    bool land();
    bool disarm();
    bool flowControl(bool);
    bool cmdrpythrust(geometry_msgs::Quaternion &rpytmsg, bool sendyaw = false);
    bool cmdvelguided(geometry_msgs::Vector3 &vel_cmd, double &yaw_inp);
    bool cmdvel_yaw_rate_guided(geometry_msgs::Vector3 &vel_cmd, double &yaw_rate);
    bool cmdvel_yaw_angle_guided(geometry_msgs::Vector3 &vel_cmd, double &yaw_angle);
    bool cmdwaypoint(geometry_msgs::Vector3 &desired_pos, double desired_yaw = 0);
};
}
#endif // DJI_HIL_PARSHER_H
