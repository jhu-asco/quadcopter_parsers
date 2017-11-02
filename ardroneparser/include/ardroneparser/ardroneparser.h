#ifndef ARDRONE_PARSER_H
#define ARDRONE_PARSHER_H
/* This header is subclassed from parser.h and will include data from Ardrone Quadcopter. It will also provide interfaces for commanding the Ardrone Quadcopter. It uses the official ardrone_autonomy drivers to do the commanding and getting the navdata

The commands for roll pitch and yaw are in units of radians. The thrust is normalized to -1 to 1 since we dont know the units for ar-drone drivers

This class uses internal locking to ensure the quaddata is available to Qt thread without bumping into ros serial. This is enough since the order of working is not important to us. Read internal vs external locking: http://www.boost.org/doc/libs/1_55_0/doc/html/thread/synchronization.html
*/
#include "parsernode/parser.h"
#include <ardrone_autonomy/Navdata.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>

using namespace std;
namespace ardroneparser{
class ArdroneParser: public parsernode::Parser
	{

		private://Members depicting the state of the quadcopter
      ros::NodeHandle nh_;///< Internal node handle
			parsernode::common::quaddata data;
			boost::mutex spin_mutex;
			
		protected:
			//Subscribers
			ros::Subscriber navdata_sub;
			//Publishers
			ros::Publisher takeoff_pub;
			ros::Publisher land_pub;
			ros::Publisher reset_pub;
			ros::Publisher cmdrpyt_pub;

			//ServiceClients
			ros::ServiceClient imurecalib_client;

		public:
			ArdroneParser();
			~ArdroneParser()
			{
				navdata_sub.shutdown();
				takeoff_pub.shutdown();
				reset_pub.shutdown();
				cmdrpyt_pub.shutdown();
				imurecalib_client.shutdown();
			}
			void initialize();
			void navCallback(const ardrone_autonomy::Navdata::ConstPtr navdata_);
			bool cmdrpythrust(geometry_msgs::Quaternion &rpytmsg, bool sendyaw = false);
			bool land();
			bool disarm();
			bool takeoff();
			bool reset();
			bool calibrateimubias();
			void estimatethrustbias();
      void setmode(std::string mode)
      {
        //TODO NOT IMPLEMENTED
      }
			void reset_attitude(double roll, double pitch, double yaw)
			{
				//Not Implemented
			}
		  void getquaddata(parsernode::common::quaddata &d1);
			void setlogdir(string logdir)
			{
				//TODO NOT IMPLEMENTED
				return;
			}
			void controllog(bool logswitch)
			{
				//TODO NOT IMPLEMENTED
				return;
			}
      bool flowControl(bool) {
				//TODO NOT IMPLEMENTED
        return false;
      }
			void setaltitude(double altitude_)//Set the altitude value in the data
			{
				spin_mutex.lock();
				data.altitude = altitude_;
				spin_mutex.unlock();
			}
			void grip(int state)
			{
				//NOT Needed
				return;
			}
      bool cmdvelguided(geometry_msgs::Vector3 &vel_cmd, double &yaw_ang) {
        return false;
      }
      bool cmdvel_yaw_rate_guided(geometry_msgs::Vector3 &vel_cmd, double &yaw_rate) {
        return false;
      }
      bool cmdvel_yaw_angle_guided(geometry_msgs::Vector3 &vel_cmd, double &yaw_ang) {
        return false;
      }
      bool cmdwaypoint(geometry_msgs::Vector3 &desired_pos, double desired_yaw = 0) {
        return false;
      }
	};
};
#endif // ARDRONE_PARSHER_H
