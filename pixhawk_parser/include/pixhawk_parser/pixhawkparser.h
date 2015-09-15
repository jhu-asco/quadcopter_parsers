#ifndef APM_PARSER_H
#define APM_PARSHER_H
/* This header is subclassed from parser.h and will include data from Ardrone Quadcopter. It will also provide interfaces for commanding the Ardrone Quadcopter. It uses the official ardrone_autonomy drivers to do the commanding and getting the navdata

The commands for roll pitch and yaw are in units of radians. The thrust is normalized to -1 to 1 since we dont know the units for ar-drone drivers

This class uses internal locking to ensure the quaddata is available to Qt thread without bumping into ros serial. This is enough since the order of working is not important to us. Read internal vs external locking: http://www.boost.org/doc/libs/1_55_0/doc/html/thread/synchronization.html
*/
#include <tf/LinearMath/Transform.h>
#include "parsernode/parser.h"//main parser base class
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>
//Messages:
//#include "mavlink/Mavlink.h"

#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Empty.h"

#include "geometry_msgs/Vector3.h"

#include <ardupilotmega/mavlink.h>//Mavlink messages
#include "common_definitions.h"

// Copied from mavlink_ros:

#include <sys/time.h>
#include <time.h>
#include <bitset>         // std::bitset


// Standard includes
#include <iostream>
#include <cstdlib>
#include <unistd.h>
#include <cmath>
#include <string.h>
#include <inttypes.h>
#include <fstream>

// Serial includes
#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#ifdef __linux
#include <sys/ioctl.h>
#endif

#include <boost/thread.hpp>

//Reconfig includes:
#include <dynamic_reconfigure/server.h>
#include <pixhawk_parser/PixhawkTuningInterfaceConfig.h>

#define THROTCHAN 2
#define SERVONUM 4

#ifndef FILE_BUFFER_SIZE
#define FILE_BUFFER_SIZE 1024
#endif
//#define NOFJOINTS 2
//#define ANGRES 0.088 

using namespace std;
namespace pixhawk_parser{
class PixhawkParser: public parsernode::Parser
	{

		private://Members depicting the state of the quadcopter
			parsernode::common::quaddata data;
			boost::mutex spin_mutex;
			uint8_t targetsys_id;
			uint8_t targetcomp_id;
			//float RC_TRIM[SERVONUM], RC_MIN[SERVONUM], RC_MAX[SERVONUM];
			float *RC_TRIM; float *RC_MIN; float *RC_MAX;
			//uint16_t *RC_ID;
			//uint16_t *ARM_ID;
			//float* ARM_TRIM; float *ARM_MIN; float *ARM_MAX;
			ros::Timer rctimer;
			ros::Timer armtimer;
			double basealtitude;
			geometry_msgs::Quaternion rctimerrpytmsg;
			//int rctimercount;
			int intmode;//Internal mode not intending to expose 
			ofstream cmdfile;//Cmd logging
			ofstream servofile;//Raw servo pwm logging
			ofstream rcinputfile;//Raw rc input logging
			ofstream imufile;//Imu data logging
			//Create Buffers for each of these files:
			char cmdfile_buffer[FILE_BUFFER_SIZE];//Buffer for ofstream
			char imufile_buffer[FILE_BUFFER_SIZE];//Buffer for ofstream
			char servofile_buffer[FILE_BUFFER_SIZE];//Buffer for ofstream
			char rcinputfile_buffer[FILE_BUFFER_SIZE];//Buffer for ofstream
			bool enable_log;
		  bool countstar;
		  //float *goalpwm;
			//float *temparm_pwm;
			//float kparm;
			/* Members from Mavlink_ros*/
			// Settings
			struct timeval tv;		  ///< System time
			int baud;
			int sysid;             ///< The unique system id of this MAV, 0-127. Has to be consistent across the system
			int compid;
			int serial_compid;
			std::string port;              ///< The serial port name, e.g. /dev/ttyUSB0
			bool silent;              ///< Wether console output should be enabled
			bool verbose;             ///< Enable verbose output
			bool debug;               ///< Enable debug functions and output
			bool pc2serial;			  ///< Enable PC to serial push mode (send more stuff from pc over serial)
			int fd;
			std::string frame_id;
			//ros::Timer serialtimer;
			boost::thread* serial_recvthread;
			double gain_throttle; //This is conversion factor from throttle cmd in Newtons to pwm
			double intercept_throttle; //This is constant addition in the linear conversion (mx + c) this is c and above one is m
      geometry_msgs::Vector3 prev_imudata;

    private:
      boost::shared_ptr<ros::NodeHandle> private_nh_;///For Reconfigure and personal publishing

      boost::shared_ptr<dynamic_reconfigure::Server<PixhawkTuningInterfaceConfig> > reconfigserver;

      dynamic_reconfigure::Server<PixhawkTuningInterfaceConfig>::CallbackType reconfigcallbacktype;

			ros::Publisher ekf_status_pub;

      struct Current_Params{
        PixhawkTuningInterfaceConfig current_tuning_params_;
        std::map<std::string, MAV_PARAM_TYPE> param_type;
      }current_params_;

      int parameter_find_count;

			///Reconfig Callback
      void reconfigCallback(PixhawkTuningInterfaceConfig &tuning_params, uint32_t level);

		protected:
			//Publishers:
			//ros::Publisher mavlink_pub;

			bool heartbeatinit;
			//Personal Callback functions:
			//void paramsetreqCallback(std_msgs::String datatype);
			void modereqCallback(const std_msgs::String &datatype);
			void datareqCallback(const std_msgs::String &datatype);
			void paramreqCallback(const std_msgs::Empty &emptymsg);
			void rctimerCallback(const ros::TimerEvent &event);
			void armtimerCallback(const ros::TimerEvent &event);
			//Functions from mavlink_ros for serial control:
			int open_port(std::string& port);
			bool setup_port(int , int , int , int , bool , bool );
			void close_port(int fd);
			void serialtimerCallback();
			void mavlinkPublish(const mavlink_message_t &msg);

			//Subscribers:
			//ros::Subscriber mavlink_sub;
			/*
				 ros::Subscriber paramenq_sub;
				 ros::Subscriber datareq_sub;
				 ros::Subscriber commandreq_sub;
				 ros::Subscriber rcoverridereq_sub;
				 ros::Subscriber modereq_sub;
				 ros::Subscriber paramsetreq_sub;
				 ros::Subscriber armsetreq_sub;
			 */

		public:
			PixhawkParser();
			~PixhawkParser()
			{
				//Setup the data to be stopped:
				std_msgs::String dataparseval;
				dataparseval.data = "ATTITUDE START 0";
				PixhawkParser::datareqCallback(dataparseval);
				dataparseval.data = "EXTENDED START 0";
				PixhawkParser::datareqCallback(dataparseval);
				dataparseval.data = "RADIO START 0";
				PixhawkParser::datareqCallback(dataparseval);
				dataparseval.data = "RAW START 0";
				PixhawkParser::datareqCallback(dataparseval);
				dataparseval.data = "EXTRA3 START 0";
				PixhawkParser::datareqCallback(dataparseval);
				tcflush(fd, TCIOFLUSH); 
  			close_port(fd);//Shutdown the serial port
				//usleep(10000);
				//First stop the data
				//mavlink_pub.shutdown();
				//mavlink_sub.shutdown();
				cmdfile.close();
				servofile.close();
				rcinputfile.close();
				imufile.close();
				//serial_recvthread->join();
			}
			void initialize(ros::NodeHandle &nh_);
			bool cmdrpythrust(geometry_msgs::Quaternion &rpytmsg, bool sendyaw = false);
			inline void sendradio(uint16_t r1,uint16_t r2,uint16_t r3,uint16_t r4);
			//void foldarm();
			//void setarmpwm(double *armpwm);
			//void setarmpwm(std::vector<float> &armpwm);
			//For gripper the angle is either -ve or +ve (does not care abt  values) Also
			// the angles are set about the trim value of 180 degrees. The angles should not exceed +/-90 for now
			//Angle input is in degrees
			//void setarmangles(std::vector<float> &armangles);
			//void setarmangles(double *armangles);
			void grip(int state);
      void prearmCalibrate();//Preflight Calibration
      inline void setParameter(std::string id, float parameter_value, MAV_PARAM_TYPE param_type = MAV_PARAM_TYPE_UINT8);
			bool land();
			bool reset();
			bool takeoff();
			bool disarm();
			bool calibrateimubias();
			void getquaddata(parsernode::common::quaddata &d1);
			void estimatethrustbias();
			void reset_attitude(double roll, double pitch, double yaw);
			void setlogdir(string logdir)
			{
				cmdfile.open((logdir+"/cmd.dat").c_str());
				servofile.open((logdir+"/servo.dat").c_str());
				rcinputfile.open((logdir+"/rcinput.dat").c_str());
				imufile.open((logdir+"/imu.dat").c_str());

				cmdfile.precision(10);
				servofile.precision(10);
				rcinputfile.precision(10);
				imufile.precision(10);
				//Create Buffer:
				imufile.rdbuf()->pubsetbuf(imufile_buffer, FILE_BUFFER_SIZE);
				cmdfile.rdbuf()->pubsetbuf(cmdfile_buffer, FILE_BUFFER_SIZE);
				servofile.rdbuf()->pubsetbuf(servofile_buffer, FILE_BUFFER_SIZE);
				imufile.rdbuf()->pubsetbuf(imufile_buffer, FILE_BUFFER_SIZE);
				//#DEBUG Print Buffer size:
				std::cout<<"Imu file buffer size: "<<imufile.rdbuf()->in_avail();

				cmdfile<<"#Time\t Roll \t Pitch \t Yaw \t Thrust"<<endl;
				servofile<<"#Time\t SERVO_1\t SERVO_2\t SERVO_3\t SERVO_4\t TIME_US\t BATT_VOLTS"<<endl;
				rcinputfile<<"#Time\t RC_1\t RC_2\t RC_3\t RC_4"<<endl;
				imufile<<"#Time\t Roll \t Pitch \t Yaw"<<endl;
			}
			void controllog(bool logswitch)
			{
				enable_log = logswitch;
			}
			void setaltitude(double altitude_)//Set the altitude value in the data
			{
				spin_mutex.lock();
				data.altitude = altitude_;
				spin_mutex.unlock();
			}
	};
};
#endif // ARDRONE_PARSHER_H
