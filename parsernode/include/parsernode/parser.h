/* This program is for parsing various quadcopter data and storing them as internal variables. This is a parent library and 
provides virtual functions for the subclasses to implement. 
	The data that needs to be stored (copied from ardrone_autonomy should be similar or more can be added based on generality). 
		* batteryPercent: The remaining charge of the drone's battery (%)
		* state: The Drone's current state as a string form
		* rotX: Left/right tilt in radians (rotation about the X axis)
		* rotY: Forward/backward tilt in radians (rotation about the Y axis)
		* rotZ: Orientation in radians (rotation about the Z axis)
		* magX, magY, magZ: Magnetometer readings (Unit undecided)
		* pressure: Pressure sensed by Drone's barometer (Unit undecided)
		* temp : Temperature sensed by Drone's sensor(Unit undecided)
		* wind_speed: Estimated wind speed (Unit undecided)
		* wind_angle: Estimated wind angle (Unit undecided)
		* altd: Estimated altitude (m)
		* motor1..4: Motor PWM values (Raw units provided by driver) 
		* vx, vy, vz: Linear velocity (m/s)
		* ax, ay, az: Linear acceleration (g)
		* tm: Timestamp of the data returned by the Drone returned as number of micro-seconds passed since Drone's boot-up.

		The parser needs to convert all the data into NWU format for the user interface and controllers to make sense of it. It should also publish the Imu data in the sensor_msgs/Imu format if it is not already done by the driver. This will be useful to maintain standard format for controllers to use.
		The rpyt commands needs to be given in the form of a quaternion with x,y,z representing roll pitch yaw and w representing thrust. the max and min values for rpy are -Pi to Pi. The max and min values for thrust are set in the parser. This will be later set as parameters or inputs from UI
		Possible States for quadstate which can be used in the UI:
			* 0 -> Unknown
			* 1 -> Init
			* 2 -> Landed
			* 3 -> Flying
			* 4 -> Taking off
//Controller should ensure that the commanded angles are between -pi to pi 
*/
#ifndef PARSER_H
#define PARSER_H
#include <fstream>
#include <iostream>
#include "ros/ros.h"
#include "ros/time.h"
#include <geometry_msgs/TransformStamped.h>
//#include <tf/transform_broadcaster.h>//Looks like problems with boost signals need to additional work to figure this out 
#include <geometry_msgs/Vector3.h>

#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#include <sensor_msgs/Imu.h>

#include <parsernode/common.h>
#include <boost/thread/mutex.hpp>

//#include "controllers/ctrl_command.h"//for roll pitch, yaw and thrust
//#include "controllers/PIDGains.h"
//#include <DspFilters/Butterworth.h>
using namespace std;
namespace parsernode{
class Parser
{
	public: //Virtual Functions for commanding the drone. These should be implemented by the subclasses based on the quadcopter we are working with.
		virtual bool takeoff()=0;//Take off the quadcopter
		virtual bool land()=0;//Land the quadcopter
		virtual bool disarm()=0;//Disarm the quadcopter
		virtual bool flowControl(bool)=0;//Enable or disable control of quadcopter
		virtual bool calibrateimubias()=0;//Calibrate imu of the quadcopter based on sample data
		virtual bool cmdrpythrust(geometry_msgs::Quaternion &rpytmsg)=0;//Command the euler angles of the quad and the thrust. the msg format is (x,y,z,w) -> (roll, pitch, yaw, thrust). The thrust value will be adjusted based on whethere there is thrust bias or not.
		virtual bool cmdrpyawratethrust(geometry_msgs::Quaternion &rpytmsg)=0;//Command the euler angles of the quad and the thrust. the msg format is (x,y,z,w) -> (roll, pitch, yawrate, thrust). The thrust value will be adjusted based on whethere there is thrust bias or not.
    virtual bool cmdvel_yaw_rate_guided(geometry_msgs::Vector3 &vel_cmd, double &yaw_rate)=0;
    virtual bool cmdvel_yaw_angle_guided(geometry_msgs::Vector3 &vel_cmd, double &yaw_ang)=0;
    virtual bool cmdwaypoint(geometry_msgs::Vector3 &desired_pos, double desired_yaw = 0)=0;
		virtual void grip(int state)=0;//Tri State Gripper
		virtual void reset_attitude(double roll, double pitch, double yaw)=0;

		//PluginLib initialization function
		virtual void initialize()=0;
		virtual void getquaddata(common::quaddata &d1)=0;
		virtual ~Parser()
		{
		}
		virtual void setaltitude(double altitude_)=0;//Set the altitude value in the data
		virtual void setlogdir(string logdir)=0;//Set whether to log data or not
		virtual void controllog(bool logswitch)=0;
    //const ros::NodeHandle &nh;
  public:
    bool initialized;
    bool imu_reset;///< If IMU Data changed abruptly due to reset, recalibration etc
	protected: 
    Parser():initialized(false){};
	private:
			common::quaddata data;
};
};
#endif //Parser_H

