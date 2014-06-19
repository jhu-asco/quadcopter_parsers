/* This is the plugin which is dependent on Parser base class. 
*/
#include <pluginlib/class_list_macros.h>
#include <rqt_quadcopter_parsers/ardroneparser.h>
namespace rqt_quadcopter_parsers{


ArdroneParser::ArdroneParser()
{
}

//PluginLib Initialization function
void ArdroneParser::initialize(ros::NodeHandle &nh_)
{
	//Re initialize the base members parameters as needed:
	data.mass = 0.42;
	data.thrustbias = data.mass*9.81; 
	data.thrustmax = data.thrustbias + 0.2*9.81;//Additional payload of 200gm
	data.thrustmin = data.thrustbias - 0.2*9.81;
	data.rpbound = M_PI/4;//This is the physical limit enforced by many drivers. This is not the same as the controller bound on angles
	//Setup the subscribers and services needed
	navdata_sub = nh_.subscribe("/ardrone/navdata",1,&ArdroneParser::navCallback,this);
	takeoff_pub = nh_.advertise<std_msgs::Empty>("/ardrone/takeoff",1);
	land_pub = nh_.advertise<std_msgs::Empty>("/ardrone/land",10);
	reset_pub = nh_.advertise<std_msgs::Empty>("/ardrone/reset",1);
	cmdrpyt_pub = nh_.advertise<geometry_msgs::Twist>("/cmd_vel",1);
	imurecalib_client = nh_.serviceClient<std_srvs::Empty>("/ardrone/imu_recalib");
}

void ArdroneParser::navCallback(const ardrone_autonomy::Navdata::ConstPtr navdata_)
{
	spin_mutex.lock();
	//Battery Status
	data.batterypercent = navdata_->batteryPercent;
	//Status
	switch(navdata_->state)
	{
		case 0:
			data.quadstate = "Unknown DISARMED";
			data.armed = false;
			break;
		case 1:
			data.quadstate = "Init DISARMED";
			data.armed = false;
			break;
		case 2:
			data.quadstate = "Landed DISARMED";
			data.armed = false;
			break;
		case 3:
			data.quadstate = "Flying ARMED";
			data.armed = true;
			break;
		case 4:
			data.quadstate = "Hovering ARMED";
			data.armed = true;
			break;
		case 6:
			data.quadstate = "Taking off ARMED";
			data.armed = true;
			break;
		case 7:
			data.quadstate = "Flying ARMED";
			data.armed = true;
			break;
		case 8:
			data.quadstate = "Landing DISARMED";
			data.armed = false;
		/*case 5:
			data.quadstate = "Test";
			break;
		case 7:
			data.quadstate = "Goto Fix Point";
			break;
		case 8:
			data.quadstate = "Landing";
			break;
		case 9:
			data.quadstate = "Looping";
			break;
			*/
		default:
			sprintf(&data.quadstate[0],"%d",navdata_->state);
			data.armed = false;
			//data.quadstate = std::to_string(navdata_->state);
			break;
	}
	//Magnetometer
	data.magdata.x = navdata_->magX; data.magdata.y = navdata_->magY; data.magdata.z = navdata_->magZ;

	//pressure sensor
	data.pressure =  navdata_->pressure;

	//temperature sensor
	data.temperature = navdata_->temp;

	//wind sensing...
	data.wind_speed = navdata_->wind_speed;
	data.wind_angle = navdata_->wind_angle;

	//Roll pitch yaw data radians 
	data.rpydata.y = navdata_->rotY*(M_PI/180); 
	data.rpydata.x = navdata_->rotX*(M_PI/180);
	data.rpydata.z = navdata_->rotZ*(M_PI/180);
/*	if(data.rpydata.x < 0)
		data.rpydata.x = -M_PI - navdata_->rotX*(M_PI/180);
	else 
		data.rpydata.x = M_PI - navdata_->rotX*(M_PI/180);

	if(data.rpydata.z < 0)
		data.rpydata.z = -M_PI - navdata_->rotZ*(M_PI/180);
	else 
		data.rpydata.z =M_PI - navdata_->rotZ*(M_PI/180);
		*/

	//altitude.rserinstance
	data.altitude = navdata_->altd/100;//to get into meters

	//Velocities
	data.linvel.x = navdata_->vx/1000.0; data.linvel.y = navdata_->vy/1000.0; data.linvel.z = navdata_->vz/1000.0;

	//Acc values
	data.linacc.x = navdata_->ax; data.linacc.y = navdata_->ay; data.linacc.z = navdata_->az;

	//Motor pwm values
	data.motorpwm[0] = navdata_->motor1;
	data.motorpwm[1] = navdata_->motor2;
	data.motorpwm[2] = navdata_->motor3;
	data.motorpwm[3] = navdata_->motor4;

	//time stamp
	data.timestamp = navdata_->tm;
	spin_mutex.unlock();
}
bool ArdroneParser::takeoff()//Virtual override function
{
	std_msgs::Empty emptymsg;
	takeoff_pub.publish(emptymsg);
	return true;
}
bool ArdroneParser::land()
{
	data.armed = false;//Disable arming to ensure the controller wont send commands
	std_msgs::Empty emptymsg;
	for(int count = 0;count < 10;count++)
		land_pub.publish(emptymsg);
	return true;
}
bool ArdroneParser::disarm()
{
	std_msgs::Empty emptymsg;
	for(int count = 0;count < 10;count++)
		land_pub.publish(emptymsg);
	return true;
}
bool ArdroneParser::reset()
{
	std_msgs::Empty emptymsg;
	reset_pub.publish(emptymsg);
	return true;
}
bool ArdroneParser::calibrateimubias()
{
	std_srvs::Empty emptymsg;
	bool res = false;
	if(imurecalib_client.call(emptymsg))
	{
		ROS_INFO("Calibrating bias imu");
		res = true;
	}
	else
	{
		ROS_WARN("Failed to call imubias calib");
		res = false;
	}
	return res;
}
bool ArdroneParser::cmdrpythrust(geometry_msgs::Quaternion &rpytmsg, bool sendyaw)
{
	geometry_msgs::Twist cmdmsg;

	cmdmsg.linear.x = parsernode::common::map(rpytmsg.y-data.rpydata.x,-data.rpbound, data.rpbound, -1,1);//Pitch moving forward/backward
	cmdmsg.linear.y = parsernode::common::map(-rpytmsg.x-data.rpydata.y,-data.rpbound, data.rpbound, -1,1);//Roll moving Left/Right
	cmdmsg.angular.z  = rpytmsg.z;//rateYaw 
	cmdmsg.linear.z = parsernode::common::map(rpytmsg.w, data.thrustmin,data.thrustmax,-1,1);//Thrust
	cmdmsg.angular.x = 1;
	cmdmsg.angular.y = 1;//Random values to make sure it does not go into autohover mode
	ROS_INFO("Commandrpyt: %f\t%f\t%f\t%f\t",cmdmsg.linear.x,cmdmsg.linear.y
			,cmdmsg.angular.z,cmdmsg.linear.z);//Later can be changed to debug
	cmdrpyt_pub.publish(cmdmsg);
	return true;
}
void ArdroneParser::getquaddata(parsernode::common::quaddata &d1)
{
	spin_mutex.lock();
	d1 = data;
	spin_mutex.unlock();
	return;
}
void ArdroneParser::estimatethrustbias()
{
	//Not Implemented for Ardrone
	return;
}

};
PLUGINLIB_DECLARE_CLASS(rqt_quadcopter_parsers, ArdroneParser, rqt_quadcopter_parsers::ArdroneParser, parsernode::Parser)
