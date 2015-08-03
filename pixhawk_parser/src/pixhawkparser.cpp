/* This is the plugin which is dependent on Parser base class. 
 */
#include <pluginlib/class_list_macros.h>
#include <pixhawk_parser/pixhawkparser.h>
namespace pixhawk_parser{

	PixhawkParser::PixhawkParser()
	{
	}
	//PluginLib Initialization function
	void PixhawkParser::initialize(ros::NodeHandle &nh_)
	{ 
		heartbeatinit = false;
		enable_log = false;
		targetcomp_id = MAV_COMP_ID_SYSTEM_CONTROL;
		targetsys_id = 1;
		gain_throttle = 81.6;//Throttle gain should be integrated forward to get the right value
		intercept_throttle = 189.5;
		//RC_TRIM = new float[SERVONUM]{1516,1516,1923,1514};
		//RC_MIN = new float[SERVONUM]{1108,1103,1105,1101};
		//RC_MAX = new float[SERVONUM]{1932,1929,1926,1926};

		//RC_ID = new uint16_t[SERVONUM]{56,61,66,71};//Arducopter code 3.1.2
		//RC_ID = new uint16_t[SERVONUM]{53,58,63,68}; Old code
		RC_TRIM = new float[SERVONUM]{1515,1516,1109,1514};
		RC_MIN = new float[SERVONUM]{1093,1090,1106,1097};
		RC_MAX = new float[SERVONUM]{1916,1914,1936,1923};
		/*ARM_ID = new uint16_t[NOFJOINTS]{1,2};
		ARM_MIN = new float[NOFJOINTS]{937,0};
		ARM_MAX = new float[NOFJOINTS]{2337,3070};
		ARM_TRIM = new float[NOFJOINTS]{2167,2047};//When arm is outstretched
		//goalpwm = new float[NOFJOINTS]{1500,1500,1500};//Initialization
		goalpwm = new float[NOFJOINTS+1]{ARM_TRIM[0],ARM_MIN[1],500};//Initialization can use malloc These are the default values when it does not have any joy input
		temparm_pwm = new float[NOFJOINTS]{ARM_TRIM[0],ARM_MIN[1]};//Dont need the last one which is just a gripper also use malloc
		//kparm = 0.02;
		kparm = 0.2;
		*/
		/*
			 for(int count1 = 0;count1 < SERVONUM;count1++)
			 {
			 RC_ID[count1] = 56+5*count1; 
			 RC_TRIM[count1] = 1515; RC_MIN[count1] = 1100; RC_MAX[count1] = 1929;
			 }
		 */
		//Re initialize the base members parameters as needed:
		data.mass = 0.25;//Start with small  values
		data.thrustbias = data.mass*9.81; //We can estimate this later
		data.thrustmax = data.thrustbias + 0.5*9.81;//Additional payload of 500gm
		data.thrustmin = 0;
		data.rpbound = M_PI/4;//This is the physical limit enforced by many drivers. This is not the same as the controller bound on angles
		//Setup the subscribers and services needed
		//Publishers:
		//mavlink_pub = nh_.advertise<mavlink_ros::Mavlink>("/mavlink/to", 1000);
		//mavlink_sub = nh_.subscribe("/mavlink/from",1000,&PixhawkParser::mavlinkCallback,this);
		//rctimercount = 0;
		rctimerrpytmsg.x = 0; rctimerrpytmsg.y = 0; rctimerrpytmsg.z = 0; rctimerrpytmsg.w = 0;//Initialize the rpytmsg used by rctimer
		intmode = -1;
		rctimer = nh_.createTimer(ros::Duration(0.02), &PixhawkParser::rctimerCallback,this);//50Hz
		//armtimer = nh_.createTimer(ros::Duration(0.1), &PixhawkParser::armtimerCallback,this);//10Hz
		rctimer.stop();//Stop the timer to begin with
		//armtimer.stop();
		//armtimer.start();
		//Initialized Mavlink_ros variables from parameters:
		nh_.param<int>("/serial/sysid", sysid, 255);
		nh_.param<int>("/serial/compid", compid, 110);
		nh_.param<int>("/serial/baudrate", baud, 115200);
		nh_.param<std::string>("/serial/port", port, "/dev/ttyUSB0");
		silent = true;              ///< Wether console output should be enabled
		verbose = false;             ///< Enable verbose output
		debug = false;               ///< Enable debug functions and output
		pc2serial = true;			  ///< Enable PC to serial push mode (send more stuff from pc over serial)
		serial_compid = 0;//Not needed for parameters
		silent = false;
		verbose = false;
		debug = false;
		pc2serial= true;
		frame_id = std::string("fcu");//DONT KNOW IF WE NEED THIS
		//SETUP SERIAL PORT:
		if (!silent)
			printf("Trying to connect to %s.. ", port.c_str());
		fd = open_port(port);
		if (fd == -1)
		{
			if (!silent)
				fprintf(stderr, "failure, could not open port.\n");
			exit(EXIT_FAILURE);
		}
		else
		{
			if (!silent)
				printf("success.\n");
		}
		if (!silent)
			printf("Trying to configure %s.. ", port.c_str());
		bool setup = setup_port(fd, baud, 8, 1, false, false);
		if (!setup)
		{
			if (!silent)
				fprintf(stderr, "failure, could not configure port.\n");
			exit(EXIT_FAILURE);
		}
		else
		{
			if (!silent)
				printf("success.\n");
		}
		//Initialize timer for reading serial data:
		//serialtimer = nh_.createTimer(ros::Duration(0.002), &PixhawkParser::serialtimerCallback,this);//500Hz
		//serialtimer.start();
		serial_recvthread =  new boost::thread(boost::bind(&PixhawkParser::serialtimerCallback, this));
		if (!silent)
			printf("\nREADY, waiting for serial/ROS data.\n");
		int noErrors = 0;
		if (fd == -1 || fd == 0)
		{
			if (!silent)
				fprintf(stderr, "Connection attempt to port %s with %d baud, 8N1 failed, exiting.\n", port.c_str(), baud);
			exit(EXIT_FAILURE);
		}
		else
		{
			if (!silent)
				fprintf(stderr, "\nConnected to %s with %d baud, 8 data bits, no parity, 1 stop bit (8N1)\n", port.c_str(), baud);
		}
	}




	/*
	Removed this function
		 void PixhawkParser::armtimerCallback(const ros::TimerEvent &event)
		 {
	//Sending arm pwm 
	mavlink_message_t mavmsg;
	mavlink_arm_ctrl_pwm_t armpwm_msg;
	for(int count1 = 0;count1 < NOFJOINTS;count1++)//Dont need to do the last one as it is a gripper
	{
	//temparm_pwm[count1] = goalpwm[count1]>temparm_pwm[count1]?(temparm_pwm[count1]+3):(goalpwm[count1] < temparm_pwm[count1]?(temparm_pwm[count1]-3):temparm_pwm[count1]);
	temparm_pwm[count1] = (temparm_pwm[count1] + kparm*(goalpwm[count1] - temparm_pwm[count1]));
	}
	armpwm_msg.pwm1 = (uint16_t)temparm_pwm[0];
	armpwm_msg.pwm2 = (uint16_t)temparm_pwm[1];
	armpwm_msg.pwm3 = (uint16_t)goalpwm[2];
	//Add log for arm pwm also TODO
	cout<<"Armpwm: "<<armpwm_msg.pwm1<<"\t"<<armpwm_msg.pwm2<<"\t"<<armpwm_msg.pwm3<<endl;//Will put this in the data for gui TODO
	mavlink_msg_arm_ctrl_pwm_encode(hostsysid,hostcompid,&mavmsg,&armpwm_msg);
	mavlink_ros::Mavlink mavlink_ros_msg;
	castmavmsgtoros(mavlink_ros_msg,mavmsg); 
	if(ros::ok())
	mavlink_pub.publish(mavlink_ros_msg);
	}
	 */
	void PixhawkParser::rctimerCallback(const ros::TimerEvent &event)
	{
		switch(intmode)
		{
			case 0://LAND
				if(data.altitude <= (basealtitude+0.05))//Verify this manually and also put some margin 5 cm
				{
					ROS_INFO("Disarming");
					rctimer.stop();	
					PixhawkParser::disarm();
				}
				else
				{
					rctimerrpytmsg.w -= 0.001*9.81;//Constant difference
					PixhawkParser::cmdrpythrust(rctimerrpytmsg);
				}
				break;
			case 1://Estimate Thrustbias:
				ROS_INFO("Estimating Thrust Bias");
				//if(data.linacc.z > 2)//It is to begin with 9.81 and should reduce as the thrust is raised
				if((data.altitude <= (basealtitude+0.005))&&(rctimerrpytmsg.w < 1.2*data.thrustbias))//Verify this manually and also put some margin
				{
					rctimerrpytmsg.w += 0.001*9.81;//Constant difference
					PixhawkParser::cmdrpythrust(rctimerrpytmsg);
				}
				else
				{
					data.thrustbias = rctimerrpytmsg.w;//Using integrator now so thrust bias may not be necessary	
					data.thrustmax = data.thrustbias + 0.5*9.81;//Additional payload of 500gm
					ROS_INFO("Estimated thrustbias: %f\t Difference in altitude: %f",data.thrustbias, (data.altitude - basealtitude));
					PixhawkParser::disarm();
					//rctimerrpytmsg.x = 0; rctimerrpytmsg.y = 0; rctimerrpytmsg.z = 0; rctimerrpytmsg.w = 0;
					//PixhawkParser::cmdrpythrust(rctimerrpytmsg);//
					rctimer.stop();	
				}
				/*rctimercount--;
					if(rctimercount == 0)
					{
					rctimer.stop();	
					}
				 */
				break;
		}
	}

	bool PixhawkParser::takeoff()//Virtual override function
	{
		geometry_msgs::Quaternion rpytmsg;
		for(int count = 0;count < 30;count++)//3Sec
		{
			PixhawkParser::sendradio(RC_TRIM[0],RC_TRIM[1],RC_MIN[2],RC_MAX[3]);
			usleep(100000);
		}
		PixhawkParser::sendradio(RC_TRIM[0],RC_TRIM[1],RC_MIN[2],RC_TRIM[3]);
		basealtitude = data.altitude;//Storing the ground pressure for us to be able to land and estimate thrustbias
		return true;
		/*
			 mavlink_message_t mavmsg;
		//construct command
		mavlink_command_long_t mavcommandreq;
		mavcommandreq.param1 = 10; //Hz
		mavcommandreq.target_system = targetsys_id;
		mavcommandreq.target_component = targetcomp_id;
		mavcommandreq.command = MAV_CMD_COMPONENT_ARM_DISARM;
		mavcommandreq.param1 = 1.0f;//for arming
		//encode the mavlink_data_stream_t into mavlink_message_t
		mavlink_msg_command_long_encode(hostsysid,hostcompid,&mavmsg,&mavcommandreq);
		//sysid and compid are write now just hardcoded.Later on will change
		mavlink_ros::Mavlink mavlink_ros_msg;
		castmavmsgtoros(mavlink_ros_msg,mavmsg); 
		ROS_INFO("Arming Quadcopter");
		mavlink_pub.publish(mavlink_ros_msg);
		ROS_INFO("Publishing done");
		return true;
		 */
		//Trying another way to arm:
	}

	bool PixhawkParser::reset()
	{
		//Not implemented for pixhawk
		return false;
	}

	bool PixhawkParser::land()
	{
		std_msgs::String modeparse;
		modeparse.data = "STABILIZE";
		PixhawkParser::modereqCallback(modeparse);
		//double diff = (data.thrustbias - data.thrustmin)/20.0;
		rctimerrpytmsg.x = 0; rctimerrpytmsg.y = 0; rctimerrpytmsg.z = 0;rctimerrpytmsg.w = 0.9*data.thrustbias;//Some bias to go downwards
		intmode = 0;//Land
		rctimer.start();
		//Some way of knowing that it stopped
		//usleep(1e6);
		//PixhawkParser::disarm();
		return true;
	}

	bool PixhawkParser::calibrateimubias()
	{
		return false;
		//Not Implemented as of now
	}

	void PixhawkParser::estimatethrustbias()
	{
		//set data.thrustbias by trying to lift the quadcopter off the ground:TODO
		//Check az and pressure while increasing the thrust and see if makes any change:
		//setup a timer to do thrust increase so it will still respond to land and disarm commands
		rctimerrpytmsg.x = 0; rctimerrpytmsg.y = 0; rctimerrpytmsg.z = 0; rctimerrpytmsg.w = 0;
		intmode = 1;//Estimate Thrust bias
		rctimer.start();
	}

	void PixhawkParser::reset_attitude(double &roll, double &pitch, double &yaw)
	{
		//Convert into NED frame from vrpn frame
		mavlink_message_t mavmsg;
		//construct command
		mavlink_attitude_t attitude_reset_req;
		attitude_reset_req.roll =  parsernode::common::map_angle(roll);
		attitude_reset_req.pitch =  -parsernode::common::map_angle(pitch);
		attitude_reset_req.yaw =  -parsernode::common::map_angle(yaw);
		//encode the mavlink_data_stream_t into mavlink_message_t
		mavlink_msg_attitude_encode(hostsysid,hostcompid,&mavmsg,&attitude_reset_req);
		//sysid and compid are write now just hardcoded.Later on will change
		//castmavmsgtoros(mavlink_ros_msg,mavmsg); 
		PixhawkParser::mavlinkPublish(mavmsg);
		//mavlink_pub.publish(mavlink_ros_msg);
		//ROS_INFO("Publishing Reset req done");[DEBUG]
	}

	bool PixhawkParser::disarm()
	{
		rctimer.stop();	//Stop the timer just in case
		for(int count = 0;count < 80;count++)//4Sec
		{
			PixhawkParser::sendradio(RC_TRIM[0],RC_TRIM[1],RC_MIN[2],RC_MIN[3]);
			usleep(50000);
		}
		PixhawkParser::sendradio(RC_TRIM[0],RC_TRIM[1],RC_MIN[2],RC_TRIM[3]);
		return true;
		/*
			 mavlink_message_t mavmsg;
		//construct command
		mavlink_command_long_t mavcommandreq;
		mavcommandreq.param1 = 0; //Hz
		mavcommandreq.target_system = targetsys_id;
		mavcommandreq.target_component = targetcomp_id;
		mavcommandreq.command = MAV_CMD_COMPONENT_ARM_DISARM;
		mavcommandreq.param1 = 0.0f;
		//encode the mavlink_data_stream_t into mavlink_message_t
		mavlink_msg_command_long_encode(hostsysid,hostcompid,&mavmsg,&mavcommandreq);
		//sysid and compid are write now just hardcoded.Later on will change
		mavlink_ros::Mavlink mavlink_ros_msg;
		castmavmsgtoros(mavlink_ros_msg,mavmsg); 
		ROS_INFO("Disarming Quadcopter");
		mavlink_pub.publish(mavlink_ros_msg);
		ROS_INFO("Publishing done");
		return true;
		 */
	}


	inline void PixhawkParser::sendradio(uint16_t r1,uint16_t r2,uint16_t r3,uint16_t r4)
	{
		mavlink_message_t mavmsg;
		//construct rc override command
		mavlink_rc_channels_override_t overridemsg;
		overridemsg.target_system = targetsys_id;
		overridemsg.target_component = targetcomp_id;

		//Set data from the msg

		//Controller should ensure that the commanded angles are between -pi to pi 
		overridemsg.chan1_raw = r1;
		overridemsg.chan2_raw = r2;
		overridemsg.chan3_raw = r3;
		overridemsg.chan4_raw = r4;
		overridemsg.chan5_raw = 900;
		overridemsg.chan6_raw = 900;
		overridemsg.chan7_raw = 900;
		overridemsg.chan8_raw = 900;

		mavlink_msg_rc_channels_override_encode(hostsysid,hostcompid,&mavmsg,&overridemsg);
		//mavlink_ros::Mavlink mavlink_ros_msg;
		//castmavmsgtoros(mavlink_ros_msg,mavmsg); 
		//ROS_INFO("Publishing rc_override %u\t%u\t%u\t%u",overridemsg.chan1_raw,overridemsg.chan2_raw,overridemsg.chan3_raw,overridemsg.chan4_raw);
		PixhawkParser::mavlinkPublish(mavmsg);
		//mavlink_pub.publish(mavlink_ros_msg);
	}

	bool PixhawkParser::cmdrpythrust(geometry_msgs::Quaternion &rpytmsg, bool sendyaw)
	{ 
		//Looks like mavlink has changed and its easier to send rpyt commands TODO
		//mavlink_attitude_control_t attitudemsg;
		mavlink_message_t mavmsg;
		//construct rc override command
		mavlink_rc_channels_override_t overridemsg;
		overridemsg.target_system = targetsys_id;
		overridemsg.target_component = targetcomp_id;

		//Set data from the msg

		//Controller should ensure that the commanded angles are between -pi to pi 
		rpytmsg.y = -rpytmsg.y;//Converting NWU to NED for commanding

    if(rpytmsg.x > 0)
    {
      overridemsg.chan1_raw = (uint16_t)parsernode::common::map(rpytmsg.x,0, data.rpbound, RC_TRIM[0]+1,RC_MAX[0]);//ROll
    }
    else
    {
      overridemsg.chan1_raw = (uint16_t)parsernode::common::map(rpytmsg.x,-data.rpbound, 0, RC_MIN[0],RC_TRIM[0]-1);//ROll
    }

    if(rpytmsg.y > 0)
    {
      overridemsg.chan2_raw = (uint16_t)parsernode::common::map(rpytmsg.y,0, data.rpbound, RC_TRIM[1]+1,RC_MAX[1]);//PITCH
    }
    else
    {
      overridemsg.chan2_raw = (uint16_t)parsernode::common::map(rpytmsg.y,-data.rpbound, 0, RC_MIN[1],RC_TRIM[1]-1);//PITCH
    }

		overridemsg.chan3_raw = (uint16_t)parsernode::common::map(rpytmsg.w, data.thrustmin,data.thrustmax,RC_MIN[2],RC_MAX[2]);//Thrust
		//uint16_t throttlepwm = (uint16_t)(rpytmsg.w * gain_throttle + intercept_throttle);
		//overridemsg.chan3_raw = (throttlepwm>RC_MAX[2])?RC_MAX[2]:(throttlepwm< RC_MIN[2])?RC_MIN[2]:throttlepwm;

		if(!sendyaw)
		{
			overridemsg.chan4_raw = (uint16_t)RC_TRIM[3];//yaw is default trim
		}
		else
		{
			overridemsg.chan4_raw = (uint16_t)parsernode::common::map(-rpytmsg.z, -1,1,RC_MIN[3],RC_MAX[3]);//Yaw is normalized between -1 to 1 for now as it is the rate of change and not exactly an angle to command
		}
		overridemsg.chan5_raw = 900;
		overridemsg.chan6_raw = 900;
		overridemsg.chan7_raw = 900;
		overridemsg.chan8_raw = 900;
		mavlink_msg_rc_channels_override_encode(hostsysid,hostcompid,&mavmsg,&overridemsg);
		//mavlink_ros::Mavlink mavlink_ros_msg;
		//castmavmsgtoros(mavlink_ros_msg,mavmsg); 
		//ROS_INFO("Publishing rc_override %u\t%u\t%u\t%u",overridemsg.chan1_raw,overridemsg.chan2_raw,overridemsg.chan3_raw,overridemsg.chan4_raw);
		PixhawkParser::mavlinkPublish(mavmsg);
		//	mavlink_pub.publish(mavlink_ros_msg);
		if(enable_log)
		{
			//log the data:
			cmdfile<<(ros::Time::now().toNSec())<<"\t"<<overridemsg.chan1_raw<<"\t"<<overridemsg.chan2_raw<<"\t"<<overridemsg.chan4_raw<<"\t"<<overridemsg.chan3_raw<<endl;
		}
		//ROS_INFO("Publishing done");
		return true;
	}
	void PixhawkParser::grip(int state)//TriState Gripper
	{ 
		//Sending arm pwm 
		mavlink_message_t mavmsg;
    mavlink_command_long_t servo_msg;
    servo_msg.command = MAV_CMD_DO_SET_SERVO;
    servo_msg.target_system = targetsys_id;
    servo_msg.param1 = 9;//http://rover.ardupilot.com/wiki/common-autopilots/common-pixhawk-overview/ : RC9 is Pin 50 AUX 1

    mavlink_command_long_t relay_msg;
    relay_msg.command = MAV_CMD_DO_SET_RELAY;
    relay_msg.target_system = targetsys_id;
    relay_msg.param1 = 0;//First pin which is 54. Pin2 is 55 Only two pins supported for virtual io in Pixhawk

    if(state == 0)
    {
      servo_msg.param2 = float(0);//set 0 pwm
      relay_msg.param2 = 0.f;//Switch off
    }
    else if (state == 1)
    {
      servo_msg.param2 = float(10000);//Run in one direction
      relay_msg.param2 = 0.f;//Switch off
    }
    else if (state == -1)
    {
      servo_msg.param2 = float(10000);//Run in other direction
      relay_msg.param2 = 1.f;//Switch On
    }
		if(ros::ok())
    {
      mavlink_msg_command_long_encode(hostsysid,hostcompid,&mavmsg,&servo_msg);
			PixhawkParser::mavlinkPublish(mavmsg);

      mavlink_msg_command_long_encode(hostsysid,hostcompid,&mavmsg,&relay_msg);
			PixhawkParser::mavlinkPublish(mavmsg);
    }
	}

  inline void PixhawkParser::setParameter(std::string id, float parameter_value, MAV_PARAM_TYPE param_type)
  {
    mavlink_message_t mavlink_msg;
    mavlink_param_set_t parameter_msg;
    //construct command
    parameter_msg.target_system = targetsys_id;
    parameter_msg.target_component = targetcomp_id;
    strcpy(parameter_msg.param_id, id.c_str());
    parameter_msg.param_value = parameter_value;
    parameter_msg.param_type = param_type;
    mavlink_msg_param_set_encode(hostsysid, hostcompid,&mavlink_msg,&parameter_msg);
		ROS_INFO("Publishing Parameter set");
		//mavlink_pub.publish(mavlink_ros_msg);
		PixhawkParser::mavlinkPublish(mavlink_msg);
  }
	/*
	void PixhawkParser::foldarm()//Folding the arm:
	{
		goalpwm[0] = ARM_TRIM[0];
		goalpwm[1] = ARM_MIN[1];
		goalpwm[2] = 500;//Neutral gripper
		/////////Removing armtimer
		//Sending arm pwm 
		mavlink_message_t mavmsg;
		mavlink_arm_ctrl_pwm_t armpwm_msg;
		armpwm_msg.pwm1 = (uint16_t)goalpwm[0];
		armpwm_msg.pwm2 = (uint16_t)goalpwm[1];
		armpwm_msg.pwm3 = (uint16_t)goalpwm[2];
		//Add log for arm pwm also TODO
		cout<<"Armpwm: "<<armpwm_msg.pwm1<<"\t"<<armpwm_msg.pwm2<<"\t"<<armpwm_msg.pwm3<<endl;//Will put this in the data for gui TODO
		mavlink_msg_arm_ctrl_pwm_encode(hostsysid,hostcompid,&mavmsg,&armpwm_msg);
		//mavlink_ros::Mavlink mavlink_ros_msg;
		//castmavmsgtoros(mavlink_ros_msg,mavmsg); 
		if(ros::ok())
			PixhawkParser::mavlinkPublish(mavmsg);
			//mavlink_pub.publish(mavlink_ros_msg);
	}
	void PixhawkParser::setarmpwm(double *armpwm)//Expecting the armpwm to be of length 3
	{ 
		//	assert(armpwm.size() >NOFJOINTS);//Needs to atleast three for the three arms
		//construct arm pwm command
		//	countstar = true;
		//goalpwm[0] = parsernode::common::map(armpwm[0],-1.0, 1.0, 1000,2000);//Map betn -1 to 1 to rcmin to rcmax
		//goalpwm[1] = parsernode::common::map(armpwm[1],-1.0, 1.0, 1000,2000);//Map betn -1 to 1 to rcmin to rcmax
		//for(int count1 = 0;count1 < NOFJOINTS;count1++)
		goalpwm[0] = parsernode::common::map(armpwm[0],-1.0, 1.0,0.5*ARM_TRIM[1],1.5*ARM_TRIM[1]);//Map betn -1 to 1 to rcmin to rcmax
		goalpwm[1] = parsernode::common::map(armpwm[1],-1.0, 1.0,ARM_MIN[0],ARM_MAX[0]);//Map betn -1 to 1 to rcmin to rcmax
		goalpwm[NOFJOINTS] = 500 +  1000*armpwm[NOFJOINTS];//Just copying the value input for gripper it is tristate with 500 neutral > 700 close and < 300 open
		//////////Removing armtimer
		//Sending arm pwm 
		mavlink_message_t mavmsg;
		mavlink_arm_ctrl_pwm_t armpwm_msg;
		armpwm_msg.pwm1 = (uint16_t)goalpwm[0];
		armpwm_msg.pwm2 = (uint16_t)goalpwm[1];
		armpwm_msg.pwm3 = (uint16_t)goalpwm[2];
		//Add log for arm pwm also TODO
		cout<<"Armpwm: "<<armpwm_msg.pwm1<<"\t"<<armpwm_msg.pwm2<<"\t"<<armpwm_msg.pwm3<<endl;//Will put this in the data for gui TODO
		mavlink_msg_arm_ctrl_pwm_encode(hostsysid,hostcompid,&mavmsg,&armpwm_msg);
		//mavlink_ros::Mavlink mavlink_ros_msg;
		//castmavmsgtoros(mavlink_ros_msg,mavmsg); 
		if(ros::ok())
			PixhawkParser::mavlinkPublish(mavmsg);
			//mavlink_pub.publish(mavlink_ros_msg);
/////////////PREV COMMENT BEGIN
		if(armpwm[NOFJOINTS] > 0)
			goalpwm[NOFJOINTS] = 1700;//Some Voltage This is for gripper not actual pwm in the new gripper case
			else
			goalpwm[NOFJOINTS] = 0;//No Voltage
/////////////PREV COMMENT END
	}
	//void PixhawkParser::setarmangles(std::vector<float> &armangles)//angles given from gcop IK notation
	void PixhawkParser::setarmangles(double *armangles)//angles given from gcop IK notation angle in radians
	{ 
		//assert(armangles.size() >NOFJOINTS);//two rotary joints and one gripper TODO separate gripper
		for(int count1 = 0;count1 < NOFJOINTS;count1++)
		{
			if(armangles[count1] > 2*M_PI)//Can make this into continous domain for greater than 2pi also TODO
				armangles[count1] = 2*M_PI;
			goalpwm[count1] = ARM_TRIM[count1] - armangles[count1]*180/(M_PI*ANGRES);//Absolute Angle should be between 0 to 360 Converts into the right frame
			//Also the angles are constrained to workspace on the microcontroller side already so no need to do it here
		}
		goalpwm[NOFJOINTS] = 500 +  1000*armangles[NOFJOINTS];//Just copying the value input for gripper it is tristate with 500 neutral > 700 close and < 300 open
		//////////Removing armtimer
		//Sending arm pwm 
		mavlink_message_t mavmsg;
		mavlink_arm_ctrl_pwm_t armpwm_msg;
		armpwm_msg.pwm1 = (uint16_t)goalpwm[0];
		armpwm_msg.pwm2 = (uint16_t)goalpwm[1];
		armpwm_msg.pwm3 = (uint16_t)goalpwm[2];
		//Add log for arm pwm also TODO
		cout<<"Armpwm: "<<armpwm_msg.pwm1<<"\t"<<armpwm_msg.pwm2<<"\t"<<armpwm_msg.pwm3<<endl;//Will put this in the data for gui TODO
		mavlink_msg_arm_ctrl_pwm_encode(hostsysid,hostcompid,&mavmsg,&armpwm_msg);
		//mavlink_ros::Mavlink mavlink_ros_msg;
		//castmavmsgtoros(mavlink_ros_msg,mavmsg); 
		if(ros::ok())
			PixhawkParser::mavlinkPublish(mavmsg);
		//	mavlink_pub.publish(mavlink_ros_msg);
	}
*/
	void PixhawkParser::getquaddata(parsernode::common::quaddata &d1)
	{
		spin_mutex.lock();
		d1 = data;
		spin_mutex.unlock();
		return;
	}

	//Personal Callback functions:
	/*void PixhawkParser::paramsetreqCallback(std_msgs::String datatype)
	{
		mavlink_message_t mavmsg;
		mavlink_param_set_t paramsetmsg; 
		//construct command
		paramsetmsg.target_system = targetsys_id;
		paramsetmsg.target_component = targetcomp_id;

		string data = datatype.data;
		istringstream iss(data);
		string substr;
		iss>>substr;
		ROS_INFO("\n%s",substr.c_str());
		sprintf(paramsetmsg.param_id,"%s",substr.c_str());
		iss>>paramsetmsg.param_value;//assume a float value is typed
		ROS_INFO("%f",paramsetmsg.param_value);
		iss>>paramsetmsg.param_type;
		//paramsetmsg.param_type = MAVLINK_TYPE_INT16_T;
		//encode 
		mavlink_msg_param_set_encode(hostsysid,hostcompid,&mavmsg,&paramsetmsg);
		//mavlink_ros::Mavlink mavlink_ros_msg;
		//castmavmsgtoros(mavlink_ros_msg,mavmsg); 
		ROS_INFO("Publishing Parameter set");
		//mavlink_pub.publish(mavlink_ros_msg);
		PixhawkParser::mavlinkPublish(mavmsg);
		ROS_INFO("Publishing done");
	}*/ 

	void PixhawkParser::modereqCallback(const std_msgs::String &datatype)
	{
		mavlink_message_t mavmsg;
		//construct command
		mavlink_set_mode_t mavmodereq;
		mavmodereq.base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;//This is the base mode to make sure we do not get kicked out. Then we add the custom mode which redefines the base mode
		mavmodereq.target_system = targetsys_id;
		string data = datatype.data;
		istringstream iss(data);
		string substr;
		iss>>substr;
		ROS_INFO("\n%s",substr.c_str());
		if(substr == "STABILIZE")
		{
			mavmodereq.custom_mode = STABILIZE;
		} 
		else  if(substr == "ALT_HOLD")
		{
			mavmodereq.custom_mode = ALT_HOLD;
		} 
		else  if(substr == "LAND")
		{
			mavmodereq.custom_mode = LAND;
		} 
		else
		{
			ROS_INFO("Invalid Arguments");
			return;
		}
		//encode the mavlink_data_stream_t into mavlink_message_t
		mavlink_msg_set_mode_encode(hostsysid,hostcompid,&mavmsg,&mavmodereq);
		//sysid and compid are write now just hardcoded.Later on will change
		//mavlink_ros::Mavlink mavlink_ros_msg;
		//castmavmsgtoros(mavlink_ros_msg,mavmsg); 
		ROS_INFO("Publishing mode %s", datatype.data.c_str());
		PixhawkParser::mavlinkPublish(mavmsg);
		//mavlink_pub.publish(mavlink_ros_msg);
		ROS_INFO("Publishing done");
	}

	void PixhawkParser::serialtimerCallback()//This is a thread now
	{
		mavlink_status_t lastStatus;
		lastStatus.packet_rx_drop_count = 0;

		//if (debug) printf("Checking for new data on serial port\n");
		// Block until data is available, read only one byte to be able to continue immediately
		//char buf[MAVLINK_MAX_PACKET_LEN];
		while(ros::ok())
		{
			uint8_t cp;
			mavlink_message_t message;
			mavlink_status_t status;
			uint8_t msgReceived = false;
			//tcflush(fd, TCIFLUSH);
			if (read(fd, &cp, 1) > 0)
			{
				// Check if a message could be decoded, return the message in case yes
				msgReceived = mavlink_parse_char(MAVLINK_COMM_1, cp, &message, &status);
				if (lastStatus.packet_rx_drop_count != status.packet_rx_drop_count)
				{
					if (verbose || debug)
						printf("ERROR: DROPPED %d PACKETS\n", status.packet_rx_drop_count);
					if (debug)
					{
						unsigned char v = cp;
						fprintf(stderr, "%02x ", v);
					}
				}
				lastStatus = status;
			}
			else
			{
				if (!verbose)
					fprintf(stderr, "ERROR: Could not read from port %s\n", port.c_str());
					//exit(EXIT_FAILURE);//Exit if we cannot read from the port
			}

			// If a message could be decoded, handle it
			if (msgReceived)
			{
				//if (verbose || debug) std::cout << std::dec << "Received and forwarded serial port message with id " << static_cast<unsigned int>(message.msgid) << " from system " << static_cast<int>(message.sysid) << std::endl;

				// Do not send images over serial port

				// DEBUG output
				if (debug)
				{
					fprintf(stderr, "Forwarding SERIAL -> ROS: ");
					unsigned int i;
					uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
					unsigned int messageLength = mavlink_msg_to_send_buffer(buffer, &message);
					if (messageLength > MAVLINK_MAX_PACKET_LEN)
					{
						fprintf(stderr, "\nFATAL ERROR: MESSAGE LENGTH IS LARGER THAN BUFFER SIZE\n");
					}
					else
					{
						for (i = 0; i < messageLength; i++)
						{
							unsigned char v = buffer[i];
							fprintf(stderr, "%02x ", v);
						}
						fprintf(stderr, "\n");
					}
				}

				if (verbose || debug)
					ROS_INFO("Received message from serial with ID #%d (sys:%d|comp:%d):\n", message.msgid, message.sysid,
							message.compid);


				//	mavlink_message_t message;
				//	castrosmsgtomav(mavlink_ros_msg,message);
				switch(message.msgid)
				{
					case MAVLINK_MSG_ID_PARAM_VALUE:
            {
              //ROS_INFO("Param received");
              mavlink_param_value_t paramvalue;
              mavlink_msg_param_value_decode(&message,&paramvalue);
              ROS_INFO("Param_index: %d\t Param_id: %s\t Param_value: %f",paramvalue.param_index, paramvalue.param_id, paramvalue.param_value);
              //We only care about parameters we need like RC params. Others for now ignore
              char id[16];//ID
              for (int i = 0;i < SERVONUM;i++)
              {
                sprintf(id,"RC%d_MIN",i+1);
                if(!strcmp(paramvalue.param_id, id))
                {
                  RC_MIN[i] = paramvalue.param_value;
                  printf("RC_MIN[%d]: %f\n",i,RC_MIN[i]);
                  break;
                }

                sprintf(id,"RC%d_TRIM",i+1);
                if(!strcmp(paramvalue.param_id, id))
                {
                  RC_TRIM[i] = paramvalue.param_value;
                  printf("RC_TRIM[%d]: %f\n",i,RC_TRIM[i]);
                  break;
                }

                sprintf(id,"RC%d_MAX",i+1);
                if(!strcmp(paramvalue.param_id, id))
                {
                  RC_MAX[i] = paramvalue.param_value;
                  printf("RC_MAX[%d]: %f\n",i,RC_MAX[i]);
                  break;
                }
              }
            }
						break;
					case MAVLINK_MSG_ID_RAW_IMU:
						{
							mavlink_raw_imu_t rawimu_val;
							mavlink_msg_raw_imu_decode(&message,&rawimu_val);
							spin_mutex.lock();
							data.magdata.x = rawimu_val.xmag;
							data.magdata.y = rawimu_val.ymag;
							data.magdata.z = rawimu_val.zmag;
							data.linacc.x = rawimu_val.xacc;
							data.linacc.y = rawimu_val.yacc;
							data.linacc.z = rawimu_val.zacc;
							spin_mutex.unlock();
						}
						break;
					case MAVLINK_MSG_ID_RAW_PRESSURE:
						{
							mavlink_raw_pressure_t pressuremsg;
							mavlink_msg_raw_pressure_decode(&message,&pressuremsg);
							spin_mutex.lock();
							data.pressure = pressuremsg.press_abs;
							//ROS_INFO("Data Pressure: %f\t%f\t%f",pressuremsg.press_abs,pressuremsg.press_diff1,pressuremsg.press_diff2);
							data.temperature = pressuremsg.temperature;
							spin_mutex.unlock();
						}
					case MAVLINK_MSG_ID_ATTITUDE:
						{
							mavlink_attitude_t attitudemsg;
							mavlink_msg_attitude_decode(&message, &attitudemsg);
							spin_mutex.lock();
							tf::Quaternion qtr;
							//qtr.setEulerZYX(0,0,M_PI);
							//tf::Transform imutovrpn(qtr,tf::Vector3(0,0,0));//Pure rotation from NED to NWU
							//tf::Vector3 imutransformed = imutovrpn
							data.rpydata.x = parsernode::common::map_angle(attitudemsg.roll);
							data.rpydata.y = -parsernode::common::map_angle(attitudemsg.pitch);//Transforming NED to NWU
							data.rpydata.z = -parsernode::common::map_angle(attitudemsg.yaw);//Transforming NED to NWU
							//Transforming to NWU frame Convert pitch to pitch - PI (invert it)
							//ensure the data is in between -Pi to Pi
							spin_mutex.unlock();
							if(enable_log)
							{
								//log the data:
								imufile<<(ros::Time::now().toNSec())<<"\t"<<data.rpydata.x<<"\t"<<data.rpydata.y<<"\t"<<data.rpydata.z<<endl;
							}
						}
						break;
					case MAVLINK_MSG_ID_SYS_STATUS: 
						{
							mavlink_sys_status_t extended_statusmsg;
							mavlink_msg_sys_status_decode(&message,&extended_statusmsg);
							spin_mutex.lock();
							//ROS_INFO("Sys: %d", extended_statusmsg.voltage_battery);
							data.batterypercent = ((float)extended_statusmsg.voltage_battery)/1000.0f;//in volts
							spin_mutex.unlock();
						}
						break;
					case MAVLINK_MSG_ID_RC_CHANNELS_RAW:
						{
							mavlink_rc_channels_raw_t rcmessage;
							mavlink_msg_rc_channels_raw_decode(&message,&rcmessage);
							//printf("RAW RC input: \t R1: %u\t R2 %u \t R_3 %u\t R_4 %u\tR_5 %u\t R_6 %u \t R_7 %u\t R_8 %u \n", rcmessage.chan1_raw, rcmessage.chan2_raw, rcmessage.chan3_raw, rcmessage.chan4_raw, rcmessage.chan5_raw, rcmessage.chan6_raw, rcmessage.chan7_raw, rcmessage.chan8_raw);
						}
						break;
					case MAVLINK_MSG_ID_SERVO_OUTPUT_RAW:
						{
							mavlink_servo_output_raw_t servooutmessage;
							mavlink_msg_servo_output_raw_decode(&message,&servooutmessage);
							//ROS_INFO("RAW Servo Channels output: \n\t SERVO_1: %u \n\t SERVO_2 %u \n\t SERVO_3 %u\n\t SERVO_4 %u \n\t SERVO_5 %u \n\t SERVO_6 %u \n\t SERVO_7 %u \n\t SERVO_8 %u \n", servooutmessage.servo1_raw, servooutmessage.servo2_raw, servooutmessage.servo3_raw, servooutmessage.servo4_raw, servooutmessage.servo5_raw, servooutmessage.servo6_raw, servooutmessage.servo7_raw, servooutmessage.servo8_raw);
							if(enable_log)
							{
								//log the data:
								servofile<<(ros::Time::now().toNSec())<<"\t"<<servooutmessage.servo1_raw<<"\t"<<servooutmessage.servo2_raw<<"\t"<<servooutmessage.servo3_raw<<"\t"<<servooutmessage.servo4_raw<<endl;
							}
						}
						break;
					case MAVLINK_MSG_ID_HEARTBEAT:
						{
							mavlink_heartbeat_t heartbeat;
							mavlink_msg_heartbeat_decode(&message,&heartbeat);
							//set the target_sys id to this one:
							if(!heartbeatinit)
							{
								//Read the parameters etc
								heartbeatinit = true;
								targetsys_id = message.sysid;
								ROS_INFO("targetsys_id: %d",targetsys_id);
								//Request Parameters:
								std_msgs::Empty emptymsg;
								PixhawkParser::paramreqCallback(emptymsg);
								//Setup the data to be requested:
								std_msgs::String dataparseval;
								//dataparseval.data = "ALL START 10";
								//PixhawkParser::datareqCallback(dataparseval);
								dataparseval.data = "ATTITUDE START 20";//30
								PixhawkParser::datareqCallback(dataparseval);
								usleep(50000);
								dataparseval.data = "EXTENDED START 2";//For battery data etc 2 times a second should be more than enough 
								PixhawkParser::datareqCallback(dataparseval);
								usleep(50000);
								dataparseval.data = "RADIO START 0";//30
								PixhawkParser::datareqCallback(dataparseval);
								usleep(50000);
								dataparseval.data = "RAW START 0";//20 Not needed right now
								PixhawkParser::datareqCallback(dataparseval);

                //Set CH7 to 34:
                PixhawkParser::setParameter("CH7_OPT", 34.0, MAV_PARAM_TYPE_INT8);
							}
							//fprintf(stdout,"Sys_status: %s MODE: %d\n",base_mode_map(heartbeat.base_mode).c_str(),heartbeat.custom_mode);
							spin_mutex.lock();
							switch(heartbeat.system_status)
							{
								case MAV_STATE_UNINIT:
									data.quadstate = "UNINTIALIZED ";
									break;
								case MAV_STATE_BOOT:
									data.quadstate = "BOOTING ";
									break;
								case MAV_STATE_CALIBRATING:
									data.quadstate = "CALIBRATING ";
									break;
								case MAV_STATE_STANDBY:
									data.quadstate = "STANDBY ";
									break;
								case MAV_STATE_ACTIVE:
									data.quadstate = "ACTIVE ";
									break;
								case MAV_STATE_CRITICAL:
									data.quadstate = "CRITICAL ";
									break;
								case MAV_STATE_EMERGENCY:
									data.quadstate = "EMERGENCY ";
									break;
								case MAV_STATE_POWEROFF:
									data.quadstate = "POWEROFF ";
									break;
								case MAV_STATE_ENUM_END:
									data.quadstate = "ENUM_END ";
									break;
							}
							if(heartbeat.base_mode & MAV_MODE_FLAG_STABILIZE_ENABLED)
							{
								data.quadstate +=  " STABILIZE";
							}
							if(heartbeat.base_mode & MAV_MODE_FLAG_SAFETY_ARMED)
							{
								data.quadstate +=  " ARMED";
								data.armed = true;
							}
							else
							{
								data.armed = false;
								data.quadstate += " DISARMED";
							}
							spin_mutex.unlock();
							//map functions are all in common.h and other .h in mavlink include for converting the enum types to strings
						}
						break;
					case MAVLINK_MSG_ID_STATUSTEXT:
						{
							mavlink_statustext_t statusmsg;
							mavlink_msg_statustext_decode(&message,&statusmsg);
							ROS_INFO("Status: %s",statusmsg.text);
						}
						break;
				}
			}
		}
	}

	void PixhawkParser::mavlinkPublish(const mavlink_message_t &msg)
	{
		//static uint8_t mavlink_crcs[] = MAVLINK_MESSAGE_CRCS;

		//Copy payload from mavlink_msg (from ROS) to the new "real" mavlink message
		//copy(mavlink_ros_msg.payload64.begin(), mavlink_ros_msg.payload64.end(), msg.payload64);

		//mavlink_finalize_message_chan(&msg, mavlink_ros_msg.sysid, mavlink_ros_msg.compid, MAVLINK_COMM_0,
				//mavlink_ros_msg.len, mavlink_crcs[msg.msgid]);

		/**
		 * Send mavlink_message to UART
		 */
		if (verbose)
			ROS_INFO("Sent Mavlink from ROS to UART, Message-ID: [%i]", msg.msgid);

		// Send message over serial port
		static uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
		int messageLength = mavlink_msg_to_send_buffer(buffer, &msg);
		if (debug)
			printf("Writing %d bytes\n", messageLength);
		int written = write(fd, (char*)buffer, messageLength);
		tcflush(fd, TCOFLUSH);
		if (messageLength != written)
			fprintf(stderr, "ERROR: Wrote %d bytes but should have written %d\n", written, messageLength);
	}

	void PixhawkParser::datareqCallback(const std_msgs::String &datatype)
	{
		mavlink_message_t mavmsg;
		//construct command
		mavlink_request_data_stream_t mavdatastreamreq;
		//mavdatastreamreq.req_message_rate = 10; //Hz
		mavdatastreamreq.req_message_rate = 30; //Hz
		mavdatastreamreq.target_system = targetsys_id;
		mavdatastreamreq.target_component = targetcomp_id;
		string data = datatype.data;
		istringstream iss(data);
		string substr;
		iss>>substr;
		ROS_INFO("\n%s",substr.c_str());
		if(substr == "ALL")
		{
			mavdatastreamreq.req_stream_id = MAV_DATA_STREAM_ALL;
		} 
		else  if(substr == "ATTITUDE")
		{
			mavdatastreamreq.req_stream_id = MAV_DATA_STREAM_EXTRA1;
		} 
		else  if(substr == "RAW")
		{
			mavdatastreamreq.req_stream_id = MAV_DATA_STREAM_RAW_SENSORS;
		} 
		else if(substr == "RADIO")
		{
			mavdatastreamreq.req_stream_id = MAV_DATA_STREAM_RC_CHANNELS;
		}
		else if(substr == "EXTENDED")
		{
			mavdatastreamreq.req_stream_id = MAV_DATA_STREAM_EXTENDED_STATUS;
		}
		else 
		{
			ROS_INFO("Invalid Arguments");
			return;
		}
		iss>>substr;
		ROS_INFO("%s",substr.c_str());
		if(substr == "STOP")
		{
			mavdatastreamreq.start_stop = 0;
		}
		else if(substr == "START")
		{
			mavdatastreamreq.start_stop = 1;
		}
		else 
		{
			ROS_INFO("Invalid Arguments");
			return;
		}
		//Get the freq from the data too:
		iss>>mavdatastreamreq.req_message_rate; //Hz

		//encode the mavlink_data_stream_t into mavlink_message_t
		mavlink_msg_request_data_stream_encode(hostsysid,hostcompid,&mavmsg,&mavdatastreamreq);
		//mavlink_ros::Mavlink mavlink_ros_msg;
		//castmavmsgtoros(mavlink_ros_msg,mavmsg); 
		ROS_INFO("Publishing Data req");
		PixhawkParser::mavlinkPublish(mavmsg);
		//mavlink_pub.publish(mavlink_ros_msg);
		ROS_INFO("Publishing done");
	}

	void PixhawkParser::paramreqCallback(const std_msgs::Empty &emptymsg)
	{
		//publish param_list message to get the parameters:
		mavlink_message_t mavmsg;
		mavlink_param_request_list_t mavparamreq;
		mavparamreq.target_system = targetsys_id;
		mavparamreq.target_component = targetcomp_id;
		//encode the mavlink_param_request_list_t into mavlink_message_t
		mavlink_msg_param_request_list_encode(hostsysid,hostcompid,&mavmsg,&mavparamreq);
		//sysid and compid are write now just hardcoded.Later on will change
		//mavlink_ros::Mavlink mavlink_ros_msg;
		//castmavmsgtoros(mavlink_ros_msg,mavmsg); 
		ROS_INFO("\nPublishing Paramreq");
		PixhawkParser::mavlinkPublish(mavmsg);
		//mavlink_pub.publish(mavlink_ros_msg);
		ROS_INFO("Publishing done");
	}
	/* These functions are from MAVLINK SERIAL NODE COPIED HERE 
	 */
	/**
	 *
	 *
	 * Returns the file descriptor on success or -1 on error.
	 */

	int PixhawkParser::open_port(std::string& port)
	{
		int fd; /* File descriptor for the port */

		// Open serial port
		// O_RDWR - Read and write
		// O_NOCTTY - Ignore special chars like CTRL-C
		fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
		if (fd == -1)
		{
			/* Could not open the port. */
			return (-1);
		}
		else
		{
			fcntl(fd, F_SETFL, 0);
		}

		return (fd);
	}

	bool PixhawkParser::setup_port(int fd, int baud, int data_bits, int stop_bits, bool parity, bool hardware_control)
	{
		//struct termios options;

		struct termios config;
		if (!isatty(fd))
		{
			fprintf(stderr, "\nERROR: file descriptor %d is NOT a serial port\n", fd);
			return false;
		}
		if (tcgetattr(fd, &config) < 0)
		{
			fprintf(stderr, "\nERROR: could not read configuration of fd %d\n", fd);
			return false;
		}
		//
		// Input flags - Turn off input processing
		// convert break to null byte, no CR to NL translation,
		// no NL to CR translation, don't mark parity errors or breaks
		// no input parity check, don't strip high bit off,
		// no XON/XOFF software flow control
		//
		config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);
		//
		// Output flags - Turn off output processing
		// no CR to NL translation, no NL to CR-NL translation,
		// no NL to CR translation, no column 0 CR suppression,
		// no Ctrl-D suppression, no fill characters, no case mapping,
		// no local output processing
		//
		config.c_oflag &= ~(OCRNL | ONLCR | ONLRET | ONOCR | OFILL | OPOST);

#ifdef OLCUC
		config.c_oflag &= ~OLCUC;
#endif

#ifdef ONOEOT
		config.c_oflag &= ~ONOEOT;
#endif

		//
		// No line processing:
		// echo off, echo newline off, canonical mode off,
		// extended input processing off, signal chars off
		//
		config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);
		//
		// Turn off character processing
		// clear current char size mask, no parity checking,
		// no output processing, force 8 bit input
		//
		config.c_cflag &= ~(CSIZE | PARENB);
		config.c_cflag |= CS8;
		//
		// One input byte is enough to return from read()
		// Inter-character timer off
		//
		config.c_cc[VMIN] = 1;
		config.c_cc[VTIME] = 10; // was 0

		// Get the current options for the port
		//tcgetattr(fd, &options);

		switch (baud)
		{
			case 1200:
				if (cfsetispeed(&config, B1200) < 0 || cfsetospeed(&config, B1200) < 0)
				{
					fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
					return false;
				}
				break;
			case 1800:
				cfsetispeed(&config, B1800);
				cfsetospeed(&config, B1800);
				break;
			case 9600:
				cfsetispeed(&config, B9600);
				cfsetospeed(&config, B9600);
				break;
			case 19200:
				cfsetispeed(&config, B19200);
				cfsetospeed(&config, B19200);
				break;
			case 38400:
				if (cfsetispeed(&config, B38400) < 0 || cfsetospeed(&config, B38400) < 0)
				{
					fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
					return false;
				}
				break;
			case 57600:
				if (cfsetispeed(&config, B57600) < 0 || cfsetospeed(&config, B57600) < 0)
				{
					fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
					return false;
				}
				break;
			case 115200:
				if (cfsetispeed(&config, B115200) < 0 || cfsetospeed(&config, B115200) < 0)
				{
					fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
					return false;
				}
				break;

				// These two non-standard (by the 70'ties ) rates are fully supported on
				// current Debian and Mac OS versions (tested since 2010).
			case 460800:
				if (cfsetispeed(&config, 460800) < 0 || cfsetospeed(&config, 460800) < 0)
				{
					fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
					return false;
				}
				break;
			case 921600:
				if (cfsetispeed(&config, 921600) < 0 || cfsetospeed(&config, 921600) < 0)
				{
					fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
					return false;
				}
				break;
			default:
				fprintf(stderr, "ERROR: Desired baud rate %d could not be set, aborting.\n", baud);
				return false;

				break;
		}

		//
		// Finally, apply the configuration
		//
		if (tcsetattr(fd, TCSAFLUSH, &config) < 0)
		{
			fprintf(stderr, "\nERROR: could not set configuration of fd %d\n", fd);
			return false;
		}
		return true;
	}
	void PixhawkParser::close_port(int fd)
	{
		close(fd);
	}
};
PLUGINLIB_DECLARE_CLASS(pixhawk_parser, PixhawkParser, pixhawk_parser::PixhawkParser, parsernode::Parser)
