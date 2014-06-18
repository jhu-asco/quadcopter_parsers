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
		//RC_TRIM = new float[SERVONUM]{1516,1516,1923,1514};
		//RC_MIN = new float[SERVONUM]{1108,1103,1105,1101};
		//RC_MAX = new float[SERVONUM]{1932,1929,1926,1926};

		RC_ID = new uint16_t[SERVONUM]{56,61,66,71};//Arducopter code 3.1.2
		//RC_ID = new uint16_t[SERVONUM]{53,58,63,68}; Old code
		RC_TRIM = new float[SERVONUM]{1515,1516,1109,1514};
		RC_MIN = new float[SERVONUM]{1102,1103,1104,1101};
		RC_MAX = new float[SERVONUM]{1927,1929,1926,1927};
		ARM_ID = new uint16_t[NOFJOINTS]{1,2};
		ARM_MIN = new float[NOFJOINTS]{937,0};
		ARM_MAX = new float[NOFJOINTS]{2337,3070};
		ARM_TRIM = new float[NOFJOINTS]{2167,2047};//When arm is outstretched
		//goalpwm = new float[NOFJOINTS]{1500,1500,1500};//Initialization
		goalpwm = new float[NOFJOINTS+1]{ARM_TRIM[0],ARM_MIN[1],500};//Initialization can use malloc These are the default values when it does not have any joy input
		temparm_pwm = new float[NOFJOINTS]{ARM_TRIM[0],ARM_MIN[1]};//Dont need the last one which is just a gripper also use malloc
		//kparm = 0.02;
		kparm = 0.2;
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
		mavlink_pub = nh_.advertise<mavlink_ros::Mavlink>("/mavlink/to", 1000);
		mavlink_sub = nh_.subscribe("/mavlink/from",1000,&PixhawkParser::mavlinkCallback,this);
		//rctimercount = 0;
		rctimerrpytmsg.x = 0; rctimerrpytmsg.y = 0; rctimerrpytmsg.z = 0; rctimerrpytmsg.w = 0;//Initialize the rpytmsg used by rctimer
		intmode = -1;
		rctimer = nh_.createTimer(ros::Duration(0.02), &PixhawkParser::rctimerCallback,this);//50Hz
		//armtimer = nh_.createTimer(ros::Duration(0.1), &PixhawkParser::armtimerCallback,this);//10Hz
		rctimer.stop();//Stop the timer to begin with
		//armtimer.stop();
		//armtimer.start();
		//For now we dont need any subscribers from personal Callbacks as this is intended to be used as a library
	}
/*
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

	bool PixhawkParser::disarm()
	{
		rctimer.stop();	//Stop the timer just in case
		for(int count = 0;count < 30;count++)//3Sec
		{
			PixhawkParser::sendradio(RC_TRIM[0],RC_TRIM[1],RC_MIN[2],RC_MIN[3]);
			usleep(100000);
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
		mavlink_ros::Mavlink mavlink_ros_msg;
		castmavmsgtoros(mavlink_ros_msg,mavmsg); 
		ROS_INFO("Publishing rc_override %u\t%u\t%u\t%u",overridemsg.chan1_raw,overridemsg.chan2_raw,overridemsg.chan3_raw,overridemsg.chan4_raw);
		mavlink_pub.publish(mavlink_ros_msg);
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

		overridemsg.chan1_raw = (uint16_t)rqt_quadcoptergui::common::map(rpytmsg.x,-data.rpbound, data.rpbound, RC_MIN[0],RC_MAX[0]);//ROll

		overridemsg.chan2_raw = (uint16_t)rqt_quadcoptergui::common::map(rpytmsg.y,-data.rpbound, data.rpbound, RC_MIN[1],RC_MAX[1]);//PITCH

		overridemsg.chan3_raw = (uint16_t)rqt_quadcoptergui::common::map(rpytmsg.w, data.thrustmin,data.thrustmax,RC_MIN[2],RC_MAX[2]);//Thrust

		if(!sendyaw)
		{
			overridemsg.chan4_raw = (uint16_t)RC_TRIM[3];//yaw is default trim
		}
		else
		{
			overridemsg.chan4_raw = (uint16_t)rqt_quadcoptergui::common::map(-rpytmsg.z, -1,1,RC_MIN[3],RC_MAX[3]);//Yaw is normalized between -1 to 1 for now as it is the rate of change and not exactly an angle to command
		}
		overridemsg.chan5_raw = 900;
		overridemsg.chan6_raw = 900;
		overridemsg.chan7_raw = 900;
		overridemsg.chan8_raw = 900;
		mavlink_msg_rc_channels_override_encode(hostsysid,hostcompid,&mavmsg,&overridemsg);
		mavlink_ros::Mavlink mavlink_ros_msg;
		castmavmsgtoros(mavlink_ros_msg,mavmsg); 
		ROS_INFO("Publishing rc_override %u\t%u\t%u\t%u",overridemsg.chan1_raw,overridemsg.chan2_raw,overridemsg.chan3_raw,overridemsg.chan4_raw);
		mavlink_pub.publish(mavlink_ros_msg);
		if(enable_log)
		{
			//log the data:
			cmdfile<<(ros::Time::now().toNSec())<<"\t"<<overridemsg.chan1_raw<<"\t"<<overridemsg.chan2_raw<<"\t"<<overridemsg.chan4_raw<<"\t"<<overridemsg.chan3_raw<<endl;
		}
		//ROS_INFO("Publishing done");
		return true;
	}
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
		mavlink_ros::Mavlink mavlink_ros_msg;
		castmavmsgtoros(mavlink_ros_msg,mavmsg); 
		if(ros::ok())
			mavlink_pub.publish(mavlink_ros_msg);
	}
	void PixhawkParser::setarmpwm(double *armpwm)//Expecting the armpwm to be of length 3
	{ 
	//	assert(armpwm.size() >NOFJOINTS);//Needs to atleast three for the three arms
		//construct arm pwm command
		//	countstar = true;
		//goalpwm[0] = rqt_quadcoptergui::common::map(armpwm[0],-1.0, 1.0, 1000,2000);//Map betn -1 to 1 to rcmin to rcmax
		//goalpwm[1] = rqt_quadcoptergui::common::map(armpwm[1],-1.0, 1.0, 1000,2000);//Map betn -1 to 1 to rcmin to rcmax
		//for(int count1 = 0;count1 < NOFJOINTS;count1++)
		goalpwm[0] = rqt_quadcoptergui::common::map(armpwm[0],-1.0, 1.0,0.5*ARM_TRIM[1],1.5*ARM_TRIM[1]);//Map betn -1 to 1 to rcmin to rcmax
		goalpwm[1] = rqt_quadcoptergui::common::map(armpwm[1],-1.0, 1.0,ARM_MIN[0],ARM_MAX[0]);//Map betn -1 to 1 to rcmin to rcmax
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
		mavlink_ros::Mavlink mavlink_ros_msg;
		castmavmsgtoros(mavlink_ros_msg,mavmsg); 
		if(ros::ok())
			mavlink_pub.publish(mavlink_ros_msg);

		/*if(armpwm[NOFJOINTS] > 0)
			goalpwm[NOFJOINTS] = 1700;//Some Voltage This is for gripper not actual pwm in the new gripper case
		else
			goalpwm[NOFJOINTS] = 0;//No Voltage
			*/
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
		mavlink_ros::Mavlink mavlink_ros_msg;
		castmavmsgtoros(mavlink_ros_msg,mavmsg); 
		if(ros::ok())
			mavlink_pub.publish(mavlink_ros_msg);

	}
	void PixhawkParser::getquaddata(rqt_quadcoptergui::common::quaddata &d1)
	{
		spin_mutex.lock();
		d1 = data;
		spin_mutex.unlock();
		return;
	}

	//Personal Callback functions:
	void PixhawkParser::paramsetreqCallback(std_msgs::String datatype)
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
		mavlink_ros::Mavlink mavlink_ros_msg;
		castmavmsgtoros(mavlink_ros_msg,mavmsg); 
		ROS_INFO("Publishing Parameter set");
		mavlink_pub.publish(mavlink_ros_msg);
		ROS_INFO("Publishing done");
	}

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
		mavlink_ros::Mavlink mavlink_ros_msg;
		castmavmsgtoros(mavlink_ros_msg,mavmsg); 
		ROS_INFO("Publishing mode %s", datatype.data.c_str());
		mavlink_pub.publish(mavlink_ros_msg);
		ROS_INFO("Publishing done");
	}

	void PixhawkParser::mavlinkCallback(const mavlink_ros::Mavlink &mavlink_ros_msg)
	{
		mavlink_message_t message;
		castrosmsgtomav(mavlink_ros_msg,message);
		switch(message.msgid)
		{
			case MAVLINK_MSG_ID_PARAM_VALUE:
				{
					//ROS_INFO("Param received");
					mavlink_param_value_t paramvalue;
					mavlink_msg_param_value_decode(&message,&paramvalue);
					ROS_INFO("Param_index: %d\t Param_id: %s\t Param_value: %f",paramvalue.param_index, paramvalue.param_id, paramvalue.param_value);
					//We only care about parameters we need like RC params. Others for now ignore
					for (int i = 0;i < SERVONUM;i++)
					{
						if(paramvalue.param_index ==RC_ID[i])
						{
							RC_MIN[i] = paramvalue.param_value;
							ROS_INFO("RC_MIN[%d]: %f",i,RC_MIN[i]);
						}
						else if(paramvalue.param_index == (RC_ID[i]+1))
						{
							RC_TRIM[i] = paramvalue.param_value;
							ROS_INFO("RC_TRIM[%d]: %f",i,RC_TRIM[i]);
						}
						else if(paramvalue.param_index == (RC_ID[i]+2))
						{
							RC_MAX[i] = paramvalue.param_value;
							ROS_INFO("RC_MAX[%d]: %f",i,RC_MAX[i]);
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
					data.rpydata.x = rqt_quadcoptergui::common::map_angle(attitudemsg.roll);
					data.rpydata.y = -rqt_quadcoptergui::common::map_angle(attitudemsg.pitch);//Transforming NED to NWU
					data.rpydata.z = -rqt_quadcoptergui::common::map_angle(attitudemsg.yaw);//Transforming NED to NWU
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
		mavlink_ros::Mavlink mavlink_ros_msg;
		castmavmsgtoros(mavlink_ros_msg,mavmsg); 
		ROS_INFO("Publishing Data req");
		mavlink_pub.publish(mavlink_ros_msg);
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
		mavlink_ros::Mavlink mavlink_ros_msg;
		castmavmsgtoros(mavlink_ros_msg,mavmsg); 
		ROS_INFO("\nPublishing Paramreq");
		mavlink_pub.publish(mavlink_ros_msg);
		ROS_INFO("Publishing done");
	}

};
PLUGINLIB_DECLARE_CLASS(pixhawk_parser, PixhawkParser, pixhawk_parser::PixhawkParser, rqt_quadcoptergui::Parser)
	//TODO Done setting up all the callback functions although need to modify this to ensure they are uptodate with new mavlink protocol; Also given that a kalman filter based position controller is already running onboard I have to dig their code and see if we can implement our arm based position control by modifying their code. Also have to verify how accurate their posn controller is. If it is like 1-2cm then we can do nice manipulation. Then based on vision we have to get the same stuff.
