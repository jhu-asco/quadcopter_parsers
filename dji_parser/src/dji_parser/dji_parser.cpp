/* This is the plugin which is dependent on Parser base class. 
 */
#include <pluginlib/class_list_macros.h>
#include <dji_parser/dji_parser.h>
#include <tf/tf.h>
#include <ros/ros.h>

namespace dji_parser{

DjiParser::DjiParser(): global_ref_lat(0), global_ref_long(0), sdk_opened(false)
                        ,quad_status(0) ,ctrl_mode(0), sdk_status(0), gps_health(0)
{
  this->initialized = false;
}

//PluginLib Initialization function
void DjiParser::initialize(ros::NodeHandle &nh_)
{
  enable_log = false;
  //Set Data Properties
  spin_mutex.lock();
  data.mass = 2.8;//Start with small  values
  data.thrustbias = data.mass*9.81; //We can estimate this later
  data.thrustmax = 3.4*9.81;//Additional payload of 1Kg
  data.thrustmin = 0;
  data.rpbound = M_PI/4;//This is the physical limit enforced by many drivers. This is not the same as the controller bound on angles
  spin_mutex.unlock();

  //Initialize DJI:
  DJI_SDK::init_parameters_and_activate(nh_, user_act_data_, std::bind(&DjiParser::receiveDJIData, this));
  //Wait till dji is initialized properly:
  ros::Time current_time = ros::Time::now();
  while(ros::ok())
  {
    spin_mutex.lock();
    if(this->initialized)
    {
      spin_mutex.unlock();
      break;
    }
    spin_mutex.unlock();
    if((ros::Time::now() - current_time).toSec() > 5.0)//Wait for few secs
    {
      ROS_WARN("Timeout initializing");
      break;
    }
    ros::Rate(10).sleep();//sleep for 0.1 secs
  }
  if(this->initialized)
  {
    ROS_INFO("Initialized dji");
  }
}

//Extend Functions from Paser:

bool DjiParser::takeoff()//Virtual override function
{
    //First request sdk control:
    bool takeoff_result = false;
    if(this->initialized)
    {
      if(!sdk_opened)//If sdk is not opened try opening it
        flowControl(true);

      if(sdk_opened)//If sdk opened successfully then takeoff
      {
        ROS_INFO("Calling Takeoff");
        int res = DJI_Pro_Status_Ctrl(4, 0);
        ROS_INFO("Takeoff Res: %d", res);
        if(res == 0)
          takeoff_result = true;
      }
    }

    return takeoff_result;
}

bool DjiParser::land()
{
  bool land_result = false;
  if(this->initialized)
  {
    if(!sdk_opened)//If sdk is not opened try opening it
      flowControl(true);

    if(sdk_opened)
    {
      int res = DJI_Pro_Status_Ctrl(6, 0);
      if(res == 0)
        land_result = true;
    }
  }
  return land_result;
}

bool DjiParser::disarm()
{
  flowControl(false);
}

bool DjiParser::flowControl(bool request)
{
  bool result = false;

  if(!(this->initialized))
    return result;
  int sdk_req = request?1:0;
  if((request && !sdk_opened)||(!request && sdk_opened))
    DJI_Pro_Control_Management(sdk_req, NULL);
  
  //Wait for 0.5 secs until sdk is opened/closed
  ros::Time current_time = ros::Time::now();
  while(ros::ok())
  {
    spin_mutex.lock();
    sdk_opened = (sdk_status == 0)?false:(sdk_status == 1)?true:false;
    if((request && sdk_opened)||(!request && !sdk_opened))
    {
      result = true;
      spin_mutex.unlock();
      break;
    }
    else if((ros::Time::now() - current_time).toSec() > 5.0)//Wait for few secs
    {
      ROS_INFO("Timeout");
      result = false;
      spin_mutex.unlock();
      break;
    }
    spin_mutex.unlock();
    ros::Rate(10).sleep();//sleep for 0.1 secs
  }
  return result;
}

bool DjiParser::calibrateimubias()
{
  return false;
  //Not Implemented as of now
}

bool DjiParser::cmdrpythrust(geometry_msgs::Quaternion &rpytmsg, bool sendyaw)
{
  if(this->initialized)
  {
    if(sdk_opened)
    {
      attitude_data_t user_ctrl_data;
      user_ctrl_data.ctrl_flag = HORIZ_ATT | VERT_TRU | HORIZ_GND;
      if(sendyaw)
      {
        user_ctrl_data.ctrl_flag = user_ctrl_data.ctrl_flag | YAW_ANG | YAW_GND;
        user_ctrl_data.roll_or_x = rpytmsg.x*(180/M_PI);
        user_ctrl_data.pitch_or_y = rpytmsg.y*(180/M_PI);
        user_ctrl_data.thr_z = rpytmsg.w;
        user_ctrl_data.yaw = rpytmsg.z*(180/M_PI);
        DJI_Pro_Attitude_Control(&user_ctrl_data);
      }
      else
      {
        user_ctrl_data.ctrl_flag = user_ctrl_data.ctrl_flag | YAW_RATE | YAW_BODY;
        user_ctrl_data.roll_or_x = rpytmsg.x*(180/M_PI);
        user_ctrl_data.pitch_or_y = rpytmsg.y*(180/M_PI);
        user_ctrl_data.thr_z = rpytmsg.w;
        user_ctrl_data.yaw = 0;
        DJI_Pro_Attitude_Control(&user_ctrl_data);
      }
    }
  }
  return true;
}

void DjiParser::reset_attitude(double roll, double pitch, double yaw)
{
    //NOT IMPLEMENTED
    return;
}

bool DjiParser::cmdvelguided(geometry_msgs::Vector3 &vel_cmd, double &yaw_ang)
{
  if(this->initialized)
  {
    if(sdk_opened)
    {
      //Convert velocity from NWU frame to NED frame
      //Also velocity in z direction is set such that positive velocity means going up
      attitude_data_t user_ctrl_data;
      //user_ctrl_data.ctrl_flag = HORIZ_VEL | VERT_VEL | HORIZ_BODY | YAW_ANG | YAW_BODY;
      user_ctrl_data.ctrl_flag = HORIZ_VEL | VERT_VEL | HORIZ_GND | YAW_ANG | YAW_GND;
      user_ctrl_data.roll_or_x = vel_cmd.x;
      user_ctrl_data.pitch_or_y = -vel_cmd.y;
      user_ctrl_data.thr_z = vel_cmd.z;
      user_ctrl_data.yaw = -yaw_ang*(180/M_PI);
      DJI_Pro_Attitude_Control(&user_ctrl_data);
    }
  }
  return true;
}

void DjiParser::grip(int state)//TriState Gripper
{
    //NOT IMPLEMENTED
    return;
}

void DjiParser::getquaddata(parsernode::common::quaddata &d1)
{
  spin_mutex.lock();
  //Create Quad State String based on existing status:
  switch(quad_status)
  {
    case STANDBY :
      data.quadstate = "STANDBY";
      break;
    case TAKEOFF :
      data.quadstate = "TAKEOFF";
      break;
    case IN_AIR :
      data.quadstate = "IN_AIR";
      break;
    case LANDING :
      data.quadstate = "LANDING";
      break;
    case FINISH_LANDING :
      data.quadstate = "FINISH_LANDING";
      break;
  }
  switch(ctrl_mode)
  {
    case 0:
      data.quadstate += " RC";
      break;
    case 1:
      data.quadstate += " APP";
      break;
    case 2:
      data.quadstate += " SER";
      break;
  }
  switch(sdk_status)
  {
    case 0:
      data.quadstate += " SCLOSE";
      sdk_opened = false;
      break;
    case 1:
      data.quadstate += " SOPEN";
      sdk_opened = true;
      break;
  }
  data.quadstate += (std::string(" G") + std::to_string(gps_health));
  d1 = data;//Copy data
  spin_mutex.unlock();
  return;
}

void DjiParser::setmode(std::string mode)
{
  //Dont need to implement as the commands can change them easily
}

//Receive DJI Data:
void DjiParser::receiveDJIData()
{
  //Uses DJI SDK In This Function Look in dji_sdk package/include/lib/DJI_LIB for further help
  sdk_std_msg_t recv_sdk_std_msgs;
  unsigned short msg_flags;
  DJI_Pro_Get_Broadcast_Data(&recv_sdk_std_msgs, &msg_flags);

  //static ros::Time starting_time = ros::Time::now();
  //static uint64_t starting_timestamp =

  spin_mutex.lock();
  data.timestamp = (recv_sdk_std_msgs.time_stamp*(1.0/600.0));

  //update attitude msg
  if ((msg_flags & ENABLE_MSG_Q) && (msg_flags & ENABLE_MSG_W)) {
    tf::Quaternion qt(recv_sdk_std_msgs.q.q1, recv_sdk_std_msgs.q.q2,recv_sdk_std_msgs.q.q3,recv_sdk_std_msgs.q.q0);
    tf::Matrix3x3 mat;
    mat.setRotation(qt);
    mat.getEulerYPR(data.rpydata.z, data.rpydata.y, data.rpydata.x);
    data.rpydata.y = -data.rpydata.y;
    data.rpydata.z = -data.rpydata.z;
    if(enable_log)
      imufile<<data.timestamp<<"\t"<<data.rpydata.x<<"\t"<<data.rpydata.y<<"\t"<<data.rpydata.z<<"\t"<<recv_sdk_std_msgs.w.x<<"\t"<<recv_sdk_std_msgs.w.y<<"\t"<<recv_sdk_std_msgs.w.z<<endl;
  }

  //update velocity msg
  if ((msg_flags & ENABLE_MSG_V)) {
    data.linvel.x = recv_sdk_std_msgs.v.x;
    data.linvel.y = recv_sdk_std_msgs.v.y;
    data.linvel.z = recv_sdk_std_msgs.v.z;
    if(enable_log)
      velfile<<data.timestamp<<"\t"<<data.linvel.x<<"\t"<<data.linvel.y<<"\t"<<data.linvel.z<<endl;
  }

  //update acceleration msg
  if ((msg_flags & ENABLE_MSG_A)) {
    data.linacc.x = recv_sdk_std_msgs.a.x;
    data.linacc.y = recv_sdk_std_msgs.a.y;
    data.linacc.z = recv_sdk_std_msgs.a.z;
    if(enable_log)
      accfile<<data.timestamp<<"\t"<<data.linacc.x<<"\t"<<data.linacc.y<<"\t"<<data.linacc.z<<endl;
  }

  //update rc_channel msg
  if ((msg_flags & ENABLE_MSG_RC)) {
    data.servo_in[0] = (int16_t)recv_sdk_std_msgs.rc.roll;
    data.servo_in[1] = (int16_t)recv_sdk_std_msgs.rc.pitch;
    data.servo_in[2] = (int16_t)recv_sdk_std_msgs.rc.throttle;
    data.servo_in[3] = (int16_t)recv_sdk_std_msgs.rc.yaw;
    if(enable_log)
      rcinputfile<<data.timestamp<<"\t"<<data.servo_in[0]<<"\t"<<data.servo_in[1]<<"\t"<<data.servo_in[2]<<"\t"<<data.servo_in[3]<<endl;
    /*rc_channels.mode = recv_sdk_std_msgs.rc.mode;
    rc_channels.gear = recv_sdk_std_msgs.rc.gear;
    rc_channels_publisher.publish(rc_channels);
    */
  }

  //update compass msg
  if ((msg_flags & ENABLE_MSG_MAG)) {
    data.magdata.x = (double)recv_sdk_std_msgs.mag.x;
    data.magdata.y = (double)recv_sdk_std_msgs.mag.y;
    data.magdata.z = (double)recv_sdk_std_msgs.mag.z;
    if(enable_log)
      magfile<<data.timestamp<<"\t"<<data.magdata.x<<"\t"<<data.magdata.y<<"\t"<<data.magdata.z<<endl;
  }

   //Fix Bug with Data Status TODO

  //update flight_status 
  if ((msg_flags & ENABLE_MSG_STATUS)) {
    quad_status = recv_sdk_std_msgs.status;
    if(quad_status == IN_AIR)
      data.armed = true;
    else if(quad_status == FINISH_LANDING)
      data.armed = false;
  }

  //update battery msg
  if ((msg_flags & ENABLE_MSG_BATTERY)) {
    data.batterypercent = (double)recv_sdk_std_msgs.battery_remaining_capacity;
  }

  //update flight control info
  if ((msg_flags & ENABLE_MSG_DEVICE)) {
    //flight_control_info.serial_req_status = recv_sdk_std_msgs.ctrl_info.serial_req_status;
    ctrl_mode = recv_sdk_std_msgs.ctrl_info.cur_ctrl_dev_in_navi_mode; 
  }

  //update obtaincontrol msg
  if ((msg_flags & ENABLE_MSG_TIME)) {
    //SDK Permission
    sdk_status = recv_sdk_std_msgs.obtained_control; 

    //update activation msg
    if(recv_sdk_std_msgs.activation)
    {
      this->initialized = true;
//      ROS_INFO("Initialized DJI");
    }
    else
    {
      this->initialized = false;
    }
  }

  if ((msg_flags & ENABLE_MSG_POS)) {
    //Initialize ref
    if(global_ref_lat == 0 && global_ref_long == 0)
    {
      global_ref_lat = recv_sdk_std_msgs.pos.lati * 180.0 / C_PI;
      global_ref_long = recv_sdk_std_msgs.pos.longti * 180.0 / C_PI;
    }

    //update local_position msg
    DJI_SDK::gps_convert_ned(
        data.localpos.x,
        data.localpos.y,
        recv_sdk_std_msgs.pos.longti * 180.0 / C_PI,
        recv_sdk_std_msgs.pos.lati * 180.0 / C_PI,
        global_ref_long,
        global_ref_lat
        );
    data.localpos.z = recv_sdk_std_msgs.pos.height;
    //Altitude is not used: recv_sdk_std_msgs.pos.alti
    gps_health = (uint8_t)recv_sdk_std_msgs.pos.health_flag;
    if(enable_log)
      localposfile<<data.timestamp<<"\t"<<data.localpos.x<<"\t"<<data.localpos.y<<"\t"<<data.localpos.z<<"\t"<<gps_health<<endl;
  }
  spin_mutex.unlock();
}

};
PLUGINLIB_DECLARE_CLASS(dji_parser, DjiParser, dji_parser::DjiParser, parsernode::Parser)
