/* This is the plugin which is dependent on Parser base class. 
 */
#include <pluginlib/class_list_macros.h>
#include <dji_parser/dji_parser.h>
#include <tf/tf.h>
#include <ros/ros.h>

namespace dji_parser{

DjiParser::DjiParser()
{
  this->initialized = false;
}

//PluginLib Initialization function
void DjiParser::initialize(ros::NodeHandle &nh_)
{
  enable_log = false;
  //Set Data Properties
  data.mass = 2.8;//Start with small  values
  data.thrustbias = data.mass*9.81; //We can estimate this later
  data.thrustmax = 3.4*9.81;//Additional payload of 1Kg
  data.thrustmin = 0;
  data.rpbound = M_PI/4;//This is the physical limit enforced by many drivers. This is not the same as the controller bound on angles

  //Create a DJI Drone class to receive dji data:
  dji_core.reset(new DJIDrone(nh_));
  //Wait till dji is initialized properly:
  ros::Time current_time = ros::Time::now();
  while(!dji_core->activation)
  {
      if((ros::Time::now() - current_time).toSec() > 5.0)//Wait for few secs
          break;
      ros::spinOnce();
      ros::Rate(10).sleep();//sleep for 0.1 secs
  }
  if(dji_core->activation)
  {
      this->initialized = true;
      ROS_INFO("Initialized");
  }
}

//Extend Functions from Paser:

bool DjiParser::takeoff()//Virtual override function
{
    //First request sdk control:
    bool takeoff_result = false;
    if(dji_core && (this->initialized))
    {
      bool sdk_opened = false;
      if(!dji_core->sdk_permission_opened)//If sdk is not opened try opening it
        sdk_opened = flowControl(true);
      else 
        sdk_opened = true;//If sdk is already opened 

      if(sdk_opened)//If sdk opened successfully then takeoff
        takeoff_result =  dji_core->takeoff();
    }

    if(takeoff_result)
        data.armed = true;
    return takeoff_result;
}

bool DjiParser::land()
{
  if(dji_core && (this->initialized))
  {
    bool sdk_opened = false;
    if(!dji_core->sdk_permission_opened)//If sdk is not opened try opening it
      sdk_opened = flowControl(true);
    else 
      sdk_opened = true;//If sdk is already opened 

    if(sdk_opened)
      return dji_core->landing();
  }
  return false;
}

bool DjiParser::disarm()
{
  return flowControl(false);
}

bool DjiParser::flowControl(bool request)
{
  bool result = false;

  if(!dji_core || !(this->initialized))
    return result;

  if(request)
  {
    if(!dji_core->sdk_permission_opened)
       result = dji_core->request_sdk_permission_control();
     else
       result = true;
  }
  else
  {
    if(dji_core->sdk_permission_opened)
      result = dji_core->release_sdk_permission_control();
    else
      result = true;
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
  if(dji_core && (this->initialized))
  {
    if(dji_core->sdk_permission_opened)
    {
      //Check if mode is matching:
      unsigned char control_mode = HORIZ_ATT | VERT_TRU | HORIZ_GND;
      if(sendyaw)
      {
        control_mode = control_mode | YAW_ANG | YAW_GND;
        return dji_core->attitude_control(control_mode, rpytmsg.x*(180/M_PI), rpytmsg.y*(180/M_PI), rpytmsg.w, rpytmsg.z*(180/M_PI));
      }
      else
      {
        control_mode = control_mode | YAW_RATE | YAW_BODY;
        return dji_core->attitude_control(control_mode, rpytmsg.x*(180/M_PI), rpytmsg.y*(180/M_PI), rpytmsg.w, 0);//0 yaw rate
      }
    }
  }
  return false;
}

void DjiParser::reset_attitude(double roll, double pitch, double yaw)
{
    //NOT IMPLEMENTED
    return;
}

bool DjiParser::cmdvelguided(geometry_msgs::Vector3 &vel_cmd, double &yaw_rate)
{
  if(dji_core && (this->initialized))
  {
    if(dji_core->sdk_permission_opened)
    {
      unsigned char control_mode = HORIZ_VEL | VERT_VEL | HORIZ_BODY | YAW_RATE | YAW_BODY;
      //Convert velocity from NWU frame to NED frame
      //Also velocity in z direction is set such that positive velocity means going up
      return dji_core->attitude_control(control_mode, vel_cmd.x, -vel_cmd.y, vel_cmd.z, -yaw_rate*(180/M_PI));
    }
  }
  return false;
}

void DjiParser::grip(int state)//TriState Gripper
{
    //NOT IMPLEMENTED
    return;
}

void DjiParser::getquaddata(parsernode::common::quaddata &d1)
{
  //this->initialized = dji_core->activation;
  data.batterypercent = double(dji_core->power_status.percentage);
  switch(dji_core->flight_status)
  {
  case STANDBY :
    data.quadstate = "STANDBY";
    break;
  case TAKEOFF :
    data.quadstate = "TAKEOFF";
    break;
  case IN_AIR :
    data.quadstate = "IN_AIR";
    data.armed = true;
    break;
  case LANDING :
    data.quadstate = "LANDING";
    break;
  case FINISH_LANDING :
    data.quadstate = "FINISH_LANDING";
    data.armed = false;
    break;
  }

  /*Flight Control Info.cur_ctrl_dev_in_navi_mode:  0->rc  1->app  2->serial*/
  switch(dji_core->flight_control_info.cur_ctrl_dev_in_navi_mode)
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
  /*Flight Control Info.serial_req_status: 1->open  0->closed*/
  switch(dji_core->flight_control_info.serial_req_status)
  {
  case 0:
    data.quadstate += " SCLOSE";
    break;
  case 1:
    data.quadstate += " SOPEN";
    break;
  }
  //TODO: What is local_posbase_use_height in dji_drone.h

  //Add data from quadcopter
  {
    //RPY:
    tf::Quaternion qt(dji_core->attitude_quaternion.q1, dji_core->attitude_quaternion.q2, dji_core->attitude_quaternion.q3, dji_core->attitude_quaternion.q0);
    tf::Matrix3x3 mat;
    mat.setRotation(qt);
    mat.getEulerYPR(data.rpydata.z, data.rpydata.y, data.rpydata.x);
  }
  //Compass:
  data.magdata.x = double(dji_core->compass.x); data.magdata.y = double(dji_core->compass.y); data.magdata.z = double(dji_core->compass.z);
  //Altitude:
  data.altitude = dji_core->global_position.height;
  //linvel:
  data.linvel.x = double(dji_core->velocity.vx); data.linvel.y = double(dji_core->velocity.vy); data.linvel.z = double(dji_core->velocity.vz);
  //rc data:
  data.servo_in[0] = (int16_t)dji_core->rc_channels.roll; data.servo_in[1] = (int16_t)dji_core->rc_channels.pitch; data.servo_in[2] = (int16_t)dji_core->rc_channels.yaw; data.servo_in[3] = (int16_t)dji_core->rc_channels.throttle;
  //localpos:
  data.localpos.x = dji_core->local_position.x;  data.localpos.y = dji_core->local_position.y; data.localpos.z = dji_core->local_position.z; 
  d1 = data;//Copy data
  return;
}

void DjiParser::setmode(std::string mode)
{
  //Dont need to implement as the commands can change them easily
}

};
PLUGINLIB_DECLARE_CLASS(dji_parser, DjiParser, dji_parser::DjiParser, parsernode::Parser)
