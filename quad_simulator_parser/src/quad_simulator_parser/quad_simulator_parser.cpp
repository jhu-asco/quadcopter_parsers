/* This is the plugin which is dependent on Parser base class. 
 */
#include <pluginlib/class_list_macros.h>
#include <quad_simulator_parser/quad_simulator_parser.h>
#include <tf/tf.h>
#include <ros/ros.h>

namespace quad_simulator_parser{

void QuadSimParser::setRCInputs(const sensor_msgs::Joy &joy_msg)
{
   rcin[0] = -(int16_t)parsernode::common::map(joy_msg.axes[0],-0.435,0.435,-10000,10000);
   rcin[1] = -(int16_t)parsernode::common::map(joy_msg.axes[1],-0.479,0.479,-10000,10000);
   rcin[2] = (int16_t)parsernode::common::map(joy_msg.axes[2],-0.6635,0.6635,-10000,10000);
   rcin[3] = (int16_t)parsernode::common::map(joy_msg.axes[5],-0.52,0.52,-10000,10000);
}

QuadSimParser::QuadSimParser()
{
  this->initialized = true;
}

//PluginLib Initialization function
void QuadSimParser::initialize(ros::NodeHandle &nh_)
{
    //Add Subscriber to a joystick for rcinput
    joy_sub_ = nh_.subscribe("joy",5,&QuadSimParser::setRCInputs,this);
    state_.Clear();//Sets to default values of Quadrotor state
    prev_vel_cmd_time_ = ros::Time::now();
    prev_rpy_cmd_time_ = ros::Time::now();
    for(int i = 0; i < 4; i++)
        rcin[i] = 0;
    //rcin[3] is thrust set to base value:
    rcin[2] = parsernode::common::map(9.81/(sys_.kt),10,100,-10000,10000);
    ROS_INFO("RCIn2: %d",rcin[2]);
    rpyt_ratemode = false;
    enable_qrotor_control_ = false;
    armed = false;
}


//Extend Functions from Paser:

bool QuadSimParser::takeoff()//Virtual override function
{
  enable_qrotor_control_ = true;
  armed = true;
  state_.p(2) = 0.5;//Set Height to 0.5 m when takeoff
  ROS_INFO("Taking Off");
  return true;
}


bool QuadSimParser::land()
{
  state_.Clear();
  ROS_INFO("Landed");
  armed = false;
  enable_qrotor_control_ = false;
  return true;
}


bool QuadSimParser::disarm()
{
  ROS_INFO("Disarmed");
  state_.Clear();
  enable_qrotor_control_ = false;
  armed = false;
  return true;
}


bool QuadSimParser::flowControl(bool request)
{
  ROS_INFO("Enabling Control of quadcopter");
  enable_qrotor_control_ = request;
  return true;
}


bool QuadSimParser::calibrateimubias()
{
  //Not Implemented as of now
  return false;
}

bool QuadSimParser::cmdrpythrust(geometry_msgs::Quaternion &rpytmsg, bool sendyaw)
{
  if(enable_qrotor_control_)
  {
      ros::Time current_time = ros::Time::now();
      double dt = (current_time - prev_rpy_cmd_time_).toSec();
      //Propagate Qrotor State
      Vector4d control;
      if(rpyt_ratemode)
      {
        control<<rpytmsg.w, rpytmsg.x, rpytmsg.y, rpytmsg.z;
      }
      else
      {
        control<<rpytmsg.w, (rpytmsg.x-state_.u(0))/dt, (rpytmsg.y-state_.u(1))/dt,(rpytmsg.z-state_.u(2))/dt;
        //ROS_INFO("Rate Computed: %f,%f,%f",control[1], control[2], control[3]);
      }
      if(!sendyaw)
          control(3) = 0;
      QRotorIDState temp_state;
      //int number_of_steps = (dt/0.01);
      //ROS_INFO("Number of steps: %d",number_of_steps);
      //for(int count = 0; count< number_of_steps; count++)
      //{
      if(dt > 0.03)
        dt = 0.02;//Cap the dt for simulation
      sys_.Step(temp_state,0,state_,control,dt,0,0,0,0);
      //  state_ = temp_state;
      //}
      prev_rpy_cmd_time_ = current_time;
      state_ = temp_state;
  }
  else
  {
      ROS_WARN("Quadrotor control not enabled");
  }
  return true;
}


void QuadSimParser::reset_attitude(double roll, double pitch, double yaw)
{
    SO3 &so3 = SO3::Instance();
    Vector3d rpy(roll, pitch, yaw);
    so3.q2g(state_.R,rpy);
    return;
}

bool QuadSimParser::cmdvelguided(geometry_msgs::Vector3 &vel_cmd, double &yaw_ang)
{
  if(enable_qrotor_control_)
  {
      Vector3d pos = state_.p;
      ros::Time current_time = ros::Time::now();
      double dt = (current_time - prev_vel_cmd_time_).toSec();
      prev_vel_cmd_time_ = ros::Time::now();
      state_.Clear();
      state_.p = pos + Vector3d(vel_cmd.x, vel_cmd.y, vel_cmd.z)*dt;//Clear everything but current position=> Holds current position
      //Set Yaw
      SO3 &so3 = SO3::Instance();
      Vector3d rpy(0,0,yaw_ang);
      so3.q2g(state_.R,rpy);
      //Propagate Qrotor State
      //TODO: Create a thread which keeps moving the quadrotor. When other modes of propagating are called, should stop this thread
  }
  else
  {
      ROS_WARN("Quadrotor control not enabled");
  }
  return true;
}


bool QuadSimParser::cmdwaypoint(geometry_msgs::Vector3 &desired_pos, double desired_yaw)
{
    if(enable_qrotor_control_)
    {
      state_.Clear();
      //Set Qrotor at way point straight away
      state_.p<<desired_pos.x, desired_pos.y, desired_pos.z;
      SO3 &so3 = SO3::Instance();
      Vector3d rpy(0,0,desired_yaw);
      so3.q2g(state_.R,rpy);
    }
    else
    {
      ROS_WARN("Quadrotor control not enabled");
    }
  return true;
}


void QuadSimParser::grip(int state)//TriState Gripper
{
  //NOT IMPLEMENTED
  return;
}

void QuadSimParser::getquaddata(parsernode::common::quaddata &d1)
{

  SO3 &so3 = SO3::Instance();
  //Return data from state
  d1.altitude = state_.p(2);
  d1.batterypercent = 100;
  d1.quadstate = "";
  if(armed)
      d1.quadstate += "ARMED ";
  if(enable_qrotor_control_)
      d1.quadstate += "ENABLE_CONTROL ";
  if(rpyt_ratemode)
      d1.quadstate += "RATE MODE ";
  Vector3d rpy;
  so3.g2q(rpy,state_.R);
  d1.rpydata.x = rpy(0); d1.rpydata.y = rpy(1); d1.rpydata.z = rpy(2);
  d1.omega.x = state_.w(0); d1.omega.y = state_.w(1); d1.omega.z = state_.w(2);
  d1.linvel.x = state_.v(0); d1.linvel.y = state_.v(1); d1.linvel.z = state_.v(2);
  d1.linacc.x = sys_.acc(0); d1.linacc.y = sys_.acc(1); d1.linacc.z = sys_.acc(2);
  d1.localpos.x = state_.p(0); d1.localpos.y = state_.p(1); d1.localpos.z = state_.p(2);
  d1.timestamp = ros::Time::now().toSec();
  d1.rpbound = M_PI/6;
  d1.mass = 1.0;
  d1.thrustbias = 9.81/(sys_.kt);
  d1.armed = armed;
  for(int i = 0; i < 4; i++)
    d1.servo_in[i] = rcin[i];
  return;
}


void QuadSimParser::setmode(std::string mode)
{
  if(strcmp(mode.c_str(),"rpyt_rate")==0)
  {
    rpyt_ratemode = true;
  }
  else if(strcmp(mode.c_str(),"rpyt_angle")==0)
  {
    rpyt_ratemode = false;
  }
  return;
}


};
PLUGINLIB_DECLARE_CLASS(quad_simulator_parser, QuadSimParser, quad_simulator_parser::QuadSimParser, parsernode::Parser)
