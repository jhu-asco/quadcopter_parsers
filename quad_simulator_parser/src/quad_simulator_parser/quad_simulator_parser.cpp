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
   rcin[3] = -(int16_t)parsernode::common::map(joy_msg.axes[5],-0.52,0.52,-10000,10000);
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
    //Add Publisher:
    global_ref_pub = nh_.advertise<sensor_msgs::NavSatFix>("gps/fix",10, true);//Latched gps fix publisher
    {
      //Publish 0 as ref msg:
      sensor_msgs::NavSatFix nav_fix_msg;
      nav_fix_msg.altitude = 0;
      nav_fix_msg.latitude = 0;
      nav_fix_msg.longitude = 0;
      nav_fix_msg.header.stamp = ros::Time::now();
      global_ref_pub.publish(nav_fix_msg);
    }
    gps_pub = nh_.advertise<sensor_msgs::NavSatFix>("gps",1);//Gps publisher
    gps_pub_timer_ = nh_.createTimer(ros::Duration(1.0), &QuadSimParser::gpsTimerCallback, this);

    state_.Clear();//Sets to default values of Quadrotor state
    prev_vel_cmd_time_ = ros::Time::now();
    //prev_rpy_cmd_time_ = ros::Time::now();
    for(int i = 0; i < 4; i++)
        rcin[i] = 0;
    //sys_.kp[2] = 5;
    //rcin[3] is thrust set to base value:
    rcin[2] = parsernode::common::map(9.81/(sys_.kt),10,100,-10000,10000);
    ROS_INFO("RCIn2: %d",rcin[2]);
    rpyt_ratemode = false;
    vel_yaw_ratemode = false;
    enable_qrotor_control_ = false;
    armed = false;
    RpytCmdStruct rpyt_cmd;
    rpyt_cmd.time = ros::Time(0);
    rpyt_cmd.dt = 0.02;
    rpyt_cmds.push(rpyt_cmd);
    nh_.param<double>("/reconfig/delay_send_time", delay_send_time_,0.2);
    kt_decrease_timer_ = nh_.createTimer(ros::Duration(5.0), &QuadSimParser::ktTimerCallback, this);
}

void QuadSimParser::ktTimerCallback(const ros::TimerEvent& )
{
  if(sys_.kt > 0.1)
    sys_.kt -= 0.00005;//Decrease kt by a small amount
}

void QuadSimParser::gpsTimerCallback(const ros::TimerEvent& )
{
  sensor_msgs::NavSatFix nav_fix_msg;
  nav_fix_msg.altitude = state_.p[2];
  ned_convert_gps(state_.p[0], state_.p[1], nav_fix_msg.latitude, nav_fix_msg.longitude);
  nav_fix_msg.header.stamp = ros::Time::now();
  gps_pub.publish(nav_fix_msg);
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
      RpytCmdStruct &current_cmd = rpyt_cmds.front();
      ros::Time current_time = ros::Time::now();
      double tdiff = (current_time - current_cmd.time).toSec();
      if(tdiff < delay_send_time_-0.02)
      {
        cout<<"Tdiff: "<<tdiff<<endl;
        RpytCmdStruct rpyt_cmd;
        rpyt_cmd.time = current_time;
        if(rpyt_cmds.empty())
          rpyt_cmd.dt = 0.02;
        else
          rpyt_cmd.dt = (current_time - rpyt_cmds.back().time).toSec();
        rpyt_cmd.rpytmsg = rpytmsg;
        rpyt_cmds.push(rpyt_cmd);
        geometry_msgs::Vector3 initial_state_vel;
        initial_state_vel.x = state_.v[0];
        initial_state_vel.y = state_.v[1];
        initial_state_vel.z = state_.v[2];
        SO3 &so3 = SO3::Instance();
        double yaw_ang = so3.yaw(state_.R);
        bool temp_vel_yaw_ratemode = vel_yaw_ratemode;
        cmdvelguided(initial_state_vel, yaw_ang);
        vel_yaw_ratemode = temp_vel_yaw_ratemode;
        state_.u[2] = yaw_ang;
        //ROS_INFO("Yaw set: %f",yaw_ang);
      }
      else if (tdiff > 2*delay_send_time_)
      {
        {
          queue<RpytCmdStruct> empty;
          rpyt_cmds.swap(empty);
        }
        cout<<"Tdiff: "<<tdiff<<endl;
        RpytCmdStruct rpyt_cmd;
        rpyt_cmd.time = current_time;
        if(rpyt_cmds.empty())
          rpyt_cmd.dt = 0.02;
        else
          rpyt_cmd.dt = (current_time - rpyt_cmds.back().time).toSec();
        rpyt_cmd.rpytmsg = rpytmsg;
        rpyt_cmds.push(rpyt_cmd);
        geometry_msgs::Vector3 initial_state_vel;
        initial_state_vel.x = state_.v[0];
        initial_state_vel.y = state_.v[1];
        initial_state_vel.z = state_.v[2];
        SO3 &so3 = SO3::Instance();
        double yaw_ang = so3.yaw(state_.R);
        bool temp_vel_yaw_ratemode = vel_yaw_ratemode;
        vel_yaw_ratemode = false;
        cmdvelguided(initial_state_vel, yaw_ang);
        vel_yaw_ratemode = temp_vel_yaw_ratemode;
        //ROS_INFO("Yaw set: %f",yaw_ang);
        state_.u[2] = yaw_ang;
      }
      else
      {
        double &dt = current_cmd.dt;
        //Propagate Qrotor State
        Vector4d control;
        if(rpyt_ratemode)
        {
          control<<current_cmd.rpytmsg.w, current_cmd.rpytmsg.x, current_cmd.rpytmsg.y, current_cmd.rpytmsg.z;
        }
        else
        {
          control<<current_cmd.rpytmsg.w, (current_cmd.rpytmsg.x-state_.u(0)), (current_cmd.rpytmsg.y-state_.u(1)),(current_cmd.rpytmsg.z-state_.u(2));
          for(int j = 0; j < 3; j++)
          {
            control[j+1] = control[j+1]>M_PI?control[j+1]-2*M_PI:(control[j+1]<-M_PI)?control[j+1]+2*M_PI:control[j+1];
            control[j+1] /= dt;
          }
          //ROS_INFO("Rate Computed: %f,%f,%f; rpytmsg.z: %f",control[1], control[2], control[3], current_cmd.rpytmsg.z);
          //ROS_INFO("current_cmdyaw: %f, state.yaw: %f", current_cmd.rpytmsg.z, state_.u(2));
        }
        if(!sendyaw)
          control(3) = 0;
        QRotorIDState temp_state;
        //int number_of_steps = (dt/0.01);
        //ROS_INFO("Number of steps: %d",number_of_steps);
        //for(int count = 0; count< number_of_steps; count++)
        //{
        if(dt > 0.03)
          dt = 0.03;//Cap the dt for simulation
        sys_.Step(temp_state,0,state_,control,dt,0,0,0,0);
        //cout<<"sys.kt: "<<sys_.kt<<", thrust: "<<control[0]<<endl;
        //  state_ = temp_state;
        //}
        state_ = temp_state;
        rpyt_cmds.pop();//Delete the Last Element
        //Add current element
        RpytCmdStruct rpyt_cmd;
        rpyt_cmd.rpytmsg = rpytmsg;
        rpyt_cmd.time = current_time;
        rpyt_cmd.dt = (current_time - rpyt_cmds.back().time).toSec();
        rpyt_cmds.push(rpyt_cmd);
      }
      
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

bool QuadSimParser::cmdvelguided(geometry_msgs::Vector3 &vel_cmd, double &yaw_inp)
{
  if(enable_qrotor_control_)
  {
      Vector3d pos = state_.p;
      Matrix3d R1 = state_.R;
      ros::Time current_time = ros::Time::now();
      double dt = (current_time - prev_vel_cmd_time_).toSec();
      if(dt > 0.03)
        dt = 0.03;//Dont give too long dt
      prev_vel_cmd_time_ = ros::Time::now();
      state_.Clear();
      state_.v = Vector3d(vel_cmd.x, vel_cmd.y, vel_cmd.z);
      state_.p = pos + state_.v*dt;//Clear everything but current position=> Holds current position
      if(vel_yaw_ratemode)
      {
        state_.w[2] = yaw_inp;
        SO3 &so3 = SO3::Instance();
        Vector3d rpy(0,0,yaw_inp*dt);
        so3.q2g(state_.R,rpy);
        state_.R = R1*state_.R;
      }
      else
      {
        //Set Yaw
        SO3 &so3 = SO3::Instance();
        Vector3d rpy(0,0,yaw_inp);
        so3.q2g(state_.R,rpy);
      }
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
  //cout<<"rpy: "<<d1.rpydata.x<<", "<<d1.rpydata.y<<", "<<d1.rpydata.z<<endl;
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
  else if(strcmp(mode.c_str(),"vel_angle")==0)
  {
    vel_yaw_ratemode = false;
  }
  else if(strcmp(mode.c_str(),"vel_rate")==0)
  {
    vel_yaw_ratemode = true;
    ROS_INFO("Vel Yaw ratemode");
  }
  return;
}


};
PLUGINLIB_DECLARE_CLASS(quad_simulator_parser, QuadSimParser, quad_simulator_parser::QuadSimParser, parsernode::Parser)
