#include <quad_simulator_parser/quad_simulator.h>
#include <tf/tf.h>
#include <ros/ros.h>

namespace quad_simulator{

void QuadSimulator::setRCInputs(const sensor_msgs::Joy &joy_msg)
{
   rcin[0] = -(int16_t)parsernode::common::map(joy_msg.axes[0],-0.435,0.435,-10000,10000);
   rcin[1] = -(int16_t)parsernode::common::map(joy_msg.axes[1],-0.479,0.479,-10000,10000);
   rcin[2] = (int16_t)parsernode::common::map(joy_msg.axes[2],-0.6635,0.6635,-10000,10000);
   rcin[3] = -(int16_t)parsernode::common::map(joy_msg.axes[5],-0.52,0.52,-10000,10000);
}

QuadSimulator::QuadSimulator()
    : parsernode::Parser(), delay_send_time_(0.2), battery_percent_(100),
      takeoff_altitude_(0.5), so3(gcop::SO3::Instance()),
      use_perfect_time_(false) {
  this->initialize();
  this->initialized = true;
}

//PluginLib Initialization function
void QuadSimulator::initialize()
{
    state_.Clear();//Sets to default values of Quadrotor state
    prev_vel_cmd_time_ = Clock::now();
    for(int i = 0; i < 4; i++)
        rcin[i] = 0;
    rcin[2] = parsernode::common::map(9.81/(sys_.kt),10,100,-10000,10000);
    enable_qrotor_control_ = true;
    armed = false;
    RpytCmdStruct rpyt_cmd;
    rpyt_cmd.time = TimePoint();
    rpyt_cmd.dt = 0.02;
    rpyt_cmds.push(rpyt_cmd);
    //TODO May be add a kt decrease timer for future
}

//Extend Functions from Paser:
bool QuadSimulator::takeoff()
{
  enable_qrotor_control_ = true;
  armed = true;
  state_.p(2) = takeoff_altitude_;//Set Height to 0.5 m when takeoff
  return true;
}


bool QuadSimulator::land()
{
  state_.Clear();
  armed = false;
  return true;
}


bool QuadSimulator::disarm()
{
  state_.Clear();
  enable_qrotor_control_ = false;
  armed = false;
  return true;
}


bool QuadSimulator::flowControl(bool request)
{
  enable_qrotor_control_ = request;
  return true;
}


bool QuadSimulator::calibrateimubias()
{
  throw std::runtime_error("Not implemented");
  return false;
}

bool QuadSimulator::cmdrpythrust(geometry_msgs::Quaternion &rpytmsg) {
  cmdrpythrustInternal(rpytmsg, false);
}

bool QuadSimulator::cmdrpyawratethrust(geometry_msgs::Quaternion &rpytmsg) {
  cmdrpythrustInternal(rpytmsg, true);
}

bool QuadSimulator::cmdrpythrustInternal(geometry_msgs::Quaternion &rpytmsg, bool rp_angle_yawrate_mode)
{
  if(enable_qrotor_control_)
  {
      RpytCmdStruct &current_cmd = rpyt_cmds.front();
      TimePoint current_time = Clock::now();
      double tdiff = delay_send_time_;
      if (!use_perfect_time_) {
        tdiff =
            ((std::chrono::duration<double>)(current_time - current_cmd.time))
                .count();
      }
      if(tdiff < delay_send_time_-0.02)
      {
        RpytCmdStruct rpyt_cmd;
        rpyt_cmd.time = current_time;
        if(rpyt_cmds.empty())
          rpyt_cmd.dt = 0.02;
        else
          rpyt_cmd.dt = (current_time - rpyt_cmds.back().time).count()/2e3;
        rpyt_cmd.rpytmsg = rpytmsg;
        rpyt_cmds.push(rpyt_cmd);
        geometry_msgs::Vector3 initial_state_vel;
        initial_state_vel.x = state_.v[0];
        initial_state_vel.y = state_.v[1];
        initial_state_vel.z = state_.v[2];
        double yaw_ang = so3.yaw(state_.R);
        cmdvel_yaw_angle_guided(initial_state_vel, yaw_ang);
        state_.u[2] = yaw_ang;
      }
      else if (tdiff > 2*delay_send_time_)
      {
        {
          queue<RpytCmdStruct> empty;
          rpyt_cmds.swap(empty);
        }
        RpytCmdStruct rpyt_cmd;
        rpyt_cmd.time = current_time;
        rpyt_cmd.dt = 0.02;
        rpyt_cmd.rpytmsg = rpytmsg;
        rpyt_cmds.push(rpyt_cmd);
        geometry_msgs::Vector3 initial_state_vel;
        initial_state_vel.x = state_.v[0];
        initial_state_vel.y = state_.v[1];
        initial_state_vel.z = state_.v[2];
        double yaw_ang = so3.yaw(state_.R);
        cmdvel_yaw_angle_guided(initial_state_vel, yaw_ang);
        state_.u[2] = yaw_ang;
      }
      else
      {
        double &dt = current_cmd.dt;
        //Propagate Qrotor State
        Eigen::Vector4d control;
        control<<current_cmd.rpytmsg.w, (current_cmd.rpytmsg.x-state_.u(0)), (current_cmd.rpytmsg.y-state_.u(1)),(current_cmd.rpytmsg.z-state_.u(2));
        for(int j = 0; j < 3; j++)
        {
          control[j+1] = control[j+1]>M_PI?control[j+1]-2*M_PI:(control[j+1]<-M_PI)?control[j+1]+2*M_PI:control[j+1];
          control[j+1] /= dt;
        }
        if(rp_angle_yawrate_mode)
        {
          control[3] = current_cmd.rpytmsg.z;
        }
        gcop::QRotorIDState temp_state;
        if(dt > 0.03)
          dt = 0.03;//Cap the dt for simulation
        sys_.Step(temp_state,0,state_,control,dt,0,0,0,0);
        state_ = temp_state;
        rpyt_cmds.pop();//Delete the Last Element
        //Add current element
        RpytCmdStruct rpyt_cmd;
        rpyt_cmd.rpytmsg = rpytmsg;
        rpyt_cmd.time = current_time;
        if (use_perfect_time_) {
          rpyt_cmd.dt = 0.02;
        } else {
          rpyt_cmd.dt = ((std::chrono::duration<double>)(current_time -
                                                         rpyt_cmds.back().time))
                            .count();
        }
        rpyt_cmds.push(rpyt_cmd);
      }
  }
  else
  {
      std::cerr<<"Quadrotor control not enabled"<<std::endl;
  }
  return true;
}


void QuadSimulator::reset_attitude(double roll, double pitch, double yaw)
{
    Eigen::Vector3d rpy(roll, pitch, yaw);
    so3.q2g(state_.R,rpy);
    return;
}

bool QuadSimulator::cmdvel_yaw_rate_guided(geometry_msgs::Vector3 &vel_cmd, double &yaw_rate)
{
  return cmdvelguided(vel_cmd, yaw_rate, true);
}

bool QuadSimulator::cmdvel_yaw_angle_guided(geometry_msgs::Vector3 &vel_cmd, double &yaw_angle)
{
  return cmdvelguided(vel_cmd, yaw_angle, false);
}

bool QuadSimulator::cmdvelguided(geometry_msgs::Vector3 &vel_cmd, double &yaw_inp, bool vel_yaw_ratemode)
{
  if(enable_qrotor_control_)
  {
      Eigen::Vector3d pos = state_.p;
      Eigen::Matrix3d R1 = state_.R;
      TimePoint current_time = Clock::now();
      double dt = 0.02;
      if (!use_perfect_time_) {
        dt =
            ((std::chrono::duration<double>)(current_time - prev_vel_cmd_time_))
                .count();
        if (dt > 0.03)
          dt = 0.03; // Dont give too long dt
        prev_vel_cmd_time_ = Clock::now();
      }
      state_.Clear();
      state_.v = Eigen::Vector3d(vel_cmd.x, vel_cmd.y, vel_cmd.z);
      //Clear everything but current position=> Holds current position
      state_.p = pos + state_.v*dt;
      if(vel_yaw_ratemode)
      {
        state_.w[2] = yaw_inp;
        Eigen::Vector3d rpy(0,0,yaw_inp*dt);
        so3.q2g(state_.R,rpy);
        state_.R = R1*state_.R;
      }
      else
      {
        //Set Yaw
        Eigen::Vector3d rpy(0,0,yaw_inp);
        so3.q2g(state_.R,rpy);
      }
      //TODO: Create a thread which keeps moving the quadrotor. When other modes of propagating are called, should stop this thread
  }
  else
  {
      std::cerr<<"Quadrotor control not enabled"<<std::endl;
  }
  return true;
}


bool QuadSimulator::cmdwaypoint(geometry_msgs::Vector3 &desired_pos, double desired_yaw)
{
    if(enable_qrotor_control_)
    {
      state_.Clear();
      //Set Qrotor at way point straight away
      state_.p<<desired_pos.x, desired_pos.y, desired_pos.z;
      Eigen::Vector3d rpy(0,0,desired_yaw);
      so3.q2g(state_.R,rpy);
    }
    else
    {
      std::cerr<<"Quadrotor control not enabled"<<std::endl;
    }
  return true;
}


void QuadSimulator::grip(int state)
{
  throw std::runtime_error("Grip not implemented");
  return;
}

void QuadSimulator::getquaddata(parsernode::common::quaddata &d1)
{

  //Return data from state
  d1.altitude = state_.p(2);
  d1.batterypercent = battery_percent_;
  d1.quadstate = "";
  if(armed)
      d1.quadstate += "ARMED ";
  if(enable_qrotor_control_)
      d1.quadstate += "ENABLE_CONTROL ";
  Eigen::Vector3d rpy;
  so3.g2q(rpy,state_.R);
  d1.rpydata.x = rpy(0); d1.rpydata.y = rpy(1); d1.rpydata.z = rpy(2);
  d1.omega.x = state_.w(0); d1.omega.y = state_.w(1); d1.omega.z = state_.w(2);
  d1.linvel.x = state_.v(0); d1.linvel.y = state_.v(1); d1.linvel.z = state_.v(2);
  d1.linacc.x = sys_.acc(0); d1.linacc.y = sys_.acc(1); d1.linacc.z = sys_.acc(2);
  d1.localpos.x = state_.p(0); d1.localpos.y = state_.p(1); d1.localpos.z = state_.p(2);
  d1.timestamp = ((std::chrono::duration<double>)(Clock::now().time_since_epoch())).count();
  d1.rpbound = M_PI/6;
  d1.mass = 1.0;
  d1.thrustbias = 9.81/(sys_.kt);
  d1.armed = armed;
  d1.rc_sdk_control_switch = enable_qrotor_control_;
  for(int i = 0; i < 4; i++)
    d1.servo_in[i] = rcin[i];
  return;
}
}
