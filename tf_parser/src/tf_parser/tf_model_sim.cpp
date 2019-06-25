#include <tf_parser/tf_model_sim.h>
#include <algorithm>

namespace tf_parser {

TFModelSim::TFModelSim()
    : parsernode::Parser(), delay_send_time_(0.02), battery_percent_(100),
      takeoff_altitude_(0.5),
      use_perfect_time_(false),
      //model_("/home/matt/spo/python/deep/model/logs/variable_log/2019-06-05_11-12-41/") {
      model_("/home/matt/spo/python/deep/model/logs/variable_log/2019-06-20_09-37-52/") {
  this->initialize();
  this->initialized = true;
}

void TFModelSim::setRCInputs(const sensor_msgs::Joy &joy_msg)
{
   rcin[0] = -(int16_t)parsernode::common::map(joy_msg.axes[0],-0.435,0.435,-10000,10000);
   rcin[1] = -(int16_t)parsernode::common::map(joy_msg.axes[1],-0.479,0.479,-10000,10000);
   rcin[2] = (int16_t)parsernode::common::map(joy_msg.axes[2],-0.6635,0.6635,-10000,10000);
   rcin[3] = -(int16_t)parsernode::common::map(joy_msg.axes[5],-0.52,0.52,-10000,10000);
   // If one switch is flipped make it manual
   if(joy_msg.axes[3] < 0.5) {
     flowControl(false);
     rc_sdk_switch = false;
   } else {
     rc_sdk_switch = true;
   }
}

//PluginLib Initialization function
void TFModelSim::initialize()
{
    state_.Clear();//Sets to default values of Quadrotor state
    prev_vel_cmd_time_ = Clock::now();
    for(int i = 0; i < 4; i++)
        rcin[i] = 0;
    rcin[2] = parsernode::common::map(9.81, 10, 100, -10000, 10000);
    flowControl(true);
    armed = false;
    RpytCmdStruct rpyt_cmd;
    rpyt_cmd.rpytmsg.w = 9.81;
    rpyt_cmd.rpytmsg.x = rpyt_cmd.rpytmsg.y = rpyt_cmd.rpytmsg.z = 0;
    rpyt_cmd.time = TimePoint();
    rpyt_cmd.dt = 0.02;
    rpyt_cmds.push(rpyt_cmd);
}


//Extend Functions from Paser:
bool TFModelSim::takeoff()
{
  flowControl(true);
  armed = true;
  state_.p(2) = takeoff_altitude_; //Set Height to 0.5 m when takeoff
  return true;
}


bool TFModelSim::land()
{
  state_.Clear();
  armed = false;
  return true;
}


bool TFModelSim::disarm()
{
  state_.Clear();
  flowControl(false);
  armed = false;
  return true;
}


bool TFModelSim::flowControl(bool request)
{
  rc_sdk_switch = request;
  enable_qrotor_control_ = request;
  return true;
}


bool TFModelSim::calibrateimubias()
{
  throw std::runtime_error("Not implemented");
  return false;
}

bool TFModelSim::cmdrpythrust(geometry_msgs::Quaternion &rpytmsg) {
  cmdrpythrustInternal(rpytmsg, false);
}

bool TFModelSim::cmdrpyawratethrust(geometry_msgs::Quaternion &rpytmsg) {
  cmdrpythrustInternal(rpytmsg, true);
}

bool TFModelSim::cmdrpythrustInternal(geometry_msgs::Quaternion &rpytmsg, bool rp_angle_yawrate_mode)
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
        double yaw_ang = state_.rpy(2);
        cmdvel_yaw_angle_guided(initial_state_vel, yaw_ang);
        //state_.u[2] = yaw_ang;
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
        double yaw_ang = state_.rpy(2);
        cmdvel_yaw_angle_guided(initial_state_vel, yaw_ang);
        //state_.u[2] = yaw_ang;
      }
      else
      {
        double &dt = current_cmd.dt;
        //Propagate Qrotor State
        Eigen::Vector4d control;
        control << current_cmd.rpytmsg.w, current_cmd.rpytmsg.x, current_cmd.rpytmsg.y, current_cmd.rpytmsg.z;
        for(int j = 0; j < 3; j++)
        {
          control[j+1] = control[j+1]>M_PI?control[j+1]-2*M_PI:(control[j+1]<-M_PI)?control[j+1]+2*M_PI:control[j+1];
        }
        if(dt > 0.03)
          dt = 0.03; //Cap the dt for simulation

        double control_bounds[4][2] = {{8., 12.}, {-M_PI/10, M_PI/10}, {-M_PI/10, M_PI/10}, {-M_PI, M_PI}};
        for(int j = 0; j < control.size(); j++)
        {
          control[j] = std::min(std::max(control[j], control_bounds[j][0]), control_bounds[j][1]);
        }

        State next_state;
        Eigen::Vector3f control_in(control(1), control(2), control(0));
        model_.predict(state_, control_in, next_state, dt);

        if(rp_angle_yawrate_mode)
        {
          next_state.rpy(2) = state_.rpy(2) + control(3) * dt;
          if(next_state.rpy(2) > M_PI) next_state.rpy(2) -= 2 * M_PI;
          if(next_state.rpy(2) < -M_PI) next_state.rpy(2) += 2 * M_PI;
        }
        else
        {
          next_state.rpy(2) = control(3);
        }

        state_ = next_state;

        double rpy_bounds[2][2] = {{-M_PI/9, M_PI/9}, {-M_PI/9, M_PI/9}};
        for(int j = 0; j < 2; j++)
        {
          state_.rpy[j] = std::min(std::max(state_.rpy[j], rpy_bounds[j][0]), rpy_bounds[j][1]);
        }

        double a_b_bounds[3][2] = {{-2.0, 2.0}, {-2.0, 2.0}, {-2.0, 2.0}};
        for(int j = 0; j < state_.a_b.size(); j++)
        {
          state_.a_b[j] = std::min(std::max(state_.a_b[j], a_b_bounds[j][0]), a_b_bounds[j][1]);
        }

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


void TFModelSim::reset_attitude(double roll, double pitch, double yaw)
{
    state_.rpy << roll, pitch, yaw;
    return;
}

bool TFModelSim::cmdvel_yaw_rate_guided(geometry_msgs::Vector3 &vel_cmd, double &yaw_rate)
{
  return cmdvelguided(vel_cmd, yaw_rate, true);
}

bool TFModelSim::cmdvel_yaw_angle_guided(geometry_msgs::Vector3 &vel_cmd, double &yaw_angle)
{
  return cmdvelguided(vel_cmd, yaw_angle, false);
}

bool TFModelSim::cmdvelguided(geometry_msgs::Vector3 &vel_cmd, double &yaw_inp, bool vel_yaw_ratemode)
{
  if(enable_qrotor_control_)
  {
      Eigen::Vector3d pos = state_.p;
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
        state_.rpy(2) += yaw_inp * dt;
        if (state_.rpy(2) > M_PI) { state_.rpy(2) -= 2*M_PI;}
        if (state_.rpy(2) < -M_PI) { state_.rpy(2) += 2*M_PI;}
      }
      else
      {
        state_.rpy(2) = yaw_inp;
      }
  }
  else
  {
      std::cerr << "Quadrotor control not enabled" << std::endl;
  }
  return true;
}


bool TFModelSim::cmdwaypoint(geometry_msgs::Vector3 &desired_pos, double desired_yaw)
{
  if(enable_qrotor_control_)
  {
    state_.Clear();
    //Set Qrotor at way point straight away
    state_.p << desired_pos.x, desired_pos.y, desired_pos.z;
    state_.rpy << 0, 0, desired_yaw;
  }
  else
  {
    std::cerr<<"Quadrotor control not enabled"<<std::endl;
  }
  return true;
}


void TFModelSim::grip(int state)
{
  throw std::runtime_error("Grip not implemented");
  return;
}

void TFModelSim::getquaddata(parsernode::common::quaddata &d1)
{

  //Return data from state
  d1.altitude = state_.p(2);
  d1.batterypercent = battery_percent_;
  d1.quadstate = "";
  if(armed)
      d1.quadstate += "ARMED ";
  if(enable_qrotor_control_)
      d1.quadstate += "ENABLE_CONTROL ";
  d1.rpydata.x = state_.rpy(0); d1.rpydata.y = state_.rpy(1); d1.rpydata.z = state_.rpy(2);
  d1.omega.x = state_.w(0); d1.omega.y = state_.w(1); d1.omega.z = state_.w(2);
  d1.linvel.x = state_.v(0); d1.linvel.y = state_.v(1); d1.linvel.z = state_.v(2);
  d1.linacc.x = state_.a_b(0); d1.linacc.y = state_.a_b(1); d1.linacc.z = state_.a_b(2);
  d1.localpos.x = state_.p(0); d1.localpos.y = state_.p(1); d1.localpos.z = state_.p(2);
  d1.timestamp = ((std::chrono::duration<double>)(Clock::now().time_since_epoch())).count();
  d1.rpbound = M_PI/6;
  d1.mass = 1.0;
  d1.thrustbias = 9.81;
  d1.armed = armed;
  d1.rc_sdk_control_switch = rc_sdk_switch;
  for(int i = 0; i < 4; i++)
    d1.servo_in[i] = rcin[i];
  return;
}
};
