/* This is the plugin which is dependent on Parser base class.
 * It provides code implementation for DJI Matrice Quadcopter in
 * HIL mode.
 * In this mode, the data from Quadrotor is provided. But functions
 * such as arming, takeoff etc only modify the height of the quadrotor
 * and state of the quadrotor without sending those commands
 * to the low-level API. This can be used to test controllers and estimators
 * without actually flying the Quadrotor.
 */
#include <pluginlib/class_list_macros.h>
#include <dji_parser/dji_hil_parser.h>
#include <tf/tf.h>
#include <ros/ros.h>
#include <stdexcept>

// Flags valid for M100
// Change flags for M600
// based on shift bit
#define HAS_Q 0x0002
#define HAS_A 0x0004
#define HAS_V 0x0008
#define HAS_W 0x0010
#define HAS_POS 0x0020
#define HAS_MAG 0x0040
#define HAS_RC 0x0080
#define HAS_GIMBAL 0x0100
#define HAS_STATUS 0x0200
#define HAS_BATTERY 0x0400
#define HAS_DEVICE 0x0800


using namespace DJI::onboardSDK;

namespace dji_parser{

DjiHILParser::DjiHILParser(): global_ref_lat(0), global_ref_long(0), sdk_opened(false)
                        ,quad_status(0) ,ctrl_mode(0), sdk_status(0), gps_health(0)
                        , rpyt_ratemode(true), vel_yaw_ratemode(false)
{
  this->initialized = false;
}

//PluginLib Initialization function
void DjiHILParser::initialize(ros::NodeHandle &nh_)
{
  enable_log = false;
  //Set Data Properties
  spin_mutex.lock();
  gps_pub_rate = 1.;
  last_gps_pub_time = ros::Time::now();
  data.mass = 2.8;//Start with small  values
  data.thrustbias = data.mass*9.81; //We can estimate this later
  data.thrustmax = 3.4*9.81;//Additional payload of 1Kg
  data.thrustmin = 0;
  data.rpbound = M_PI/4;//This is the physical limit enforced by many drivers. This is not the same as the controller bound on angles
  spin_mutex.unlock();
  //Initialize ros publishers:
  global_ref_pub = nh_.advertise<sensor_msgs::NavSatFix>("gps/fix",10, true);//Latched gps fix publisher
  gps_pub = nh_.advertise<sensor_msgs::NavSatFix>("gps",1);//Gps publisher

  //Initialize DJI:
  init_parameters_and_activate(nh_, &user_act_data_, DjiHILParser::statReceiveDJIData);
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
      throw std::runtime_error("Timeout initializing DJI!");
      break;
    }
    ros::Rate(10).sleep();//sleep for 0.1 secs
  }
  if(this->initialized)
  {
    ROS_INFO("Initialized dji");
  }

  tf_timer_ = nh_.createTimer(ros::Duration(0.1), &DjiHILParser::tfTimerCallback, this);
}

void DjiHILParser::tfTimerCallback(const ros::TimerEvent&)
{
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(data.localpos.x, data.localpos.y, data.localpos.z) );
  tf::Quaternion q;
  q.setRPY(data.rpydata.x, data.rpydata.y, data.rpydata.z);
  transform.setRotation(q);
  tf_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "quad"));
}

//Extend Functions from Paser:

bool DjiHILParser::takeoff()//Virtual override function
{
    spin_mutex.lock();
    data.localpos.z = 1.0;
    data.altitude = 1.0;
    spin_mutex.unlock();
    return true;
}

bool DjiHILParser::land()
{
    spin_mutex.lock();
    data.localpos.z = 0.0;
    data.altitude = 0.0;
    spin_mutex.unlock();
  return true;
}

bool DjiHILParser::disarm()
{
  flowControl(false);
}

bool DjiHILParser::flowControl(bool request)
{
  bool result = false;

  if(!(this->initialized))
    return result;
  if((request && !sdk_opened)||(!request && sdk_opened))
  {
    //coreAPI->activate(&user_act_data_);
    coreAPI->setControl(request);
  }

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

bool DjiHILParser::calibrateimubias()
{
  return false;
  //Not Implemented as of now
}

bool DjiHILParser::cmdrpythrust(geometry_msgs::Quaternion &, bool)
{
  return true;
}

void DjiHILParser::reset_attitude(double roll, double pitch, double yaw)
{
    //NOT IMPLEMENTED
    return;
}

bool DjiHILParser::cmdvel_yaw_rate_guided(geometry_msgs::Vector3 &, double &)
{
  return true;
}

bool DjiHILParser::cmdvel_yaw_angle_guided(geometry_msgs::Vector3 &, double &)
{
  return true;
}

bool DjiHILParser::cmdvelguided(geometry_msgs::Vector3 &, double &)
{
  return true;
}

bool DjiHILParser::cmdwaypoint(geometry_msgs::Vector3 &, double)
{
  return true;
}

void DjiHILParser::grip(int state)//TriState Gripper
{
    //NOT IMPLEMENTED
    return;
}

void DjiHILParser::getquaddata(parsernode::common::quaddata &d1)
{
  spin_mutex.lock();
  //Create Quad State String based on existing status:
  data.quadstate = "";
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

void DjiHILParser::setmode(std::string mode)
{
  //Dont need to implement as the commands can change them easily
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
  }
}

void DjiHILParser::init(std::string device, unsigned int baudrate) {
  printf("--- Connection Info ---\n");
  printf("Serial port: %s\n", device.c_str());
  printf("Baudrate: %u\n", baudrate);
  printf("-----\n");

  HardDriver_Manifold* m_hd = new HardDriver_Manifold(device, baudrate);
  m_hd->init();

  coreAPI = new CoreAPI( (HardDriver*)m_hd );
  //no log output while running hotpoint mission
  coreAPI->setHotPointData(false);

  flight = new Flight(coreAPI);

  int ret;
  ret = pthread_create(&m_recvTid, 0, APIRecvThread, (void *)coreAPI);
  if(0 != ret)
    ROS_FATAL("Cannot create new thread for readPoll!");
  else
    ROS_INFO("Succeed to create thread for readPoll");

  version_data = coreAPI->getDroneVersion(2);//Timeout in sec
  ROS_INFO("Hardware type: %s", version_data.hwVersion);
  // If true implies the hardware is either A3 etc
  if(strcmp(version_data.hwVersion, "M100") != 0) {
    shift_bit = 2;
    rc_f_pwm = 10000;
    hardware_type = A3;
  }
  else {
    shift_bit = 0;
    rc_f_pwm = 8000;
    hardware_type = MATRICE;
  }
}

int DjiHILParser::init_parameters_and_activate(ros::NodeHandle& nh_, DJI::onboardSDK::ActivateData* user_act_data,
  CallBack broadcast_function)
{
  std::string serial_name;
  int baud_rate;
  int app_id;
  int app_api_level;
  int app_version;
  std::string app_bundle_id;
  std::string enc_key;
  nh_.param("serial_name", serial_name, std::string("/dev/cu.usbserial-A603T4HK"));
  nh_.param("baud_rate", baud_rate, 230400);
  nh_.param("app_id", app_id, 1022384);
  nh_.param("app_api_level", app_api_level, 2);
  nh_.param("app_version", app_version, 1);
  nh_.param("app_bundle_id", app_bundle_id, std::string("12345678901234567890123456789012"));
  nh_.param("enc_key", enc_key,
      std::string("e7bad64696529559318bb35d0a8c6050d3b88e791e1808cfe8f7802150ee6f0d"));
  // Initialize serial port
  init(serial_name.c_str(), baud_rate);
   // activation
  user_act_data->ID = app_id;
  user_act_data->version = version_data.fwVersion;
  strcpy((char*) user_act_data->iosID, app_bundle_id.c_str());
  user_act_data->encKey = new char[65];//Create a char on heap
  strcpy(user_act_data->encKey, enc_key.c_str());
  printf("=================================================\n");
  printf("app id: %d\n", user_act_data->ID);
  printf("app version: 0x0%X\n", user_act_data->version);
  printf("app key: %s\n", user_act_data->encKey);
  printf("=================================================\n");

  coreAPI->activate(user_act_data);
  coreAPI->setBroadcastCallback(broadcast_function, (DJI::UserData)this);

  return 0;
}

void* DjiHILParser::APIRecvThread(void* param) {
  CoreAPI* p_coreAPI = (CoreAPI*)param;
  while(true) {
    p_coreAPI->readPoll();
    p_coreAPI->sendPoll();
    usleep(1000);
  }
}

void DjiHILParser::landingCb(DJI::onboardSDK::CoreAPI *, Header * header, void * userData)
{
  ((CbResponse*)userData)->succeeded = header->length - EXC_DATA_SIZE <= 2; // this check is how the flight API determines success, so I guess it is corrcet?
  ((CbResponse*)userData)->received = true;
}

void DjiHILParser::takeoffCb(DJI::onboardSDK::CoreAPI *, Header * header, void * userData)
{
  ((CbResponse*)userData)->succeeded = header->length - EXC_DATA_SIZE <= 2; // this check is how the flight API determines success, so I guess it is corrcet?
  ((CbResponse*)userData)->received = true;
}

void DjiHILParser::statReceiveDJIData(DJI::onboardSDK::CoreAPI *, Header *, void * userData)
{
  ((DjiHILParser*)userData)->receiveDJIData();
}

//Receive DJI Data:
void DjiHILParser::receiveDJIData()
{
  DJI::onboardSDK::BroadcastData bc_data = coreAPI->getBroadcastData();
  unsigned short msg_flags = bc_data.dataFlag;

  //static ros::Time starting_time = ros::Time::now();
  //static uint64_t starting_timestamp =

  spin_mutex.lock();
  data.timestamp = bc_data.timeStamp.time*(1.0/400.0);// + 1e-9*bc_data.timeStamp.nanoTime; //(bc_data.timeStamp.time*(1.0/600.0)); //TODO: ensure this scaling is correct for new sdk
  //update attitude msg
  if ((msg_flags & HAS_Q) && (msg_flags & HAS_W)) {
    tf::Quaternion qt(bc_data.q.q1, bc_data.q.q2, bc_data.q.q3, bc_data.q.q0);
    tf::Matrix3x3 mat;
    mat.setRotation(qt);
    mat.getEulerYPR(data.rpydata.z, data.rpydata.y, data.rpydata.x);
    data.rpydata.y = -data.rpydata.y;
    data.rpydata.z = -data.rpydata.z;
    data.omega.x = bc_data.w.x;
    data.omega.y = -bc_data.w.y;
    data.omega.z = -bc_data.w.z;
    if(enable_log)
    {
      imufile<<data.timestamp<<"\t"<<data.rpydata.x<<"\t"<<data.rpydata.y<<"\t"<<data.rpydata.z<<"\t"
        <<bc_data.w.x<<"\t"<<-bc_data.w.y<<"\t"<<-bc_data.w.z<<endl;
    }
  }

  //update velocity msg
  if ((msg_flags & HAS_V)) {
    data.linvel.x = bc_data.v.x;
    data.linvel.y = -bc_data.v.y;
    data.linvel.z = bc_data.v.z;
    if(enable_log)
      velfile<<data.timestamp<<"\t"<<data.linvel.x<<"\t"<<data.linvel.y<<"\t"<<data.linvel.z<<endl;
  }

  //update acceleration msg
  if ((msg_flags & HAS_A)) {
    data.linacc.x = bc_data.a.x;
    data.linacc.y = -bc_data.a.y;
    data.linacc.z = -bc_data.a.z;
    if(enable_log)
      accfile<<data.timestamp<<"\t"<<data.linacc.x<<"\t"<<data.linacc.y<<"\t"<<data.linacc.z<<endl;
  }

  //update rc_channel msg
  if ((msg_flags & (HAS_RC<<shift_bit))) {
    data.servo_in[0] = (int16_t)bc_data.rc.roll;
    data.servo_in[1] = (int16_t)bc_data.rc.pitch;
    data.servo_in[2] = (int16_t)bc_data.rc.throttle;
    data.servo_in[3] = (int16_t)bc_data.rc.yaw;
    if(enable_log)
      rcinputfile<<data.timestamp<<"\t"<<data.servo_in[0]<<"\t"<<data.servo_in[1]<<"\t"
        <<data.servo_in[2]<<"\t"<<data.servo_in[3]<<endl;
    // 8000 is obtained by testing with matrice and dji radio
    if(bc_data.rc.mode == rc_f_pwm) {
      data.rc_sdk_control_switch = true;
    }
    else {
      data.rc_sdk_control_switch = false;
    }
    /*rc_channels.mode = recv_sdk_std_msgs.rc.mode;
    rc_channels.gear = recv_sdk_std_msgs.rc.gear;
    rc_channels_publisher.publish(rc_channels);
    */
  }

  //update compass msg
  if (msg_flags & (HAS_MAG<<shift_bit)) {
    data.magdata.x = (double)bc_data.mag.x;
    data.magdata.y = (double)bc_data.mag.y;
    data.magdata.z = (double)bc_data.mag.z;
    if(enable_log)
      magfile<<data.timestamp<<"\t"<<data.magdata.x<<"\t"<<data.magdata.y<<"\t"<<data.magdata.z<<endl;
  }

   //Fix Bug with Data Status TODO

  //update flight_status
  if (msg_flags & (HAS_STATUS<<shift_bit)) {
    quad_status = bc_data.status;
    if(quad_status == IN_AIR)
      data.armed = true;
    else if(quad_status == FINISH_LANDING)
      data.armed = false;
  }

  //update battery msg
  if(hardware_type == MATRICE) {
    if (msg_flags & (HAS_BATTERY<<shift_bit)) {
      data.batterypercent = (double)bc_data.battery;
    }
  } else {
    data.batterypercent = 100; // A3 does not support battery voltage measurement wtf
  }

  //update flight control info
  if (msg_flags & (HAS_DEVICE<<shift_bit)) {
    //flight_control_info.serial_req_status = recv_sdk_std_msgs.ctrl_info.serial_req_status;
    //ctrl_mode = bc_data.ctrlInfo.flightStatus; //TODO: I don't think this is correct, but I'm not sure how to get conrol mode
    ctrl_mode = bc_data.ctrlInfo.mode;
    sdk_status = bc_data.ctrlInfo.flightStatus;
    if(enable_log)
    {
      statusfile<<data.timestamp<<"\t"<<int(quad_status)<<"\t"<<int(sdk_status)<<"\t"<<int(ctrl_mode)<<"\t"<<int(gps_health)<<"\t"<<int(vel_yaw_ratemode)<<"\t"<<int(rpyt_ratemode)<<endl;
    }
  }

  //update obtaincontrol msg
  //if ((msg_flags & HAS_TIME)) {
    //SDK Permission
    //sdk_status = bc_data.controlStatus; // whether control is obtained

    //update activation msg
  if(bc_data.activation == 0)
  {
    this->initialized = true;
    //      ROS_INFO("Initialized DJI");
  }
  else
  {
    this->initialized = false;
    if(bc_data.activation != 255)// default value
    {
      ROS_ERROR("Activation error: %d", bc_data.activation);
    }
  }
  //}

  if ((msg_flags & HAS_POS)) {
    //Initialize ref
    if(global_ref_lat == 0 && global_ref_long == 0)
    {
      global_ref_lat = bc_data.pos.latitude * 180.0 / C_PI;
      global_ref_long = bc_data.pos.longitude * 180.0 / C_PI;
      sensor_msgs::NavSatFix nav_fix_msg;
      nav_fix_msg.altitude = 0;
      nav_fix_msg.latitude = global_ref_lat;
      nav_fix_msg.longitude = global_ref_long;
      nav_fix_msg.header.stamp = ros::Time::now();
      global_ref_pub.publish(nav_fix_msg);
    }
    if((ros::Time::now()-last_gps_pub_time).toSec() >= 1./gps_pub_rate)
    {
      double glat = bc_data.pos.latitude * 180.0 / C_PI;
      double glong = bc_data.pos.longitude * 180.0 / C_PI;
      sensor_msgs::NavSatFix nav_fix_msg;
      nav_fix_msg.altitude = bc_data.pos.height;
      nav_fix_msg.latitude = glat;
      nav_fix_msg.longitude = glong;
      nav_fix_msg.header.stamp = last_gps_pub_time = ros::Time::now();
      gps_pub.publish(nav_fix_msg);
    }

    //update local_position msg
    DJI_SDK::gps_convert_ned(
        data.localpos.x,
        data.localpos.y,
        bc_data.pos.longitude * 180.0 / C_PI,
        bc_data.pos.latitude * 180.0 / C_PI,
        global_ref_long,
        global_ref_lat
        );
    data.localpos.y = - data.localpos.y;//NED to NWU format
    // Z position is not set. It is set by takeoff and land commands
    gps_health = (uint8_t)bc_data.pos.health;
    if(enable_log)
    {
      localposfile<<data.timestamp<<"\t"<<data.localpos.x<<"\t"<<data.localpos.y<<"\t"<<data.localpos.z
        <<endl;
    }
  }
  spin_mutex.unlock();
}

};
PLUGINLIB_DECLARE_CLASS(dji_parser, DjiHILParser, dji_parser::DjiHILParser, parsernode::Parser)
