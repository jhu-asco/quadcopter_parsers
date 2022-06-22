/* This is the plugin which is dependent on Parser base class. 
 */
#include <pluginlib/class_list_macros.h>
#include <dji_parser/dji_parser.h>
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


using namespace DJI::OSDK;

namespace dji_parser{

DjiParser::DjiParser(): nh_("~uav"), global_ref_lat(0), global_ref_long(0), global_ref_x(0), global_ref_y(0), global_ref_z(0)//, sdk_opened(false)
                        ,quad_status(0) ,ctrl_mode(0), sdk_status(0), gps_health(0),rc_f_pwm(-8000),
    R_FLU2FRD(tf::Matrix3x3(1,  0,  0, 0, -1,  0, 0,  0, -1)),
    R_ENU2NED(tf::Matrix3x3(0,  1,  0, 1,  0,  0, 0,  0, -1))
{
  this->initialized = false;
}

DjiParser::~DjiParser()
{
  disarm();
  if (vehicle) {
    cleanUpSubscribe();
    delete vehicle;
  }
}

//PluginLib Initialization function
void DjiParser::initialize()
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
  init_parameters_and_activate(nh_);
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
    if(!flowControl(false)) {
      ROS_INFO("Error FlowControl False");
    }
  }

  tf_timer_ = nh_.createTimer(ros::Duration(0.1), &DjiParser::tfTimerCallback, this);
}

void DjiParser::tfTimerCallback(const ros::TimerEvent&) 
{
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(data.localpos.x, data.localpos.y, data.localpos.z) );
  tf::Quaternion q;
  q.setRPY(data.rpydata.x, data.rpydata.y, data.rpydata.z);
  transform.setRotation(q);
  tf_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "quad"));
}

//Extend Functions from Paser:

bool DjiParser::takeoff()//Virtual override function
{
  if(!flowControl(true)) {
    ROS_ERROR("Could not get flow control");
    return false;
  }
  ACK::ErrorCode ack = vehicle->control->takeoff(WAIT_TIMEOUT);
  if (ACK::getError(ack)) {
    ROS_ERROR("Takeoff Failed");
    ACK::getErrorCodeMessage(ack, __func__);
    return false;
  }
  return true;
}

bool DjiParser::land()
{
  if(!flowControl(true)) {
    ROS_ERROR("Could not get flow control");
    return false;
  }
  ACK::ErrorCode ack = vehicle->control->land(WAIT_TIMEOUT);
  if (ACK::getError(ack)) {
    ROS_ERROR("Land Failed");
    ACK::getErrorCodeMessage(ack, __func__);
    return false;
  }
  return true;
}

bool DjiParser::disarm()
{
  ROS_ERROR("Disarm Start");
  ACK::ErrorCode ack = vehicle->control->disArmMotors(WAIT_TIMEOUT);
  if (ACK::getError(ack))
  {
    ROS_ERROR("Disarm Failed");
    ACK::getErrorCodeMessage(ack, __func__);
    return false;
  }
  ROS_ERROR("FlowControl False");
  if(!flowControl(false)) {
    ROS_ERROR("Could not release flow control");
    return false;
  }
  return true;
}

bool DjiParser::flowControl(bool request)
{

  bool result = false;

  if(!(this->initialized))
    return false;
  
  ROS_ERROR("Flow Control");
  if (request) {
    ROS_ERROR("Request: True");
  } else {
    ROS_ERROR("Request: False");
  }
  if (sdk_status == 1) {
    ROS_ERROR("sdk: True");
  } else {
    ROS_ERROR("sdk: False");
  }
  ACK::ErrorCode ack;
  bool called = false;
  if (request && !(sdk_status == 1)) {
    ROS_ERROR("OBTAIN");
    ack = vehicle->obtainCtrlAuthority(WAIT_TIMEOUT);
    called = true;
  }else if (!request && (sdk_status == 1)) {
    ROS_ERROR("RELEASE");
    ack = vehicle->releaseCtrlAuthority(WAIT_TIMEOUT);
    called = true;
  }
  if (called) 
  {
    if (ACK::getError(ack))
    {
      ROS_ERROR("Flow Control Failed");
      ACK::getErrorCodeMessage(ack, __func__);
      //sdk_opened = !request;
      return false;
    }

    ROS_ERROR("WAITING");
    ros::Time current_time = ros::Time::now();
    while(ros::ok())
    {
      ROS_ERROR("TOP");
      spin_mutex.lock();
      //sdk_opened = (sdk_status==1);
      if ((request == (sdk_status == 1))) {
        ROS_ERROR("SUCCESS");
        result = true;
        spin_mutex.unlock();
        break;
      }
      else if ((ros::Time::now() - current_time).toSec() > 5.0) {
        ROS_ERROR("Timeout");
        result = false;
        spin_mutex.unlock();
        break;
      }
      spin_mutex.unlock();
      ros::Rate(10).sleep();
    }
  } else {
    result = true;
  }
  ROS_ERROR("DONE");
  return result;
}

bool DjiParser::calibrateimubias()
{
  return false;
  //Not Implemented as of now
}

bool DjiParser::cmdrpyawratethrust(geometry_msgs::Quaternion &rpytmsg)
{
  uint8_t flag = Control::HorizontalLogic::HORIZONTAL_ANGLE |
                 Control::VerticalLogic::VERTICAL_THRUST |
                 Control::HorizontalCoordinate::HORIZONTAL_BODY |
                 Control::YawLogic::YAW_RATE |
                 Control::StableMode::STABLE_ENABLE;
  float roll,pitch,yaw,thrust;
  roll = rpytmsg.x;
  pitch = rpytmsg.y;
  yaw = rpytmsg.z;
  thrust = rpytmsg.w;
  return setFlightMovement(flag, roll, pitch, thrust, yaw);
}

bool DjiParser::cmdrpythrust(geometry_msgs::Quaternion &rpytmsg)
{
  uint8_t flag = Control::HorizontalLogic::HORIZONTAL_ANGLE |
                 Control::VerticalLogic::VERTICAL_THRUST |
                 Control::HorizontalCoordinate::HORIZONTAL_BODY |
                 Control::YawLogic::YAW_ANGLE |
                 Control::StableMode::STABLE_ENABLE;
  float roll,pitch,yaw,thrust;
  roll = rpytmsg.x;
  pitch = rpytmsg.y;
  yaw = rpytmsg.z;
  thrust = rpytmsg.w;
  return setFlightMovement(flag, roll, pitch, thrust, yaw);
}

bool DjiParser::setFlightMovement(uint8_t flag, float x_in, float y_in, float z_in, float yaw_in) {
  //TODO: Double check this works.
  if(this->initialized && (sdk_status == 1)) {
    uint8_t HORI  = (flag & 0xC0);
    uint8_t VERT  = (flag & 0x30);
    uint8_t YAW   = (flag & 0x08);
    uint8_t FRAME = (flag & 0x06);
    uint8_t HOLD  = (flag & 0x01);

    double xCmd, yCmd, zCmd, yawCmd;
    if (FRAME == Control::HORIZONTAL_GROUND)
    {
      // 1.1 Horizontal channels
      if ( (HORI == Control::HORIZONTAL_VELOCITY) || (HORI == Control::HORIZONTAL_POSITION) )
      {
        xCmd = y_in;
        yCmd = x_in;
      }
      else
      {
        //ROS_DEBUG("GROUND frame is specified, but angle and rate command is generated in body frame");
        xCmd = (180/M_PI)*(x_in);
        yCmd = (180/M_PI)*(-y_in);
      }

      // 1.2 Verticle Channel
      if ( (VERT == Control::VERTICAL_VELOCITY) || (VERT == Control::VERTICAL_POSITION) )
      {
        zCmd = z_in;
      }
      else
      {
        //ROS_WARN_THROTTLE(1.0, "GROUND frame is specified, but thrust command is generated in body frame");
        zCmd = z_in;
      }
    }
    else if(FRAME == Control::HORIZONTAL_BODY)
    {
      // 2.1 Horizontal channels
      if ( (HORI == Control::HORIZONTAL_VELOCITY) || (HORI == Control::HORIZONTAL_POSITION) )
      {
        // The X and Y Vel and Pos should be only based on rotation after Yaw,
        // whithout roll and pitch. Otherwise the behavior will be weird.

        // Transform from F-R to F-L
        xCmd = x_in;
        yCmd = -y_in;
      }
      else
      {
        xCmd = (180/M_PI)*(x_in);
        yCmd = (180/M_PI)*(-y_in);
      }
  
      // 2.2 Vertical channel
      if ( (VERT == Control::VERTICAL_VELOCITY) || (VERT == Control::VERTICAL_POSITION)  )
      {
        //ROS_WARN_THROTTLE(1.0, "BODY frame is specified, but hight and z-velocity is generated in ground frame");
        zCmd = z_in;
      }
      else
      {
        zCmd = z_in;
      }
    }
    // The behavior of yaw should be the same in either frame
    if ( YAW == Control::YAW_ANGLE )
    {
      tf::Matrix3x3 rotationSrc;
      rotationSrc.setRPY(0.0, 0.0, yaw_in);

      //The last term should be transpose, but since it's symmetric ...
      tf::Matrix3x3 rotationDes (R_ENU2NED * rotationSrc * R_FLU2FRD);
  
      double temp1, temp2;
      rotationDes.getRPY(temp1, temp2, yawCmd);
  
      yawCmd = (180/M_PI)*(yawCmd);
    }
    else if (YAW == Control::YAW_RATE)
    {
      yawCmd = (180/M_PI)*(-yaw_in);
    }

    /*user_ctrl_data.flag = flag;
    user_ctrl_data.x = rpytmsg.x*(180/M_PI);
    user_ctrl_data.y = -rpytmsg.y*(180/M_PI);
    user_ctrl_data.z = rpytmsg.w;
    user_ctrl_data.yaw = -rpytmsg.z*(180/M_PI);
    user_ctrl_data.x = (user_ctrl_data.x > 30.0)?30.0:(user_ctrl_data.x < -30.0)?-30.0:user_ctrl_data.x;
    user_ctrl_data.y = (user_ctrl_data.y > 30.0)?30.0:(user_ctrl_data.y < -30.0)?-30.0:user_ctrl_data.y;*/
    Control::CtrlData ctrlData(flag, xCmd, yCmd, zCmd, yawCmd);
    vehicle->control->flightCtrl(ctrlData);
    return true;
  }
  return false;
}

void DjiParser::reset_attitude(double roll, double pitch, double yaw)
{
    //NOT IMPLEMENTED
    return;
}

bool DjiParser::cmdvel_yaw_rate_guided(geometry_msgs::Vector3 &vel_cmd, double &yaw_rate)
{
  uint8_t flag =(Control::HORIZONTAL_VELOCITY |
                 Control::VERTICAL_VELOCITY |
                 Control::HORIZONTAL_GROUND |
                 Control::YAW_RATE |
                 Control::StableMode::STABLE_ENABLE);
  bool result = setFlightMovement(flag,vel_cmd.x,vel_cmd.y,vel_cmd.z,yaw_rate);
/*  user_ctrl_data.x = vel_cmd.x;
    user_ctrl_data.y = -vel_cmd.y;
    user_ctrl_data.z = vel_cmd.z;
    user_ctrl_data.yaw = -yaw_rate*(180/M_PI);*/
  data.velocity_goal = vel_cmd;
  data.velocity_goal_yaw = yaw_rate;
  return result;
}

bool DjiParser::cmdvel_yaw_angle_guided(geometry_msgs::Vector3 &vel_cmd, double &yaw_angle)
{
  uint8_t flag =(Control::HORIZONTAL_VELOCITY |
                 Control::VERTICAL_VELOCITY |
                 Control::HORIZONTAL_GROUND |
                 Control::YAW_ANGLE |
                 Control::StableMode::STABLE_ENABLE);
  bool result = setFlightMovement(flag,vel_cmd.x,vel_cmd.y,vel_cmd.z,yaw_angle);
/*  user_ctrl_data.x = vel_cmd.x;
    user_ctrl_data.y = -vel_cmd.y;
    user_ctrl_data.z = vel_cmd.z;
    user_ctrl_data.yaw = -yaw_angle*(180/M_PI);*/
  data.velocity_goal = vel_cmd;
  data.velocity_goal_yaw = yaw_angle;
  return result;
}

bool DjiParser::cmdwaypoint(geometry_msgs::Vector3 &desired_pos, double desired_yaw)
{
  uint8_t flag =(Control::HORIZONTAL_POSITION |
		 Control::VERTICAL_POSITION |
		 Control::YAW_ANGLE |
		 Control::HORIZONTAL_GROUND |
		 Control::StableMode::STABLE_ENABLE);

  //Unclear if this should be global or local.  
/*    user_ctrl_data.x = desired_pos.x - data.localpos.x;
    user_ctrl_data.y = -(desired_pos.y - data.localpos.y);
    user_ctrl_data.z = desired_pos.z;
    user_ctrl_data.yaw = -desired_yaw*(180/M_PI);*/
    //ROS_INFO("Offset: %f,%f,%f,%f",user_ctrl_data.roll_or_x, user_ctrl_data.pitch_or_y, user_ctrl_data.thr_z, user_ctrl_data.yaw);

  bool result = setFlightMovement(flag,desired_pos.x-data.localpos.x,desired_pos.y-data.localpos.y,desired_pos.z,desired_yaw);
  data.position_goal = desired_pos;
  data.position_goal_yaw = desired_yaw;
  return result;
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
  data.quadstate = "";
  switch(quad_status)
  {
/*    case STANDBY :
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
      break;*/
    case DJI::OSDK::VehicleStatus::FlightStatus::STOPED :
      data.quadstate = "STOPPED";
      break;
    case DJI::OSDK::VehicleStatus::FlightStatus::ON_GROUND :
      data.quadstate = "ON GROUND";
      break;
    case DJI::OSDK::VehicleStatus::FlightStatus::IN_AIR :
      data.quadstate = "IN AIR";
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
      data.quadstate += " SER2";
      break;
    case 4:
      data.quadstate += " SER4";
      break;
  }
  switch(sdk_status)
  {
    case 0:
      data.quadstate += " SCLOSE";
      //sdk_opened = false;
      break;
    case 1:
      data.quadstate += " SOPEN";
      //sdk_opened = true;
      break;
  }
  data.quadstate += (std::string(" G") + std::to_string(gps_health));
  d1 = data;//Copy data
  spin_mutex.unlock();
  return;
}

ACK::ErrorCode
DjiParser::activate(int l_app_id, std::string l_enc_key)
{
  usleep(1000000);
  Vehicle::ActivateData testActivateData;
  char app_key[65];
  testActivateData.encKey = app_key;
  strcpy(testActivateData.encKey, l_enc_key.c_str());
  testActivateData.ID = l_app_id;

  ROS_DEBUG("called vehicle->activate(&testActivateData, WAIT_TIMEOUT)");
  return vehicle->activate(&testActivateData, WAIT_TIMEOUT);
}

bool DjiParser::init(std::string device, unsigned int baudrate, int app_id, std::string enc_key) {
  printf("--- Connection Info ---\n");
  printf("Serial port: %s\n", device.c_str());
  printf("Baudrate: %u\n", baudrate);
  printf("-----\n");

  bool threadSupport = true;
  bool enable_advanced_sensing = false;

#ifdef ADVANCED_SENSING
  enable_advanced_sensing = true;
  ROS_INFO("Advanced Sensing is Enabled on M210.");
#endif

  vehicle = new Vehicle(device.c_str(), baudrate, threadSupport, enable_advanced_sensing);

  if (ACK::getError(this->activate(app_id,enc_key))) {
    ROS_ERROR("activation error...");
    return false;
  }

#ifdef ADVANCED_SENSING
  vehicle->advancedSensing->setAcmDevicePath(acm_device.c_str());
#endif

  return true;
}

bool DjiParser::initReceive() {
  if(vehicle->subscribe != NULL) {
    ROS_INFO("Use data subscription to get telemetry data!");
    //Subscribe to data
    ACK::ErrorCode ack = vehicle->subscribe->verify(WAIT_TIMEOUT);
    if (ACK::getError(ack))
    {
      ROS_ERROR("Failed to subscribe");
      ACK::getErrorCodeMessage(ack, __func__);
      return false;
    }

    std::vector<Telemetry::TopicName> topicList50Hz;
    // 50 Hz package from FC
    topicList50Hz.push_back(Telemetry::TOPIC_QUATERNION);
    topicList50Hz.push_back(Telemetry::TOPIC_ACCELERATION_GROUND);
    topicList50Hz.push_back(Telemetry::TOPIC_VELOCITY);
    topicList50Hz.push_back(Telemetry::TOPIC_ANGULAR_RATE_FUSIONED);
    topicList50Hz.push_back(Telemetry::TOPIC_CONTROL_DEVICE);
    int nTopic50Hz    = topicList50Hz.size();
    if (vehicle->subscribe->initPackageFromTopicList(DjiParser::PACKAGE_ID_50HZ, nTopic50Hz,
  						   topicList50Hz.data(), 1, 50))
    {
      ack = vehicle->subscribe->startPackage(DjiParser::PACKAGE_ID_50HZ, WAIT_TIMEOUT);
      if (ACK::getError(ack))
      {
        vehicle->subscribe->removePackage(DjiParser::PACKAGE_ID_50HZ, WAIT_TIMEOUT);
        ROS_ERROR("Failed to start 50Hz package");
        ACK::getErrorCodeMessage(ack, __func__);
        return false;
      }
      else
      {
        ROS_INFO("Start 50Hz package");
        vehicle->subscribe->registerUserPackageUnpackCallback(
        PACKAGE_ID_50HZ, static50HzData, (UserData) this);
      }
    }

    std::vector<Telemetry::TopicName> topicList10Hz;
    topicList10Hz.push_back(Telemetry::TOPIC_GPS_FUSED);
    topicList10Hz.push_back(Telemetry::TOPIC_HEIGHT_FUSION);
    topicList10Hz.push_back(Telemetry::TOPIC_POSITION_VO);
    topicList10Hz.push_back(Telemetry::TOPIC_RC/*_FULL_RAW_DATA*/);
    topicList10Hz.push_back(Telemetry::TOPIC_GIMBAL_ANGLES);
    topicList10Hz.push_back(Telemetry::TOPIC_GIMBAL_STATUS);
    topicList10Hz.push_back(Telemetry::TOPIC_GPS_SIGNAL_LEVEL);
    int nTopic10Hz    = topicList10Hz.size();
    if (vehicle->subscribe->initPackageFromTopicList(DjiParser::PACKAGE_ID_10HZ, nTopic10Hz,
                                                     topicList10Hz.data(), 1, 10))
    {
      ack = vehicle->subscribe->startPackage(DjiParser::PACKAGE_ID_10HZ, WAIT_TIMEOUT);
      if (ACK::getError(ack))
      {
        vehicle->subscribe->removePackage(DjiParser::PACKAGE_ID_10HZ, WAIT_TIMEOUT);
        ROS_ERROR("Failed to start 10Hz package");
        ACK::getErrorCodeMessage(ack, __func__);
        return false;
      }
      else
      {
        ROS_INFO("Start 10Hz package");
        vehicle->subscribe->registerUserPackageUnpackCallback(
                PACKAGE_ID_10HZ, static10HzData, this);
      }
    }

    std::vector<Telemetry::TopicName> topicList1Hz;
    topicList1Hz.push_back(Telemetry::TOPIC_STATUS_FLIGHT);
    topicList1Hz.push_back(Telemetry::TOPIC_BATTERY_INFO);
    int nTopic1Hz    = topicList1Hz.size();
    if (vehicle->subscribe->initPackageFromTopicList(DjiParser::PACKAGE_ID_1HZ, nTopic1Hz,
                                                     topicList1Hz.data(), 1, 1))
    {
      ack = vehicle->subscribe->startPackage(DjiParser::PACKAGE_ID_1HZ, WAIT_TIMEOUT);
      if (ACK::getError(ack))
      {
        vehicle->subscribe->removePackage(DjiParser::PACKAGE_ID_1HZ, WAIT_TIMEOUT);
        ROS_ERROR("Failed to start 1Hz package");
        ACK::getErrorCodeMessage(ack, __func__);
        return false;
      }
      else
      {
        ROS_INFO("Start 1Hz package");
        vehicle->subscribe->registerUserPackageUnpackCallback(
                PACKAGE_ID_1HZ, static1HzData, this);
      }
    }
    ros::Duration(1).sleep();
    return true;
  } else {
    //Set up broadcast
    ACK::ErrorCode broadcast_set_freq_ack;
    ROS_INFO("Use legacy data broadcast to get telemetry data!");

    uint8_t defaultFreq[16];
    setUpDefaultFreq(defaultFreq);
    broadcast_set_freq_ack =
      vehicle->broadcast->setBroadcastFreq(defaultFreq, WAIT_TIMEOUT);
    if (ACK::getError(broadcast_set_freq_ack))
    {
      ACK::getErrorCodeMessage(broadcast_set_freq_ack, __func__);
      return false;
    }
    // register a callback function whenever a broadcast data is in
    vehicle->broadcast->setUserBroadcastCallback(
      &DjiParser::staticParserBroadcastCallback, this);
    ros::Duration(1).sleep();
    return true;
  }
  return true;
}

void
DjiParser::setUpDefaultFreq(uint8_t freq[16])
{
  freq[0]  = DataBroadcast::FREQ_50HZ;//Timestamp
  freq[1]  = DataBroadcast::FREQ_50HZ;//Attitude
  freq[2]  = DataBroadcast::FREQ_50HZ;//Acc
  freq[3]  = DataBroadcast::FREQ_50HZ;//Vel
  freq[4]  = DataBroadcast::FREQ_50HZ;//Ang Vel
  freq[5]  = DataBroadcast::FREQ_10HZ;//GPS pos, alt, height
//  freq[6]  = DataBroadcast::FREQ_50HZ;//Magnetometer
  freq[7]  = DataBroadcast::FREQ_10HZ;//RC
  freq[8]  = DataBroadcast::FREQ_10HZ;//Gimbal Angles
  freq[9]  = DataBroadcast::FREQ_1HZ;//Status
  freq[10] = DataBroadcast::FREQ_1HZ;//Battery
  freq[11] = DataBroadcast::FREQ_50HZ;//Control Device
  //freq[12] = DataBroadcast::FREQ_10HZ;//Reserved
}

void DjiParser::ParserBroadcastCallback(Vehicle *vehicle, RecvContainer recvFrame)
{
  using namespace DJI::OSDK;

  uint16_t data_enable_flag = vehicle->broadcast->getPassFlag();
  if (!data_enable_flag) {
    return;
  }
  auto quat = vehicle->broadcast->getQuaternion();
  auto angrate = vehicle->broadcast->getAngularRate();
  auto vel = vehicle->broadcast->getVelocity();
  auto acc = vehicle->broadcast->getAcceleration();
  auto control = vehicle->broadcast->getSDKInfo();
  auto gps_pos = vehicle->broadcast->getGlobalPosition();
  auto gpshealth = gps_pos.health;
  auto rc = vehicle->broadcast->getRC();
//  auto gimbal = vehicle->broadcast->getGimbal();
  auto fstatus = vehicle->broadcast->getStatus().flight;
  auto batt = vehicle->broadcast->getBatteryInfo();
  

  //UPDATE QUAD DATA
  this->spin_mutex.lock();
  //quat
  tf::Quaternion qt(quat.q1, quat.q2, quat.q3, quat.q0);
  tf::Matrix3x3 mat;
  mat.setRotation(qt);
  mat.getEulerYPR(this->data.rpydata.z, this->data.rpydata.y, this->data.rpydata.x);
  //FLIP FOR DJI RD->LU switch.
  this->data.rpydata.y = -(this->data.rpydata.y);
  this->data.rpydata.z = -(this->data.rpydata.z);
  //angrate
  this->data.omega.x = angrate.x;
  this->data.omega.y = -angrate.y;
  this->data.omega.z = -angrate.z;
  //LOG
  if(this->enable_log)
  {
    this->imufile<<this->data.timestamp<<"\t"<<this->data.rpydata.x<<"\t"<<this->data.rpydata.y<<"\t"<<this->data.rpydata.z
	    <<"\t"<<this->data.omega.x<<"\t"<<-(this->data.omega.y)<<"\t"<<-(this->data.omega.z)<<endl;
  }
  //vel
  this->data.linvel.x = vel.x;
  this->data.linvel.y = -vel.y;
  this->data.linvel.z = vel.z;
  if(this->enable_log){
      this->velfile<<this->data.timestamp<<"\t"<<this->data.linvel.x<<"\t"<<this->data.linvel.y<<"\t"<<this->data.linvel.z
	      <<endl;
  }
  this->data.linacc.x = acc.x;
  this->data.linacc.y = -acc.y;
  this->data.linacc.z = acc.z;
  if(this->enable_log)
    this->accfile<<this->data.timestamp<<"\t"<<this->data.linacc.x<<"\t"<<this->data.linacc.y<<"\t"<<this->data.linacc.z
	    <<endl;
  //control
  this->ctrl_mode = control.deviceStatus;
  this->sdk_status = control.flightStatus;
  if(this->enable_log)
  {
    this->statusfile<<this->data.timestamp<<"\t"<<int(this->quad_status)<<"\t"<<int(this->sdk_status)<<"\t"<<int(this->ctrl_mode)<<endl;
  }
  //gps_pos
  //Initialize ref
  if(this->global_ref_lat == 0 && this->global_ref_long == 0)
  {
    this->global_ref_lat = gps_pos.latitude * 180.0 / M_PI;
    this->global_ref_long = gps_pos.longitude * 180.0 / M_PI;
    sensor_msgs::NavSatFix nav_fix_msg;
    nav_fix_msg.altitude = 0;
    nav_fix_msg.latitude = this->global_ref_lat;
    nav_fix_msg.longitude = this->global_ref_long;
    nav_fix_msg.header.stamp = ros::Time::now();
    this->global_ref_pub.publish(nav_fix_msg);
  }
  if((ros::Time::now()-this->last_gps_pub_time).toSec() >= 1./this->gps_pub_rate)
  {
    double glat = gps_pos.latitude * 180.0 / M_PI;
    double glong = gps_pos.longitude * 180.0 / M_PI;
    sensor_msgs::NavSatFix nav_fix_msg;
    nav_fix_msg.altitude = gps_pos.height;
    nav_fix_msg.latitude = glat;
    nav_fix_msg.longitude = glong;
    nav_fix_msg.header.stamp = this->last_gps_pub_time = ros::Time::now();
    this->gps_pub.publish(nav_fix_msg);
  }
  //update local_position msg
  DJI_SDK::gps_convert_ned(this->data.localpos.x,this->data.localpos.y,gps_pos.longitude * 180.0 / M_PI,
		           gps_pos.latitude * 180.0 / M_PI,this->global_ref_long,this->global_ref_lat);
  this->data.localpos.y = -(this->data.localpos.y);//NED to NWU format
  this->data.localpos.z = gps_pos.height;
  this->gps_health = gpshealth;
  if(this->enable_log)
  {
    this->localposfile<<this->data.timestamp<<"\t"<<this->data.localpos.x<<"\t"<<this->data.localpos.y<<"\t"
	    <<this->data.localpos.z<<endl;
  }
  //rc
  this->data.servo_in[0] = (int16_t)rc.roll;
  this->data.servo_in[1] = (int16_t)rc.pitch;
  this->data.servo_in[2] = (int16_t)rc.throttle;
  this->data.servo_in[3] = (int16_t)rc.yaw;
  if(this->enable_log)
    this->rcinputfile<<this->data.timestamp<<"\t"<<this->data.servo_in[0]<<"\t"<<this->data.servo_in[1]<<"\t"
      <<this->data.servo_in[2]<<"\t"<<this->data.servo_in[3]<<endl;
  // -8000 is obtained by testing with matrice and dji radio
  /*cout << "RC: mode: " << rc.mode << " lW: " << rc.leftWheel << " rW: " << rc.rightWheelButton 
       << " rc.rcC1: " << rc.rcC1 << " rc.rcC2: " << rc.rcC2 
       << " rc.rcD1: " << rc.rcD1 << " rc.rcD2: " << rc.rcD2
       << " rc.rcD3: " << rc.rcD1 << " rc.rcD4: " << rc.rcD2
       << " rc.rcD5: " << rc.rcD1 << " rc.rcD6: " << rc.rcD2
       << " rc.rcD7: " << rc.rcD1 << " rc.rcD8: " << rc.rcD2 << std::endl;*/
  if(rc.mode == this->rc_f_pwm) {
    this->data.rc_sdk_control_switch = true;
  }
  else {
    this->data.rc_sdk_control_switch = false;
  }
  this->quad_status = fstatus;
  if(this->quad_status == DJI::OSDK::VehicleStatus::FlightStatus::IN_AIR)
    this->data.armed = true;
  else if(this->quad_status == DJI::OSDK::VehicleStatus::FlightStatus::STOPED)//Yes, the API has a typo.
    this->data.armed = false;
  this->data.batterypercent = (double)batt.percentage;
  this->initialized = true;
  this->spin_mutex.unlock();
}

void DjiParser::cleanUpSubscribe() {
  if (vehicle->subscribe != NULL) {
    vehicle->subscribe->removePackage(DjiParser::PACKAGE_ID_1HZ,WAIT_TIMEOUT);
    vehicle->subscribe->removePackage(DjiParser::PACKAGE_ID_10HZ,WAIT_TIMEOUT);
    vehicle->subscribe->removePackage(DjiParser::PACKAGE_ID_50HZ,WAIT_TIMEOUT);
  }
}

int DjiParser::init_parameters_and_activate(ros::NodeHandle& nh_)
{
  std::string serial_name;
  int app_id;
  int app_version;
  std::string enc_key;
  std::string drone_version;
  bool user_select_broadcast;
  int baud_rate;
  int app_api_level;
  std::string app_bundle_id;
  nh_.param("serial_name",   serial_name, std::string("/dev/ttym200"));
  nh_.param("app_id",        app_id, 1113137);
  nh_.param("app_version",   app_version, 1);
  nh_.param("enc_key",       enc_key, std::string("58660c19755d9007702c9e94520ff072fba7978b3ea422375db21f1e7371790b"));
  nh_.param("drone_version", drone_version, std::string("M210")); // choose M100 as default
  nh_.param("use_broadcast", user_select_broadcast, false);
  nh_.param("baud_rate", baud_rate, 230400);
  nh_.param("app_api_level", app_api_level, 2);
  nh_.param("app_version", app_version, 1);
  ROS_INFO("Serial name: %s",serial_name.c_str());
  // Initialize vehicle and activate
  if (!init(serial_name.c_str(), baud_rate,app_id,enc_key)) {
    ROS_ERROR("Vehicle Init Failed");
    return -1;
  }
  if (!initReceive()) {
    ROS_ERROR("Receive Setup Failed");
  }

  return 0;
}

void DjiParser::static50HzData(Vehicle *vehicle, RecvContainer recvFrame, DJI::OSDK::UserData userData)
{
  DjiParser *p = (DjiParser *) userData;
  p->get50HzData(vehicle, recvFrame);
}

void DjiParser::static10HzData(Vehicle *vehicle, RecvContainer recvFrame, DJI::OSDK::UserData userData)
{
  DjiParser *p = (DjiParser *) userData;
  p->get10HzData(vehicle, recvFrame);
}

void DjiParser::static1HzData(Vehicle *vehicle, RecvContainer recvFrame, DJI::OSDK::UserData userData)
{
  DjiParser *p = (DjiParser *) userData;
  p->get1HzData(vehicle, recvFrame);
}

void DjiParser::staticParserBroadcastCallback(Vehicle *vehicle, RecvContainer recvFrame, DJI::OSDK::UserData userData)
{
  DjiParser *p = (DjiParser *) userData;
  p->ParserBroadcastCallback(vehicle, recvFrame);
}

void DjiParser::get50HzData(Vehicle *vehicle, RecvContainer recvFrame)
{

  uint8_t* data = recvFrame.recvData.raw_ack_array;
  ROS_ASSERT(DjiParser::PACKAGE_ID_50HZ == *data );
//  data++;
//  Telemetry::TimeStamp packageTimeStamp = * (reinterpret_cast<Telemetry::TimeStamp *>(data));

//  ros::Time msg_time = ros::Time::now();

  //READ DATA
  Telemetry::TypeMap<Telemetry::TOPIC_QUATERNION>::type quat =
          vehicle->subscribe->getValue<Telemetry::TOPIC_QUATERNION>();
  Telemetry::TypeMap<Telemetry::TOPIC_ACCELERATION_GROUND>::type acc =
          vehicle->subscribe->getValue<Telemetry::TOPIC_ACCELERATION_GROUND>();
  Telemetry::TypeMap<Telemetry::TOPIC_VELOCITY>::type vel =
          vehicle->subscribe->getValue<Telemetry::TOPIC_VELOCITY>();
  Telemetry::TypeMap<Telemetry::TOPIC_ANGULAR_RATE_FUSIONED>::type angrate =
          vehicle->subscribe->getValue<Telemetry::TOPIC_ANGULAR_RATE_FUSIONED>();
  Telemetry::TypeMap<Telemetry::TOPIC_CONTROL_DEVICE>::type control =
          vehicle->subscribe->getValue<Telemetry::TOPIC_CONTROL_DEVICE>();
  //UPDATE QUAD DATA
  this->spin_mutex.lock();
  //quat
  tf::Quaternion qt(quat.q1, quat.q2, quat.q3, quat.q0);
  tf::Matrix3x3 mat;
  mat.setRotation(qt);
  mat.getEulerYPR(this->data.rpydata.z, this->data.rpydata.y, this->data.rpydata.x);
  //FLIP FOR DJI RD->LU switch.
  this->data.rpydata.y = -(this->data.rpydata.y);
  this->data.rpydata.z = -(this->data.rpydata.z);
  //angrate
  this->data.omega.x = angrate.x;
  this->data.omega.y = -angrate.y;
  this->data.omega.z = -angrate.z;
  //LOG
  if(this->enable_log)
  {
    this->imufile<<this->data.timestamp<<"\t"<<this->data.rpydata.x<<"\t"<<this->data.rpydata.y<<"\t"<<this->data.rpydata.z
	    <<"\t"<<this->data.omega.x<<"\t"<<-(this->data.omega.y)<<"\t"<<-(this->data.omega.z)<<endl;
  }
  //vel
  this->data.linvel.x = vel.data.x;
  this->data.linvel.y = -vel.data.y;
  this->data.linvel.z = vel.data.z;
  if(this->enable_log){
      this->velfile<<this->data.timestamp<<"\t"<<this->data.linvel.x<<"\t"<<this->data.linvel.y<<"\t"<<this->data.linvel.z
	      <<endl;
  }
  this->data.linacc.x = acc.x;
  this->data.linacc.y = -acc.y;
  this->data.linacc.z = acc.z;
  if(this->enable_log)
    this->accfile<<this->data.timestamp<<"\t"<<this->data.linacc.x<<"\t"<<this->data.linacc.y<<"\t"<<this->data.linacc.z
	    <<endl;
  //control
  this->ctrl_mode = control.deviceStatus;
  this->sdk_status = control.flightStatus;
  //cout << "CTRL MODE: " << (int) this->ctrl_mode << std::endl;
  //cout << "SDK STATUS: " << (int) this->sdk_status << std::endl;
  if(this->enable_log)
  {
    this->statusfile<<this->data.timestamp<<"\t"<<int(this->quad_status)<<"\t"<<int(this->sdk_status)<<"\t"<<int(this->ctrl_mode)<<endl;
  }
  this->spin_mutex.unlock();
}

void DjiParser::get10HzData(Vehicle *vehicle, RecvContainer recvFrame)
{

  uint8_t* data = recvFrame.recvData.raw_ack_array;
  ROS_ASSERT(DjiParser::PACKAGE_ID_10HZ == *data );
//  data++;
//  Telemetry::TimeStamp packageTimeStamp = * (reinterpret_cast<Telemetry::TimeStamp *>(data));

//  ros::Time msg_time = ros::Time::now();

  //READ DATA
  Telemetry::TypeMap<Telemetry::TOPIC_GPS_FUSED>::type gps_pos =
          vehicle->subscribe->getValue<Telemetry::TOPIC_GPS_FUSED>();
  Telemetry::TypeMap<Telemetry::TOPIC_HEIGHT_FUSION>::type height =
          vehicle->subscribe->getValue<Telemetry::TOPIC_HEIGHT_FUSION>();
  Telemetry::TypeMap<Telemetry::TOPIC_RC/*_FULL_RAW_DATA*/>::type rc =
          vehicle->subscribe->getValue<Telemetry::TOPIC_RC/*_FULL_RAW_DATA*/>();
  Telemetry::TypeMap<Telemetry::TOPIC_GPS_SIGNAL_LEVEL>::type gpshealth =
          vehicle->subscribe->getValue<Telemetry::TOPIC_GPS_SIGNAL_LEVEL>();
//  Telemetry::TypeMap<Telemetry::TOPIC_GIMBAL_ANGLES>::type gimbal_ang =
//          vehicle->subscribe->getValue<Telemetry::TOPIC_GIMBAL_ANGLES>();
//  Telemetry::TypeMap<Telemetry::TOPIC_GIMBAL_STATUS>::type gimbal_status =
//          vehicle->subscribe->getValue<Telemetry::TOPIC_GIMBAL_STATUS>();
  //UPDATE QUAD DATA
  this->spin_mutex.lock();
  //gps_pos
  //Initialize ref
  if(this->global_ref_lat == 0 && this->global_ref_long == 0)
  {
    this->global_ref_lat = gps_pos.latitude * 180.0 / M_PI;
    this->global_ref_long = gps_pos.longitude * 180.0 / M_PI;
    sensor_msgs::NavSatFix nav_fix_msg;
    nav_fix_msg.altitude = 0;
    nav_fix_msg.latitude = this->global_ref_lat;
    nav_fix_msg.longitude = this->global_ref_long;
    nav_fix_msg.header.stamp = ros::Time::now();
    this->global_ref_pub.publish(nav_fix_msg);
  }
  if((ros::Time::now()-this->last_gps_pub_time).toSec() >= 1./this->gps_pub_rate)
  {
    double glat = gps_pos.latitude * 180.0 / M_PI;
    double glong = gps_pos.longitude * 180.0 / M_PI;
    sensor_msgs::NavSatFix nav_fix_msg;
    nav_fix_msg.altitude = height;
    nav_fix_msg.latitude = glat;
    nav_fix_msg.longitude = glong;
    nav_fix_msg.header.stamp = this->last_gps_pub_time = ros::Time::now();
    this->gps_pub.publish(nav_fix_msg);
  }
  //update local_position msg
  DJI_SDK::gps_convert_ned(this->data.localpos.x,this->data.localpos.y,gps_pos.longitude * 180.0 / M_PI,
		           gps_pos.latitude * 180.0 / M_PI,this->global_ref_long,this->global_ref_lat);
  this->data.localpos.y = -(this->data.localpos.y);//NED to NWU format
//  cout << "local_pos: long: " << gps_pos.longitude << " lat: " << gps_pos.latitude << std::endl 
//       << " ref_long: " << this->global_ref_long << " ref_lat: " << this->global_ref_lat << std::endl
//       << " x: " << this->data.localpos.x << " y: " << this->data.localpos.y << std::endl;
  this->data.localpos.z = height;
  this->gps_health = gpshealth;
  if(this->enable_log)
  {
    this->localposfile<<this->data.timestamp<<"\t"<<this->data.localpos.x<<"\t"<<this->data.localpos.y<<"\t"
	    <<this->data.localpos.z<<endl;
  }
  //rc
  this->data.servo_in[0] = (int16_t)rc.roll;
  this->data.servo_in[1] = (int16_t)rc.pitch;
  this->data.servo_in[2] = (int16_t)rc.throttle;
  this->data.servo_in[3] = (int16_t)rc.yaw;
  if(this->enable_log)
    this->rcinputfile<<this->data.timestamp<<"\t"<<this->data.servo_in[0]<<"\t"<<this->data.servo_in[1]<<"\t"
      <<this->data.servo_in[2]<<"\t"<<this->data.servo_in[3]<<endl;
  // -8000 is obtained by testing with matrice and dji radio
  if(rc.mode == this->rc_f_pwm) {
    this->data.rc_sdk_control_switch = true;
  }
  else {
    this->data.rc_sdk_control_switch = false;
  }
  this->spin_mutex.unlock();
}

void DjiParser::get1HzData(Vehicle *vehicle, RecvContainer recvFrame)
{
  uint8_t* data = recvFrame.recvData.raw_ack_array;
  ROS_ASSERT(DjiParser::PACKAGE_ID_1HZ == *data );
//  data++;
//  Telemetry::TimeStamp packageTimeStamp = * (reinterpret_cast<Telemetry::TimeStamp *>(data));

//  ros::Time msg_time = ros::Time::now();

  //READ DATA
  Telemetry::TypeMap<Telemetry::TOPIC_STATUS_FLIGHT>::type fstatus =
          vehicle->subscribe->getValue<Telemetry::TOPIC_STATUS_FLIGHT>();
  Telemetry::TypeMap<Telemetry::TOPIC_BATTERY_INFO>::type batt =
          vehicle->subscribe->getValue<Telemetry::TOPIC_BATTERY_INFO>();
  //UPDATE QUAD DATA
  this->spin_mutex.lock();
  this->quad_status = fstatus;
  if(this->quad_status == DJI::OSDK::VehicleStatus::FlightStatus::IN_AIR)
    this->data.armed = true;
  else if(this->quad_status == DJI::OSDK::VehicleStatus::FlightStatus::STOPED)//Yes, the API has a typo.
    this->data.armed = false;
  this->data.batterypercent = (double)batt.percentage;
  this->initialized = true;
  this->spin_mutex.unlock();
}

};
PLUGINLIB_EXPORT_CLASS(dji_parser::DjiParser, parsernode::Parser)
