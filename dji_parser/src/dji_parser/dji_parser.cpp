/* This is the plugin which is dependent on Parser base class. 
 */
#include <pluginlib/class_list_macros.h>
#include <dji_parser/dji_parser.h>
#include <tf/tf.h>
#include <ros/ros.h>

using namespace DJI::onboardSDK;

namespace dji_parser{

DjiParser::DjiParser(): global_ref_lat(0), global_ref_long(0), sdk_opened(false)
                        ,quad_status(0) ,ctrl_mode(0), sdk_status(0), gps_health(0), rpyt_ratemode(true)
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
  //Initialize ros publishers:
  global_ref_pub = nh_.advertise<sensor_msgs::NavSatFix>("gps/fix",10, true);//Latched gps fix publisher

  //Initialize DJI:
  init_parameters_and_activate(nh_, &user_act_data_, DjiParser::statReceiveDJIData);
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
        CbResponse cb_response;
        flight->task(DJI::onboardSDK::Flight::TASK::TASK_TAKEOFF, DjiParser::takeoffCb,
          (DJI::UserData) &cb_response);
        // TODO: may need to do some locking here? Not sure...
        while(!cb_response.received)
          ros::Duration(.02).sleep();          
        ROS_INFO("Takeoff Res: %d", cb_response.succeeded);
        takeoff_result = cb_response.succeeded;
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
      CbResponse cb_response;
      flight->task(DJI::onboardSDK::Flight::TASK::TASK_LANDING, DjiParser::landingCb,
          (DJI::UserData) &cb_response);
      while(!cb_response.received)
        ros::Duration(.02).sleep();          
      ROS_INFO("Landing Res: %d", cb_response.succeeded);
      land_result = cb_response.succeeded;
    }
  }
  return land_result;
}

bool DjiParser::disarm()
{
  //TODO: There is a new arm and disarm protocol in 3.1
  flowControl(false);
}

bool DjiParser::flowControl(bool request)
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
      FlightData user_ctrl_data;
      user_ctrl_data.flag = Flight::HorizontalLogic::HORIZONTAL_ANGLE | 
                            Flight::VerticalLogic::VERTICAL_THRUST | 
                            Flight::HorizontalCoordinate::HORIZONTAL_BODY;
      if(sendyaw)
      {
        if(rpyt_ratemode)
        {
          user_ctrl_data.flag = user_ctrl_data.flag | 
                                Flight::YawLogic::YAW_PALSTANCE | 
                                Flight::SmoothMode::SMOOTH_ENABLE; // TODO: double check what smoothmode is (I guess smooth mode smoothly changes the control from current value to the new value; Whereas non-smooth abruptly changes the value thereby making it go to the desired command as fast as possible)
        }
        else
        {
          user_ctrl_data.flag = user_ctrl_data.flag | 
                                Flight::YawLogic::YAW_ANGLE | 
                                Flight::SmoothMode::SMOOTH_ENABLE;
        }
        // TODO: double check these are all valid in new sdk
        user_ctrl_data.x = rpytmsg.x*(180/M_PI);
        user_ctrl_data.y = -rpytmsg.y*(180/M_PI);
        user_ctrl_data.z = rpytmsg.w;
        user_ctrl_data.yaw = -rpytmsg.z*(180/M_PI);
        flight->setFlight(&user_ctrl_data);
      }
      else
      {
        user_ctrl_data.flag = user_ctrl_data.flag | 
                              Flight::YawLogic::YAW_PALSTANCE | 
                              Flight::SmoothMode::SMOOTH_ENABLE;
                                                           //   TODO: In v3.1 there is something called 
                                                           //   SmoothMode which is not explained
        user_ctrl_data.x = rpytmsg.x*(180/M_PI);
        user_ctrl_data.y = -rpytmsg.y*(180/M_PI);
        user_ctrl_data.z = rpytmsg.w;
        user_ctrl_data.yaw = 0;
        flight->setFlight(&user_ctrl_data);
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
      FlightData user_ctrl_data;
      user_ctrl_data.flag = Flight::HorizontalLogic::HORIZONTAL_VELOCITY | 
                            Flight::VerticalLogic::VERTICAL_VELOCITY |
                            Flight::HorizontalCoordinate::HORIZONTAL_GROUND |
                            Flight::YawLogic::YAW_ANGLE |
                            Flight::SmoothMode::SMOOTH_ENABLE;
      user_ctrl_data.x = vel_cmd.x;
      user_ctrl_data.y = -vel_cmd.y;
      user_ctrl_data.z = vel_cmd.z;
      user_ctrl_data.yaw = -yaw_ang*(180/M_PI);
      flight->setFlight(&user_ctrl_data);
    }
  }
  return true;
}

bool DjiParser::cmdwaypoint(geometry_msgs::Vector3 &desired_pos, double desired_yaw)
{
  if(sdk_opened)
  {
    //Convert position from NWU frame to NED frame
    FlightData user_ctrl_data;
    user_ctrl_data.flag = Flight::HorizontalLogic::HORIZONTAL_POSITION | // TODO: Ensure this is relative pos command
                          Flight::VerticalLogic::VERTICAL_POSITION |
                          Flight::HorizontalCoordinate::HORIZONTAL_GROUND |
                          Flight::YawLogic::YAW_ANGLE |
                          Flight::SmoothMode::SMOOTH_ENABLE;
    spin_mutex.lock();
    user_ctrl_data.x = desired_pos.x - data.localpos.x;
    user_ctrl_data.y = -(desired_pos.y - data.localpos.y);
    user_ctrl_data.z = desired_pos.z;
    spin_mutex.unlock();
    user_ctrl_data.yaw = -desired_yaw*(180/M_PI);
    //ROS_INFO("Offset: %f,%f,%f,%f",user_ctrl_data.roll_or_x, user_ctrl_data.pitch_or_y, user_ctrl_data.thr_z, user_ctrl_data.yaw);
    flight->setFlight(&user_ctrl_data);
  }
  else
  {
    ROS_WARN("SDK Not Open");
  }
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
  if(strcmp(mode.c_str(),"rpyt_rate")==0)
  {
    rpyt_ratemode = true;
  }
  else if(strcmp(mode.c_str(),"rpyt_angle")==0)
  {
    rpyt_ratemode = false;
  }
}

void DjiParser::init(std::string device, unsigned int baudrate) {
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

  coreAPI->getSDKVersion();
}

int DjiParser::init_parameters_and_activate(ros::NodeHandle& nh_, ActivateData* user_act_data,
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
   // activation
  user_act_data->ID = app_id;
  user_act_data->version = SDK_VERSION;
  //user_act_data->version = 0x03010a00
  strcpy((char*) user_act_data->iosID, app_bundle_id.c_str());
  user_act_data->encKey = new char[65];//Create a char on heap
  strcpy(user_act_data->encKey, enc_key.c_str());
  printf("=================================================\n");
  printf("app id: %d\n", user_act_data->ID);
  printf("app version: 0x0%X\n", user_act_data->version);
  printf("app key: %s\n", user_act_data->encKey);
  printf("=================================================\n");

  
  init(serial_name.c_str(), baud_rate);

  coreAPI->activate(user_act_data, NULL);
  coreAPI->setBroadcastCallback(broadcast_function, (DJI::UserData)this);

  return 0;
}

void* DjiParser::APIRecvThread(void* param) {
  CoreAPI* p_coreAPI = (CoreAPI*)param;
  while(true) {
    p_coreAPI->readPoll();
    p_coreAPI->sendPoll();
    usleep(1000);
  }
}

void DjiParser::landingCb(DJI::onboardSDK::CoreAPI *, Header * header, void * userData)
{
  ((CbResponse*)userData)->succeeded = header->length - EXC_DATA_SIZE <= 2; // this check is how the flight API determines success, so I guess it is corrcet?
  ((CbResponse*)userData)->received = true;
}

void DjiParser::takeoffCb(DJI::onboardSDK::CoreAPI *, Header * header, void * userData)
{
  ((CbResponse*)userData)->succeeded = header->length - EXC_DATA_SIZE <= 2; // this check is how the flight API determines success, so I guess it is corrcet?
  ((CbResponse*)userData)->received = true;
}

void DjiParser::statReceiveDJIData(DJI::onboardSDK::CoreAPI *, Header *, void * userData)
{
  ((DjiParser*)userData)->receiveDJIData();
}

//Receive DJI Data:
void DjiParser::receiveDJIData()
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
    data.linvel.z = -bc_data.v.z;
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
  if ((msg_flags & HAS_RC)) {
    data.servo_in[0] = (int16_t)bc_data.rc.roll;
    data.servo_in[1] = (int16_t)bc_data.rc.pitch;
    data.servo_in[2] = (int16_t)bc_data.rc.throttle;
    data.servo_in[3] = (int16_t)bc_data.rc.yaw;
    if(enable_log)
      rcinputfile<<data.timestamp<<"\t"<<data.servo_in[0]<<"\t"<<data.servo_in[1]<<"\t"
        <<data.servo_in[2]<<"\t"<<data.servo_in[3]<<endl;
    /*rc_channels.mode = recv_sdk_std_msgs.rc.mode;
    rc_channels.gear = recv_sdk_std_msgs.rc.gear;
    rc_channels_publisher.publish(rc_channels);
    */
  }

  //update compass msg
  if ((msg_flags & HAS_MAG)) {
    data.magdata.x = (double)bc_data.mag.x;
    data.magdata.y = (double)bc_data.mag.y;
    data.magdata.z = (double)bc_data.mag.z;
    if(enable_log)
      magfile<<data.timestamp<<"\t"<<data.magdata.x<<"\t"<<data.magdata.y<<"\t"<<data.magdata.z<<endl;
  }

   //Fix Bug with Data Status TODO

  //update flight_status 
  if ((msg_flags & HAS_STATUS)) {
    quad_status = bc_data.status;
    if(quad_status == IN_AIR)
      data.armed = true;
    else if(quad_status == FINISH_LANDING)
      data.armed = false;
  }

  //update battery msg
  if ((msg_flags & HAS_BATTERY)) {
    data.batterypercent = (double)bc_data.battery;
  }

  //update flight control info
  if ((msg_flags & HAS_DEVICE)) {
    //flight_control_info.serial_req_status = recv_sdk_std_msgs.ctrl_info.serial_req_status;
    //ctrl_mode = bc_data.ctrlInfo.flightStatus; //TODO: I don't think this is correct, but I'm not sure how to get conrol mode
    ctrl_mode = bc_data.ctrlInfo.mode;
    sdk_status = bc_data.ctrlInfo.flightStatus;
  }

  //update obtaincontrol msg
  //if ((msg_flags & HAS_TIME)) {
    //SDK Permission
    //sdk_status = bc_data.controlStatus; // whether control is obtained

    //update activation msg
  if(bc_data.activation)
  {
    this->initialized = true;
    //      ROS_INFO("Initialized DJI");
  }
  else
  {
    this->initialized = false;
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
    data.localpos.z = bc_data.pos.height;
    //Altitude is not used: recv_sdk_std_msgs.pos.alti
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
PLUGINLIB_DECLARE_CLASS(dji_parser, DjiParser, dji_parser::DjiParser, parsernode::Parser)
