#include <pluginlib/class_list_macros.h>
#include <dji_parser/dji_hil_parser.h>
#include <tf/tf.h>
#include <ros/ros.h>
#include <stdexcept>

using namespace DJI::OSDK;

namespace dji_parser{

DjiHILParser::DjiHILParser(): DjiParser() 
{
  this->initialized = false;
  data.batterypercent = 100;
}

//Extend Functions from Paser:

bool DjiHILParser::takeoff()//Virtual override function
{
    ROS_INFO("DJI HIL takeoff");
    spin_mutex.lock();
    data.localpos.z = 1.0;
    data.altitude = 1.0;
    data.armed = true;
    spin_mutex.unlock();
    return true;
}

bool DjiHILParser::land()
{
    spin_mutex.lock();
    data.localpos.z = 0.0;
    data.altitude = 0.0;
    data.armed = false;
    spin_mutex.unlock();
  return true;
}

bool DjiHILParser::disarm()
{
  flowControl(false);
}

bool DjiHILParser::flowControl(bool request)
{
  return true;
}

bool DjiHILParser::cmdrpyawratethrust(geometry_msgs::Quaternion &)
{
  return true;
}

bool DjiHILParser::cmdrpythrust(geometry_msgs::Quaternion &)
{
  return true;
}

bool DjiHILParser::cmdvel_yaw_rate_guided(geometry_msgs::Vector3 &, double &)
{
  return true;
}

bool DjiHILParser::cmdvel_yaw_angle_guided(geometry_msgs::Vector3 &, double &)
{
  return true;
}

bool DjiHILParser::cmdwaypoint(geometry_msgs::Vector3 &, double)
{
  return true;
}
//Receive DJI Data:
/*
void DjiHILParser::getquaddata(parsernode::common::quaddata &d1)
{
  //Record data
  spin_mutex.lock();
  bool armed = data.armed;
  double batterypercent = data.batterypercent;
  double z = data.localpos.z;
  cout << "pre-z: " << z << " armed: " << armed << " batt: " << batterypercent << std::endl;
  spin_mutex.unlock();
  DjiParser::getquaddata(d1);
  //Reset data
  spin_mutex.lock();
  d1.armed = armed;
  d1.batterypercent = batterypercent;
  d1.localpos.z = z;
  ROS_INFO("post-z: %d",d1.localpos.z);
  spin_mutex.unlock();
}*/
void DjiHILParser::get50HzData(Vehicle *vehicle, RecvContainer recvFrame)
{
  //Record data
  this->spin_mutex.lock();
  bool armed = this->data.armed;
  double batterypercent = this->data.batterypercent;
  double z = this->data.localpos.z;
  this->spin_mutex.unlock();
  //Run Parent Version
  DjiParser::get50HzData(vehicle, recvFrame);
  //Overwrite
  this->spin_mutex.lock();
  this->data.armed = armed;
  this->data.batterypercent = batterypercent;
  this->data.localpos.z = z;
  this->spin_mutex.unlock();
}

void DjiHILParser::get10HzData(Vehicle *vehicle, RecvContainer recvFrame)
{
  //Record data
  this->spin_mutex.lock();
  bool armed = this->data.armed;
  double batterypercent = this->data.batterypercent;
  double z = this->data.localpos.z;
  this->spin_mutex.unlock();
  //Run Parent Version
  DjiParser::get10HzData(vehicle, recvFrame);
  //Overwrite
  this->spin_mutex.lock();
  this->data.armed = armed;
  this->data.batterypercent = batterypercent;
  this->data.localpos.z = z;
  this->spin_mutex.unlock();
}

void DjiHILParser::get1HzData(Vehicle *vehicle, RecvContainer recvFrame)
{
  //Record data
  this->spin_mutex.lock();
  bool armed = this->data.armed;
  double batterypercent = this->data.batterypercent;
  double z = this->data.localpos.z;
  this->spin_mutex.unlock();
  //Run Parent Version
  DjiParser::get1HzData(vehicle, recvFrame);
  //Overwrite
  this->spin_mutex.lock();
  this->data.armed = armed;
  this->data.batterypercent = batterypercent;
  this->data.localpos.z = z;
  this->spin_mutex.unlock();
}

};
PLUGINLIB_EXPORT_CLASS(dji_parser::DjiHILParser, parsernode::Parser)
