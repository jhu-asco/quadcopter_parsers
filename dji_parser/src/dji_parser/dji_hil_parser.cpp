#include <pluginlib/class_list_macros.h>
#include <dji_parser/dji_hil_parser.h>
#include <tf/tf.h>
#include <ros/ros.h>
#include <stdexcept>

using namespace DJI::onboardSDK;

namespace dji_parser{

DjiHILParser::DjiHILParser(): DjiParser() 
{
  this->initialized = false;
}

//Extend Functions from Paser:

bool DjiHILParser::takeoff()//Virtual override function
{
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

bool DjiHILParser::cmdrpythrust(geometry_msgs::Quaternion &, bool)
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

bool DjiHILParser::cmdvelguided(geometry_msgs::Vector3 &, double &)
{
  return true;
}

bool DjiHILParser::cmdwaypoint(geometry_msgs::Vector3 &, double)
{
  return true;
}
//Receive DJI Data:
void DjiHILParser::receiveDJIData()
{
  //Record data
  spin_mutex.lock();
  bool armed = data.armed;
  bool batterypercent = data.batterypercent;
  double z = data.localpos.z;
  spin_mutex.unlock();
  DjiParser::receiveDJIData();
  //Reset data
  spin_mutex.lock();
  data.armed = armed;
  data.batterypercent = batterypercent;
  data.localpos.z = z;
  spin_mutex.unlock();
}
};
PLUGINLIB_DECLARE_CLASS(dji_parser, DjiHILParser, dji_parser::DjiHILParser, parsernode::Parser)
