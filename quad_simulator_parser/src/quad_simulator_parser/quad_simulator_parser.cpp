/* This is the plugin which is dependent on Parser base class. 
 */
#include <pluginlib/class_list_macros.h>
#include <quad_simulator_parser/quad_simulator_parser.h>
#include <tf/tf.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>

namespace quad_simulator_parser{

QuadSimParser::QuadSimParser():quad_simulator::QuadSimulator(), nh_("~uav")
{
}

//PluginLib Initialization function
void QuadSimParser::initialize()
{
    //Add Subscriber to a joystick for rcinput
    joy_sub_ = nh_.subscribe("joy",5,&QuadSimParser::setRCInputs,(quad_simulator::QuadSimulator*)this);

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

    nh_.param<double>("/reconfig/delay_send_time", delay_send_time_,0.2);
    kt_decrease_timer_ = nh_.createTimer(ros::Duration(5.0), &QuadSimParser::ktTimerCallback, this);

    tf_timer_ = nh_.createTimer(ros::Duration(0.1), &QuadSimParser::tfTimerCallback, this);

    quad_simulator::QuadSimulator::initialize();
}

void QuadSimParser::tfTimerCallback(const ros::TimerEvent&) 
{
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(state_.p[0], state_.p[1], state_.p[2]) );
  Eigen::Vector3d rpy;
  so3.g2q(rpy, state_.R);
  tf::Quaternion q;
  q.setRPY(rpy(0), rpy(1), rpy(2));
  transform.setRotation(q);
  tf_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "quad_sim"));
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
  ned_convert_gps(state_.p[0], -state_.p[1], nav_fix_msg.longitude, nav_fix_msg.latitude);
  nav_fix_msg.header.stamp = ros::Time::now();
  gps_pub.publish(nav_fix_msg);
}

};
PLUGINLIB_EXPORT_CLASS(quad_simulator_parser::QuadSimParser, parsernode::Parser)
