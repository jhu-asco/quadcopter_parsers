#include "tf_parser/tf_parser.h"

#include <tf/tf.h>
#include <pluginlib/class_list_macros.h>

namespace tf_parser {
TFParser::TFParser()
    : tf_parser::TFModelSim(),
      nh_("~uav") {
}

void TFParser::initialize() {
  tf_timer_ = nh_.createTimer(ros::Duration(0.1), &TFParser::tfTimerCallback, this);

  tf_parser::TFModelSim::initialize();
}

void TFParser::tfTimerCallback(const ros::TimerEvent&)
{
  tf::Transform transform;
  tf::Quaternion q;
  transform.setOrigin( tf::Vector3(state_.p[0], state_.p[1], state_.p[2]) );
  q.setRPY(state_.rpy(0), state_.rpy(1), state_.rpy(2));
  transform.setRotation(q);
  tf_broadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "quad_sim"));
}
};

PLUGINLIB_DECLARE_CLASS(tf_parser, TFParser, tf_parser::TFParser, parsernode::Parser)
