#pragma once

#include "tf_parser/tf_model_sim.h"
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

namespace tf_parser {
class TFParser : public tf_parser::TFModelSim {
public:
    TFParser();
    /**
    * @brief Use for initializing
    */
    virtual void initialize();
private:
    void tfTimerCallback(const ros::TimerEvent&);

    tf::TransformBroadcaster tf_broadcaster_;
    ros::NodeHandle nh_;
    ros::Timer tf_timer_;
};
}
