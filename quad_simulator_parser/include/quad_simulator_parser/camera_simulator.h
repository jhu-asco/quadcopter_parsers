#ifndef _CAMERA_SIMULATOR_H_
#define _CAMERA_SIMULATOR_H_

#include <ros/ros.h>
#include <Eigen/Dense>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <object_renderer/virtual_image_handler.h>
#include <object_renderer/camera_render_application.h>
class CameraSimulator
{
public:
  CameraSimulator(ros::NodeHandle, ros::NodeHandle);
private:
  void cameraRenderCallback(const ros::TimerEvent&);
  void publishDepthMat(const cv::Mat& depth, ros::Time stamp);

  ros::NodeHandle nh;
  ros::Publisher image_pub;
  ros::Publisher depth_pub;
  ros::Publisher cam_info_pub;

  std::string body_frame_name;
  std::string world_frame_name;
  std::string model_name, resource_path;
  ros::Timer camera_timer;

  Eigen::Matrix4d cam_transform;
  cv::Mat K;

  VirtualImageHandler* vih;
  tf::TransformListener tflistener;
};

#endif
