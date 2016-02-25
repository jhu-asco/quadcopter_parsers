#include "quad_simulator_parser/camera_simulator.h"
int main(int argc, char** argv)
{
  ros::init(argc, argv, "camera_simulator");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  CameraSimulator cs(nh, nh_private);
  ros::spin();
  return 0;
}
