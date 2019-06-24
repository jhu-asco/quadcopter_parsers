#pragma once
#include <Eigen/Dense>

struct State {
  Eigen::Vector3d p;
  Eigen::Vector3d v;
  Eigen::Vector3d rpy;
  Eigen::Vector3d w;
  // This is the acceleration (in m/s^2) in the pitch/roll
  // compensated body frame (i.e., one only needs to
  // transform the frame based on the yaw to get world frame acc)
  Eigen::Vector3d a_b;

  State();
  void Clear();
};
