#pragma once
#include <Eigen/Dense>

struct State {
  Eigen::Vector3d p;
  Eigen::Vector3d v;
  Eigen::Vector3d rpy;
  Eigen::Vector3d w;
  Eigen::Vector3d a_b;

  State();
  void Clear();
};
