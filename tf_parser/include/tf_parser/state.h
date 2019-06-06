#pragma once
#include <Eigen/Dense>

struct State {
  Eigen::Vector3d p;
  Eigen::Vector3d v;
  Eigen::Vector3d rpy;
  Eigen::Vector3d w;
  Eigen::Vector3d a_b;

  State() : p(0, 0, 0), v(0, 0, 0), w(0, 0, 0), rpy(0, 0, 0), a_b(0, 0, 0) {}
  void Clear() {
    p.setZero();
    v.setZero();
    w.setZero();
    rpy.setZero();
    a_b.setZero();
  }
};
