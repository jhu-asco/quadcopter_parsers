#include "tf_parser/state.h"

State::State() : p(0, 0, 0), v(0, 0, 0), w(0, 0, 0), rpy(0, 0, 0), a_b(0, 0, 0) {}

void State::Clear() {
  p.setZero();
  v.setZero();
  w.setZero();
  rpy.setZero();
  a_b.setZero();
}
