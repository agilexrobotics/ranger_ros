/**
 * @Kit       : Qt-Creator: Desktop
 * @Author    : Wang Zhe
 * @Date      : 2021-04-21  20:10:56
 * @FileName  : ranger_model.hpp
 * @Mail      : zhe.wang@agilex.ai
 * Copyright  : AgileX Robotics (2021)
 **/

#ifndef RANGER_MODEL_HPP
#define RANGER_MODEL_HPP

#include "ascent/Ascent.h"
#include "ascent/Utility.h"
#include "ranger_base/ranger_params.hpp"

namespace westonrobot {
/*
 * Bicycle kinematics model (center of weelbase):
 *  dot_x = v(t) * cos(theta(t))
 *  dot_y = v(t) * sin(theta(t))
 *  dot_theta = 2 * v(t)/L * tan(delta(t)) // twice of ackerman's
 * State: (x, y, theta)
 * Control input: (v, delta) - velocity, steering angle of front wheel
 */
class BicycleKinematics

{
 public:
  struct CtrlInput {
    CtrlInput(double vel = 0.0, double steer = 0.0) : v(vel), delta(steer) {
      double v{0.0};
      double delta{0.0};
    }

    double v;
    double delta;
  };
  using control_t = CtrlInput;

  BicycleKinematics(control_t u) : u_(u) {}

  // x1 = x, x2 = y, x3 = theta
  void operator()(const asc::state_t &x, asc::state_t &xd, const double);

 private:
  control_t u_{0.0, 0.0};

  static constexpr double L = RangerParams::wheelbase;
};
}  // namespace westonrobot
#endif  // RANGER_MODEL_HPP
