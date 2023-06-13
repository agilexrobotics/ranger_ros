/**
 * @Kit       : Qt-Creator: Desktop
 * @Author    : Wang Zhe
 * @Date      : 2021-04-21  20:29:35
 * @FileName  : ranger_model.cpp
 * @Mail      : zhe.wang@agilex.ai
 * Copyright  : AgileX Robotics (2021)
 **/

#include "ranger_base/ranger_model.hpp"
#include <cmath>
#include <cstdint>

namespace westonrobot {

// x1 = x, x2 = y, x3 = theta
void BicycleKinematics::operator()(const asc::state_t &x, asc::state_t &xd,
                                   const double) {
  xd[0] = u_.v * std::cos(x[2]);
  xd[1] = u_.v * std::sin(x[2]);
  xd[2] = (2 * u_.v / L) * std::tan(u_.delta);
}
}  // namespace westonrobot
