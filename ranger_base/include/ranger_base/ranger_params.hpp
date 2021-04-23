/**
 * @Kit       : Qt-Creator: Desktop
 * @Author    : Wang Zhe
 * @Date      : 2021-04-20  11:41:28
 * @FileName  : ranger_params.hpp
 * @Mail      : zhe.wang@agilex.ai
 * Copyright  : AgileX Robotics (2021)
 **/

#ifndef RANGER_PARAMS_HPP
#define RANGER_PARAMS_HPP
namespace westonrobot {
struct RangerParams {
  static constexpr double track =
      0.36;  // in meter (left & right wheel distance)
  static constexpr double wheelbase =
      0.36;  // in meter (front & rear wheel distance)

  static constexpr double max_linear_speed = 1.5;      // in m/s
  static constexpr double max_angular_speed = 0.7853;  // in rad/s
  static constexpr double max_speed_cmd = 10.0;        // in rad/s
  static constexpr double max_steer_angle =
      0.698;  // in rad,  inner wheel
  static constexpr double max_steer_angle_central = 0.698;  // max central angle
};
}  // namespace westonrobot
#endif  // RANGER_PARAMS_HPP
