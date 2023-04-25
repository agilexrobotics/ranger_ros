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
  static constexpr double track = 0.56
      /*0.56*/;  // in meter (left & right wheel distance) //ranger-mini 0.364
  static constexpr double wheelbase = 0.90
      /*0.89*/;  // in meter (front & rear wheel distance) //ranger-mini 0.494

  static constexpr double max_linear_speed = 2.7;      // in m/s
  static constexpr double max_angular_speed = 0.7853;  // in rad/s
  static constexpr double max_speed_cmd = 10.0;        // in rad/s

  static constexpr double max_steer_angle_central = 0.6981; //~= 30.58 degree
  //static constexpr double max_steer_angle_slide = 0.69813; // 40 degree
  static constexpr double max_steer_angle_slide = 1.570; // 40 degree
  static constexpr double max_round_angle = 1.0057; // 40 degree
  static constexpr double min_turn_radius = 0.810330349 /*0.810330349*/; //ranger-mini 0.536
};
struct RangerMiniParams {
  static constexpr double track = 0.364
      /*0.56*/;  // in meter (left & right wheel distance) //ranger-mini 0.364
  static constexpr double wheelbase = 0.494
      /*0.89*/;  // in meter (front & rear wheel distance) //ranger-mini 0.494

  static constexpr double max_linear_speed = 2.7;      // in m/s
  static constexpr double max_angular_speed = 0.7853;  // in rad/s
  static constexpr double max_speed_cmd = 10.0;        // in rad/s

  static constexpr double max_steer_angle_central = 0.6981; //~= 30.58 degree
  //static constexpr double max_steer_angle_slide = 0.69813; // 40 degree
  static constexpr double max_steer_angle_slide = 1.570; // 40 degree
  static constexpr double max_round_angle = 0.935671;
  static constexpr double min_turn_radius = 0.536 /*0.810330349*/; //ranger-mini 0.536
};
}  // namespace westonrobot
#endif  // RANGER_PARAMS_HPP
