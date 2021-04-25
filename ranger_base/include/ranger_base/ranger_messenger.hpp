/**
 * @Kit       : Qt-Creator: Desktop
 * @Author    : Wang Zhe
 * @Date      : 2021-04-20  11:43:51
 * @FileName  : ranger_messenger.hpp
 * @Mail      : zhe.wang@agilex.ai
 * Copyright  : AgileX Robotics (2021)
 **/

#ifndef RANGER_MESSENGER_HPP
#define RANGER_MESSENGER_HPP

#include <geometry_msgs/Twist.h>
#include <ranger_msgs/RangerSetting.h>
#include <ranger_msgs/RangerStatus.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <string>
#include "ascent/Ascent.h"
#include "ascent/Utility.h"
#include "ranger_base/ranger_model.hpp"
#include "ranger_base/ranger_params.hpp"
#include "ugv_sdk/ranger_base.hpp"

//#include <Eigen/Core>
#include <eigen3/Eigen/Core>

using namespace ros;
using namespace ros::master;

namespace westonrobot {

template <typename SystemModel>
class SystemPropagator {
 public:
  asc::state_t Propagate(asc::state_t init_state,
                         typename SystemModel::control_t u, double t0,
                         double tf, double dt) {
    double t = t0;
    asc::state_t x = init_state;
    while (t <= tf) {
      integrator_(SystemModel(u), x, t, dt);
    }
    return x;
  }

 private:
  asc::RK4 integrator_;
};

class RangerROSMessenger {
 public:
  explicit RangerROSMessenger(ros::NodeHandle *nh);
  RangerROSMessenger(RangerBase *ranger, ros::NodeHandle *nh);

  std::string odom_frame_;
  std::string base_frame_;
  std::string odom_topic_name_;
  bool pub_odom_tf_;

  bool simulated_robot_ = false;
  int sim_control_rate_ = 50;

  void SetupSubscription();

  void PublishStateToROS();
  void PublishSimStateToROS(double linear, double angular);

  void GetCurrentMotionCmdForSim(double &linear, double &angular);

 private:
  RangerBase *ranger_;
  ros::NodeHandle *nh_;

  std::mutex twist_mutex_;
  geometry_msgs::Twist current_twist_;
  uint8_t motion_mode_{0};

  ros::Publisher odom_publisher_;
  ros::Publisher status_publisher_;
  ros::Subscriber motion_cmd_subscriber_;
  ros::Subscriber light_cmd_subscriber_;
  ros::Subscriber ranger_setting_sub_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  static constexpr double l = RangerParams::wheelbase;
  static constexpr double w = RangerParams::track;
  static constexpr double s = w / 2.0;                    // half of track
  static constexpr double steer_angle_tolerance = 0.005;  // ~+-0.287 degrees

  // speed variables
  double linear_speed_ = 0.0;  // inear velocity
  double angular_vel_ = 0.0;   // angule velocity
  double x_linear_vel_ = 0.0;  // x direction linear velocity
  double y_linear_vel_ = 0.0;  // y direction linear velocity
  double position_x_ = 0.0;
  double position_y_ = 0.0;
  double theta_ = 0.0;

  // the transform of current position based on the origin
  Eigen::Matrix4d curr_transform_{};

  SystemPropagator<BicycleKinematics> model_;

  ros::Time last_time_;
  ros::Time current_time_;

  void TwistCmdCallback(const geometry_msgs::Twist::ConstPtr &msg);
  //   void LightCmdCallback(const ranger_msgs::RangerLightCmd::ConstPtr &msg);
  void RangerSettingCbk(const ranger_msgs::RangerSetting::ConstPtr &msg);
  double ConvertInnerAngleToCentral(double angle);
  double ConvertCentralAngleToInner(double angle);
  void PublishOdometryToROS(double linear, double angle_vel,
                            double x_linear_vel, double y_linear_vel,
                            double dt);
};
}  // namespace westonrobot

#endif  // RANGER_MESSENGER_HPP
