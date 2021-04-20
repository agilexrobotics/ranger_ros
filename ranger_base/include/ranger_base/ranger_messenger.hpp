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

#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <string>

#include "ugv_sdk/ranger_base.hpp"
namespace westonrobot {
class RangerROSMessenger {
 public:
  explicit RangerROSMessenger(ros::NodeHandle *nh);
  RangerROSMessenger(RangerBase *ranger, ros::NodeHandle *nh);

  std::string odom_frame_;
  std::string base_frame_;
  std::string odom_topic_name_;

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

  ros::Publisher odom_publisher_;
  ros::Publisher status_publisher_;
  ros::Subscriber motion_cmd_subscriber_;
  ros::Subscriber light_cmd_subscriber_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  // speed variables
  double linear_speed_ = 0.0;
  double angular_speed_ = 0.0;
  double position_x_ = 0.0;
  double position_y_ = 0.0;
  double theta_ = 0.0;

  ros::Time last_time_;
  ros::Time current_time_;

  void TwistCmdCallback(const geometry_msgs::Twist::ConstPtr &msg);
  void LightCmdCallback(const ranger_msgs::RangerLightCmd::ConstPtr &msg);
  void PublishOdometryToROS(double linear, double angular, double dt);
};
}  // namespace westonrobot

#endif  // RANGER_MESSENGER_HPP
