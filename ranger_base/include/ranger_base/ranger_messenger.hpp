/**
* @file ranger_messenger.hpp
* @date 2021-04-20
* @brief
*
# @copyright Copyright (c) 2021 AgileX Robotics
* @copyright Copyright (c) 2023 Weston Robot Pte. Ltd.
*/

#ifndef RANGER_MESSENGER_HPP
#define RANGER_MESSENGER_HPP

#include <string>

#include <eigen3/Eigen/Core>
#include <geometry_msgs/PoseArray.h>

#include <ros/console.h>
#include <ros/ros.h>

#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include <ranger_msgs/SystemState.h>
#include <ranger_msgs/MotionState.h>
#include <ranger_msgs/ActuatorStateArray.h>
#include <ranger_msgs/RangerLightCmd.h>
#include <ranger_msgs/TriggerParkMode.h>
#include <ranger_msgs/BatteryState.h>

#include "ranger_base/ranger_params.hpp"
#include "ugv_sdk/mobile_robot/ranger_robot.hpp"

using namespace ros::master;

namespace westonrobot {
class RangerROSMessenger {
  struct RobotParams {
    double track;
    double wheelbase;
    double max_linear_speed;
    double max_angular_speed;
    double max_speed_cmd;
    double max_steer_angle_ackermann;
    double max_steer_angle_parallel;
    double max_round_angle;
    double min_turn_radius;
  };

  enum class RangerSubType { kRanger = 0, kRangerMiniV1, kRangerMiniV2 };

 public:
  RangerROSMessenger(ros::NodeHandle* nh);

  void Run();

 private:
  void LoadParameters();
  void SetupSubscription();
  void PublishStateToROS();
  void PublishSimStateToROS(double linear, double angular);
  void TwistCmdCallback(const geometry_msgs::Twist::ConstPtr& msg);
  void LightCmdCallback(const ranger_msgs::RangerLightCmd::ConstPtr &msg);
  double CalculateSteeringAngle(geometry_msgs::Twist msg, double& radius);
  void UpdateOdometry(double linear, double angular, double angle, double dt);
  double ConvertInnerAngleToCentral(double angle);
  double ConvertCentralAngleToInner(double angle);
  bool TriggerParkingService(ranger_msgs::TriggerParkMode::Request &req, ranger_msgs::TriggerParkMode::Response &res); 

  ros::NodeHandle* nh_;
  std::shared_ptr<RangerRobot> robot_;
  RangerSubType robot_type_;
  RobotParams robot_params_;

  // constants
  const double steer_angle_tolerance_ = 0.005;  // ~+-0.287 degrees

  // parameters
  std::string robot_model_;
  std::string port_name_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string odom_topic_name_;
  int update_rate_;
  bool publish_odom_tf_;

  uint8_t motion_mode_ = 0;
  bool parking_mode_;

  ros::Publisher system_state_pub_;
  ros::Publisher motion_state_pub_;
  ros::Publisher actuator_state_pub_;
  ros::Publisher odom_pub_;
  ros::Publisher battery_state_pub_;

  ros::Subscriber motion_cmd_sub_;
  ros::Subscriber light_cmd_subscriber_;

  ros::ServiceServer trigger_parking_server;

  tf2_ros::TransformBroadcaster tf_broadcaster_;

  // odom variables
  ros::Time last_time_;
  ros::Time current_time_;
  double position_x_ = 0.0;
  double position_y_ = 0.0;
  double theta_ = 0.0;
};
}  // namespace westonrobot

#endif  // RANGER_MESSENGER_HPP
