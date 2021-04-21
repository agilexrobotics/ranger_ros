/**
 * @Kit       : Qt-Creator: Desktop
 * @Author    : Wang Zhe
 * @Date      : 2021-04-20  11:50:26
 * @FileName  : ranger_messenger.cpp
 * @Mail      : zhe.wang@agilex.ai
 * Copyright  : AgileX Robotics (2021)
 **/

#include "ranger_base/ranger_messenger.hpp"
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include "ranger_msgs/RangerStatus.h"

using namespace ros;

namespace westonrobot {
RangerROSMessenger::RangerROSMessenger(ros::NodeHandle *nh)
    : ranger_(nullptr), nh_(nh) {}

RangerROSMessenger::RangerROSMessenger(RangerBase *ranger, ros::NodeHandle *nh)
    : ranger_(ranger), nh_(nh) {}

void RangerROSMessenger::SetupSubscription() {
  odom_publisher_ = nh_->advertise<nav_msgs::Odometry>(odom_topic_name_, 50);
  status_publisher_ =
      nh_->advertise<ranger_msgs::RangerStatus>("/ranger_status", 10);

  motion_cmd_subscriber_ = nh_->subscribe<geometry_msgs::Twist>(
      "/cmd_vel", 5, &RangerROSMessenger::TwistCmdCallback, this);
}

void RangerROSMessenger::PublishStateToROS() {
  current_time_ = ros::Time::now();
  double dt = (current_time_ - last_time_).toSec();
  static bool init_run = true;
  if (init_run) {
    last_time_ = current_time_;
    init_run = false;
    return;
  }

  auto state = ranger_->GetRangerState();

  ranger_msgs::RangerStatus status_msg;
  status_msg.header.stamp = current_time_;
  status_msg.linear_velocity = state.motion_state.linear_velocity;
  status_msg.angular_velocity = state.motion_state.angular_velocity;
  status_msg.lateral_velocity = state.motion_state.lateral_velocity;
  status_msg.steering_angle = state.motion_state.steering_angle;

  

  status_msg.vehicle_state = state.system_state.vehicle_state;
  status_msg.control_mode = state.system_state.control_mode;
  status_msg.error_code = state.system_state.error_code;
  status_msg.battery_voltage = state.system_state.battery_voltage;

  status_publisher_.publish(status_msg);

  PublishOdometryToROS(state.motion_state.linear_velocity,
                       state.motion_state.angular_velocity, dt);
  last_time_ = current_time_;
}

void RangerROSMessenger::PublishSimStateToROS(double linear, double angular) {}

void RangerROSMessenger::GetCurrentMotionCmdForSim(double &linear,
                                                   double &angular) {
  std::lock_guard<std::mutex> lg(twist_mutex_);
  linear = current_twist_.linear.x;
  angular = current_twist_.angular.z;
}

void RangerROSMessenger::TwistCmdCallback(
    const geometry_msgs::Twist::ConstPtr &msg) {
  if (!simulated_robot_) {
    ranger_->SetMotionCommand(msg->linear.x, msg->angular.z);
  } else {
    std::lock_guard<std::mutex> lg(twist_mutex_);
    current_twist_ = *msg.get();
  }
}

void RangerROSMessenger::PublishOdometryToROS(double linear, double angular,
                                              double dt) {
  linear_speed_ = linear;
  angular_speed_ = angular;
}

}  // namespace westonrobot
