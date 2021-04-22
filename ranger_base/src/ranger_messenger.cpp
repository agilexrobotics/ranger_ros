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
#include "ranger_base/ranger_params.hpp"
#include "ranger_msgs/RangerStatus.h"

using namespace ros;
using namespace ranger_msgs;

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
  ranger_setting_sub_ = nh_->subscribe<ranger_msgs::RangerSetting>(
      "/ranger_setting", 1, &RangerROSMessenger::RangerSettingCbk, this);
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
  status_msg.steering_angle = state.motion_state.steering_angle / 180.0 * M_PI;

  status_msg.vehicle_state = state.system_state.vehicle_state;
  status_msg.control_mode = state.system_state.control_mode;
  status_msg.error_code = state.system_state.error_code;
  status_msg.battery_voltage = state.system_state.battery_voltage;

  status_publisher_.publish(status_msg);

  PublishOdometryToROS(status_msg.lateral_velocity, status_msg.steering_angle,
                       dt);

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
  double steer_cmd = msg->angular.z;
  if (steer_cmd > RangerParams::max_steer_angle_central) {
    steer_cmd = RangerParams::max_steer_angle_central;
  }
  if (steer_cmd < -RangerParams::max_steer_angle_central) {
    steer_cmd = -RangerParams::max_steer_angle_central;
  }

  if (!simulated_robot_) {
    double phi_i = ConvertCentralAngleToInner(steer_cmd);

    double phi_degree = -(phi_i / M_PI * 180.0);
    // std::cout << "set steering angle: " << phi_degree << std::endl;
    ranger_->SetMotionCommand(msg->linear.x, phi_degree);
  } else {
    std::lock_guard<std::mutex> lg(twist_mutex_);
    current_twist_ = *msg.get();
  }
}

void RangerROSMessenger::RangerSettingCbk(
    const ranger_msgs::RangerSetting::ConstPtr &msg) {
  auto mode = msg->motion_mode;
  switch (mode) {
    case RangerSetting::MOTION_MODE_ACKERMAN: {
      ranger_->SetMotionMode(RangerSetting::MOTION_MODE_ACKERMAN);
      break;
    }
    case RangerSetting::MOTION_MODE_SLIDE: {
      ranger_->SetMotionMode(RangerSetting::MOTION_MODE_SLIDE);
      break;
    }
    case RangerSetting::MOTION_MODE_ROUND: {
      ranger_->SetMotionMode(RangerSetting::MOTION_MODE_ROUND);
      break;
    }
    case RangerSetting::MOTION_MODE_SLOPING: {
      ranger_->SetMotionMode(RangerSetting::MOTION_MODE_SLOPING);
      break;
    }
    default:
      ROS_WARN("ranger motion mode not support %d", mode);
      break;
  }
}
double RangerROSMessenger::ConvertInnerAngleToCentral(double angle) {
  double phi = 0;
  double phi_i = angle;
  if (phi_i > steer_angle_tolerance) {
    // left turn
    double r = l / std::tan(phi_i) + w;
    phi = std::atan(l / r);
  } else if (phi_i < -steer_angle_tolerance) {
    // right turn
    double r = l / std::tan(-phi_i) + w;
    phi = std::atan(l / r);
    phi = -phi;
  }
  return phi;
}

double RangerROSMessenger::ConvertCentralAngleToInner(double angle) {
  double phi = angle;
  double phi_i = 0;
  if (phi > steer_angle_tolerance) {
    // left turn
    phi_i =
        std::atan(l * std::sin(phi) / (l * std::cos(phi) - w * std::sin(phi)));
  } else if (phi < -steer_angle_tolerance) {
    // right turn
    phi = -phi;
    phi_i =
        std::atan(l * std::sin(phi) / (l * std::cos(phi) - w * std::sin(phi)));
    phi_i = -phi_i;
  }
  return phi_i;
}
void RangerROSMessenger::PublishOdometryToROS(double linear, double angular,
                                              double dt) {
  linear_speed_ = linear;
  angular_angle_ = angular;
}

}  // namespace westonrobot
