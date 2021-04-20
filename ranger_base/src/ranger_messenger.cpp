/**
 * @Kit       : Qt-Creator: Desktop
 * @Author    : Wang Zhe
 * @Date      : 2021-04-20  11:50:26
 * @FileName  : ranger_messenger.cpp
 * @Mail      : zhe.wang@agilex.ai
 * Copyright  : AgileX Robotics (2021)
 **/

#include "ranger_base/ranger_messenger.hpp"
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include "ranger_msgs/RangerStatus.h"

namespace westonrobot {
RangerROSMessenger::RangerROSMessenger(ros::NodeHandle *nh)
    : ranger_(nullptr), nh_(nh) {}

RangerROSMessenger::RangerROSMessenger(RangerBase *ranger, ros::NodeHandle *nh)
    : ranger_(ranger), nh_(nh) {}

void RangerROSMessenger::SetupSubscription() {
  // odom_publisher_ = nh_->ad
}

void RangerROSMessenger::PublishStateToROS()
{

}

void RangerROSMessenger::PublishSimStateToROS(double linear, double angular)
{

}

void RangerROSMessenger::GetCurrentMotionCmdForSim(double &linear, double &angular)
{

}

void RangerROSMessenger::TwistCmdCallback(const geometry_msgs::Twist::ConstPtr &msg)
{

}

void RangerROSMessenger::PublishOdometryToROS(double linear, double angular, double dt)
{

}

}  // namespace westonrobot
