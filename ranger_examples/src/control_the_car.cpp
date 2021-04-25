/**
 * @Kit       : Qt-Creator: Desktop
 * @Author    : Wang Zhe
 * @Date      : 2021-04-23  18:20:37
 * @FileName  : control_the_car.cpp
 * @Mail      : zhe.wang@agilex.ai
 * Copyright  : AgileX Robotics (2021)
 **/

#include <memory>

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

#include <ranger_msgs/RangerSetting.h>
#include <ranger_msgs/RangerStatus.h>
#include "ranger_base/ranger_messenger.hpp"
#include "ugv_sdk/ranger_base.hpp"

using namespace ranger_msgs;
using namespace westonrobot;

void Ackermann();
void Slide();
void Round();
void Sloping();

std::shared_ptr<RangerBase> robot;
double l_v = 0.0, angle_z = 0.0;

int main(int argc, char *argv[]) {
  // setup ROS node
  ros::init(argc, argv, "control_the_car");
  ros::NodeHandle node(""), private_node("~");

  // instantiate a robot object
  robot = std::make_shared<RangerBase>();

  robot->Connect("can0");
  robot->EnableCommandedMode();

  Ackermann();
  // Slide();
  // Round();
  // Sloping();

  // publish robot state at 50Hz while listening to twist commands
  ros::Rate rate(50);
  while (ros::ok()) {
    robot->SetMotionCommand(l_v, angle_z);
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}
void Ackermann() {
  robot->SetMotionMode(0);
  // or
  robot->SetMotionMode(RangerSetting::MOTION_MODE_ACKERMAN);
  l_v = 0.1;                  // m/s
  angle_z = 30 / 180 * M_PI;  // rad
}
void Slide() {
  robot->SetMotionMode(1);
  // or
  robot->SetMotionMode(RangerSetting::MOTION_MODE_SLIDE);
  l_v = 0.1;                  // m/s
  angle_z = 30 / 180 * M_PI;  // rad
}
void Round() {
  robot->SetMotionMode(2);
  // or
  robot->SetMotionMode(RangerSetting::MOTION_MODE_ROUND);
  l_v = 0.1;
  // angle_z is not used
}
void Sloping() {
  robot->SetMotionMode(3);
  // or
  robot->SetMotionMode(RangerSetting::MOTION_MODE_SLOPING);
  l_v = 0.1;
  // angle_z is not used
}
