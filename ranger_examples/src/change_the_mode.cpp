/**
 * @Kit       : Qt-Creator: Desktop
 * @Author    : Wang Zhe
 * @Date      : 2021-04-23  18:20:58
 * @FileName  : change_the_mode.cpp
 * @Mail      : zhe.wang@agilex.ai
 * Copyright  : AgileX Robotics (2021)
 **/
#include <memory>

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

#include "ranger_base/ranger_messenger.hpp"
#include "ugv_sdk/ranger_base.hpp"

using namespace westonrobot;

std::shared_ptr<RangerBase> robot;

int main(int argc, char *argv[]) {
  // setup ROS node
  ros::init(argc, argv, "change_the_mode");
  ros::NodeHandle node(""), private_node("~");

  // instantiate a robot object
  robot = std::make_shared<RangerBase>();

  robot->Connect("can0");
  robot->EnableCommandedMode();

  // publish robot state at 50Hz while listening to twist commands
  ros::Rate rate(50);
  while (ros::ok()) {
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}
