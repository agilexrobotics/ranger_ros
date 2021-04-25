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

#include <ranger_msgs/RangerSetting.h>
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

  // 0 Arckmann 1 Slide 2 round, 3 Sloping
  robot->SetMotionMode(0);
  //  robot->SetMotionMode(1);
  //  robot->SetMotionMode(2);
  //  robot->SetMotionMode(3);

  robot->SetMotionCommand(0.1, 30.0/180.0 * M_PI); // steer angle = 30°

  // publish robot state at 50Hz while listening to twist commands
  ros::Rate rate(50);
  while (ros::ok()) {
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}
