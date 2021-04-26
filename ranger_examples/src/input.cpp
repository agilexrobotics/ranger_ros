/**
 * @Kit       : Qt-Creator: Desktop
 * @Author    : Wang Zhe
 * @Date      : 2021-04-26  18:47:25
 * @FileName  : input.cpp
 * @Mail      : zhe.wang@agilex.ai
 * Copyright  : AgileX Robotics (2021)
 **/

#include <memory>

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <ranger_msgs/RangerSetting.h>
#include "ranger_base/ranger_messenger.hpp"
#include "ugv_sdk/ranger_base.hpp"

using namespace westonrobot;

std::shared_ptr<RangerBase> robot;

int main(int argc, char *argv[]) {
  // setup ROS node
  ros::init(argc, argv, "input");
  ros::NodeHandle node(""), private_node("~");

  // instantiate a robot object
  robot = std::make_shared<RangerBase>();

  robot->Connect("can0");
  robot->EnableCommandedMode();

  ////-----------------direct control by call the sdk function-------------
  // 0 Arckmann 1 Slide 2 round, 3 Sloping
  robot->SetMotionMode(0);
  //  robot->SetMotionMode(1);
  //  robot->SetMotionMode(2);
  //  robot->SetMotionMode(3);

  //  robot->SetMotionCommand(0.1, 30.0/180.0 * M_PI); // steer angle = 30°

  ////----------------control by ros topic---------------------------------
  ros::Publisher motion_mode =
      node.advertise<ranger_msgs::RangerSetting>("/ranger_setting", 1);
  ranger_msgs::RangerSetting setting;
  setting.motion_mode = ranger_msgs::RangerSetting::MOTION_MODE_ACKERMAN;
  motion_mode.publish(setting);

  ////------------------move by ros topic --------------------------------
  ros::Publisher move_cmd =
      node.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
  geometry_msgs::Twist cmd;
  cmd.linear.x = 0.1;                   // the motor will run at 0.1m/s
  cmd.angular.z = 30.0 / 180.0 * M_PI;  // the heading angle of the car

  // publish robot state at 50Hz while listening to twist commands
  ros::Rate rate(50);
  while (ros::ok()) {
    ros::spinOnce();

    // /cmd_vel topic must send at 50Hz, even stop need send 0m/s
    move_cmd.publish(cmd);

    rate.sleep();
  }
  return 0;
}
