/**
 * @Kit       : Qt-Creator: Desktop
 * @Author    : Wang Zhe
 * @Date      : 2021-04-26  18:47:49
 * @FileName  : output.cpp
 * @Mail      : zhe.wang@agilex.ai
 * Copyright  : AgileX Robotics (2021)
 **/

#include <ranger_msgs/RangerStatus.h>
#include <ros/ros.h>
#include <iostream>

void StatusCallback(ranger_msgs::RangerStatus::ConstPtr msg) {
  std::cout << "linear velocity: " << msg->linear_velocity << std::endl;
  std::cout << "angular velocity: " << msg->angular_velocity << std::endl;
  std::cout << "x direction linear velocity: " << msg->x_linear_vel
            << std::endl;
  std::cout << "y direction linear linear velocity: " << msg->y_linear_vel
            << std::endl;
  std::cout << "rotate radius: " << msg->motion_radius << std::endl;
  std::cout << "car heading angle: " << msg->steering_angle << std::endl;
  // ...etc
}

int main(int argc, char *argv[]) {
  // setup ROS node
  ros::init(argc, argv, "output");
  ros::NodeHandle node(""), private_node("~");

  ros::Subscriber status_sub = node.subscribe<ranger_msgs::RangerStatus>(
      "/ranger_status", 10, StatusCallback);

  ros::Rate rate(50);
  while (ros::ok()) {
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}
