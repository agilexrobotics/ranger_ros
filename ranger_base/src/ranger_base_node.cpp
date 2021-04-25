/**
 * @Kit       : Qt-Creator: Desktop
 * @Author    : Wang Zhe
 * @Date      : 2021-04-20  11:52:52
 * @FileName  : ranger_base_node.cpp
 * @Mail      : zhe.wang@agilex.ai
 * Copyright  : AgileX Robotics (2021)
 **/

#include <memory>

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

#include <signal.h>
#include <stdlib.h>
#include <unistd.h>
#include "ranger_base/ranger_messenger.hpp"
#include "ugv_sdk/ranger_base.hpp"

using namespace westonrobot;

std::shared_ptr<RangerBase> robot;

void SignalHandler(int s) {
  printf("Caught signal %d, program exit\n", s);
  exit(EXIT_FAILURE);
}
void controlSingal() {
  struct sigaction sigIntHandler;
  sigIntHandler.sa_handler = SignalHandler;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;
  sigaction(SIGINT, &sigIntHandler, NULL);
}
int main(int argc, char **argv) {
  // setup ROS node
  ros::init(argc, argv, "ranger_node");
  ros::NodeHandle node(""), private_node("~");

  controlSingal();

  // instantiate a robot object
  robot = std::make_shared<RangerBase>();
  RangerROSMessenger messenger(robot.get(), &node);

  // fetch parameters before connecting to robot
  std::string port_name;
  private_node.param<std::string>("port_name", port_name, std::string("can0"));
  private_node.param<std::string>("odom_frame", messenger.odom_frame_,
                                  std::string("odom"));
  private_node.param<std::string>("base_frame", messenger.base_frame_,
                                  std::string("base_link"));
  private_node.param<bool>("simulated_robot", messenger.simulated_robot_,
                           false);
  private_node.param<int>("control_rate", messenger.sim_control_rate_, 50);
  private_node.param<std::string>("odom_topic_name", messenger.odom_topic_name_,
                                  std::string("odom"));
  private_node.param<bool>("pub_odom_tf", messenger.pub_odom_tf_, false);

  if (!messenger.simulated_robot_) {
    // connect to robot and setup ROS subscription
    if (port_name.find("can") != std::string::npos) {
      robot->Connect(port_name);
      robot->EnableCommandedMode();
      ROS_INFO("Using CAN bus to talk with the robot");
    }
  }
  messenger.SetupSubscription();

  // publish robot state at 50Hz while listening to twist commands
  ros::Rate rate(50);
  while (true) {
    if (!messenger.simulated_robot_) {
      messenger.PublishStateToROS();
    } else {
      double linear, angular;
      messenger.GetCurrentMotionCmdForSim(linear, angular);
      messenger.PublishSimStateToROS(linear, angular);
    }
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
