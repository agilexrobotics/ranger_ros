//-----------------------------------------------------------------------------
// Copyright(c) 2016-2025 松灵机器人
// All rights reserved.
//
// File: chassis_service.h
// Version: 1.0.00
//
// Create:  Feb.24 2025
// Author:  youguo.chu@agilex.ai
// Summary: 底盘操作服务类
//-----------------------------------------------------------------------------

#ifndef CHASSIS_SERVICE_H_
#define CHASSIS_SERVICE_H_

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nlohmann/json.hpp>
#include "can/frame/handler/can_handler.h"

using json = nlohmann::json;

namespace agx::nav {

/** 底盘操作服务类 */
class ChassisService
{
public:
  ChassisService();
  ~ChassisService();
  void launch();

private:
  /** 订阅发送给底盘的数据 */
  void sub_cmd_vel_callback(geometry_msgs::Twist cmd_vel);

private:
  ros::NodeHandle m_private_nh;
  /** 订阅发送给底盘速度信息 */
  ros::Subscriber m_sub_cmd_vel;

};

}

#endif
