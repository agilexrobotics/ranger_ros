//-----------------------------------------------------------------------------
// Copyright(c) 2016-2025 松灵机器人
// All rights reserved.
//
// File: frame_311.h
// Version: 1.0.00
//
// Create:  Feb.21 2025
// Author:  youguo.chu@agilex.ai
// Summary: 311帧解析
//-----------------------------------------------------------------------------

#ifndef FRAME_312_H_
#define FRAME_312_H_

#include <cmath>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <nlohmann/json.hpp>
#include "can/chassis_status.h"
#include "can/frame/feedback/base_frame.h"
#include <agx_bringup/BackWheelOdometry.h>

using json = nlohmann::json;

namespace agx::nav
{

  class Frame312: public BaseFrame
  {
  public:
    Frame312();
    ~Frame312() = default;

    virtual void parse(const struct can_frame &frame);

  private:
    ros::NodeHandle m_nh;
    ros::NodeHandle m_private_nh;

    // 发布器
    ros::Publisher m_odom_pub;

    // 数据成员
    double m_left_odom;
    double m_right_odom;

    // 话题名称参数
    std::string m_odom_topic;
  };

} // namespace agx::nav

#endif
