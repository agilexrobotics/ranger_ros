#include "can/frame/feedback/frame_221.h"
#include <ros/ros.h>
#include "global_state.hpp"

namespace agx::nav
{

  Frame221::Frame221() : m_private_nh("~")
  {
    m_max_count = 50;
    // 添加自定义消息发布器
    m_chassis_pub = m_private_nh.advertise<agx_bringup::ChassisMotion>("/chassis_motion_feedback", m_max_count);
  }

  void Frame221::parse(const struct can_frame &frame)
  {
    try
    {
      auto &gs = GlobalState::getInstance();

      // 解析线速度 (单位: m/s)
      int8_t speed_high = static_cast<int8_t>(frame.data[0]);
      int8_t speed_low = static_cast<int8_t>(frame.data[1]);
      double linear_speed = get_data(speed_high, speed_low) / 1000.0;

      // 解析旋转角速度 (单位: rad/s)
      int8_t rotate_angle_high = static_cast<int8_t>(frame.data[2]);
      int8_t rotate_angle_low = static_cast<int8_t>(frame.data[3]);
      double rotate_angular = get_data(rotate_angle_high, rotate_angle_low) / 1000.0;

      // 解析转向角 (单位: rad)
      int8_t angle_high = static_cast<int8_t>(frame.data[6]);
      int8_t angle_low = static_cast<int8_t>(frame.data[7]);
      double steering_angle = get_data(angle_high, angle_low) / 1000.0;

      gs.setLinearVelocity(linear_speed);
      gs.setAngularVelocity(rotate_angular);
      gs.setSteeringAngle(steering_angle);

      // 创建自定义消息
      agx_bringup::ChassisMotion chassis_msg;
      chassis_msg.linear_speed = linear_speed;
      chassis_msg.rotate_angular = rotate_angular;
      chassis_msg.steering_angle = steering_angle;

      // 发布自定义消息
      m_chassis_pub.publish(chassis_msg);
    }
    catch (...)
    {
      ROS_ERROR("[BU] => Parse 221 frame failed");
    }
  }

  double Frame221::get_data(int8_t high, int8_t low)
  {
    return (static_cast<int16_t>(high) << 8) | (static_cast<uint16_t>(low) & 0xFF);
  }

} // namespace agx::nav