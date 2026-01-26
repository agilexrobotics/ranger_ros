#include "can/frame/feedback/frame_271.h"
#include <ros/ros.h>

namespace agx::nav {

Frame271::Frame271() : m_private_nh("~")
{
  m_max_count = 50;
  m_steering_pub = m_private_nh.advertise<agx_bringup::SteeringAngles>("/steering_angles", m_max_count);
}

void Frame271::parse(const struct can_frame& frame)
{
  try
  {
    // 解析车轮角度数据
    int8_t angle_0 = static_cast<int8_t>(frame.data[0]);
    int8_t angle_1 = static_cast<int8_t>(frame.data[1]);
    int8_t angle_2 = static_cast<int8_t>(frame.data[2]);
    int8_t angle_3 = static_cast<int8_t>(frame.data[3]);
    int8_t angle_4 = static_cast<int8_t>(frame.data[4]);
    int8_t angle_5 = static_cast<int8_t>(frame.data[5]);
    int8_t angle_6 = static_cast<int8_t>(frame.data[6]);
    int8_t angle_7 = static_cast<int8_t>(frame.data[7]);

    // 创建转向角度消息
    agx_bringup::SteeringAngles steering_msg;
    
    // 转换角度数据 (单位: 0.001rad -> rad)
    steering_msg.steering01 = get_data(angle_0, angle_1) / 1000.0;
    steering_msg.steering02 = get_data(angle_2, angle_3) / 1000.0;
    steering_msg.steering03 = get_data(angle_4, angle_5) / 1000.0;
    steering_msg.steering04 = get_data(angle_6, angle_7) / 1000.0;

    // 发布转向角度消息
    m_steering_pub.publish(steering_msg);
  }
  catch (...)
  {
    ROS_ERROR("[BU] => Parse 271 frame failed");
  }
}

double Frame271::get_data(int8_t high, int8_t low)
{
  return (static_cast<int16_t>(high) << 8) | (static_cast<uint16_t>(low) & 0xFF);
}

} // namespace agx::nav