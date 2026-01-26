#include "can/frame/feedback/frame_281.h"
#include <ros/ros.h>

namespace agx::nav {

Frame281::Frame281() : m_private_nh("~")
{
  m_max_count = 50;
  m_steering_pub = m_private_nh.advertise<agx_bringup::WheelSpeeds>("/wheel_speeds", m_max_count);
}

void Frame281::parse(const struct can_frame& frame)
{
  try
  {
    // 解析转向速度数据
    int8_t speed_0 = static_cast<int8_t>(frame.data[0]);
    int8_t speed_1 = static_cast<int8_t>(frame.data[1]);
    int8_t speed_2 = static_cast<int8_t>(frame.data[2]);
    int8_t speed_3 = static_cast<int8_t>(frame.data[3]);
    int8_t speed_4 = static_cast<int8_t>(frame.data[4]);
    int8_t speed_5 = static_cast<int8_t>(frame.data[5]);
    int8_t speed_6 = static_cast<int8_t>(frame.data[6]);
    int8_t speed_7 = static_cast<int8_t>(frame.data[7]);

    // 创建轮速消息
    agx_bringup::WheelSpeeds speed_msg;
    
    // 转换速度数据 (单位: mm/s -> m/s)
    speed_msg.wheel01 = get_data(speed_0, speed_1) / 1000.0;
    speed_msg.wheel02 = get_data(speed_2, speed_3) / 1000.0;
    speed_msg.wheel03 = get_data(speed_4, speed_5) / 1000.0;
    speed_msg.wheel04 = get_data(speed_6, speed_7) / 1000.0;

    // 发布转向速度消息
    m_steering_pub.publish(speed_msg);
  }
  catch (...)
  {
    ROS_ERROR("[BU] => Parse 281 frame failed");
  }
}

double Frame281::get_data(int8_t high, int8_t low)
{
  return (static_cast<int16_t>(high) << 8) | (static_cast<uint16_t>(low) & 0xFF);
}

} // namespace agx::nav