#include "can/frame/feedback/frame_241.h"
#include <ros/ros.h>

namespace agx::nav {

Frame241::Frame241() : m_private_nh("~")
{
  m_max_count = 50;
  m_remote_pub = m_private_nh.advertise<agx_bringup::RemoteControlStatus>("/remote_control_status", m_max_count);
}

void Frame241::parse(const struct can_frame& frame)
{
  try
  {
    int8_t sw = static_cast<int8_t>(frame.data[0]);
    
    // 创建遥控器状态消息
    agx_bringup::RemoteControlStatus remote_msg;
    
    // 解析开关状态
    remote_msg.SWA = static_cast<uint8_t>(sw & 0x03);
    remote_msg.SWB = static_cast<uint8_t>((sw >> 2) & 0x03);
    remote_msg.SWC = static_cast<uint8_t>((sw >> 4) & 0x03);
    remote_msg.SWD = static_cast<uint8_t>((sw >> 6) & 0x03);
    
    // 解析摇杆数据
    remote_msg.rightLR = static_cast<int8_t>(frame.data[1]);
    remote_msg.rightUD = static_cast<int8_t>(frame.data[2]);
    remote_msg.leftUD = static_cast<int8_t>(frame.data[3]);
    remote_msg.leftLR = static_cast<int8_t>(frame.data[4]);
    remote_msg.leftVRA = static_cast<int8_t>(frame.data[5]);
    
    // 发布遥控器状态消息
    m_remote_pub.publish(remote_msg);
  }
  catch (...)
  {
    ROS_ERROR("[BU] => Parse 241 frame failed");
  }
}

} // namespace agx::nav