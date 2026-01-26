#include "can/frame/feedback/frame_231.h"
#include <ros/ros.h>

namespace agx::nav {

Frame231::Frame231() : m_private_nh("~")
{
  m_max_count = 50;
  m_light_pub = m_private_nh.advertise<agx_bringup::LightControlStatus>("/light_control_status", m_max_count);
}

void Frame231::parse(const struct can_frame& frame)
{
  try
  {
    int8_t enable = static_cast<int8_t>(frame.data[0]);
    int8_t mode = static_cast<int8_t>(frame.data[1]);
    
    // 创建灯光控制状态消息
    agx_bringup::LightControlStatus light_msg;
    
    // 设置使能状态
    light_msg.enable = enable;
    if (enable == 0) {
      light_msg.enable_text = "Disabled";
    } else if (enable == 1) {
      light_msg.enable_text = "Enabled";
    } else {
      light_msg.enable_text = "Unknown";
    }
    
    // 设置灯光模式
    light_msg.mode = mode;
    if (mode == 0) {
      light_msg.mode_text = "Normally Off";
    } else if (mode == 1) {
      light_msg.mode_text = "Normally On";
    } else {
      light_msg.mode_text = "Unknown";
    }
    
    // 发布灯光控制状态消息
    m_light_pub.publish(light_msg);
  }
  catch (...)
  {
    ROS_ERROR("[BU] => Parse 231 frame failed");
  }
}

} // namespace agx::nav