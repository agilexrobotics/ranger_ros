#include "can/frame/feedback/frame_361.h"
#include <ros/ros.h>

namespace agx::nav {

Frame361::Frame361() : m_private_nh("~")
{
  m_max_count = 2;
  m_bms_pub = m_private_nh.advertise<agx_bringup::BMSStatus>("/bms_status", 50);
}

void Frame361::parse(const struct can_frame& frame)
{
  try
  {
    // 创建BMS状态消息
    agx_bringup::BMSStatus bms_msg;
    
    // 解析SOC (State of Charge) - 电量百分比
    bms_msg.soc = static_cast<uint8_t>(frame.data[0]);
    
    // 解析SOH (State of Health) - 健康度百分比
    bms_msg.soh = static_cast<uint8_t>(frame.data[1]);
    
    // 解析电压 (单位: 0.01V -> V)
    int8_t vol_high = static_cast<int8_t>(frame.data[2]);
    int8_t vol_low = static_cast<int8_t>(frame.data[3]);
    bms_msg.voltage = get_data(vol_high, vol_low) / 100.0;
    
    // 解析电流 (单位: 0.1A -> A)
    int8_t current_high = static_cast<int8_t>(frame.data[4]);
    int8_t current_low = static_cast<int8_t>(frame.data[5]);
    bms_msg.current = get_data(current_high, current_low) / 10.0;
    
    // 解析温度 (单位: 0.1°C -> °C)
    int8_t temp_high = static_cast<int8_t>(frame.data[6]);
    int8_t temp_low = static_cast<int8_t>(frame.data[7]);
    bms_msg.temperature = get_data(temp_high, temp_low) / 10.0;
    
    // 充电状态 (需要根据实际协议解析，这里假设在data[0]的高位)
    bms_msg.status = (frame.data[0] >> 6) & 0x03;  // 假设状态在SOC字节的高2位
    
    // 发布BMS状态消息
    m_bms_pub.publish(bms_msg);
  }
  catch (...)
  {
    ROS_ERROR("[BU] => Parse 361 frame failed");
  }
}

double Frame361::get_data(int8_t high, int8_t low)
{
  return (static_cast<int16_t>(high) << 8) | (static_cast<uint16_t>(low) & 0xFF);
}

} // namespace agx::nav