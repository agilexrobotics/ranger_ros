#include "can/frame/feedback/frame_362.h"
#include <ros/ros.h>

namespace agx::nav {

Frame362::Frame362() : m_private_nh("~")
{
  m_max_count = 2;
  m_alarm_pub = m_private_nh.advertise<agx_bringup::BMSAlarmWarning>("/bms_alarm_warning", 50);
}

void Frame362::parse(const struct can_frame& frame)
{
  try
  {
    // 解析报警和警告数据
    uint8_t alarm1 = static_cast<uint8_t>(frame.data[0]);
    uint8_t alarm2 = static_cast<uint8_t>(frame.data[1]);
    uint8_t warn1 = static_cast<uint8_t>(frame.data[2]);
    uint8_t warn2 = static_cast<uint8_t>(frame.data[3]);
    
    // 创建BMS报警警告消息
    agx_bringup::BMSAlarmWarning alarm_msg;
    
    // 设置原始数据
    alarm_msg.alarm1 = alarm1;
    alarm_msg.alarm2 = alarm2;
    alarm_msg.warn1 = warn1;
    alarm_msg.warn2 = warn2;
    
    // 解析具体的报警和警告位
    parse_alarm_warning(alarm1, alarm2, warn1, warn2, alarm_msg);
    
    // 发布BMS报警警告消息
    m_alarm_pub.publish(alarm_msg);
  }
  catch (...)
  {
    ROS_ERROR("[BU] => Parse 362 frame failed");
  }
}

void Frame362::parse_alarm_warning(uint8_t alarm1, uint8_t alarm2, uint8_t warn1, uint8_t warn2, agx_bringup::BMSAlarmWarning& msg)
{
  // 解析报警1 (alarm1)
  msg.over_voltage_alarm = (alarm1 >> 6) & 1;      // BIT1: 过压报警
  msg.under_voltage_alarm = (alarm1 >> 5) & 1;     // BIT2: 欠压报警
  msg.high_temp_alarm = (alarm1 >> 4) & 1;         // BIT3: 高温报警
  msg.low_temp_alarm = (alarm1 >> 3) & 1;          // BIT4: 低温报警
  msg.discharge_overcurrent_alarm = alarm1 & 1;    // BIT7: 放电过流报警
  
  // 解析报警2 (alarm2)
  msg.charge_overcurrent_alarm = (alarm2 >> 7) & 1; // BIT0: 充电过流报警
  
  // 解析警告1 (warn1)
  msg.over_voltage_warning = (warn1 >> 6) & 1;     // BIT1: 过压警告
  msg.under_voltage_warning = (warn1 >> 5) & 1;    // BIT2: 欠压警告
  msg.high_temp_warning = (warn1 >> 4) & 1;        // BIT3: 高温警告
  msg.low_temp_warning = (warn1 >> 3) & 1;         // BIT4: 低温警告
  msg.discharge_overcurrent_warning = warn1 & 1;    // BIT7: 放电过流警告
  
  // 解析警告2 (warn2)
  msg.charge_overcurrent_warning = (warn2 >> 7) & 1; // BIT0: 充电过流警告
}

} // namespace agx::nav