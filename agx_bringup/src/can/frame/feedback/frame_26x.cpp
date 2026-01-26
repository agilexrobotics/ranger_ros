#include "can/frame/feedback/frame_26x.h"
#include <ros/ros.h>

namespace agx::nav {

Frame26x::Frame26x() : m_private_nh("~")
{
  m_max_count = 10;
  m_motor_array_pub = m_private_nh.advertise<agx_bringup::MotorLowArray>("/motor_low_speed_feedback", 50);
  
  // 初始化1-8号电机数据存储
  for (int i = 1; i <= 8; ++i) {
    m_motor_data[i] = nlohmann::json::object();
  }
  
  m_received_count = 0;
}

void Frame26x::parse(const struct can_frame& frame)
{
  try
  {
    // 检查是否是0x261-0x268帧
    if (frame.can_id < 0x261 || frame.can_id > 0x268) {
      return;
    }
    
    // 计算电机编号 (0x261->1, 0x262->2, ..., 0x268->8)
    int motor_id = frame.can_id - 0x260;
    
    std::lock_guard<std::mutex> lock(m_data_mutex);
    
    nlohmann::json motor_result;
    motor_driver_low(frame, motor_result);
    
    // 添加电机基本信息
    motor_result["motor_id"] = motor_id;
    motor_result["can_id"] = frame.can_id;
    
    // 检查是否是新数据
    bool had_previous_data = false;
    if (!m_motor_data[motor_id].empty() && 
        m_motor_data[motor_id].contains("motor_id")) {
      had_previous_data = true;
    }
    
    // 存储JSON数据
    m_motor_data[motor_id] = motor_result;
    
    if (!had_previous_data) {
      m_received_count++;
    }
    
    ROS_DEBUG_STREAM("[BU] => 26x电机" << motor_id << "数据，已收集: " << m_received_count << "/8");
    
    // 检查是否收集完整
    if (m_received_count >= 8) {
      check_and_publish_complete_set();
    }
  }
  catch (const std::exception& e)
  {
    ROS_ERROR_STREAM("[BU] => 解析26x帧" << std::hex << frame.can_id << "失败: " << e.what());
  }
  catch (...)
  {
    ROS_ERROR_STREAM("[BU] => 解析26x帧" << std::hex << frame.can_id << "失败: 未知错误");
  }
}

void Frame26x::check_and_publish_complete_set()
{
  if (m_received_count >= 8) {
    try {
      agx_bringup::MotorLowArray motor_array;
      
      for (int i = 1; i <= 8; ++i) {
        if (!m_motor_data[i].empty()) {
          agx_bringup::MotorLow motor_low;
          json_to_motor_low(m_motor_data[i], motor_low);
          motor_array.motors.push_back(motor_low);
        }
      }
      
      m_motor_array_pub.publish(motor_array);
            
    } catch (const std::exception& e) {
      ROS_ERROR_STREAM("[BU] => 发布26x数据失败: " << e.what());
    }
    
    reset_collection();
  }
}

void Frame26x::json_to_motor_low(const nlohmann::json& json_data, agx_bringup::MotorLow& motor_low)
{
  // 从JSON转换为自定义消息
  motor_low.motor_id = json_data.value("motor_id", 0);
  motor_low.can_id = json_data.value("can_id", 0);
  motor_low.driver_vol = json_data.value("driverVol", 0.0f) * 0.1;
  motor_low.driver_temp = json_data.value("driverTemp", 0.0f);
  motor_low.motor_temp = json_data.value("motorTemp", 0.0f);
  motor_low.driver_status = json_data.value("driverStatus", 0);
  motor_low.enable_status = json_data.value("enable_status", false);
  motor_low.error_status = json_data.value("error_status", false);
  motor_low.sensor_status = json_data.value("sensor_status", false);
  motor_low.over_temp_driver = json_data.value("over_temp_driver", false);
  motor_low.over_current = json_data.value("over_current", false);
  motor_low.over_temp_motor = json_data.value("over_temp_motor", false);
  motor_low.under_voltage = json_data.value("under_voltage", false);
}

void Frame26x::reset_collection()
{
  m_received_count = 0;
  for (int i = 1; i <= 8; ++i) {
    m_motor_data[i] = nlohmann::json::object();
  }
}

} // namespace agx::nav