#include "can/chassis_service.h"

namespace agx::nav {

ChassisService::ChassisService() : m_private_nh("~")
{}

ChassisService::~ChassisService()
{}

void ChassisService::launch()
{
  ROS_INFO("[BU] => 启动底盘操作服务");
  m_sub_cmd_vel = m_private_nh.subscribe("/sub_cmd_vel", 1, &ChassisService::sub_cmd_vel_callback, this);
}

/** 订阅发送给底盘的数据 */
void ChassisService::sub_cmd_vel_callback(geometry_msgs::Twist cmd_vel)
{
  CanHandler &can_handler = CanHandler::get_instance();
  json data = {
    { "linear_x", cmd_vel.linear.x },
    { "linear_y", cmd_vel.linear.y },
    { "angular", cmd_vel.angular.z }
  };
  can_handler.handle(agx::nav::Instruction::CTRL_MOTION, data);
}

}
