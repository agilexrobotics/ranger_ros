#include "bring_up.h"

namespace agx::nav {

BringUp::BringUp() : m_chassis_service(new ChassisService)
{
  ROS_INFO("[BU] => 启动底盘通信模块");

  // 启动底盘数据通信
  SocketCan &socket_can = SocketCan::get_instance();
  socket_can.launch();

  // 启动底盘操控服务
  m_chassis_service->launch();
}

BringUp::~BringUp()
{}

}
