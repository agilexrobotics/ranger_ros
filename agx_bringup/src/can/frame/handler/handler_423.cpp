#include "can/frame/handler/handler_423.h"

namespace agx::nav {

Handler423::Handler423()
{}

Handler423::~Handler423()
{}

/**
 * @brief 处理消息报文
 * @param[in] data 消息格式为{
 *   "mode":     // 0电流 1电压
 * }
 */
void Handler423::handle(const json& data)
{
  try
  {
    SocketCan &socket_can = SocketCan::get_instance();
    int8_t mode = data["mode"];
    if (mode != 0 || mode != 1)
    {
      ROS_ERROR("[BU] => 控制模式设置错误，不是0或1");
      return;
    }
    struct can_frame frame = { 0 };
    get_can_frame(frame, agx::nav::Instruction::CTRL_SWITCH, mode, zero, zero, zero, zero, zero, zero, zero);
    socket_can.send_frame(frame);
  }
  catch (...)
  {
    ROS_ERROR("[BU] => 写423帧失败");
  }
}

}
