#include "can/frame/handler/handler_121.h"

namespace agx::nav {

Handler121::Handler121()
{}

Handler121::~Handler121()
{}

/**
 * @brief 处理消息报文
 * @param[in] data 消息格式为 
 *   "enable":,   // 0无效1使能
 *   "value":     // 0常关1常开
 */
void Handler121::handle(const json& data)
{
  try
  {
    SocketCan &socket_can = SocketCan::get_instance();
    int8_t enable = data["enable"];
    int8_t value = data["value"];
    struct can_frame frame = { 0 };
    get_can_frame(frame, agx::nav::Instruction::LIGHT_CTRL, enable, value, zero, zero, zero, zero, zero, zero);
    socket_can.send_frame(frame);
  }
  catch (...)
  {
    ROS_ERROR("[BU] => 写121帧失败");
  }
}

}
