#include "can/frame/handler/handler_441.h"

namespace agx::nav {

Handler441::Handler441()
{}

Handler441::~Handler441()
{}

/**
 * @brief 处理消息报文
 * @param[in] data 消息格式为{
 *   "inst":     
 * }
 */
void Handler441::handle(const json& data)
{
  try
  {
    SocketCan &socket_can = SocketCan::get_instance();
    int8_t inst = data["inst"];
    struct can_frame frame = { 0 };
    get_can_frame(frame, agx::nav::Instruction::CLEAR_FAULT, inst, zero, zero, zero, zero, zero, zero, zero);
    socket_can.send_frame(frame);
  }
  catch (...)
  {
    ROS_ERROR("[BU] => 写441帧失败");
  }
}

}
