#include "can/frame/handler/handler_141.h"

namespace agx::nav {

Handler141::Handler141()
{}

Handler141::~Handler141()
{}

/**
 * @brief 处理消息报文
 * @param[in] data 消息格式为{
 *   "model":     
 * }
 */
void Handler141::handle(const json& data)
{
  try
  {
    SocketCan &socket_can = SocketCan::get_instance();
    int8_t model = data["model"];
    struct can_frame frame = { 0 };
    get_can_frame(frame, agx::nav::Instruction::MOTION_MODEL, model, zero, zero, zero, zero, zero, zero, zero);
    socket_can.send_frame(frame);
  }
  catch (...)
  {
    ROS_ERROR("[BU] => 写141帧失败");
  }
}

}
