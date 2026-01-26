#ifndef SOCKET_CAN_H_
#define SOCKET_CAN_H_

#include <ros/ros.h>
#include <iostream>
#include <thread>
#include <map>
#include <cstring>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include "can/chassis_status.h"
#include "can/frame/feedback/frame_211.h"
#include "can/frame/feedback/frame_221.h"
#include "can/frame/feedback/frame_231.h"
#include "can/frame/feedback/frame_241.h"
#include "can/frame/feedback/frame_271.h"
#include "can/frame/feedback/frame_281.h"
#include "can/frame/feedback/frame_291.h"
#include "can/frame/feedback/frame_311.h"
#include "can/frame/feedback/frame_312.h"
#include "can/frame/feedback/frame_361.h"
#include "can/frame/feedback/frame_362.h"
#include "can/frame/feedback/frame_371.h"
#include "can/frame/feedback/frame_25x.h"
#include "can/frame/feedback/frame_26x.h"

using json = nlohmann::json;

namespace agx::nav {

/**
 * can通信
 */
class SocketCan
{
public:
  static SocketCan &get_instance() {
    static SocketCan s;
    return s;
  }
  /** @brief 启动 */
  void launch();
  /**
   * @brief 发送帧数据
   * @param[in] frame 发送数据
   */
  void send_frame(const struct can_frame& frame);

private:
  SocketCan();
  ~SocketCan();
  SocketCan(const SocketCan &) = delete;
  SocketCan(const SocketCan &&) = delete;
  SocketCan &operator=(const SocketCan &) = delete;
  /** @brief 初始化can */
  void init_can();
  /** @brief 初始化报文解析 */
  void init_frame();
  /** @brief 在线程中启动 */
  void launch_thread();
  /**
   * @brief 接收帧数据
   * @return 帧数据
   */
  struct can_frame receive_frame();

private:
  ros::NodeHandle m_private_nh;
  std::map<int, std::unique_ptr<BaseFrame>> m_frame_map;
  std::unique_ptr<Frame25x> m_frame_25x;  // 处理所有251-258帧
  std::unique_ptr<Frame26x> m_frame_26x;  // 处理所有261-268帧
  int m_sock;
  bool m_debug;
  std::string m_interface;
};

}

#endif