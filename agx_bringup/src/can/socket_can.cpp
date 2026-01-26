#include "can/socket_can.h"

namespace agx::nav {

SocketCan::SocketCan(): m_private_nh("~")
{
  m_private_nh.param<bool>("debug", m_debug, false);
  m_private_nh.param<std::string>("interface", m_interface, "can0");

  init_can();
  init_frame();
}

SocketCan::~SocketCan()
{
  close(m_sock);
}

/** @brief 启动 */
void SocketCan::launch()
{
  ROS_INFO("[BU] => 启动can通信模块");
  std::thread socket_thread(&SocketCan::launch_thread, this);
  socket_thread.detach();
}

/** @brief 在线程中启动 */
void SocketCan::launch_thread()
{
  ROS_INFO("[BU] => 启动can采集线程");
  while (ros::ok())
  {
    struct can_frame frame = receive_frame();
    
    if (m_debug)
    {
      std::cout << "Received CAN frame: ID=0x" << std::hex << frame.can_id
          << ", DLC=" << std::dec << static_cast<int>(frame.can_dlc)
          << ", Data=";
      for (int i = 0; i < frame.can_dlc; i++)
      {
        std::cout << std::hex << static_cast<int>(frame.data[i]) << " ";
      }
      std::cout << std::endl;
    }

    // 解析can数据
    // 251-258帧由m_frame_25x处理
    if (frame.can_id >= agx::nav::Instruction::MOTOR_DRIVE_HIGH_1 && frame.can_id <= agx::nav::Instruction::MOTOR_DRIVE_HIGH_8) {
      m_frame_25x->parse(frame);
    }
    // 261-268帧由m_frame_26x处理
    else if (frame.can_id >= agx::nav::Instruction::MOTOR_DRIVE_LOW_1 && frame.can_id <= agx::nav::Instruction::MOTOR_DRIVE_LOW_8) {
      m_frame_26x->parse(frame);
    }
    // 其他帧使用映射表处理
    else {
      auto it = m_frame_map.find(frame.can_id);  
      if (it != m_frame_map.end())
      {
        it->second->parse(frame);
      }
    }

    ros::spinOnce();
  }
}

/**
 * @brief 发送帧数据
 * @param[in] frame 发送数据
 */
void SocketCan::send_frame(const struct can_frame& frame)
{
  if (write(m_sock, &frame, sizeof(frame)) != sizeof(frame))
  {
    ROS_ERROR("[BU] => Error sending CAN frame");
  }
}

/**
 * @brief 接收帧数据
 * @return 帧数据
 */
struct can_frame SocketCan::receive_frame()
{
  struct can_frame frame;
  int nbytes = read(m_sock, &frame, sizeof(frame));
  if (nbytes < 0) {
    ROS_ERROR("[BU] => Error reading CAN frame");
  }
  return frame;
}

/** @brief 初始化can */
void SocketCan::init_can()
{
  // 创建套接字
  if ((m_sock = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
  {
    ROS_ERROR("[BU] => Error while opening socket");
    throw std::runtime_error("Error while opening socket");
  }

  // 指定 CAN 接口
  struct ifreq ifr;
  std::strncpy(ifr.ifr_name, m_interface.c_str(), IFNAMSIZ - 1);
  if (ioctl(m_sock, SIOCGIFINDEX, &ifr) < 0)
  {
    ROS_ERROR("[BU] => Error getting interface index");
    throw std::runtime_error("Error getting interface index");
  }

  // 绑定套接字到 CAN 接口
  struct sockaddr_can addr;
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (bind(m_sock, (struct sockaddr*)&addr, sizeof(addr)) < 0)
  {
    ROS_ERROR("[BU] => Error binding socket to interface");
    throw std::runtime_error("Error binding socket to interface");
  }
}

/** @brief 初始化报文解析 */
void SocketCan::init_frame()
{
  // 创建聚合处理器
  m_frame_25x = std::make_unique<Frame25x>();
  m_frame_26x = std::make_unique<Frame26x>();
  
  // 初始化其他帧处理器
  std::unique_ptr<Frame211> frame_211(new Frame211());
  m_frame_map.insert(std::make_pair(agx::nav::Instruction::SYS_STATUS, std::move(frame_211)));
  
  std::unique_ptr<Frame221> frame_221(new Frame221());
  m_frame_map.insert(std::make_pair(agx::nav::Instruction::MOTION, std::move(frame_221)));
  
  std::unique_ptr<Frame231> frame_231(new Frame231());
  m_frame_map.insert(std::make_pair(agx::nav::Instruction::LIGHT_STATUS, std::move(frame_231)));
  
  std::unique_ptr<Frame241> frame_241(new Frame241());
  m_frame_map.insert(std::make_pair(agx::nav::Instruction::REMOTE_CONTROL_STATUS, std::move(frame_241)));
  
  std::unique_ptr<Frame271> frame_271(new Frame271());
  m_frame_map.insert(std::make_pair(agx::nav::Instruction::TURN_ANGLE, std::move(frame_271)));
  
  std::unique_ptr<Frame281> frame_281(new Frame281());
  m_frame_map.insert(std::make_pair(agx::nav::Instruction::TURN_SPEED, std::move(frame_281)));
  
  std::unique_ptr<Frame291> frame_291(new Frame291());
  m_frame_map.insert(std::make_pair(agx::nav::Instruction::MOTION_MODE, std::move(frame_291)));
  
  std::unique_ptr<Frame311> frame_311(new Frame311());
  m_frame_map.insert(std::make_pair(agx::nav::Instruction::ODOM_FRONT, std::move(frame_311)));

  std::unique_ptr<Frame312> frame_312(new Frame312());
  m_frame_map.insert(std::make_pair(agx::nav::Instruction::ODOM_BACK, std::move(frame_312)));
  
  std::unique_ptr<Frame361> frame_361(new Frame361());
  m_frame_map.insert(std::make_pair(agx::nav::Instruction::BMS_ALL, std::move(frame_361)));
  
  std::unique_ptr<Frame362> frame_362(new Frame362());
  m_frame_map.insert(std::make_pair(agx::nav::Instruction::BMS_ALL_STATUS, std::move(frame_362)));
  
  std::unique_ptr<Frame371> frame_371(new Frame371());
  m_frame_map.insert(std::make_pair(agx::nav::Instruction::CHARGER, std::move(frame_371)));
}

}