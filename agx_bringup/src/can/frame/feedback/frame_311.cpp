#include "can/frame/feedback/frame_311.h"

namespace agx::nav
{

  Frame311::Frame311() : m_private_nh("~")
  {
    // 获取话题名称参数
    m_private_nh.param<std::string>("front_wheel_odom_topic", m_odom_topic, "/front_wheel_odom");

    // 初始化发布器
    m_odom_pub = m_nh.advertise<agx_bringup::FrontWheelOdometry>(m_odom_topic, 10);

    // 初始化里程计值
    m_left_odom = 0.0;
    m_right_odom = 0.0;
  }

  void Frame311::parse(const struct can_frame &frame)
  {
    try
    {
      // 处理里程计
      m_left_odom = (double)((int32_t)((frame.data[3] & 0xff) | (frame.data[2] << 8) | (frame.data[1] << 16) | (frame.data[0] << 24))) / 1000.0;
      m_right_odom = (double)((int32_t)((frame.data[7] & 0xff) | (frame.data[6] << 8) | (frame.data[5] << 16) | (frame.data[4] << 24))) / 1000.0;

      // 发布消息
      agx_bringup::FrontWheelOdometry odom_msg;
      odom_msg.left_wheel_odom = m_left_odom;
      odom_msg.right_wheel_odom = m_right_odom;

      m_odom_pub.publish(odom_msg);

      ROS_DEBUG_THROTTLE(1.0,
                         "前轮里程计 - 左轮: %.3f m, 右轮: %.3f m",
                         m_left_odom, m_right_odom);
    }
    catch (...)
    {
      ROS_ERROR("[BU] => 解析311帧失败");
    }
  }

}
