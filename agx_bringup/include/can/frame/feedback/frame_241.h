#ifndef CAN_FRAME_FEEDBACK_FRAME_241_H
#define CAN_FRAME_FEEDBACK_FRAME_241_H

#include "can/frame/feedback/base_frame.h"
#include "agx_bringup/RemoteControlStatus.h"
#include <ros/ros.h>
#include "can/chassis_status.h"

namespace agx::nav {

class Frame241 : public BaseFrame
{
public:
  Frame241();
  ~Frame241() = default;
  virtual void parse(const struct can_frame& frame);

private:
  ros::NodeHandle m_private_nh;
  ros::Publisher m_remote_pub;
  int m_max_count;
};

} // namespace agx::nav

#endif