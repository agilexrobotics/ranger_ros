#ifndef CAN_FRAME_FEEDBACK_FRAME_231_H
#define CAN_FRAME_FEEDBACK_FRAME_231_H

#include "can/frame/feedback/base_frame.h"
#include "agx_bringup/LightControlStatus.h"
#include <ros/ros.h>
#include "can/chassis_status.h"

namespace agx::nav {

class Frame231 : public BaseFrame
{
public:
  Frame231();
  ~Frame231() = default;
  virtual void parse(const struct can_frame& frame);

private:
  ros::NodeHandle m_private_nh;
  ros::Publisher m_light_pub;
  int m_max_count;
};

} // namespace agx::nav

#endif