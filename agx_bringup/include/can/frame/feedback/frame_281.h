#ifndef CAN_FRAME_FEEDBACK_FRAME_281_H
#define CAN_FRAME_FEEDBACK_FRAME_281_H

#include "can/frame/feedback/base_frame.h"
#include "agx_bringup/WheelSpeeds.h"
#include <ros/ros.h>
#include "can/chassis_status.h"

namespace agx::nav {

class Frame281 : public BaseFrame
{
public:
  Frame281();
  ~Frame281() = default;
  virtual void parse(const struct can_frame& frame);

private:
  double get_data(int8_t high, int8_t low);
  
  ros::NodeHandle m_private_nh;
  ros::Publisher m_steering_pub;
  int m_max_count;
};

} // namespace agx::nav

#endif