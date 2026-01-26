#ifndef CAN_FRAME_FEEDBACK_FRAME_221_H
#define CAN_FRAME_FEEDBACK_FRAME_221_H

#include "can/frame/feedback/base_frame.h"
#include "agx_bringup/ChassisMotion.h"
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include "can/chassis_status.h"

namespace agx::nav {

class Frame221 : public BaseFrame
{
public:
  Frame221();
  ~Frame221() = default;
  virtual void parse(const struct can_frame& frame);

private:
  double get_data(int8_t high, int8_t low);
  
  ros::NodeHandle m_private_nh;
  ros::Publisher m_chassis_pub; 
  int m_max_count;
};

} // namespace agx::nav

#endif