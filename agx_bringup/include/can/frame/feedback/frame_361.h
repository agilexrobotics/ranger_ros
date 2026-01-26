#ifndef CAN_FRAME_FEEDBACK_FRAME_361_H
#define CAN_FRAME_FEEDBACK_FRAME_361_H

#include "can/frame/feedback/base_frame.h"
#include "agx_bringup/BMSStatus.h"
#include <ros/ros.h>
#include "can/chassis_status.h"

namespace agx::nav {

class Frame361 : public BaseFrame
{
public:
  Frame361();
  ~Frame361() = default;
  virtual void parse(const struct can_frame& frame);

private:
  double get_data(int8_t high, int8_t low);
  
  ros::NodeHandle m_private_nh;
  ros::Publisher m_bms_pub;
  int m_max_count;
};

} // namespace agx::nav

#endif