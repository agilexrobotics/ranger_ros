#ifndef CAN_FRAME_FEEDBACK_FRAME_26X_H
#define CAN_FRAME_FEEDBACK_FRAME_26X_H

#include "can/frame/feedback/base_frame.h"
#include "agx_bringup/MotorLowArray.h"
#include <map>
#include <mutex>
#include <atomic>

namespace agx::nav {

class Frame26x : public BaseFrame
{
public:
  Frame26x();
  ~Frame26x() = default;
  virtual void parse(const struct can_frame& frame);

private:
  void check_and_publish_complete_set();
  void reset_collection();
  void json_to_motor_low(const nlohmann::json& json_data, agx_bringup::MotorLow& motor_low);
  
  ros::NodeHandle m_private_nh;
  ros::Publisher m_motor_array_pub;   
  int m_max_count;
  
  // 数据收集相关
  std::mutex m_data_mutex;
  std::map<int, nlohmann::json> m_motor_data;  
  std::atomic<int> m_received_count{0};
};

} // namespace agx::nav

#endif