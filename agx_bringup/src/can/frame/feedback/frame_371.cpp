#include "can/frame/feedback/frame_371.h"

namespace agx::nav {

Frame371::Frame371() 
{
}

Frame371::~Frame371()
{}

void Frame371::parse(const struct can_frame& frame)
{
  try
  {
    // 充电状态 0未充电1桩充2线充
    int8_t status = static_cast<int8_t>(frame.data[0]);
    if (status == 1 || status == 2)
    {
      ROS_INFO("正在充电中");
      set_nav_detail_status(301);
    }
  }
  catch (...)
  {
    ROS_ERROR("[BU] => 解析371帧失败");
  }
}

}
