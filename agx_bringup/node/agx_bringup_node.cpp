#include "bring_up.h"
#include <ros/ros.h>
#include "global_state.hpp"
int main(int argc, char **argv)
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "agx_bringup_node");
    
    // 创建节点句柄
    ros::NodeHandle nh;
    
    try {
        GlobalState::getInstance().initialize(nh);
        
        ROS_INFO("GlobalState 初始化成功");
        
        agx::nav::BringUp bring_up;
        
        ROS_INFO("AGX 启动节点已启动");
        
        ros::spin();
    }
    catch (const std::exception& e) {
        ROS_ERROR("启动节点初始化失败: %s", e.what());
        return -1;
    }
    
    return 0;
}