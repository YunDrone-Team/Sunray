/*
Rviz Goal Bridge Node - Main Function
独立的rviz 2D Nav Goal桥接节点主函数

功能描述：
- 作为ROS节点运行
- 订阅rviz的2D Nav Goal
- 转换为UAVControlCMD并发布给UAV控制系统
- 提供安全检查和参数配置

使用方法：
rosrun sunray_uav_control rviz_goal_bridge_node

或通过launch文件启动：
roslaunch sunray_uav_control rviz_goal_bridge.launch
*/

#include <ros/ros.h>
#include "RvizGoalBridge.h"

int main(int argc, char **argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "rviz_goal_bridge_node");
    ros::NodeHandle nh("~");
    
    ROS_INFO("Starting Rviz Goal Bridge Node...");
    
    try
    {
        // 创建桥接对象
        RvizGoalBridge bridge;
        
        // 初始化
        bridge.init(nh);
        
        ROS_INFO("Rviz Goal Bridge Node initialized successfully!");
        ROS_INFO("Listening for 2D Nav Goals from rviz...");
        
        // 运行节点
        bridge.run();
    }
    catch (const std::exception& e)
    {
        ROS_ERROR("Exception in Rviz Goal Bridge Node: %s", e.what());
        return -1;
    }
    catch (...)
    {
        ROS_ERROR("Unknown exception in Rviz Goal Bridge Node");
        return -1;
    }
    
    ROS_INFO("Rviz Goal Bridge Node shutting down...");
    return 0;
}
