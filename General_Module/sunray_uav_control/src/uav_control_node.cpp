#include "UAVControl.h"

int main(int argc, char **argv)
{
    // 设置日志
    Logger::init_default();
    Logger::setPrintLevel(false);
    Logger::setPrintTime(false);
    Logger::setPrintToFile(false);
    Logger::setFilename("~/Documents/Sunray_log.txt");

    ros::init(argc, argv, "uav_control_node");
    ros::NodeHandle nh("~");
    ros::Rate rate(100.0);
    
    // 声明控制类
    UAVControl uav_ctrl;
    // 控制类初始化
    uav_ctrl.init(nh);

    // 初始化检查：等待PX4连接
    int trials = 0;
    while (ros::ok() && !uav_ctrl.px4_state.connected)
    {
        ros::spinOnce();
        ros::Duration(1.0).sleep();
        if (trials++ > 5)
            ROS_ERROR("Unable to connnect to PX4!!!");
    }

    // 主循环
    while (ros::ok())
    {
        ros::spinOnce();
        
        // 控制类主循环函数
        uav_ctrl.mainLoop();

        rate.sleep();
    }

    return 0;
}