#include "UAVControl.h"

int main(int argc, char **argv)
{
    // 设置日志
    Logger::init_default();
    Logger::setPrintLevel(false);
    Logger::setPrintTime(false);
    Logger::setPrintToFile(false);
    Logger::setFilename("~/Documents/Sunray_log.txt");

    ros::init(argc, argv, "UAVControl");
    ros::NodeHandle nh("~");
    ros::Rate rate(50.0);
    
    // 声明控制类
    UAVControl uav_ctrl;
    // 控制类初始化
    uav_ctrl.init(nh);

    // 主循环
    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
        // 控制类主循环函数
        uav_ctrl.mainLoop();
    }

    return 0;
}