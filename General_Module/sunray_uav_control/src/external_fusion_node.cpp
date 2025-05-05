#include "externalFusion.h"
#include <signal.h>

// 中断信号
void mySigintHandler(int sig)
{
    ROS_INFO("[external_fusion_node] exit...");
    ros::shutdown();
    exit(0);
}

int main(int argc, char **argv)
{
    // 设置日志
    Logger::init_default();
    Logger::setPrintLevel(false);
    Logger::setPrintTime(false);
    Logger::setPrintToFile(false);
    Logger::setFilename("~/Documents/Sunray_log.txt");

    ros::init(argc, argv, "external_fusion_node");
    ros::NodeHandle nh("~");
    ros::Rate rate(200.0);
    bool flag_printf = false; // 是否打印状态
    nh.param<bool>("flag_printf", flag_printf, true);           

    // 中断信号注册 
    signal(SIGINT, mySigintHandler);
    ExternalFusion external_fusion;
    external_fusion.init(nh);
    ros::Time now = ros::Time::now();

    // 主循环
    while (ros::ok)
    {
        ros::spinOnce();

        // 定时打印状态
        if (ros::Time::now() - now > ros::Duration(1.0) && flag_printf)
        {
            external_fusion.show_px4_state();
            now = ros::Time::now();
        }

        rate.sleep();
    }

    return 0;
}