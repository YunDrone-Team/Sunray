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
    ros::Rate rate(50);

    // 中断信号注册
    signal(SIGINT, mySigintHandler);
    ExternalFusion external_fusion;
    external_fusion.init(nh);
    ros::Time now = ros::Time::now();

    // 主循环
    while (ros::ok)
    {
        ros::spinOnce();

        if (ros::Time::now() - now > ros::Duration(1.0))
        {
            external_fusion.show_px4_state();
            now = ros::Time::now();
        }

        rate.sleep();
    }

    return 0;
}