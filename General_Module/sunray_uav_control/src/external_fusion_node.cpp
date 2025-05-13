#include "externalFusion.h"

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

    // 初始化外部估计类
    ExternalFusion external_fusion;
    external_fusion.init(nh);

    // 初始化检查：等待PX4连接
    int trials = 0;
    while (ros::ok() && !external_fusion.px4_state.connected)
    {
        ros::spinOnce();
        ros::Duration(1.0).sleep();
        if (trials++ > 5)
            Logger::print_color(int(LogColor::red), "Unable to connnect to PX4!!!");
    }

    ros::Time last_time = ros::Time::now();
    // 主循环
    while (ros::ok)
    {
        ros::spinOnce();

        // 定时打印状态
        if (ros::Time::now() - last_time > ros::Duration(1.0) && flag_printf)
        {
            external_fusion.show_px4_state();
            last_time = ros::Time::now();
        }

        rate.sleep();
    }

    return 0;
}