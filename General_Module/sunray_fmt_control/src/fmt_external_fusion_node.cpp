#include "FMTexternalFusion.h"

// 中断信号处理
void mySigintHandler(int sig)
{
    ROS_INFO("[fmt_external_fusion_node] exit...");
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

    ros::init(argc, argv, "fmt_external_fusion_node");
    ros::NodeHandle nh("~");
    ros::Rate rate(50.0);
    bool flag_printf = false; // 是否打印状态
    nh.param<bool>("flag_printf", flag_printf, true);           

    // 中断信号注册 
    signal(SIGINT, mySigintHandler);

    // 初始化FMT外部估计类
    FMTExternalFusion fmt_external_fusion;
    fmt_external_fusion.init(nh);

    // 初始化检查：等待FMT连接
    int trials = 0;
    while (ros::ok() && !fmt_external_fusion.fmt_state.connected)
    {
        ros::spinOnce();
        ros::Duration(1.0).sleep();
        if (trials++ > 5)
            Logger::print_color(int(LogColor::red), "Unable to connnect to FMT!!!");
    }

    ros::Time last_time = ros::Time::now();
    // 主循环
    while (ros::ok())
    {
        ros::spinOnce();

        // 定时打印状态
        if (ros::Time::now() - last_time > ros::Duration(1.0) && flag_printf)
        {
            fmt_external_fusion.show_fmt_state();
            last_time = ros::Time::now();
        }

        rate.sleep();
    }

    return 0;
}
