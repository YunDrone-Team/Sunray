#include "Waypoint.h"

// 中断信号
void mySigintHandler(int sig)
{
    ROS_INFO("[waypoint_mission_node] exit...");
    ros::shutdown();
    exit(0);
}

int main(int argc, char **argv)
{
    // 设置日志
    Logger::init_default();
 
    Logger::setPrintToFile(false);
    Logger::setFilename("~/Documents/Sunray_log.txt");

    ros::init(argc, argv, "waypoint_mission_node");
    ros::NodeHandle nh("~");
    ros::Rate rate(100.0);
    bool flag_printf = false; // 是否打印状态
    nh.param<bool>("flag_printf", flag_printf, true);  

    signal(SIGINT, mySigintHandler);
    ros::Duration(1.0).sleep();

    // 初始化航点任务类
    Waypoint wp;
    wp.init(nh);

    // 初始化检查：等待PX4连接
    int trials = 0;
    while (ros::ok() && !wp.uav_state.connected)
    {
        ros::spinOnce();
        ros::Duration(1.0).sleep();
        if (trials++ > 5)
            Logger::error("Unable to connnect to PX4!!!");
    }

    ros::Time last_time = ros::Time::now();
    // 主循环
    while (ros::ok())
    {
        ros::spinOnce();
        
        // 控制类主循环函数
        wp.mainLoop();

        // 定时打印状态
        if (ros::Time::now() - last_time > ros::Duration(2.0) && flag_printf)
        {
            wp.show_state();
            last_time = ros::Time::now();
        }

        rate.sleep();
    }

    return 0;
}