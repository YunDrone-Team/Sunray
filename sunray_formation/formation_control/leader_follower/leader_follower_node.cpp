#include "leader_follower.h"

// 中断信号
void mySigintHandler(int sig)
{
    ROS_INFO("[leader_follower_node] exit...");
    ros::shutdown();
    exit(0);
}


int main(int argc, char** argv)
{
    // 设置日志
    Logger::init_default();
    Logger::setPrintToFile(false);
    Logger::setFilename("~/Documents/Sunray_log.txt");
    
    ros::init(argc, argv, "leader_follower_node");
    ros::NodeHandle nh("~");
    ros::Rate rate(50.0);
    bool flag_printf = false; // 是否打印状态
    nh.param<bool>("flag_printf", flag_printf, true);  

    // 初始化无人机控制类
    LeaderFollower leader_follower;
    leader_follower.init(nh);

    // 初始化检查：等待PX4连接
    int trials = 0;
    while (ros::ok() && !leader_follower.formation.communication_timeout)
    {
        ros::spinOnce();
        ros::Duration(1.0).sleep();
        if (trials++ > 5)
            Logger::error("UAV", leader_follower.uav_id, "unable to connnect to PX4!!!");
    }
    Logger::info("UAV", leader_follower.uav_id,"connected.");

    ros::Time last_time = ros::Time::now();
    leader_follower.show_debug_info();
    // 主循环
    while (ros::ok())
    {
        // 回调函数
        ros::spinOnce();
        
        // 主循环函数
        leader_follower.mainLoop();

        // 定时打印状态
        if (ros::Time::now() - last_time > ros::Duration(5.0) && flag_printf)
        {
            leader_follower.show_debug_info();
            last_time = ros::Time::now();
        }

        rate.sleep();
    }

    return 0;
}