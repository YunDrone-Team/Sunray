#include <ros/ros.h>
#include <signal.h>

#include "vision_pose.h"

void mySigintHandler(int sig)
{
    ROS_INFO("[vision_pose_node] exit...");
    ros::shutdown();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vision_pose_node");
    ros::NodeHandle nh("~");
    ros::Rate rate(50.0);

    bool flag_printf;
    nh.param<bool>("flag_printf", flag_printf, false);

    signal(SIGINT, mySigintHandler);

    // 外部定位数据
    VISION_POSE vision_pose;
    vision_pose.init(nh);

    ros::Time time_now = ros::Time::now();
    ros::Time time_last = ros::Time::now();
    time_last.sec = time_last.sec + 2;          // 解决显示问题 

    ros::Duration(1.0).sleep();
    // 主循环
    while (ros::ok())
    {
        // 回调函数
        ros::spinOnce();

        // 定时状态打印
        time_now = ros::Time::now();
        if ((time_now - time_last).toSec() > 1.0 /* 秒 */ && flag_printf)
        {
            vision_pose.printf_debug_info();
            time_last = time_now;
        }

        rate.sleep();
    }

    return 0;
}
