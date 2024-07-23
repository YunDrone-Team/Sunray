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
    ros::Rate rate(100.0);

    bool flag_printf;
    nh.param<bool>("flag_printf", flag_printf, false);

    signal(SIGINT, mySigintHandler);
    ros::Duration(1.0).sleep();

    // 外部定位数据
    VISION_POSE vision_pose;
    vision_pose.init(nh);

    ros::Duration(1.0).sleep();

    ros::Time time_now = ros::Time::now();
    ros::Time time_last = ros::Time::now();
    time_last.sec = time_last.sec - 10;

    // 主循环
    while (ros::ok())
    {
        // 回调函数
        ros::spinOnce();
        rate.sleep();

        // 定时状态打印
        time_now = ros::Time::now();
        if ((time_now - time_last).toSec() > 1.0 /* 秒 */ && flag_printf)
        {
            vision_pose.printf_debug_info();
            vision_pose.check_timeout();
            time_last = time_now;
        }
    }

    return 0;
}
