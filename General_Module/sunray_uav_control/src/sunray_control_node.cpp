#include <ros/ros.h>
#include <signal.h>

#include "uav_control.h"
#include "vision_pose.h"

void mySigintHandler(int sig)
{
    ROS_INFO("[sunray_control_node] exit...");
    ros::shutdown();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sunray_control_node");
    ros::NodeHandle nh("~");
    ros::Rate rate(100.0);

    signal(SIGINT, mySigintHandler);
    ros::Duration(1.0).sleep();

    bool sim_mode, flag_printf;
    nh.param<bool>("sim_mode", sim_mode, true);
    nh.param<bool>("flag_printf", flag_printf, true);

    // 外部定位数据
    // VISION_POSE vision_pose;
    // vision_pose.init(nh);

    // 控制器
    UAVControl uav_control;
    uav_control.init(nh);

    ros::spinOnce();
    ros::Duration(1.0).sleep();

    ros::Time time_now = ros::Time::now();
    ros::Time time_last = ros::Time::now();
    time_last.sec = time_last.sec - 10;

    // 主循环
    while (ros::ok())
    {
        // 回调函数
        ros::spinOnce();
        // 主循环函数
        uav_control.mainloop();

        rate.sleep();
    }

    return 0;
}
