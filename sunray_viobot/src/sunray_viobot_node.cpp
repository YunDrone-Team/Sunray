#include <ros/ros.h>

#include "sunray_viobot.h"
#include <signal.h>

void mySigintHandler(int sig)
{
    ROS_INFO("[sunray_viobot_node] shut down stereo2 ...");
    ros::shutdown();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sunray_viobot_node");
    ros::NodeHandle nh("~");
    ros::Rate rate(100.0);

    signal(SIGINT, mySigintHandler);
    ros::Duration(1.0).sleep();

    VIOBOT viobot;
    viobot.init(nh, false);
    viobot.start_stereo2();

    // 主循环
    while (ros::ok())
    {
        // 回调函数
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
