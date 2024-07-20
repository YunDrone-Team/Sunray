#include <ros/ros.h>
#include <signal.h>

#include "wheeltec_robot.h"

void mySigintHandler(int sig)
{
    ROS_INFO("[control_node] exit...");
    ros::shutdown();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "wheeltec_control_node");
    ros::NodeHandle nh("~");
    ros::Rate rate(20.0);

    signal(SIGINT, mySigintHandler);
    ros::Duration(1.0).sleep();

    // 无人车控制类
    WheeltecRobot wheeltec_robot;
    wheeltec_robot.init(nh, true);

    // 无人车规划类
    


    // 主循环
    while (ros::ok())
    {
        // 回调函数
        ros::spinOnce();
        // 主循环函数
        wheeltec_robot.mainloop();
        rate.sleep();
    }

    return 0;
}
