#include <ros/ros.h>
#include <signal.h>

#include "orca.h"

void mySigintHandler(int sig)
{
    ROS_INFO("[orca_node] exit...");
    ros::shutdown();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "orca_node");
    ros::NodeHandle nh("~");
    ros::Rate rate(30.0);

    signal(SIGINT, mySigintHandler);
    ros::Duration(1.0).sleep();

    bool flag_printf;
    nh.param<bool>("flag_printf", flag_printf, true);

    // 控制器
    ORCA orca;
    orca.init(nh);

    ros::spinOnce();
    ros::Duration(1.0).sleep();

    bool arrived_all_goals{false};

    while(!orca.start_flag && ros::ok())
    {
        // 回调函数
        ros::spinOnce();
        cout << YELLOW << "ORCA wait for start! " << TAIL << endl;
        // sleep
        sleep(2.0);
    }

    // 主循环
    while (ros::ok())
    {
        // 回调函数
        ros::spinOnce();
        // 主循环函数
        arrived_all_goals = orca.orca_run();
        // sleep
        rate.sleep();
    }

    return 0;
}
