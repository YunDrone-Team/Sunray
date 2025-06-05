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
    ros::Rate rate(20.0);

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

    ros::Time start_time = ros::Time::now();
    // 主循环
    while (ros::ok())
    {
        // 回调函数
        ros::spinOnce();
        // 主循环函数
        arrived_all_goals = orca.orca_run();
        if(ros::Time::now() - start_time > ros::Duration(2.0))
        {
            if(!orca.start_flag)
            {
                cout << YELLOW << "ORCA wait for start! " << TAIL << endl;
            }
            if(orca.goal_reached_printed && ros::ok())
            {
                cout << YELLOW << "ORCA wait for goal! " << TAIL << endl;
            }
            start_time = ros::Time::now();
        }
        // sleep
        rate.sleep();
    }

    return 0;
}
