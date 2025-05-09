#include "communication_bridge.h"
#include <signal.h>

void mySigintHandler(int sig)
{
    ROS_INFO("[ground_control_node] exit...");
    ros::shutdown();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ground_control");
    ros::NodeHandle nh("~");

    signal(SIGINT, mySigintHandler);

    communication_bridge ground_control;
    ground_control.init(nh);

    // ros::spin();    100Hz太快
    ros::Rate loop_rate(20); // 设置刷新率为20Hz

    while (ros::ok()) 
    {

        ros::spinOnce();
        loop_rate.sleep();
        
    }

    return 0;
}