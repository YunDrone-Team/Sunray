#include <ros/ros.h>
#include "rc_input.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rc_control_node");
    ros::NodeHandle nh("~");
    ros::Rate rate(20); // 20Hz

    RC_Input rc_input;
    rc_input.init(nh);

    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
