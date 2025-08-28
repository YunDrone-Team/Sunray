#include <ros/ros.h>
#include "rc_input.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rc_control_node");
    ros::NodeHandle nh("~");
    ros::Rate rate(20); // 20Hz

    int uav_id;			   // 无人机编号
	std::string uav_name; // 无人机名称

	nh.param<int>("uav_id", uav_id, 1);
	nh.param<std::string>("uav_name", uav_name, "uav");

    RC_Input rc_input;
    rc_input.init(nh, uav_id);

    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
