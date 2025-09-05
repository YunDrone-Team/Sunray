#include "rc_input.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rc_input_node");
    ros::NodeHandle nh("~");
    ros::Rate rate(50.0); 

    int uav_id;			   
	std::string uav_name; 
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
