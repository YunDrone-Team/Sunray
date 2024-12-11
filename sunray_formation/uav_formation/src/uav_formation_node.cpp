#include "uav_formation.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "uav_formation_node");
    ros::NodeHandle nh("~");

    UAV_formation uav_formation();

    ros::spin();

    return 0;
}