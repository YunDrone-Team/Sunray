#include "ground_control.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ground_control");
    ros::NodeHandle nh("~");

    GroundControl ground_control;
    ground_control.init(nh);

    ros::spin();

    return 0;
}