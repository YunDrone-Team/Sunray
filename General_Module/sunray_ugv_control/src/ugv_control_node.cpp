#include "ugv_control.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ugv_control");
    ros::NodeHandle nh("~");

    UGV_CONTROL ugv_control;
    ugv_control.init(nh);

    ros::Rate loop_rate(50);
    while (ros::ok())
    {
        ros::spinOnce();
        ugv_control.mainloop();
        loop_rate.sleep();
    }
    return 0;
}