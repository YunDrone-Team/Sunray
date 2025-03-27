#include "map_generator.h"



int main(int argc, char **argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "map_generator");
    // 创建ROS节点句柄
    ros::NodeHandle nh;

    MapGenerator mg;
    mg.init(nh, -5, -5, 5, 5, 0.1, 0.1);

    ros::spinOnce();
    // Publish the Octomap
    ros::Rate loop_rate(1); // Adjust the publishing rate as needed
    while (ros::ok())
    {
        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}