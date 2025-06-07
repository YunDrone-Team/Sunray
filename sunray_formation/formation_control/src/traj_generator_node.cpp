#include "traj_generator.h"
#include <ros/ros.h>

// // sudo apt-get install libyaml-cpp-dev
int main(int argc, char **argv)
{
    ros::init(argc, argv, "traj_generator_node");
    ros::NodeHandle nh("~");

    traj_generator tg(6);
    tg.save_trajectory_flag = true;
    tg.circle_trajectory(1.5, 1.5, 0.25, 0, 0);
    tg.figure_eight(2.5, 0.8, 0.25, 0, 0);
    // ros::spin();

    return 0;
}