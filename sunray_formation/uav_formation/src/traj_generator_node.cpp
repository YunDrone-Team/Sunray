#include "traj_generator.h"
#include "printf_format.h"
#include <ros/ros.h>

using namespace sunray_logger;
// // sudo apt-get install libyaml-cpp-dev
int main(int argc, char **argv)
{

    // 设置日志
    Logger::init_default();
    Logger::setPrintLevel(false);
    Logger::setPrintTime(false);
    Logger::setPrintToFile(false);
    Logger::setFilename("~/Documents/Sunray_log.txt");

    ros::init(argc, argv, "traj_generator_node");
    ros::NodeHandle nh("~");

    traj_generator tg(6);
    tg.save_trajectory_flag = true;
    tg.circle_trajectory(1.5, 1.5, 0.25, 0, 0);
    tg.figure_eight(2.5, 0.8, 0.25, 0, 0);
    // ros::spin();

    return 0;
}