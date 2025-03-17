#include "uav_formation.h"

// sudo apt-get install libyaml-cpp-dev
int main(int argc, char** argv)
{

    // 设置日志
    Logger::init_default();
    Logger::setPrintLevel(false);
    Logger::setPrintTime(false);
    Logger::setPrintToFile(false);
    Logger::setFilename("~/Documents/Sunray_log.txt");
    
    ros::init(argc, argv, "uav_formation_node");
    ros::NodeHandle nh("~");

    UAVFormation uav_formation;
    uav_formation.init(nh);
    ros::spin();

    return 0;
}