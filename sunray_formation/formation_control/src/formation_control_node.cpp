#include "formation_control.h"

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

    SunrayFormation formation_control;
    formation_control.init(nh);
    ros::spin();

    return 0;
}