#include <ros/ros.h>
#include "printf_format.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "printf_format_node");
    ros::NodeHandle nh;

    sunray_logger::Logger::init_default();
    sunray_logger::Logger::setPrintLevel(true);
    sunray_logger::Logger::setPrintTime(true);
    sunray_logger::Logger::setPrintToFile(true);
    sunray_logger::Logger::setFilename("/home/yundrone/Sunray/General_Module/sunray_uav_control/test/log.txt");

    sunray_logger::Logger::print_color(int(LogColor::blue),LOG_BOLD, 123.456789, 123.456789);
    // for(int i = 0; i < 20000; i++)
    // {
    //     sunray_logger::Logger::info(1);
    //     sunray_logger::Logger::warning(1111);
    //     sunray_logger::Logger::error(111.11);
    //     sunray_logger::Logger::print(111.1111);
    //     sunray_logger::Logger::print_color(int(LogColor::blue),111.11111);
    // }

    // ros::spin();
    return 0;
}
