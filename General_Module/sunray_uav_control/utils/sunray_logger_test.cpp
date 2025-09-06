#include "ros_msg_utils.h"

int main(int argc, char **argv)
{
    // 本程序为Logger的测试程序



    // 初始化日志变量
    Logger::init_default();
    // 是否打印level（建议设为false，意义不大）
    Logger::setPrintLevel(true);
    // 是否打印时间（建议设为false，意义不大）
    Logger::setPrintTime(true);
    // 小数点精度
    Logger::setPrecision(2);
    // 分隔符
    Logger::setSeparator("");
    // 是否存储到文件，建议选为false（如果不需要记录日志的话）
    Logger::setPrintToFile(true);
    // 设置日志存储地址
    Logger::setFilename("~/Documents/Sunray_log.txt");

    ros::init(argc, argv, "sunray_logger_test");
    ros::NodeHandle nh("~");
    ros::Rate rate(100.0);


    // DEBUG: 颜色为绿色
    Logger::info("This is Logger::info.");
    // WARNING: 颜色为黄色
    Logger::warning("This is Logger::warning.");
    // ERROR：颜色为红色
    Logger::error("This is Logger::error.");

    // 指定颜色打印，颜色定义参见LogColor枚举
    Logger::print_color(int(LogColor::red), "This color is LogColor::red");
    Logger::print_color(int(LogColor::green), "This color is LogColor::green");
    Logger::print_color(int(LogColor::yellow), "This color is LogColor::yellow");
    Logger::print_color(int(LogColor::blue), "This color is LogColor::blue");
    Logger::print_color(int(LogColor::magenta), "This color is LogColor::magenta");
    Logger::print_color(int(LogColor::cyan), "This color is LogColor::cyan");
    Logger::print_color(int(LogColor::white), "This color is LogColor::white");
    Logger::print_color(int(LogColor::black), "This color is LogColor::black");
    Logger::print_color(int(LogColor::white_bg_green), "This color is LogColor::white_bg_green");
    Logger::print_color(int(LogColor::white_bg_red), "This color is LogColor::white_bg_red");
    Logger::print_color(int(LogColor::white_bg_yellow), "This color is LogColor::white_bg_yellow");
    Logger::print_color(int(LogColor::white_bg_blue), "This color is LogColor::white_bg_blue");
    Logger::print_color(int(LogColor::white_bg_magenta), "This color is LogColor::white_bg_magenta");
    Logger::print_color(int(LogColor::white_bg_cyan), "This color is LogColor::white_bg_cyan");
    Logger::print_color(int(LogColor::white_bg_black), "This color is LogColor::white_bg_black");
    Logger::print_color(int(LogColor::black_bg_white), "This color is LogColor::black_bg_white");
    Logger::print_color(int(LogColor::black_bg_red), "This color is LogColor::black_bg_red");
    Logger::print_color(int(LogColor::black_bg_yellow), "This color is LogColor::black_bg_yellow");
    Logger::print_color(int(LogColor::black_bg_blue), "This color is LogColor::black_bg_blue");
    Logger::print_color(int(LogColor::black_bg_magenta), "This color is LogColor::black_bg_magenta");
    Logger::print_color(int(LogColor::black_bg_cyan), "This color is LogColor::black_bg_cyan");

    Logger::print_color(int(LogColor::bold), "This color is LogColor::bold");
    Logger::print_color(int(LogColor::underline), "This color is LogColor::underline");
    Logger::print_color(int(LogColor::blink), "This color is LogColor::blink");
    Logger::print_color(int(LogColor::invert), "This color is LogColor::invert");
    Logger::print_color(int(LogColor::hidden), "This color is LogColor::hidden");
    Logger::print_color(int(LogColor::clear), "This color is LogColor::clear"); 

    return 0;
}
