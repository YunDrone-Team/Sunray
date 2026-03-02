#include <ros/ros.h>
#include <signal.h>
#include "sunray_log.hpp"
#include "sunray_log_example/localization_example.hpp"

void SigintHandler(int sig) {
    SUNRAY_WARN("节点收到退出信号，正在关闭...");
    ros::shutdown();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "localization_example_node", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;

    signal(SIGINT, SigintHandler);

    // 初始化日志：控制台显示 info 及以上，文件记录 debug 及以上
    SunrayLogConfig cfg;
    cfg.name = "localization_example";
    cfg.console_level = SunrayLogLevel::info;
    cfg.file_path = "logs/localization_example.log";
    cfg.file_level = SunrayLogLevel::debug;
    cfg.async = false;
    SunrayLogger::instance().Init(cfg);

    SUNRAY_INFO("localization_example_node 启动");

    LocalizationExample loc(nh);
    loc.Init();

    ros::spin();

    SUNRAY_ERROR("localization_example_node 已退出");

    return 0;
}
