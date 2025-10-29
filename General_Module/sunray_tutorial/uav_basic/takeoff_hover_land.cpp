/*
    起飞降落例程：takeoff_hover_land.cpp
    程序功能：自动起飞、指定点悬停、自动降落
*/

#include "ros_msg_utils.h"

int uav_id;
string node_name;
string uav_name;
sunray_msgs::UAVState uav_state;
sunray_msgs::UAVSetup uav_setup;
sunray_msgs::UAVControlCMD uav_cmd;

void mySigintHandler(int sig)
{
    std::cout << "[takeoff_hover_land] exit..." << std::endl;

    ros::shutdown();
    exit(EXIT_SUCCESS); // 或者使用 exit(0)
}

// 无人机状态回调
void uav_state_callback(const sunray_msgs::UAVState::ConstPtr &msg)
{
    uav_state = *msg;
}

int main(int argc, char **argv)
{
    // 设置日志
    Logger::init_default();

    ros::init(argc, argv, "takeoff_hover_land");
    ros::NodeHandle nh("~");
    ros::Rate rate(20.0);

    signal(SIGINT, mySigintHandler);

    node_name = ros::this_node::getName();
    node_name = "["+node_name+"]:";

    // 【参数】无人机编号
    nh.param<int>("uav_id", uav_id, 1);
    // 【参数】无人机名称
    nh.param<string>("uav_name", uav_name, "uav");
    uav_name = "/" + uav_name + std::to_string(uav_id);

    // 【订阅】无人机状态
    ros::Subscriber uav_state_sub = nh.subscribe<sunray_msgs::UAVState>(uav_name + "/sunray/uav_state", 10, uav_state_callback);
    // 【发布】无人机控制指令 （本节点 -> sunray_control_node）
    ros::Publisher control_cmd_pub = nh.advertise<sunray_msgs::UAVControlCMD>(uav_name + "/sunray/uav_control_cmd", 1);
    // 【发布】无人机设置指令（本节点 -> sunray_control_node）
    ros::Publisher uav_setup_pub = nh.advertise<sunray_msgs::UAVSetup>(uav_name + "/sunray/setup", 1);

    // 控制辅助类 - 初始化
    Control_Utils uav_control_utils;
    uav_control_utils.init(nh, uav_id, node_name);

    // 初始化检查：等待PX4连接
    int times = 0;
    while (ros::ok() && !uav_state.connected)
    {
        ros::spinOnce();
        ros::Duration(1.0).sleep();
        if (times++ > 5)
            Logger::print_color(int(LogColor::red), node_name, "Wait for UAV connect...");
    }

    // 控制辅助类 - 自动起飞
    uav_control_utils.auto_takeoff();

    // 以上: 无人机已成功起飞，进入自由任务模式
    Logger::print_color(int(LogColor::green), node_name, "Wait 5 sec and then send Hover cmd...");

    ros::Duration(5.0).sleep();
    // 发布悬停指令
    Logger::print_color(int(LogColor::green), node_name, "Send UAV Hover cmd.");
    uav_cmd.header.stamp = ros::Time::now();
    uav_cmd.cmd = sunray_msgs::UAVControlCMD::Hover;
    control_cmd_pub.publish(uav_cmd);
    ros::Duration(5).sleep();
    ros::spinOnce();

    Logger::print_color(int(LogColor::green), node_name, "Wait 5 sec and then send Land cmd...");
    ros::Duration(5.0).sleep();

    // 控制辅助类 - 自动降落
    uav_control_utils.auto_land();

    // Demo 结束
    Logger::print_color(int(LogColor::green), node_name, "Demo finished, quit!");
    
    return 0;
}