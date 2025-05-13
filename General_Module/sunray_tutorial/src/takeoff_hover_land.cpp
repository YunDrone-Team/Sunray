#include <ros/ros.h>
#include <printf_format.h>
#include "ros_msg_utils.h"
#include "printf_utils.h"
#include <csignal>


using namespace sunray_logger;
using namespace std;

/*
这些变量存储无人机的状态、目标位置、控制命令等信息：
uav_cmd：无人机的控制指令。
setup：无人机的设置指令。
stop_flag：控制任务停止的标志
*/

sunray_msgs::UAVState uav_state;
sunray_msgs::UAVControlCMD uav_cmd;
sunray_msgs::UAVSetup setup;

bool stop_flag{false};

void mySigintHandler(int sig)
{
    // ROS_INFO("[takeoff_hover_land] exit...");
    std::cout << "[takeoff_hover_land] exit..." << std::endl;

    ros::shutdown();
    exit(EXIT_SUCCESS); // 或者使用 exit(0)
}

void stop_tutorial_cb(const std_msgs::Empty::ConstPtr &msg)
{
    stop_flag = true;
}

int main(int argc, char **argv)
{
    // 设置日志
    Logger::init_default();
    Logger::setPrintLevel(false);
    Logger::setPrintTime(false);
    Logger::setPrintToFile(false);
    Logger::setFilename("~/Documents/Sunray_log.txt");

    ros::init(argc, argv, "takeoff_node");
    ros::NodeHandle nh("~");
    ros::Rate rate(20.0);

    // 获取 ROS 参数，设置默认值。如果没有在参数服务器上找到这些参数，会使用默认值。
    int uav_id;

    signal(SIGINT, mySigintHandler);

    string uav_name, target_tpoic_name;
    // 【参数】无人机编号
    nh.param<int>("uav_id", uav_id, 1);
    // 【参数】无人机名称
    nh.param<string>("uav_name", uav_name, "uav");

    uav_name = uav_name + std::to_string(uav_id);
    string topic_prefix = "/" + uav_name;
    // 【订阅】任务结束
    ros::Subscriber stop_tutorial_sub = nh.subscribe<std_msgs::Empty>(topic_prefix + "/sunray/stop_tutorial", 1, stop_tutorial_cb);

    // 【发布】无人机控制指令 （本节点 -> sunray_control_node）
    ros::Publisher control_cmd_pub = nh.advertise<sunray_msgs::UAVControlCMD>(topic_prefix + "/sunray/uav_control_cmd", 1);
    // 【发布】无人机设置指令（本节点 -> sunray_control_node）
    ros::Publisher uav_setup_pub = nh.advertise<sunray_msgs::UAVSetup>(topic_prefix + "/sunray/setup", 1);

    // 变量初始化
    uav_cmd.header.stamp = ros::Time::now();
    uav_cmd.cmd = 102;
    uav_cmd.desired_pos[0] = 0.0;
    uav_cmd.desired_pos[1] = 0.0;
    uav_cmd.desired_pos[2] = 0.0;
    uav_cmd.desired_vel[0] = 0.0;
    uav_cmd.desired_vel[1] = 0.0;
    uav_cmd.desired_vel[2] = 0.0;
    uav_cmd.desired_acc[0] = 0.0;
    uav_cmd.desired_acc[1] = 0.0;
    uav_cmd.desired_acc[2] = 0.0;
    uav_cmd.desired_att[0] = 0.0;
    uav_cmd.desired_att[1] = 0.0;
    uav_cmd.desired_att[2] = 0.0;
    uav_cmd.desired_yaw = 0.0;
    uav_cmd.desired_yaw_rate = 0.0;

    // 固定的浮点显示
    cout.setf(ios::fixed);
    // setprecision(n) 设显示小数精度为2位
    cout << setprecision(2);
    // 左对齐
    cout.setf(ios::left);
    // 强制显示小数点
    cout.setf(ios::showpoint);
    // 强制显示符号
    cout.setf(ios::showpos);

    // 打印相关信息
    cout << GREEN << ">>>>>>>>>>>>>>>> " << ros::this_node::getName() << " <<<<<<<<<<<<<<<<" << TAIL << endl;
    cout << GREEN << "uav_id                    : " << uav_id << " " << TAIL << endl;
    cout << GREEN << "uav_name                  : " << uav_name << " " << TAIL << endl;

    ros::Duration(0.5).sleep();
    // 初始化检查：等待PX4连接
    int times = 0;
    while (ros::ok() && !uav_state.connected)
    {
        ros::spinOnce();
        ros::Duration(1.0).sleep();
        if (times++ > 5)
            Logger::print_color(int(LogColor::red), "Wait for uav connect");
    }

    Logger::print_color(int(LogColor::blue), ">>>>>>> uav connected!");
    // 解锁
    setup.cmd = sunray_msgs::UAVSetup::ARM;
    uav_setup_pub.publish(setup);
    Logger::print_color(int(LogColor::blue), ">>>>>>> uav_setup_pub: arm uav!");
    ros::Duration(1.5).sleep();

    Logger::print_color(int(LogColor::blue), ">>>>>>> uav armed successfully!");

    // 切换到指令控制模式
    Logger::print_color(int(LogColor::blue), ">>>>>>> switch to CMD_CONTROL");
    setup.cmd = sunray_msgs::UAVSetup::SET_CONTROL_MODE;
    setup.control_mode = "CMD_CONTROL";
    uav_setup_pub.publish(setup);
    ros::Duration(1.0).sleep();

    Logger::print_color(int(LogColor::blue), ">>>>>>> uav switch to CMD_CONTROL successfully!");


    // 起飞
    Logger::print_color(int(LogColor::blue), ">>>>>>> UAV Takeoff");
    uav_cmd.cmd = sunray_msgs::UAVControlCMD::Takeoff;
    control_cmd_pub.publish(uav_cmd);
    ros::Duration(10).sleep();

    // 悬停
    Logger::print_color(int(LogColor::blue), ">>>>>>> UAV Hover");
    uav_cmd.cmd = sunray_msgs::UAVControlCMD::Hover;
    control_cmd_pub.publish(uav_cmd);
    ros::Duration(5).sleep();

    // 降落
    Logger::print_color(int(LogColor::blue), ">>>>>>> UAV Land");
    uav_cmd.cmd = sunray_msgs::UAVControlCMD::Land;
    control_cmd_pub.publish(uav_cmd);
    // 关键
    ros::Duration(0.5).sleep();
}