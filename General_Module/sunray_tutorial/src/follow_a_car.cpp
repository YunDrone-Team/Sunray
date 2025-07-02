/*
程序功能：根据相对位置进行标签跟踪，跟踪方向：下方物体
*/

#include <ros/ros.h>
#include "ros_msg_utils.h"
#include "printf_utils.h"
#include <sunray_msgs/TargetMsg.h>
#include <sunray_msgs/TargetsInFrameMsg.h>
#include "utils.hpp"
#include <printf_format.h>

using namespace std;
using namespace sunray_logger;

string node_name;
sunray_msgs::UAVControlCMD uav_cmd;
sunray_msgs::UAVSetup uav_setup;
sunray_msgs::UAVState uav_state;

bool stop_flag{false};

// 全局变量存储无人机和车的坐标和朝向
double x_rel, y_rel, z_rel, yaw_rel;
bool pos_flag{false};
bool tag_flag{false};
double x_vel = 0.0;
double y_vel = 0.0;
double z_vel = 0.0;
double yaw = 0.0;

MovingAverageFilter x_filter(5);
MovingAverageFilter y_filter(5);
MovingAverageFilter z_filter(5);
MovingAverageFilter yaw_filter(3);
ros::Time last_time{0};

// 无人机状态回调
void uav_state_callback(const sunray_msgs::UAVState::ConstPtr &msg)
{
    uav_state = *msg;
    pos_flag = true;
}

void stop_tutorial_cb(const std_msgs::Empty::ConstPtr &msg)
{
    stop_flag = true;
}

// 回调函数，用于获取目标二的位置和朝向
void tagCallback(const sunray_msgs::TargetsInFrameMsg::ConstPtr &msg)
{

    if (msg->targets.size() > 0)
    {
        tag_flag = true;
        last_time = ros::Time::now();
        x_rel = x_filter.filter(msg->targets[0].px);
        y_rel = y_filter.filter(-msg->targets[0].py);
        z_rel = z_filter.filter(msg->targets[0].pz);
        yaw_rel = yaw_filter.filter(-msg->targets[0].yaw);
    }
}

int main(int argc, char **argv)
{
    // 设置日志
    Logger::init_default();
    Logger::setPrintLevel(false);
    Logger::setPrintTime(false);
    Logger::setPrintToFile(false);
    Logger::setFilename("~/Documents/Sunray_log.txt");

    ros::init(argc, argv, "auto_land_node");
    ros::NodeHandle nh("~");

    ros::Rate rate(20.0);
    node_name = ros::this_node::getName();

    int uav_id;
    bool auto_takeoff = false;
    string uav_name;

    
    // 【参数】无人机编号
    nh.param<int>("uav_id", uav_id, 1);
    // 【参数】无人机名称
    nh.param<string>("uav_name", uav_name, "uav");

    double k_p_xy, k_p_z, k_p_yaw, max_vel, max_vel_z, max_yaw;
    double height;
    // 是否启动自动起飞
    nh.param<bool>("auto_takeoff", auto_takeoff, false);
    // 自动起飞高度
    nh.param<double>("height", height, 1.0);
    // 【参数】P控制参数
    nh.param<double>("k_p_xy", k_p_xy, 1.2);
    nh.param<double>("k_p_z", k_p_z, 0.5);
    nh.param<double>("k_p_yaw", k_p_yaw, 0.04);
    // 【参数】最大速度限制
    nh.param<double>("max_vel", max_vel, 0.5);
    nh.param<double>("max_vel_z", max_vel_z, 0.2);
    nh.param<double>("max_yaw", max_yaw, 0.4);
    
    uav_name = "/" + uav_name + to_string(uav_id);
    // 【订阅】无人机状态
    ros::Subscriber uav_state_sub = nh.subscribe<sunray_msgs::UAVState>(uav_name + "/sunray/uav_state", 10, uav_state_callback);
    // 【订阅】目标点位置
    ros::Subscriber target_pos_sub = nh.subscribe<sunray_msgs::TargetsInFrameMsg>(uav_name + "/sunray_detect/qrcode_detection_ros", 1, tagCallback);
    // 【订阅】任务结束
    ros::Subscriber stop_tutorial_sub = nh.subscribe<std_msgs::Empty>(uav_name + "/sunray/stop_tutorial", 1, stop_tutorial_cb);
    // 【发布】无人机控制指令 （本节点 -> sunray_control_node）
    ros::Publisher control_cmd_pub = nh.advertise<sunray_msgs::UAVControlCMD>(uav_name + "/sunray/uav_control_cmd", 1);
    // 【发布】无人机设置指令（本节点 -> sunray_control_node）
    ros::Publisher uav_setup_pub = nh.advertise<sunray_msgs::UAVSetup>(uav_name + "/sunray/setup", 1);

    // 变量初始化
    uav_cmd.header.stamp = ros::Time::now();
    uav_cmd.cmd = sunray_msgs::UAVControlCMD::Hover;
    uav_cmd.desired_pos[0] = 0.0;
    uav_cmd.desired_pos[1] = 0.0;
    uav_cmd.desired_pos[2] = 0.0;
    uav_cmd.desired_vel[0] = 0.0;
    uav_cmd.desired_vel[1] = 0.0;
    uav_cmd.desired_vel[2] = 0.0;
    uav_cmd.desired_yaw = 0.0;
    uav_cmd.desired_yaw_rate = 0.0;

    // 自动起飞
    if (auto_takeoff)
    {
        ros::Duration(3).sleep();
        // 初始化检查：等待PX4连接
        int times = 0;
        while (ros::ok() && !uav_state.connected)
        {
            ros::spinOnce();
            ros::Duration(1.0).sleep();
            if (times++ > 5)
                Logger::print_color(int(LogColor::red), node_name, ": Wait for UAV connect...");
        }
        Logger::print_color(int(LogColor::green), node_name, ": UAV connected!");

        // 切换到指令控制模式(同时，PX4模式将切换至OFFBOARD模式)
        while (ros::ok() && uav_state.control_mode != sunray_msgs::UAVSetup::CMD_CONTROL)
        {

            uav_setup.cmd = sunray_msgs::UAVSetup::SET_CONTROL_MODE;
            uav_setup.control_mode = "CMD_CONTROL";
            uav_setup_pub.publish(uav_setup);
            Logger::print_color(int(LogColor::green), node_name, ": SET_CONTROL_MODE - [CMD_CONTROL]. ");
            ros::Duration(1.0).sleep();
            ros::spinOnce();
        }
        Logger::print_color(int(LogColor::green), node_name, ": UAV control_mode set to [CMD_CONTROL] successfully!");

        // 解锁无人机
        Logger::print_color(int(LogColor::green), node_name, ": Arm UAV in 5 sec...");
        ros::Duration(1.0).sleep();
        Logger::print_color(int(LogColor::green), node_name, ": Arm UAV in 4 sec...");
        ros::Duration(1.0).sleep();
        Logger::print_color(int(LogColor::green), node_name, ": Arm UAV in 3 sec...");
        ros::Duration(1.0).sleep();
        Logger::print_color(int(LogColor::green), node_name, ": Arm UAV in 2 sec...");
        ros::Duration(1.0).sleep();
        Logger::print_color(int(LogColor::green), node_name, ": Arm UAV in 1 sec...");
        ros::Duration(1.0).sleep();
        while (ros::ok() && !uav_state.armed)
        {
            uav_setup.cmd = sunray_msgs::UAVSetup::ARM;
            uav_setup_pub.publish(uav_setup);
            Logger::print_color(int(LogColor::green), node_name, ": Arm UAV now.");
            ros::Duration(1.0).sleep();
            ros::spinOnce();
        }
        Logger::print_color(int(LogColor::green), node_name, ": Arm UAV successfully!");

        // 起飞无人机
        while (ros::ok() && abs(uav_state.position[2] - uav_state.home_pos[2] - uav_state.takeoff_height) > 0.2)
        {
            uav_cmd.cmd = sunray_msgs::UAVControlCMD::Takeoff;
            control_cmd_pub.publish(uav_cmd);
            Logger::print_color(int(LogColor::green), node_name, ": Takeoff UAV now.");
            ros::Duration(4.0).sleep();
            ros::spinOnce();
        }
        Logger::print_color(int(LogColor::green), node_name, ": Takeoff UAV successfully!");
        ros::Duration(5).sleep();

        Logger::print_color(int(LogColor::blue), ">>>>>>> move to the specified height");
        uav_cmd.cmd = sunray_msgs::UAVControlCMD::XyVelZPos;
        uav_cmd.desired_vel[0] = 0.0;
        uav_cmd.desired_vel[1] = 0.0;
        uav_cmd.desired_pos[2] = height;
        control_cmd_pub.publish(uav_cmd);
        ros::Duration(2).sleep();
    }

    ros::Duration(0.5).sleep();
    ros::spinOnce();

    // 等待订阅无人机位置
    while (!pos_flag)
    {
        Logger::print_color(int(LogColor::blue), "wait for pose!!!");
        ros::spinOnce();
        ros::Duration(1).sleep();
    }
    // 等待订阅到目标位置
    while (!tag_flag)
    {
        Logger::print_color(int(LogColor::blue), "wait for tag!!!");
        ros::spinOnce();
        ros::Duration(1).sleep();
    }

    double landing_point_num = 0;

    while (ros::ok())
    {
        ros::spinOnce();
        ros::Duration(0.05).sleep();
        if (stop_flag)
        {
            Logger::print_color(int(LogColor::blue), "land");
            uav_cmd.cmd = sunray_msgs::UAVControlCMD::Land;
            control_cmd_pub.publish(uav_cmd);
            break;
        }
        if (((ros::Time::now() - last_time).toSec()) < 0.5)
        {
            landing_point_num += 1;

            if (landing_point_num > 10)
            {
                if (yaw_rel < 1 && yaw_rel > -1)
                {
                    yaw_rel = 0; // 小角度则不再调整 避免无人机超调而产生抽搐情况
                }

                x_vel = min(max(x_rel * k_p_xy, -max_vel), max_vel);
                y_vel = min(max(y_rel * k_p_xy, -max_vel), max_vel);
                z_vel = min(max((1 - z_rel) * k_p_z, -max_vel_z), max_vel_z); // 1-z_rel 调整跟随高度 程序中定为高度1m
                // yaw = min(max(yaw_rel*k_p_yaw, -max_yaw), max_yaw);
                yaw = yaw_rel / 180.0 * M_PI; // 转为弧度制

                uav_cmd.header.stamp = ros::Time::now();
                uav_cmd.cmd = sunray_msgs::UAVControlCMD::XyzVelYawBody;
                uav_cmd.desired_vel[0] = x_vel;
                uav_cmd.desired_vel[1] = y_vel;
                uav_cmd.desired_vel[2] = z_vel;
                uav_cmd.desired_yaw = yaw;
                control_cmd_pub.publish(uav_cmd);
            }
            continue;
        }
        if ((ros::Time::now() - last_time).toSec() > 10)
        {
            Logger::print_color(int(LogColor::blue), "over time!!! Land directly");
            landing_point_num = 0;
            uav_cmd.header.stamp = ros::Time::now();
            uav_cmd.cmd = sunray_msgs::UAVControlCMD::Land;
            control_cmd_pub.publish(uav_cmd);
            // 程序结束太快会导致消息没有发送出去 因此要停止一段时间
            ros::Duration(0.5).sleep();
            break;
        }
        if ((ros::Time::now() - last_time).toSec() > 2)
        {
            Logger::print_color(int(LogColor::blue), "lost the tag !!! hovering!!! ");
            uav_cmd.cmd = sunray_msgs::UAVControlCMD::Hover;
            control_cmd_pub.publish(uav_cmd);
            continue;
        }
    }
}
