/*
程序功能：使用XyVelZPos接口进行圆形轨迹飞行
*/
#include <ros/ros.h>
#include <printf_format.h>
#include "ros_msg_utils.h"
#include "printf_utils.h"
#include <csignal>

using namespace sunray_logger;
using namespace std;

sunray_msgs::UAVControlCMD uav_cmd;
sunray_msgs::UAVState uav_state;
sunray_msgs::UAVSetup uav_setup;
string node_name;

void mySigintHandler(int sig)
{
    std::cout << "[circle_z_pos] exit..." << std::endl;

    ros::shutdown();
    exit(EXIT_SUCCESS); // 或者使用 exit(0)
}

void uav_state_callback(const sunray_msgs::UAVState::ConstPtr &msg)
{
    uav_state = *msg;
}

int main(int argc, char **argv)
{
    // 设置日志
    Logger::init_default();
    Logger::setPrintLevel(false);
    Logger::setPrintTime(false);
    Logger::setPrintToFile(false);
    Logger::setFilename("~/Documents/Sunray_log.txt");

    ros::init(argc, argv, "circle_z_pos");
    ros::NodeHandle nh("~");
    ros::Rate rate(20.0);
    node_name = ros::this_node::getName();

    int uav_id;

    signal(SIGINT, mySigintHandler);

    string uav_name;
    // 【参数】无人机编号
    nh.param<int>("uav_id", uav_id, 1);
    // 【参数】无人机名称
    nh.param<string>("uav_name", uav_name, "uav");

    uav_name = "/" + uav_name + std::to_string(uav_id);
    //  订阅无人机状态
    ros::Subscriber uav_state_sub = nh.subscribe<sunray_msgs::UAVState>(uav_name + "/sunray/uav_state", 10, uav_state_callback);

    // 【发布】无人机控制指令 （本节点 -> sunray_control_node）
    ros::Publisher control_cmd_pub = nh.advertise<sunray_msgs::UAVControlCMD>(uav_name + "/sunray/uav_control_cmd", 1);
    // 【发布】无人机设置指令（本节点 -> sunray_control_node）
    ros::Publisher uav_setup_pub = nh.advertise<sunray_msgs::UAVSetup>(uav_name + "/sunray/setup", 1);

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

    ros::Duration(0.5).sleep();
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

    // 以上: 无人机已成功起飞，进入自由任务模式

    ros::Duration(5).sleep();
    // 悬停
    Logger::print_color(int(LogColor::green), node_name, ": Send UAV Hover cmd.");
    uav_cmd.cmd = sunray_msgs::UAVControlCMD::Hover;
    control_cmd_pub.publish(uav_cmd);
    ros::Duration(5).sleep();
    ros::spinOnce();

    // Define the circle's center and radius
    double center_x = 0;
    double center_y = 0;
    double radius = 1;

    // Define the number of points on the circle
    int num_points = 50;
    // Define the proportional gain and maximum velocity
    double k_p = 1;       // proportional gain
    double max_vel = 1.0; // maximum velocity (m/s)

    double height = 0.8;

    geometry_msgs::PoseStamped pose;
    Logger::print_color(int(LogColor::green), node_name, ": Start circle.");
    ros::Time start_time = ros::Time::now();

    // 圈数
    int num = 2;
    for (int i = 0; i < num_points; i++)
    {
        double theta = i * 2 * M_PI / num_points;
        pose.pose.position.x = center_x + radius * cos(theta);
        pose.pose.position.y = center_y + radius * sin(theta);
        pose.pose.position.z = height; // fixed altitude
        start_time = ros::Time::now();
        // Send setpoints until the drone reaches the target point
        while (ros::ok())
        {
            // Calculate the distance to the target position
            double dx = pose.pose.position.x - uav_state.position[0];
            double dy = pose.pose.position.y - uav_state.position[1];

            // Calculate the desired velocity using a proportional controller
            double vx = k_p * dx;
            double vy = k_p * dy;

            // Limit the velocities to a maximum value
            vx = min(max(vx, -max_vel), max_vel);
            vy = min(max(vy, -max_vel), max_vel);

            uav_cmd.header.stamp = ros::Time::now();
            uav_cmd.cmd = sunray_msgs::UAVControlCMD::XyVelZPos;
            uav_cmd.desired_vel[0] = vx;
            uav_cmd.desired_vel[1] = vy;
            uav_cmd.desired_pos[2] = height;
            control_cmd_pub.publish(uav_cmd);

            // 闭环
            // if (fabs(current_pose.pose.position.x - pose.pose.position.x) < 0.15 &&
            //     fabs(current_pose.pose.position.y - pose.pose.position.y) < 0.15)
            // {
            //     break;
            // }

            // 开环
            if (ros::Time::now() - start_time > ros::Duration(0.6))
            {
                break;
            }
            ros::spinOnce();
            ros::Duration(0.1).sleep();

            if (num != 1 && i == num_points - 1)
            {
                num--;
                i = 0;
            }
        }
    }

    // 回到原点
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = height;
    while (ros::ok())
    {
        // Calculate the distance to the target position
        double dx = pose.pose.position.x - uav_state.position[0];
        double dy = pose.pose.position.y - uav_state.position[1];

        // Calculate the desired velocity using a proportional controller
        double vx = k_p * dx;
        double vy = k_p * dy;

        // Limit the velocities to a maximum value
        vx = min(max(vx, -max_vel), max_vel);
        vy = min(max(vy, -max_vel), max_vel);

        uav_cmd.header.stamp = ros::Time::now();
        uav_cmd.cmd = sunray_msgs::UAVControlCMD::XyVelZPos;
        uav_cmd.desired_vel[0] = vx;
        uav_cmd.desired_vel[1] = vy;
        uav_cmd.desired_pos[2] = height;
        uav_cmd.desired_yaw = 0;
        control_cmd_pub.publish(uav_cmd);

        // Check if the drone has reached the target point
        if (fabs(uav_state.position[0] - pose.pose.position.x) < 0.2 &&
            fabs(uav_state.position[1] - pose.pose.position.y) < 0.2)
        {
            // 停下1秒等待无人机速度降下来
            ros::Duration(1.0).sleep();
            break;
        }

        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }

    // 降落无人机
    while (ros::ok() && uav_state.control_mode != sunray_msgs::UAVSetup::LAND_CONTROL && uav_state.landed_state != 1)
    {
        uav_cmd.cmd = sunray_msgs::UAVControlCMD::Land;
        control_cmd_pub.publish(uav_cmd);
        Logger::print_color(int(LogColor::green), node_name, ": Land UAV now.");
        ros::Duration(4.0).sleep();
        ros::spinOnce();
    }
    // 等待降落
    while (ros::ok() && uav_state.landed_state != 1)
    {
        Logger::print_color(int(LogColor::green), node_name, ": Landing");
        ros::Duration(1.0).sleep();
        ros::spinOnce();
    }
    // 成功降落
    Logger::print_color(int(LogColor::green), node_name, ": Land UAV successfully!");

    // Demo 结束
    Logger::print_color(int(LogColor::green), node_name, ": Demo finished, quit!");
    return 0;
}
