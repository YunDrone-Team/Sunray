#include <ros/ros.h>
#include <printf_format.h>
#include "ros_msg_utils.h"
#include "printf_utils.h"

using namespace sunray_logger;
using namespace std;

geometry_msgs::PoseStamped current_pose;
sunray_msgs::UAVState uav_state;
sunray_msgs::UAVControlCMD uav_cmd;
std_msgs::Int32 uav_control_state;
geometry_msgs::PoseStamped target_pose;
sunray_msgs::UAVSetup uav_setup;
string node_name;

float target_yaw;
bool stop_flag{false};

void uav_state_cb(const sunray_msgs::UAVState::ConstPtr &msg)
{
    uav_state = *msg;
}
void uav_control_state_cb(const std_msgs::Int32::ConstPtr &msg)
{
    uav_control_state = *msg;
}
void target_pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    target_pose = *msg;
    tf2::Quaternion quaternion;
    tf2::fromMsg(msg->pose.orientation, quaternion);
    double roll, pitch, yaw;
    tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
    target_yaw = yaw;
}
void stop_tutorial_cb(const std_msgs::Empty::ConstPtr &msg)
{
    stop_flag = true;
}

void pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    current_pose = *msg;
}

int main(int argc, char **argv)
{
    // 设置日志
    Logger::init_default();
    Logger::setPrintLevel(false);
    Logger::setPrintTime(false);
    Logger::setPrintToFile(false);
    Logger::setFilename("~/Documents/Sunray_log.txt");

    ros::init(argc, argv, "circle_vel");
    ros::NodeHandle nh("~");
    ros::Rate rate(20.0);
    node_name = ros::this_node::getName();

    int uav_id;
    string uav_name, target_topic_name;
    bool sim_mode, flag_printf;
    nh.param<bool>("sim_mode", sim_mode, true);
    nh.param<bool>("flag_printf", flag_printf, true);
    // 【参数】无人机编号
    nh.param<int>("uav_id", uav_id, 1);
    // 【参数】无人机名称
    nh.param<string>("uav_name", uav_name, "uav");
    // 【参数】目标话题名称
    nh.param<string>("target_tpoic_name", target_tpoic_name, "/vrpn_client_node/target/pose");

    uav_name = "/" + uav_name + std::to_string(uav_id);
    // 【订阅】无人机状态 -- from vision_pose
    ros::Subscriber uav_state_sub = nh.subscribe<sunray_msgs::UAVState>(uav_name + "/sunray/uav_state", 1, uav_state_cb);
    // 【订阅】无人机控制信息
    ros::Subscriber uav_contorl_state_sub = nh.subscribe<std_msgs::Int32>(uav_name+ "/sunray/control_state", 1, uav_control_state_cb);
    // 【订阅】目标点位置
    if (sim_mode)
    {
        ros::Subscriber target_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>(uav_name + "/sunray/gazebo_pose", 1, target_pos_cb);
    }
    else
    {
        ros::Subscriber target_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/" + uav_name + "/pose", 1, target_pos_cb);
    }
    // 【订阅】任务结束
    ros::Subscriber stop_tutorial_sub = nh.subscribe<std_msgs::Empty>(uav_name + "/sunray/stop_tutorial", 1, stop_tutorial_cb);

    // 【发布】无人机控制指令 （本节点 -> sunray_control_node）
    ros::Publisher control_cmd_pub = nh.advertise<sunray_msgs::UAVControlCMD>(uav_name + "/sunray/uav_control_cmd", 1);
    // 【发布】无人机设置指令（本节点 -> sunray_control_node）
    ros::Publisher uav_setup_pub = nh.advertise<sunray_msgs::UAVSetup>(uav_name + "/sunray/setup", 1);

    ros::Subscriber pose_sub = nh.subscribe(uav_name+ "/mavros/local_position/pose", 10, pose_cb);
    // 变量初始化
    uav_cmd.header.stamp = ros::Time::now();
    uav_cmd.cmd_id = 0;
    uav_cmd.cmd = sunray_msgs::UAVControlCMD::Hover;
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
    uav_cmd.enable_yawRate = false;

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
    cout << GREEN << ">>>>>>>>>>>>>>>> " << node_name << " <<<<<<<<<<<<<<<<" << TAIL << endl;
    cout << GREEN << "uav_id                    : " << uav_id << " " << TAIL << endl;
    cout << GREEN << "sim_mode                  : " << sim_mode << " " << TAIL << endl;
    cout << GREEN << "uav_name                  : " << uav_name << " " << TAIL << endl;
    cout << GREEN << "target_topic_name         : " << target_topic_name << " " << TAIL << endl;

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

    double k_p = 1.0;     // proportional gain
    double z_k_p = 0.5;   // proportional gain
    double max_vel = 1.0; // maximum velocity (m/s)

    double hight = 0.6;

    // Define the circle's center and radius
    double center_x = 0;
    double center_y = 0;
    double radius = 2.0;

    double cilcle_time = 5.0;
    double dt = 2 * M_PI / cilcle_time;

    geometry_msgs::PoseStamped pose;
    ros::Time start_time = ros::Time::now();
    while (ros::ok())
    {
        double theta = (ros::Time::now() - start_time).toSec() * dt;
        // double theta = 2 * M_PI * t / cilcle_time;
        pose.pose.position.z = hight;
        ros::Time start_time = ros::Time::now();

        if (stop_flag)
        {
            uav_cmd.cmd = sunray_msgs::UAVControlCMD::Land;
            control_cmd_pub.publish(uav_cmd);
            Logger::print_color(int(LogColor::green), node_name, ": Land UAV now.");
            uav_cmd.cmd_id = uav_cmd.cmd_id + 1;
            break;
        }

        double dz = pose.pose.position.z - current_pose.pose.position.z;

        double vx = radius * sin(theta) * (2 * M_PI / cilcle_time);
        double vy = -radius * cos(theta) * (2 * M_PI / cilcle_time);
        double vz = z_k_p * dz;
        vz = min(max(vz, -max_vel), max_vel);
        std::cout << "vx: " << vx << " vy: " << vy << " vz: " << vz << std::endl;

        uav_cmd.header.stamp = ros::Time::now();
        uav_cmd.cmd = sunray_msgs::UAVControlCMD::XYZ_VEL;
        uav_cmd.desired_vel[0] = vx;
        uav_cmd.desired_vel[1] = vy;
        uav_cmd.desired_vel[2] = vz;
        uav_cmd.desired_pos[0] = 0.0;
        uav_cmd.desired_pos[1] = 0.0;
        uav_cmd.desired_pos[2] = 0.0;
        uav_cmd.desired_yaw = 0;
        uav_cmd.enable_yawRate = 0;
        uav_cmd.cmd_id = uav_cmd.cmd_id + 1;
        control_cmd_pub.publish(uav_cmd);

        if ((ros::Time::now() - start_time).toSec() > 0.1)
        {
            break;
        }
        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }

    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = hight;
    while (ros::ok())
    {
        // Calculate the distance to the target position
        double dx = pose.pose.position.x - current_pose.pose.position.x;
        double dy = pose.pose.position.y - current_pose.pose.position.y;
        double dz = pose.pose.position.z - current_pose.pose.position.z;

        // Calculate the desired velocity using a proportional controller
        double vx = k_p * dx;
        double vy = k_p * dy;
        double vz = z_k_p * dz;

        // Limit the velocities to a maximum value
        vx = min(max(vx, -max_vel), max_vel);
        vy = min(max(vy, -max_vel), max_vel);
        vz = min(max(vz, -max_vel), max_vel);

        uav_cmd.header.stamp = ros::Time::now();
        uav_cmd.cmd = sunray_msgs::UAVControlCMD::XYZ_VEL;
        uav_cmd.desired_vel[0] = vx;
        uav_cmd.desired_vel[1] = vy;
        uav_cmd.desired_vel[2] = vz;
        uav_cmd.desired_pos[0] = 0.0;
        uav_cmd.desired_pos[1] = 0.0;
        uav_cmd.desired_pos[2] = 0.0;
        uav_cmd.desired_yaw = 0;
        uav_cmd.enable_yawRate = 0;
        uav_cmd.cmd_id = uav_cmd.cmd_id + 1;
        control_cmd_pub.publish(uav_cmd);

        // Check if the drone has reached the target point
        if (fabs(current_pose.pose.position.x - pose.pose.position.x) < 0.2 &&
            fabs(current_pose.pose.position.y - pose.pose.position.y) < 0.2)
        {
            break;
        }

        ros::spinOnce();
        rate.sleep();
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
