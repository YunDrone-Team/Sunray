#include <ros/ros.h>
#include "ros_msg_utils.h"
#include "printf_utils.h"
#include <sunray_msgs/TargetMsg.h>
#include <sunray_msgs/TargetsInFrameMsg.h>
#include "utils.hpp"
#include <sunray_msgs/UAVSetup.h>
#include <sunray_msgs/UAVControlCMD.h>
#include <sunray_msgs/UAVState.h>
#include <printf_format.h>

using namespace std;
using namespace sunray_logger;
string node_name;
// 当前无人机的位姿
geometry_msgs::PoseStamped current_pose;
// 存储发送给控制模块的指令
sunray_msgs::UAVControlCMD uav_cmd;
// 目标位位姿
geometry_msgs::PoseStamped target_pose;
// 无人机设置指令
sunray_msgs::UAVSetup uav_setup;
// 目标yaw
float target_yaw;
// 停止标志
bool stop_flag{false};

sunray_msgs::UAVState uav_state;
// 自动起飞标志
// bool auto_takeoff = false;

// 全局变量存储无人机和车的坐标和朝向
double uav_x, uav_y, uav_z, uav_yaw, uav_vx, uav_vy, uav_vz;
double x_rel, y_rel, z_rel, yaw_rel;
// 可能表示是否收到当前位置信息和目标检测信息的标志位。
bool sub_flag{false};
bool tag_flag{false};

// 移动平均滤波器，用于平滑目标物相对位置和朝向的变化。
MovingAverageFilter x_filter(15);
MovingAverageFilter y_filter(15);
MovingAverageFilter z_filter(15);
MovingAverageFilter yaw_filter(15);

ros::Time last_time{0};

void uav_state_callback(const sunray_msgs::UAVState::ConstPtr &msg)
{
    uav_state = *msg;
}
/*
stop_tutorial_cb:
监听 /sunray/stop_tutorial 话题，用于接收任务结束的指令，
一旦接收到消息，stop_flag 被设置为 true。
*/
void stop_tutorial_cb(const std_msgs::Empty::ConstPtr &msg)
{
    stop_flag = true;
}

/*
pose_cb:
监听 /mavros/local_position/pose 话题，
用于更新当前无人机的位置和朝向信息，将其存储到全局变量中
*/
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    sub_flag = true;
    current_pose = *msg;
    uav_x = msg->pose.position.x;
    uav_y = msg->pose.position.y;
    uav_z = msg->pose.position.z;
    uav_yaw = tf::getYaw(msg->pose.orientation);
}

// 回调函数，用于获取目标二的位置和朝向
/*
tagCallback: 监听目标检测消息 /sunray_detect/qrcode_detection_ros，
用于获取目标的相对位置和朝向，并对其进行滤波处理。如果有目标检测到，会更新相对位置信息。
*/
void tagCallback(const sunray_msgs::TargetsInFrameMsg::ConstPtr &msg)
{

    if (msg->targets.size() > 0)
    {
        if (abs(msg->targets[0].px) > 5 || abs(msg->targets[0].py) > 5 || abs(msg->targets[0].pz) > 5)
        {
            return;
        }
        tag_flag = true;
        last_time = ros::Time::now();
        // 相机正装
        x_rel = x_filter.filter(msg->targets[0].px);
        y_rel = y_filter.filter(-msg->targets[0].py);
        z_rel = z_filter.filter(-msg->targets[0].pz);
        yaw_rel = yaw_filter.filter(-msg->targets[0].yaw);

        // // 相机反装（180旋转）
        // x_rel = x_filter.filter(-msg->targets[0].px);
        // y_rel = y_filter.filter(msg->targets[0].py);
        // z_rel = z_filter.filter(-msg->targets[0].pz);
        // // 机体系相对旋转都是一样的 由于标签识别在180度时候的数据波动异常 因此只能反向降落到标签上而不纠正180度的偏差
        // yaw_rel = yaw_filter.filter(-msg->targets[0].yaw);
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
    bool auto_takeoff = false;
    int uav_id;
    string uav_name, target_tpoic_name;
    bool sim_mode, flag_printf;
    nh.param<bool>("sim_mode", sim_mode, true);
    nh.param<bool>("flag_printf", flag_printf, true);
    nh.param<bool>("auto_takeoff", auto_takeoff, false);
    // error_xy   error_z
    double error_xy, error_z;
    // land_v
    double land_v;
    // land_time
    double land_time;
    double height;
    // 【参数】无人机编号
    nh.param<int>("uav_id", uav_id, 1);
    // 【参数】无人机名称
    nh.param<string>("uav_name", uav_name, "uav");
    // 【参数】目标话题名称 （预留，可以通过外部定位直接发送降落地点）
    nh.param<string>("target_tpoic_name", target_tpoic_name, "/vrpn_client_node/target/pose");
    // PID控制P参数和速度参数的限制
    double k_p_xy, k_p_z, k_p_yaw, max_vel, max_vel_z, max_yaw;
    nh.param<double>("k_p_xy", k_p_xy, 1.2);
    nh.param<double>("k_p_z", k_p_z, 0.5);
    nh.param<double>("k_p_yaw", k_p_yaw, 0.04); // 已弃用 预留参数 当数据输出波动较为剧烈时可以降低偏航抽搐 需要将程序中相应的计算注释打开
    nh.param<double>("max_vel", max_vel, 0.5);
    nh.param<double>("max_vel_z", max_vel_z, 0.2);
    nh.param<double>("max_yaw", max_yaw, 0.4); // 已弃用 预留参数 当数据输出波动较为剧烈时可以降低偏航抽搐 需要将程序中相应的计算注释打开
    nh.param<double>("height", height, 1.5);

    // 降落相关参数
    // 当无人机当前位置与标签位置(x,y)误差小于error_xy且z小于error_z时 无人机直接降落 不再进行姿态调整了
    nh.param<double>("error_xy", error_xy, 0.05);
    nh.param<double>("error_z", error_z, 0.25);
    // 最后的俯冲速度 根据不同的无人机重量和地效（有些无人机在靠经地面时会被自己吹出的风反向吹开）调整land_vel和last_land_time能有效解决这个问题
    nh.param<double>("land_vel", land_v, -0.3);
    // land_vel在经历last_land_time时间后会直接锁桨
    nh.param<double>("last_land_time", land_time, 1.5);
    // 还有一个没有启用的参数 当无人机与标签Z轴上的误差达到阈值阈值后也会直接降落

    uav_name = "/" + uav_name + to_string(uav_id);
   

    ros::Subscriber uav_state_sub = nh.subscribe<sunray_msgs::UAVState>(uav_name + "/sunray/uav_state", 10, uav_state_callback);
    // 【订阅】目标点位置
    ros::Subscriber target_pos_sub = nh.subscribe<sunray_msgs::TargetsInFrameMsg>(uav_name + "/sunray_detect/qrcode_detection_ros", 1, tagCallback);
    ;
    // 【订阅】任务结束
    ros::Subscriber stop_tutorial_sub = nh.subscribe<std_msgs::Empty>(uav_name + "/sunray/stop_tutorial", 1, stop_tutorial_cb);

    // 【发布】无人机控制指令 （本节点 -> sunray_control_node）
    ros::Publisher control_cmd_pub = nh.advertise<sunray_msgs::UAVControlCMD>(uav_name + "/sunray/uav_control_cmd", 1);
    // 【发布】无人机设置指令（本节点 -> sunray_control_node）
    ros::Publisher uav_setup_pub = nh.advertise<sunray_msgs::UAVSetup>(uav_name + "/sunray/setup", 1);

    ros::Subscriber pose_sub = nh.subscribe(uav_name + "/mavros/local_position/pose", 10, pose_cb);

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
    cout << GREEN << "sim_mode                  : " << sim_mode << " " << TAIL << endl;
    cout << GREEN << "uav_name                  : " << uav_name << " " << TAIL << endl;
    cout << GREEN << "target_tpoic_name         : " << target_tpoic_name << " " << TAIL << endl;

    if (auto_takeoff)
    {
        ros::Duration(0.5).sleep();
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

        Logger::print_color(int(LogColor::blue), ">>>>>>> move to trajectory start point");
        uav_cmd.cmd = sunray_msgs::UAVControlCMD::XyVelZPos;
        uav_cmd.desired_vel[0] = 0.0;
        uav_cmd.desired_vel[1] = 0.0;
        uav_cmd.desired_pos[2] = height;
        control_cmd_pub.publish(uav_cmd);
        ros::Duration(2).sleep();
    }

    ros::Duration(0.5).sleep();
    ros::spinOnce();

    geometry_msgs::PoseStamped pose;

    while (!sub_flag && ros::ok())
    {
        Logger::print_color(int(LogColor::blue), "wait for pose!!!");
        ros::spinOnce();
        ros::Duration(1).sleep();
    }
    while (!tag_flag && ros::ok())
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
        if (landing_point_num > 10 && (((abs(x_rel) < error_xy && abs(y_rel) < error_xy && abs(z_rel) < error_z)) || abs(z_rel) < 0.20))
        {
            ros::Time stop_time = ros::Time::now();
            while ((ros::Time::now() - stop_time).toSec() < land_time)
            {
                uav_cmd.header.stamp = ros::Time::now();
                uav_cmd.cmd = sunray_msgs::UAVControlCMD::XyzVelYawBody;
                uav_cmd.desired_vel[0] = 0;
                uav_cmd.desired_vel[1] = 0;
                uav_cmd.desired_vel[2] = land_v;
                uav_cmd.desired_yaw = 0;
                control_cmd_pub.publish(uav_cmd);
                ros::Duration(0.1).sleep();
            }
            Logger::print_color(int(LogColor::blue), "land successfully!!!");
            uav_setup.header.stamp = ros::Time::now();
            uav_setup.cmd = sunray_msgs::UAVSetup::EMERGENCY_KILL;
            uav_setup_pub.publish(uav_setup);
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

                uav_cmd.header.stamp = ros::Time::now();
                uav_cmd.cmd = sunray_msgs::UAVControlCMD::XyzPosYawBody;
                uav_cmd.desired_pos[0] = x_rel / 2;
                uav_cmd.desired_pos[1] = y_rel / 2;
                uav_cmd.desired_pos[2] = -0.10;
                uav_cmd.desired_yaw = yaw_rel / 180.0 * M_PI / 2;
                // uav_cmd.desired_yaw = 0;

                // 优先调整水平距离
                if (abs(z_rel) < 1 && abs(z_rel) > 0.2 && (abs(x_rel) > 0.10 || abs(y_rel) > 0.10))
                {
                    uav_cmd.desired_pos[2] = -0.05;
                }

                cout << "x_rel: " << x_rel << " y_rel: " << y_rel << " z_rel: " << z_rel << " yaw_rel: " << yaw_rel << endl;
                control_cmd_pub.publish(uav_cmd);
            }
            continue;
        }
        if ((ros::Time::now() - last_time).toSec() > 5)
        {
            Logger::print_color(int(LogColor::blue), "over time!!! Land directly");
            landing_point_num = 0;
            uav_cmd.header.stamp = ros::Time::now();
            uav_cmd.cmd = sunray_msgs::UAVControlCMD::Land;
            control_cmd_pub.publish(uav_cmd);
            break;
        }
        if ((ros::Time::now() - last_time).toSec() > 1)
        {
            Logger::print_color(int(LogColor::blue), "lost the tag !!! rising and serching");
            uav_cmd.header.stamp = ros::Time::now();
            uav_cmd.cmd = sunray_msgs::UAVControlCMD::XyzPosYawBody;
            uav_cmd.desired_pos[0] = 0;
            uav_cmd.desired_pos[1] = 0;
            uav_cmd.desired_pos[2] = 0.05;
            uav_cmd.desired_yaw = 0;
            control_cmd_pub.publish(uav_cmd);
            continue;
        }
    }
}
