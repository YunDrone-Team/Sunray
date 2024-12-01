// ros头文件
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <iostream>

// topic 头文件
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/RCIn.h>
#include <nav_msgs/Path.h>

// #include "controller_test.h"
#include "printf_utils.h"
#include <signal.h>
#include "ros_msg_utils.h"

using namespace std;
#define TRA_WINDOW 2000
sunray_msgs::UAVState uav_state;
sunray_msgs::UAVControlCMD uav_cmd;
sunray_msgs::UAVSetup setup;
sunray_msgs::UAVState uav_control_state;
std::vector<geometry_msgs::PoseStamped> posehistory_vector_;

bool is_ground_station_control = false;
bool flag = false;

void uav_state_cb(const sunray_msgs::UAVState::ConstPtr &msg)
{
    uav_state = *msg;
}

void uav_control_state_cb(const sunray_msgs::UAVState::ConstPtr &msg)
{
    uav_control_state = *msg;
}

void mySigintHandler(int sig)
{
    ROS_INFO("[uav_command_pub] exit...");
    ros::shutdown();
    exit(0);
}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "uav_command_pub");
    ros::NodeHandle nh("~");
    signal(SIGINT, mySigintHandler);
    int uav_id;
    bool sim_mode;
    string uav_name{""};
    nh.param("uav_id", uav_id, 1);
    nh.param("sim_mode", sim_mode, true);
    nh.param<string>("uav_name", uav_name, "uav");
    nh.getParam("/communication_bridge/trajectory_ground_control", is_ground_station_control);

    uav_name = uav_name + std::to_string(uav_id);
    string topic_prefix = "/" + uav_name;
    // 【订阅】状态信息
    ros::Subscriber uav_state_sub = nh.subscribe<sunray_msgs::UAVState>(topic_prefix + "/sunray/uav_state", 1, uav_state_cb);

    // 【订阅】无人机控制信息
    ros::Subscriber uav_contorl_state_sub = nh.subscribe<sunray_msgs::UAVState>(topic_prefix + "/sunray/uav_state_cmd", 1, uav_control_state_cb);

    // 【发布】UAVCommand
    ros::Publisher uav_command_pub = nh.advertise<sunray_msgs::UAVControlCMD>(topic_prefix + "/sunray/uav_control_cmd", 1);

    // 【发布】UAVSetup
    ros::Publisher setup_pub = nh.advertise<sunray_msgs::UAVSetup>(topic_prefix + "/sunray/setup", 1);

    // 用于控制器测试的类，功能例如：生成圆形轨迹，8字轨迹等
    //  Controller_Test Controller_Test;
    //  Controller_Test.printf_param();

    int CMD = 0;
    float state_desired[4];
    bool yaw_rate = false;

    uav_cmd.header.stamp = ros::Time::now();
    uav_cmd.cmd = 3;
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

    float time_trajectory = 0.0;

    while (ros::ok())
    {
        ros::spinOnce();

        if (uav_control_state.control_mode != 2)
        {
            cout << YELLOW << "Please switch to COMMAND_CONTROL mode first" << TAIL << endl;
        }

        if (!is_ground_station_control)
        {
            cout << GREEN << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>UAV Terminal Control<<<<<<<<<<<<<<<<<<<<<<<<< " << TAIL << endl;
            cout << GREEN << "setup: "
                 << YELLOW << "101 " << GREEN << "arm or dis arm,"
                 << YELLOW << " 102 " << GREEN << "control_state,"
                 << YELLOW << " 103 " << GREEN << "takeoff,"
                 << YELLOW << " 104 " << GREEN << "hover,"
                 << YELLOW << " 105 " << GREEN << "land"<< TAIL << endl;
            cout << GREEN << "CMD: "
                 << YELLOW << "1 " << GREEN << "(XYZ_POS),"
                 << YELLOW << " 2 " << GREEN << "(XY_VEL_Z_POS),"
                 << YELLOW << " 3 " << GREEN << "(XYZ_VEL),"
                 << YELLOW << " 4 " << GREEN << "(XYZ_POS_BODY),"
                 << YELLOW << " 5 " << GREEN << "(XYZ_VEL_BODY),"
                 << YELLOW << " 6 " << GREEN << "(XY_VEL_Z_POS_BODY)" << TAIL << endl;
            cin >> CMD;
        }

        switch (CMD)
        {
        case 1:
            cout << BLUE << "XyzPos" << endl;
            cout << BLUE << "desired state: --- x [m] " << endl;
            cin >> state_desired[0];
            cout << BLUE << "desired state: --- y [m]" << endl;
            cin >> state_desired[1];
            cout << BLUE << "desired state: --- z [m]" << endl;
            cin >> state_desired[2];
            state_desired[3] = state_desired[3] / 180.0 * M_PI;
            uav_cmd.header.stamp = ros::Time::now();
            uav_cmd.cmd = 1;
            uav_cmd.desired_pos[0] = state_desired[0];
            uav_cmd.desired_pos[1] = state_desired[1];
            uav_cmd.desired_pos[2] = state_desired[2];
            uav_command_pub.publish(uav_cmd);
            cout << BLUE << "pos_des [X Y Z] : " << state_desired[0] << " [ m ] " << state_desired[1] << " [ m ] " << state_desired[2] << " [ m ] " << endl;
            break;
        case 2:
            cout << BLUE << "XyzVel" << endl;
            cout << BLUE << "desired state: --- x [m/s] " << endl;
            cin >> state_desired[0];
            cout << BLUE << "desired state: --- y [m/s]" << endl;
            cin >> state_desired[1];
            cout << BLUE << "desired state: --- z [m/s]" << endl;
            cin >> state_desired[2];
            uav_cmd.header.stamp = ros::Time::now();
            uav_cmd.cmd = 2;
            uav_cmd.desired_vel[0] = state_desired[0];
            uav_cmd.desired_vel[1] = state_desired[1];
            uav_cmd.desired_vel[2] = state_desired[2];
            uav_command_pub.publish(uav_cmd);
            cout << BLUE << "vel_des [X Y] : " << state_desired[0] << " [ m/s ] " << state_desired[1] << " [ m/s ] " << " pos_des [Z] : " << state_desired[2] << " [ m ] " << endl;
            break;
        case 3:
            cout << BLUE << "XyVelZPos" << endl;
            cout << BLUE << "desired state: --- x [m/s] " << endl;
            cin >> state_desired[0];
            cout << BLUE << "desired state: --- y [m/s]" << endl;
            cin >> state_desired[1];
            cout << BLUE << "desired state: --- z [m]" << endl;
            uav_cmd.header.stamp = ros::Time::now();
            uav_cmd.cmd = 3;
            uav_cmd.desired_vel[0] = state_desired[0];
            uav_cmd.desired_vel[1] = state_desired[1];
            uav_cmd.desired_pos[2] = state_desired[2];
            uav_command_pub.publish(uav_cmd);
            cout << BLUE << "vel_des [X Y Z] : " << state_desired[0] << " [ m/s ] " << state_desired[1] << " [ m/s ] " << state_desired[2] << " [ m/s ] " << endl;
            break;
        case 4:
        { // 固定 yaw，输入速度
            cout << BLUE << "Move for XYZ_VEL in END frame, input the desired velocity and yaw angle" << endl;
            cout << BLUE << "desired velocity: --- vx [m/s] " << endl;
            cin >> state_desired[0];
            cout << BLUE << "desired velocity: --- vy [m/s]" << endl;
            cin >> state_desired[1];
            cout << BLUE << "desired velocity: --- vz [m/s]" << endl;
            cin >> state_desired[2];
            cout << BLUE << "desired state: --- yaw [deg]:" << endl;
            cin >> state_desired[3];
            state_desired[3] = state_desired[3] / 180.0 * M_PI; // 转换为弧度

            uav_cmd.header.stamp = ros::Time::now();
            uav_cmd.cmd = 4; // 对应 XYZ_VEL 的命令
            uav_cmd.desired_vel[0] = state_desired[0];
            uav_cmd.desired_vel[1] = state_desired[1];
            uav_cmd.desired_vel[2] = state_desired[2];
            uav_cmd.desired_yaw = state_desired[3]; // 固定 yaw
            uav_command_pub.publish(uav_cmd);

            cout << BLUE << "vel_des [Vx Vy Vz] : " 
                << state_desired[0] << " [ m/s ] " 
                << state_desired[1] << " [ m/s ] " 
                << state_desired[2] << " [ m/s ] " << endl;
            cout << BLUE << "yaw_fixed : " << state_desired[3] / M_PI * 180.0 << " [ deg ] " << endl;
            break;
        }
        
        // case 4:
        //     cout << BLUE << "Move for XYZ_POS in BODY frame, input the desired position and yaw (or yaw rate) angle" << endl;
        //     cout << BLUE << "desired state: --- x [m] " << endl;
        //     cin >> state_desired[0];
        //     cout << BLUE << "desired state: --- y [m]" << endl;
        //     cin >> state_desired[1];
        //     cout << BLUE << "desired state: --- z [m]" << endl;
        //     cin >> state_desired[2];
        //     cout << BLUE << "desired state: --- yaw [deg]:" << endl;
        //     cin >> state_desired[3];
        //     state_desired[4] = yaw_rate;
        //     state_desired[3] = state_desired[3] / 180.0 * M_PI;

        //     uav_cmd.header.stamp = ros::Time::now();
        //     uav_cmd.cmd = sunray_msgs::UAVControlCMD::XYZ_POS_BODY;
        //     uav_cmd.desired_pos[0] = state_desired[0];
        //     uav_cmd.desired_pos[1] = state_desired[1];
        //     uav_cmd.desired_pos[2] = state_desired[2];
        //     uav_cmd.desired_yaw = state_desired[3];
        //     uav_cmd.desired_yaw_setuprate = state_desired[3];
        //
        //     uav_command_pub.publish(uav_cmd);
        //     cout << BLUE << "pos_des [X Y Z] : " << state_desired[0] << " [ m ] " << state_desired[1] << " [ m ] " << state_desired[2] << " [ m ] " << endl;
        //     cout << BLUE << "yaw_des : " << state_desired[3] / M_PI * 180.0 << " [ deg ] " << endl;
        //     break;
        // case 5:
        //     cout << BLUE << "Move for XYZ_VEL in BODY frame, input the desired position and yaw (or yaw rate) angle" << endl;
        //     cout << BLUE << "desired state: --- x [m/s] " << endl;
        //     cin >> state_desired[0];
        //     cout << BLUE << "desired state: --- y [m/s]" << endl;
        //     cin >> state_desired[1];
        //     cout << BLUE << "desired state: --- z [m/s]" << endl;
        //     cin >> state_desired[2];
        //     cout << BLUE << "desired state: --- yaw [deg]:" << endl;
        //     cin >> state_desired[3];
        //     state_desired[4] = yaw_rate;
        //     state_desired[3] = state_desired[3] / 180.0 * M_PI;

        //     uav_cmd.header.stamp = ros::Time::now();
        //     uav_cmd.cmd = sunray_msgs::UAVControlCMD::XYZ_VEL_BODY;
        //     uav_cmd.desired_vel[0] = state_desired[0];
        //     uav_cmd.desired_vel[1] = state_desired[1];
        //     uav_cmd.desired_vel[2] = state_desired[2];
        //     uav_cmd.desired_pos[0] = 0.0;
        //     uav_cmd.desired_pos[1] = 0.0;
        //     uav_cmd.desired_pos[2] = 0.0;
        //     uav_cmd.desired_yaw = state_desired[3];
        //     uav_cmd.desired_yaw_rate = state_desired[3];
        //
        //     uav_command_pub.publish(uav_cmd);
        //     cout << BLUE << "vel_des [X Y Z] : " << state_desired[0] << " [ m/s ] " << state_desired[1] << " [ m/s ] " << state_desired[2] << " [ m/s ] " << endl;
        //     cout << BLUE << "yaw_des : " << state_desired[3] / M_PI * 180.0 << " [ deg ] " << endl;
        //     break;
        // case 6:
        //     cout << BLUE << "Move for XY_VEL_Z_POS in BODY frame, input the desired position and yaw (or yaw rate) angle" << endl;
        //     cout << BLUE << "desired state: --- x [m/s] " << endl;
        //     cin >> state_desired[0];
        //     cout << BLUE << "desired state: --- y [m/s]" << endl;
        //     cin >> state_desired[1];
        //     cout << BLUE << "desired state: --- z [m]" << endl;
        //     cin >> state_desired[2];
        //     cout << BLUE << "desired state: --- yaw [deg]:" << endl;
        //     cin >> state_desired[3];
        //     state_desired[4] = yaw_rate;
        //     state_desired[3] = state_desired[3] / 180.0 * M_PI;

        //     uav_cmd.header.stamp = ros::Time::now();
        //     uav_cmd.cmd = sunray_msgs::UAVControlCMD::XY_VEL_Z_POS_BODY;
        //     uav_cmd.desired_vel[0] = state_desired[0];
        //     uav_cmd.desired_vel[1] = state_desired[1];
        //     uav_cmd.desired_vel[2] = 0.0;
        //     uav_cmd.desired_pos[0] = 0.0;
        //     uav_cmd.desired_pos[1] = 0.0;
        //     uav_cmd.desired_pos[2] = state_desired[2];
        //     uav_cmd.desired_yaw = state_desired[3];
        //     uav_cmd.desired_yaw_rate = state_desired[3];
        //
        //     uav_command_pub.publish(uav_cmd);
        //     cout << BLUE << "vel_des [X Y] : " << state_desired[0] << " [ m/s ] " << state_desired[1] << " [ m/s ] " << " pos_des [Z] : " << state_desired[2] << " [ m ] " << endl;
        //     cout << BLUE << "yaw_des : " << state_desired[3] / M_PI * 180.0 << " [ deg ] " << endl;
        //     break;
        case 7:
        { // 固定 yaw，输入速度和高度
            cout << BLUE << "Move for XYZ_VEL in END frame, input the desired velocity and height with fixed yaw" << endl;
            cout << BLUE << "desired velocity: --- vx [m/s] " << endl;
            cin >> state_desired[0];
            cout << BLUE << "desired velocity: --- vy [m/s]" << endl;
            cin >> state_desired[1];
            cout << BLUE << "desired height: --- z [m]" << endl;
            cin >> state_desired[2];  // 高度（Z轴）
            cout << BLUE << "desired state: --- yaw [deg]:" << endl;
            cin >> state_desired[3];  // 固定yaw
            state_desired[3] = state_desired[3] / 180.0 * M_PI; // 转换为弧度

            uav_cmd.header.stamp = ros::Time::now();
            uav_cmd.cmd = 7;  // 对应 XYZ_VEL 的命令
            uav_cmd.desired_vel[0] = state_desired[0]; // X轴速度
            uav_cmd.desired_vel[1] = state_desired[1]; // Y轴速度
            uav_cmd.desired_vel[2] = 0;  // Z轴速度设为 0，因为我们要指定高度
            uav_cmd.desired_pos[2] = state_desired[2];// 固定高度
            uav_cmd.desired_yaw = state_desired[3]; // 固定 yaw
            uav_command_pub.publish(uav_cmd);

            cout << BLUE << "vel_des [Vx Vy] : " 
                << state_desired[0] << " [ m/s ] " 
                << state_desired[1] << " [ m/s ] " << endl;
            cout << BLUE << "height_des : " << state_desired[2] << " [ m ] " << endl;
            cout << BLUE << "yaw_fixed : " << state_desired[3] / M_PI * 180.0 << " [ deg ] " << endl;
            break;
        }

        case 10: 
        {   // 输入位置、速度和yaw
            cout << BLUE << "Move to a specific position with desired velocity and yaw" << endl;

            // 输入位置
            cout << BLUE << "Enter the desired position (x, y, z):" << endl;
            cout << BLUE << "desired position --- X [m]: ";
            cin >> state_desired[0];
            cout << BLUE << "desired position --- Y [m]: ";
            cin >> state_desired[1];
            cout << BLUE << "desired position --- Z [m]: ";
            cin >> state_desired[2];

            // 输入速度
            cout << BLUE << "Enter the desired velocity (V_x, V_y, V_z):" << endl;
            cout << BLUE << "desired velocity --- V_x [m/s]: ";
            cin >> state_desired[3];
            cout << BLUE << "desired velocity --- V_y [m/s]: ";
            cin >> state_desired[4];
            cout << BLUE << "desired velocity --- V_z [m/s]: ";
            cin >> state_desired[5];

            // 输入yaw
            cout << BLUE << "Enter the desired yaw angle [deg]: ";
            cin >> state_desired[6];
            state_desired[6] = state_desired[6] / 180.0 * M_PI; // 转换为弧度

            // 填充指令消息
            uav_cmd.header.stamp = ros::Time::now();
            uav_cmd.cmd = 10;  // 新的命令模式
            uav_cmd.desired_pos[0] = state_desired[0]; // X 位置
            uav_cmd.desired_pos[1] = state_desired[1]; // Y 位置
            uav_cmd.desired_pos[2] = state_desired[2]; // Z 位置
            uav_cmd.desired_vel[0] = state_desired[3]; // X 速度
            uav_cmd.desired_vel[1] = state_desired[4]; // Y 速度
            uav_cmd.desired_vel[2] = state_desired[5]; // Z 速度
            uav_cmd.desired_yaw = state_desired[6];    // 固定 yaw

            // 发布指令
            uav_command_pub.publish(uav_cmd);

            // 输出验证
            cout << BLUE << "Position [x y z]: " 
                << state_desired[0] << " [m], " 
                << state_desired[1] << " [m], " 
                << state_desired[2] << " [m]" << endl;
            cout << BLUE << "Velocity [vx vy vz]: " 
                << state_desired[3] << " [m/s], " 
                << state_desired[4] << " [m/s], " 
                << state_desired[5] << " [m/s]" << endl;
            cout << BLUE << "Yaw: " << state_desired[6] / M_PI * 180.0 << " [deg]" << endl;

            break;
        }

        case 16:
        {
            cout << BLUE << "Move for XYZ_POS in END frame, input the desired position and yaw (or yaw rate)" << endl;

            // 输入 x, y, z 位置
            cout << BLUE << "Enter desired position:" << endl;
            cout << BLUE << "x [m]: ";
            cin >> state_desired[0];
            cout << BLUE << "y [m]: ";
            cin >> state_desired[1];
            cout << BLUE << "z [m]: ";
            cin >> state_desired[2];

            // 填充 UAV 指令
            uav_cmd.header.stamp = ros::Time::now();
            uav_cmd.cmd = 16;  // 表示 XYZ_POS 模式
            uav_cmd.desired_pos[0] = state_desired[0]; // x
            uav_cmd.desired_pos[1] = state_desired[1]; // y
            uav_cmd.desired_pos[2] = state_desired[2]; // z
            uav_cmd.desired_yaw_rate = 0.0;            // 如果是固定 yaw，yaw_rate 为 0

            // 发布指令
            uav_command_pub.publish(uav_cmd);

            // 输出结果
            cout << BLUE << "Desired position [X Y Z]: " 
                << state_desired[0] << " [m], " 
                << state_desired[1] << " [m], " 
                << state_desired[2] << " [m]" << endl;
            
            break;
        }

        case 17:
        {
            cout << BLUE << "Move for XYZ_VEL in END frame, input the desired velocity and yaw (or yaw rate)" << endl;

            // 输入 vx, vy, vz 速度
            cout << BLUE << "Enter desired velocity:" << endl;
            cout << BLUE << "vx [m/s]: ";
            cin >> state_desired[0];
            cout << BLUE << "vy [m/s]: ";
            cin >> state_desired[1];
            cout << BLUE << "vz [m/s]: ";
            cin >> state_desired[2];

            // 填充 UAV 指令
            uav_cmd.header.stamp = ros::Time::now();
            uav_cmd.cmd = 17;  // 表示 XYZ_VEL 模式
            uav_cmd.desired_vel[0] = state_desired[0]; // vx
            uav_cmd.desired_vel[1] = state_desired[1]; // vy
            uav_cmd.desired_vel[2] = state_desired[2]; // vz
            
            uav_cmd.desired_yaw_rate = 0.0;            // 如果是固定 yaw，yaw_rate 为 0

            // 发布指令
            uav_command_pub.publish(uav_cmd);

            // 输出结果
            cout << BLUE << "Desired velocity [Vx Vy Vz]: " 
                << state_desired[0] << " [m/s], " 
                << state_desired[1] << " [m/s], " 
                << state_desired[2] << " [m/s]" << endl;
            break;
        }

        case 18:
        { // 固定 yaw，输入速度和高度
            cout << BLUE << "Move for XYZ_VEL in END frame, input the desired velocity and height with fixed yaw" << endl;
            cout << BLUE << "desired velocity: --- V_x [m/s] " << endl;
            cin >> state_desired[0];
            cout << BLUE << "desired velocity: --- V_y [m/s]" << endl;
            cin >> state_desired[1];
            cout << BLUE << "desired height: --- Z [m]" << endl;
            cin >> state_desired[2];  // 高度（Z轴）
            

            uav_cmd.header.stamp = ros::Time::now();
            uav_cmd.cmd = 18;  // 对应 XYZ_VEL 的命令
            uav_cmd.desired_vel[0] = state_desired[0]; // X轴速度
            uav_cmd.desired_vel[1] = state_desired[1]; // Y轴速度
            uav_cmd.desired_vel[2] = 0;  // Z轴速度设为 0，因为我们要指定高度
            uav_cmd.desired_pos[2] = state_desired[2];// 固定高度
            
            uav_command_pub.publish(uav_cmd);

            cout << BLUE << "vel_des [Vx Vy] : " 
                << state_desired[0] << " [ m/s ] " 
                << state_desired[1] << " [ m/s ] " << endl;
            cout << BLUE << "height_des : " << state_desired[2] << " [ m ] " << endl;
            break;
        }

        case 19:
        {
            cout << BLUE << "Move for XYZ_POS in BODY frame, input the desired position and yaw (or yaw rate)" << endl;

            // 输入 x, y, z 相对位置
            cout << BLUE << "Enter desired position relative to the BODY frame:" << endl;
            cout << BLUE << "x [m]: ";
            cin >> state_desired[0];
            cout << BLUE << "y [m]: ";
            cin >> state_desired[1];
            cout << BLUE << "z [m]: ";
            cin >> state_desired[2];

            // 输入 yaw
            cout << BLUE << "Enter desired yaw angle [deg]: ";
            cin >> state_desired[3];
            state_desired[3] = state_desired[3] / 180.0 * M_PI; // 转换为弧度

            // 填充 UAV 指令
            uav_cmd.header.stamp = ros::Time::now();
            uav_cmd.cmd = 19; // BODY 坐标系的位置控制
            uav_cmd.desired_pos[0] = state_desired[0]; // x
            uav_cmd.desired_pos[1] = state_desired[1]; // y
            uav_cmd.desired_pos[2] = state_desired[2]; // z
            uav_cmd.desired_yaw = state_desired[3];    // 固定 yaw
            uav_cmd.desired_yaw_rate = 0.0;            // 固定 yaw 时不设置 yaw 速率

            // 发布指令
            uav_command_pub.publish(uav_cmd);

            // 输出结果
            cout << BLUE << "Desired position in BODY frame [X Y Z]: " 
                << state_desired[0] << " [m], " 
                << state_desired[1] << " [m], " 
                << state_desired[2] << " [m]" << endl;
            cout << BLUE << "Desired yaw: " 
                << state_desired[3] / M_PI * 180.0 << " [deg]" << endl;

            break;
        }

        case 20: 
        {
            cout << BLUE << "Move with Velocity in BODY frame, input desired velocity and yaw angle" << endl;

            // 输入 x, y, z 方向速度
            cout << BLUE << "Enter desired velocity in BODY frame:" << endl;
            cout << BLUE << "x velocity [m/s]: ";
            cin >> state_desired[0];
            cout << BLUE << "y velocity [m/s]: ";
            cin >> state_desired[1];
            cout << BLUE << "z velocity [m/s]: ";
            cin >> state_desired[2];

            // 输入 yaw
            cout << BLUE << "Enter desired yaw angle [deg]: ";
            cin >> state_desired[3];
            state_desired[3] = state_desired[3] / 180.0 * M_PI; // 转换为弧度

            // 填充 UAV 指令
            uav_cmd.header.stamp = ros::Time::now();
            uav_cmd.cmd = 20; // BODY 坐标系的速度控制
            uav_cmd.desired_vel[0] = state_desired[0]; // x velocity
            uav_cmd.desired_vel[1] = state_desired[1]; // y velocity
            uav_cmd.desired_vel[2] = state_desired[2]; // z velocity
            uav_cmd.desired_yaw = state_desired[3];    // 固定 yaw
            uav_cmd.desired_yaw_rate = 0.0;            // 无需设置 yaw 速率

            // 发布指令
            uav_command_pub.publish(uav_cmd);

            // 输出结果
            cout << BLUE << "Desired velocity in BODY frame [X Y Z]: "
                << state_desired[0] << " [m/s], "
                << state_desired[1] << " [m/s], "
                << state_desired[2] << " [m/s]" << endl;
            cout << BLUE << "Desired yaw: "
                << state_desired[3] / M_PI * 180.0 << " [deg]" << endl;

            break;
        }

        case 21: 
        {
            cout << BLUE << "Move with Velocity in BODY frame (X, Y), Position in Z, and fixed Yaw" << endl;

            // 输入 X 和 Y 方向速度
            cout << BLUE << "Enter desired velocity in BODY frame:" << endl;
            cout << BLUE << "x velocity [m/s]: ";
            cin >> state_desired[0];
            cout << BLUE << "y velocity [m/s]: ";
            cin >> state_desired[1];

            // 输入 Z 方向位置
            cout << BLUE << "Enter desired Z position [m]: ";
            cin >> state_desired[2];

            // 输入 yaw
            cout << BLUE << "Enter desired yaw angle [deg]: ";
            cin >> state_desired[3];
            state_desired[3] = state_desired[3] / 180.0 * M_PI; // 转换为弧度

            // 填充 UAV 指令
            uav_cmd.header.stamp = ros::Time::now();
            uav_cmd.cmd = 21; // BODY 坐标系的速度控制 (X, Y) 和位置控制 (Z)
            uav_cmd.desired_vel[0] = state_desired[0]; // x velocity
            uav_cmd.desired_vel[1] = state_desired[1]; // y velocity
            uav_cmd.desired_pos[2] = state_desired[2]; // z position
            uav_cmd.desired_yaw = state_desired[3];    // 固定 yaw
            uav_cmd.desired_yaw_rate = 0.0;            // 无需设置 yaw 速率

            // 发布指令
            uav_command_pub.publish(uav_cmd);

            // 输出结果
            cout << BLUE << "Desired velocity in BODY frame [X Y]: "
                << state_desired[0] << " [m/s], "
                << state_desired[1] << " [m/s]" << endl;
            cout << BLUE << "Desired Z position: " 
                << state_desired[2] << " [m]" << endl;
            cout << BLUE << "Desired yaw: " 
                << state_desired[3] / M_PI * 180.0 << " [deg]" << endl;

            break;
        }

        // case 100:
        // {
        //     yaw_rate = !yaw_rate;
        //     break;
        // }
        case 101:
        {
            int arming;
            cout << BLUE << "Please select Operation 1 arm 0 disarm" << std::endl;
            cin >> arming;
            if (arming == 1)
            {
                setup.cmd = 1;
                setup_pub.publish(setup);
            }
            else if (arming == 0)
            {
                setup.cmd = 0;
                setup_pub.publish(setup);
            }
            else
            {
                std::cout << BLUE << "input error" << std::endl;
            }
            break;
        }
        case 102:
        {
            int control_state;
            cout << BLUE << "control_state 1 INIT 2 RC_CONTROL 3 CMD_CONTROL 4 LAND_CONTROL 5 WITHOUT_CONTROL" << std::endl;
            cin >> control_state;
            if (control_state == 1)
                setup.control_state = "INIT";
            else if (control_state == 2)
                setup.control_state = "RC_CONTROL";
            else if (control_state == 3)
                setup.control_state = "CMD_CONTROL";
            else if (control_state == 4)
                setup.control_state = "LAND_CONTROL";
            else if (control_state == 5)
                setup.control_state = "WITHOUT_CONTROL";
            else
            {
                std::cout << BLUE << "input error" << std::endl;
                break;
            }
            setup.cmd = 4;
            setup_pub.publish(setup);
            break;
        }
        case 103:
            uav_cmd.header.stamp = ros::Time::now();
            uav_cmd.cmd = 100;
            uav_command_pub.publish(uav_cmd);
            std::cout<<"takeoff"<<std::endl;
            break;
        case 104:
            uav_cmd.header.stamp = ros::Time::now();
            uav_cmd.cmd = 102;
            uav_command_pub.publish(uav_cmd);
            std::cout<<"hover"<<std::endl;
            break;
        case 105:
            uav_cmd.header.stamp = ros::Time::now();
            uav_cmd.cmd = 101;
            uav_command_pub.publish(uav_cmd);
            std::cout<<"land"<<std::endl;
            break;
        }

        ros::Duration(0.5).sleep();
    }
    return 0;
}
