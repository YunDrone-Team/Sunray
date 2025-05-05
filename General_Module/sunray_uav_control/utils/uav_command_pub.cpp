// ros头文件
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <iostream>

// topic 头文件
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/RCIn.h>
#include <nav_msgs/Path.h>

#include "traj_generator.h"
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

bool flag = false;
int Trjectory_mode;
int trajectory_total_time;

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
    int uav_num;
    string uav_name{""};
    nh.param("uav_id", uav_id, 1);
    nh.param("uav_num", uav_num, 1);
    nh.param<string>("uav_name", uav_name, "uav");

    //用于控制器测试的类，功能例如：生成圆形轨迹，8字轨迹等
    TRAJ_GENERATOR traj_generator;

    uav_name = uav_name + std::to_string(uav_id);
    string topic_prefix = "/" + uav_name;
    // 【订阅】状态信息
    ros::Subscriber uav_state_sub = nh.subscribe<sunray_msgs::UAVState>(topic_prefix + "/sunray/uav_state", 1, uav_state_cb);

    // 【订阅】无人机控制信息
    ros::Subscriber uav_contorl_state_sub = nh.subscribe<sunray_msgs::UAVState>(topic_prefix + "/sunray/uav_state_cmd", 1, uav_control_state_cb);

    //【发布】UAVCommand
    ros::Publisher ref_trajectory_pub = nh.advertise<nav_msgs::Path>(topic_prefix + "/sunray/reference_trajectory", 10);


    // 【发布】UAVCommand
    std::vector<ros::Publisher> uav_command_pub;
    std::vector<ros::Publisher> uav_setup_pub;
    for (int i = 0; i < uav_num; i++)
    {
        uav_name = "uav" + std::to_string(i + 1);
        topic_prefix = "/" + uav_name;
        uav_command_pub.push_back(nh.advertise<sunray_msgs::UAVControlCMD>(topic_prefix + "/sunray/uav_control_cmd", 1));
        uav_setup_pub.push_back(nh.advertise<sunray_msgs::UAVSetup>(topic_prefix + "/sunray/setup", 1));
    }

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
        cout << GREEN << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>UAV Terminal Control<<<<<<<<<<<<<<<<<<<<<<<<< " << TAIL << endl;
        cout << GREEN << "setup: "
                << YELLOW << "101 " << GREEN << "arm or dis arm,"
                << YELLOW << " 102 " << GREEN << "control_mode,"
                << YELLOW << " 103 " << GREEN << "takeoff,"
                << YELLOW << " 104 " << GREEN << "hover,"
                << YELLOW << " 105 " << GREEN << "land,"
                << YELLOW << " 106 " << GREEN << "return,"
                << YELLOW << " 107 " << GREEN << "waypoint,"
                << YELLOW << " 108 " << GREEN << "goalpoint"
                << YELLOW << " 110 " << GREEN << "tajectory test,"<< TAIL << endl;
        cout << GREEN << "CMD: "
                << YELLOW << "1 " << GREEN << "(XYZ_POS),"
                << YELLOW << " 2 " << GREEN << "(XyzVel),"
                << YELLOW << " 3 " << GREEN << "(XyVelZPos),"
                << YELLOW << " 4 " << GREEN << "(XyzPosYaw),"
                // << YELLOW << " 5 " << GREEN << "(XyzPosYawrate),"
                << YELLOW << " 6 " << GREEN << "(XyzVelYaw),"
                // << YELLOW << " 7 " << GREEN << "(XyzVelYawrate),"
                << YELLOW << " 8 " << GREEN << "(XyVelZPosYaw),"
                << YELLOW << " 14 " << GREEN << "(XyzPosYawBody),"
                << YELLOW << " 15 " << GREEN << "(XyzVelYawBody),"
                << YELLOW << " 17 " << GREEN << "(GlobalPos)" << TAIL << endl;
        cin >> CMD;

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
            uav_cmd.cmd = sunray_msgs::UAVControlCMD::XyzPos;
            ;
            uav_cmd.desired_pos[0] = state_desired[0];
            uav_cmd.desired_pos[1] = state_desired[1];
            uav_cmd.desired_pos[2] = state_desired[2];
            uav_command_pub[0].publish(uav_cmd);
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
            uav_cmd.cmd = sunray_msgs::UAVControlCMD::XyzVel;
            uav_cmd.desired_vel[0] = state_desired[0];
            uav_cmd.desired_vel[1] = state_desired[1];
            uav_cmd.desired_vel[2] = state_desired[2];
            uav_command_pub[0].publish(uav_cmd);
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
            uav_cmd.cmd = sunray_msgs::UAVControlCMD::XyVelZPos;
            uav_cmd.desired_vel[0] = state_desired[0];
            uav_cmd.desired_vel[1] = state_desired[1];
            uav_cmd.desired_pos[2] = state_desired[2];
            uav_command_pub[0].publish(uav_cmd);
            cout << BLUE << "vel_des [X Y Z] : " << state_desired[0] << " [ m/s ] " << state_desired[1] << " [ m/s ] " << state_desired[2] << " [ m/s ] " << endl;
            break;
        case 4:
        { // 固定 yaw，位置
            cout << BLUE << "XyzPosYaw " << endl;
            cout << BLUE << "desired velocity: --- x [m] " << endl;
            cin >> state_desired[0];
            cout << BLUE << "desired velocity: --- y [m]" << endl;
            cin >> state_desired[1];
            cout << BLUE << "desired velocity: --- z [m]" << endl;
            cin >> state_desired[2];
            cout << BLUE << "desired state: --- yaw [deg]:" << endl;
            cin >> state_desired[3];
            state_desired[3] = state_desired[3] / 180.0 * M_PI; // 转换为弧度

            uav_cmd.header.stamp = ros::Time::now();
            uav_cmd.cmd = sunray_msgs::UAVControlCMD::XyzPosYaw;
            uav_cmd.desired_pos[0] = state_desired[0];
            uav_cmd.desired_pos[1] = state_desired[1];
            uav_cmd.desired_pos[2] = state_desired[2];
            uav_cmd.desired_yaw = state_desired[3]; // 固定 yaw
            uav_command_pub[0].publish(uav_cmd);

            cout << BLUE << "vel_des [Vx Vy Vz] : "
                 << state_desired[0] << " [ m/s ] "
                 << state_desired[1] << " [ m/s ] "
                 << state_desired[2] << " [ m/s ] " << endl;
            cout << BLUE << "yaw_fixed : " << state_desired[3] / M_PI * 180.0 << " [ deg ] " << endl;
            break;
        }
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
        //     uav_command_pub[0].publish(uav_cmd);
        //     cout << BLUE << "vel_des [X Y Z] : " << state_desired[0] << " [ m/s ] " << state_desired[1] << " [ m/s ] " << state_desired[2] << " [ m/s ] " << endl;
        //     cout << BLUE << "yaw_des : " << state_desired[3] / M_PI * 180.0 << " [ deg ] " << endl;
        //     break;
        case 6:
            cout << BLUE << "XyzVelYaw " << endl;
            cout << BLUE << "desired state: --- x [m/s] " << endl;
            cin >> state_desired[0];
            cout << BLUE << "desired state: --- y [m/s]" << endl;
            cin >> state_desired[1];
            cout << BLUE << "desired state: --- z [m/s]" << endl;
            cin >> state_desired[2];
            cout << BLUE << "desired state: --- yaw [deg]:" << endl;
            cin >> state_desired[3];
            state_desired[4] = yaw_rate;
            state_desired[3] = state_desired[3] / 180.0 * M_PI;

            uav_cmd.header.stamp = ros::Time::now();
            uav_cmd.cmd = sunray_msgs::UAVControlCMD::XyzVelYaw;
            uav_cmd.desired_vel[0] = state_desired[0];
            uav_cmd.desired_vel[1] = state_desired[1];
            uav_cmd.desired_vel[2] = state_desired[2];
            uav_cmd.desired_yaw = state_desired[3];
            uav_cmd.desired_yaw_rate = state_desired[3];

            uav_command_pub[0].publish(uav_cmd);
            cout << BLUE << "vel_des [X Y Z] : " << state_desired[0] << " [ m/s ] " << state_desired[1] << " [ m/s ] " << state_desired[2] << " [ m/s ] " << endl;
            cout << BLUE << "yaw_des : " << state_desired[3] / M_PI * 180.0 << " [ deg ] " << endl;
            break;
        case 8:
        { // 固定 yaw，输入速度和高度
            cout << BLUE << "XyVelZPosYaw" << endl;
            cout << BLUE << "desired state: --- vx [m/s] " << endl;
            cin >> state_desired[0];
            cout << BLUE << "desired state: --- vy [m/s]" << endl;
            cin >> state_desired[1];
            cout << BLUE << "desired state: --- z [m]" << endl;
            cin >> state_desired[2]; // 高度（Z轴）
            cout << BLUE << "desired state: --- yaw [deg]:" << endl;
            cin >> state_desired[3];                            // 固定yaw
            state_desired[3] = state_desired[3] / 180.0 * M_PI; // 转换为弧度

            uav_cmd.header.stamp = ros::Time::now();
            uav_cmd.cmd = sunray_msgs::UAVControlCMD::XyVelZPosYaw;
            uav_cmd.desired_vel[0] = state_desired[0]; // X轴速度
            uav_cmd.desired_vel[1] = state_desired[1]; // Y轴速度
            uav_cmd.desired_vel[2] = 0;                // Z轴速度设为 0，因为我们要指定高度
            uav_cmd.desired_pos[2] = state_desired[2]; // 固定高度
            uav_cmd.desired_yaw = state_desired[3];    // 固定 yaw
            uav_command_pub[0].publish(uav_cmd);

            cout << BLUE << "vel_des [Vx Vy] : "
                 << state_desired[0] << " [ m/s ] "
                 << state_desired[1] << " [ m/s ] " << endl;
            cout << BLUE << "height_des : " << state_desired[2] << " [ m ] " << endl;
            cout << BLUE << "yaw_fixed : " << state_desired[3] / M_PI * 180.0 << " [ deg ] " << endl;
            break;
        }

        case 14:
        {
            cout << BLUE << "XyzPosYawBody" << endl;
            cout << BLUE << "desired state: --- x [m] " << endl;
            cin >> state_desired[0];
            cout << BLUE << "desired state: --- y [m]" << endl;
            cin >> state_desired[1];
            cout << BLUE << "desired state: --- z [m]" << endl;
            cin >> state_desired[2]; // 高度（Z轴）
            cout << BLUE << "desired state: --- yaw [deg]:" << endl;
            cin >> state_desired[3];                            // 固定yaw
            state_desired[3] = state_desired[3] / 180.0 * M_PI; // 转换为弧度

            // 填充 UAV 指令 Z轴速度设为 0，因为我们要指定高度
            uav_cmd.header.stamp = ros::Time::now();
            uav_cmd.cmd = sunray_msgs::UAVControlCMD::XyzPosYawBody;
            uav_cmd.desired_pos[0] = state_desired[0];
            uav_cmd.desired_pos[1] = state_desired[1];
            uav_cmd.desired_pos[2] = state_desired[2];
            uav_cmd.desired_yaw = state_desired[3];
            uav_cmd.desired_yaw_rate = 0.0;
            uav_command_pub[0].publish(uav_cmd);

            cout << BLUE << "pos_des [X Y Z] : " << state_desired[0] << " [ m ] " << state_desired[1] << " [ m ] " << state_desired[2] << " [ m ] " << endl;
            cout << BLUE << "yaw_des : " << state_desired[3] / M_PI * 180.0 << " [ deg ] " << endl;

            break;
        }

        case 15:
        {
            cout << BLUE << "XyVelZPosYawBody" << endl;
            cout << BLUE << "desired state: --- x [m/s] " << endl;
            cin >> state_desired[0];
            cout << BLUE << "desired state: --- y [m/s]" << endl;
            cin >> state_desired[1];
            cout << BLUE << "desired state: --- z [m]" << endl;
            cin >> state_desired[2]; // 高度（Z轴）
            cout << BLUE << "desired state: --- yaw [deg]:" << endl;
            cin >> state_desired[3];                            // 固定yaw
            state_desired[3] = state_desired[3] / 180.0 * M_PI; // 转换为弧度

            // 填充 UAV 指令 Z轴速度设为 0，因为我们要指定高度
            uav_cmd.header.stamp = ros::Time::now();
            uav_cmd.cmd = sunray_msgs::UAVControlCMD::XyVelZPosYawBody;
            uav_cmd.desired_vel[0] = state_desired[0];
            uav_cmd.desired_vel[1] = state_desired[1];
            uav_cmd.desired_pos[2] = state_desired[2];
            uav_cmd.desired_yaw = state_desired[3];
            uav_cmd.desired_yaw_rate = 0.0;
            uav_command_pub[0].publish(uav_cmd);

            cout << BLUE << "vel_des [X Y Z] : " << state_desired[0] << " [ m/s ] " << state_desired[1] << " [ m/s ] " << state_desired[2] << " [ m/s ] " << endl;
            cout << BLUE << "yaw_des : " << state_desired[3] / M_PI * 180.0 << " [ deg ] " << endl;

            break;
        }

        // case 17:
        // {
        //     cout << BLUE << "waypoint" << endl;
        //     uav_cmd.header.stamp = ros::Time::now();
        //     uav_cmd.cmd = sunray_msgs::UAVControlCMD::Waypoint;
        //     uav_command_pub[0].publish(uav_cmd);

        //     break;
        // }

        // case 17:
        // {
        //     cout << BLUE << "XyVelZPosYawBody" << endl;
        //     cout << BLUE << "latitude:" << endl;
        //     cin >> state_desired[0];
        //     cout << BLUE << "longitude:" << endl;
        //     cin >> state_desired[1];
        //     // 高度为相对高度
        //     cout << BLUE << "altitude: --- z [m]" << endl;
        //     cin >> state_desired[2]; // 高度（Z轴）
        //     cout << BLUE << "desired state: --- yaw [deg]:" << endl;
        //     cin >> state_desired[3];                            // 固定yaw
        //     state_desired[3] = state_desired[3] / 180.0 * M_PI; // 转换为弧度

        //     // 
        //     uav_cmd.header.stamp = ros::Time::now();
        //     uav_cmd.cmd = sunray_msgs::UAVControlCMD::GlobalPos;
        //     uav_cmd.latitude = state_desired[0];
        //     uav_cmd.longitude = state_desired[1];
        //     uav_cmd.altitude = state_desired[2];
        //     uav_cmd.desired_yaw = state_desired[3];
        //     uav_cmd.desired_yaw_rate = 0.0;
        //     uav_command_pub[0].publish(uav_cmd);

        //     cout << BLUE << "latitude longitude altitude: " << state_desired[0] << " " << state_desired[1] << " " << state_desired[2] << " " << endl;
        //     cout << BLUE << "yaw_des : " << state_desired[3] / M_PI * 180.0 << " [ deg ] " << endl;

        //     break;
        // }

        case 17:
        {
            cout << BLUE << "goalpoint" << endl;
            cout << BLUE << "desired state: --- x [m] " << endl;
            cin >> state_desired[0];
            cout << BLUE << "desired state: --- y [m]" << endl;
            cin >> state_desired[1];
            cout << BLUE << "desired state: --- z [m]" << endl;
            cin >> state_desired[2];
            cout << BLUE << "desired state: --- yaw [deg]:" << endl;
            cin >> state_desired[3];                            // 固定yaw
            state_desired[3] = state_desired[3] / 180.0 * M_PI; // 转换为弧度

            // 填充 UAV 指令 Z轴速度设为 0，因为我们要指定高度
            uav_cmd.header.stamp = ros::Time::now();
            uav_cmd.cmd = sunray_msgs::UAVControlCMD::Point;
            uav_cmd.desired_pos[0] = state_desired[0];
            uav_cmd.desired_pos[1] = state_desired[1];
            uav_cmd.desired_pos[2] = state_desired[2];
            uav_cmd.desired_yaw = state_desired[3];
            uav_cmd.desired_yaw_rate = 0.0;
            uav_command_pub[0].publish(uav_cmd);

            cout << BLUE << "pos_des [X Y Z] : " << state_desired[0] << " [ m ] " << state_desired[1] << " [ m ] " << state_desired[2] << " [ m ] " << endl;
            cout << BLUE << "yaw_des : " << state_desired[3] / M_PI * 180.0 << " [ deg ] " << endl;

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
                for (int i = 0; i < uav_num; i++)
                {
                    uav_setup_pub[i].publish(setup);
                }
            }
            else if (arming == 0)
            {
                setup.cmd = 0;
                for (int i = 0; i < uav_num; i++)
                {
                    uav_setup_pub[i].publish(setup);
                }
                // uav_setup_pub[0].publish(setup);
            }
            else
            {
                std::cout << BLUE << "input error" << std::endl;
            }
            break;
        }
        case 102:
        {
            int control_mode;
            cout << BLUE << "control_mode 1 INIT 2 RC_CONTROL 3 CMD_CONTROL 4 LAND_CONTROL 5 WITHOUT_CONTROL" << std::endl;
            cin >> control_mode;
            if (control_mode == 1)
                setup.control_mode = "INIT";
            else if (control_mode == 2)
                setup.control_mode = "RC_CONTROL";
            else if (control_mode == 3)
                setup.control_mode = "CMD_CONTROL";
            else if (control_mode == 4)
                setup.control_mode = "LAND_CONTROL";
            else if (control_mode == 5)
                setup.control_mode = "WITHOUT_CONTROL";
            else
            {
                std::cout << BLUE << "input error" << std::endl;
                break;
            }
            setup.cmd = 4;
            for (int i = 0; i < uav_num; i++)
            {
                uav_setup_pub[i].publish(setup);
            }
            // uav_setup_pub[0].publish(setup);
            break;
                 
                  
        }
        case 103:
            uav_cmd.header.stamp = ros::Time::now();
            uav_cmd.cmd = 100;
            for (int i = 0; i < uav_num; i++)
            {
                uav_command_pub[i].publish(uav_cmd);
            }
            // uav_command_pub[0].publish(uav_cmd);
            std::cout << "takeoff" << std::endl;
            break;
        case 104:
            uav_cmd.header.stamp = ros::Time::now();
            uav_cmd.cmd = 102;
            for (int i = 0; i < uav_num; i++)
            {
                uav_command_pub[i].publish(uav_cmd);
            }
            // uav_command_pub[0].publish(uav_cmd);
            std::cout << "hover" << std::endl;
            break;
        case 105:
            uav_cmd.header.stamp = ros::Time::now();
            uav_cmd.cmd = 101;
            for (int i = 0; i < uav_num; i++)
            {
                uav_command_pub[i].publish(uav_cmd);
            }
            // uav_command_pub[0].publish(uav_cmd);
            std::cout << "land" << std::endl;
            break;
        case 106:
            uav_cmd.header.stamp = ros::Time::now();
            uav_cmd.cmd = 104;
            for (int i = 0; i < uav_num; i++)
            {
                uav_command_pub[i].publish(uav_cmd);
            }
            // uav_command_pub[0].publish(uav_cmd);
            std::cout << "return" << std::endl;
            break;
        case 107:
            uav_cmd.header.stamp = ros::Time::now();
            uav_cmd.cmd = 103;
            uav_command_pub[0].publish(uav_cmd);
        case 110:

            cout << "For safety, please move the drone near to the trajectory start point firstly!!!" << endl;
            cout << "Please choose the trajectory type: 0 for Circle, 1 for Eight Shape, 2 for Step, 3 for Line" << endl;
            cin >> Trjectory_mode;
            cout << "Input the trajectory_total_time:" << endl;
            cin >> trajectory_total_time;
            time_trajectory = 0.0;

            while (time_trajectory < trajectory_total_time)
            {

                if (Trjectory_mode == 0)
                {
                    uav_cmd = traj_generator.Circle_trajectory_generation(time_trajectory);
                }
                else if (Trjectory_mode == 1)
                {
                    uav_cmd = traj_generator.Eight_trajectory_generation(time_trajectory);
                }
                else if (Trjectory_mode == 2)
                {
                    uav_cmd = traj_generator.Step_trajectory_generation(time_trajectory);
                }
                else if (Trjectory_mode == 3)
                {
                    uav_cmd = traj_generator.Line_trajectory_generation(time_trajectory);
                }

                uav_cmd.header.stamp = ros::Time::now();
                uav_cmd.cmd = sunray_msgs::UAVControlCMD::PosVelAccYaw;
                uav_command_pub[0].publish(uav_cmd);

                time_trajectory = time_trajectory + 0.01;
                cout << "Trajectory tracking: " << time_trajectory << " / " << trajectory_total_time << " [ s ]" << endl;

                geometry_msgs::PoseStamped reference_pose;

                reference_pose.header.stamp = ros::Time::now();
                reference_pose.header.frame_id = "world";

                reference_pose.pose.position.x = uav_cmd.desired_pos[0];
                reference_pose.pose.position.y = uav_cmd.desired_pos[1];
                reference_pose.pose.position.z = uav_cmd.desired_pos[2];

                posehistory_vector_.insert(posehistory_vector_.begin(), reference_pose);
                if (posehistory_vector_.size() > TRA_WINDOW)
                {
                    posehistory_vector_.pop_back();
                }

                nav_msgs::Path reference_trajectory;
                reference_trajectory.header.stamp = ros::Time::now();
                reference_trajectory.header.frame_id = "world";
                reference_trajectory.poses = posehistory_vector_;
                ref_trajectory_pub.publish(reference_trajectory);

                ros::Duration(0.01).sleep();
            }
        }

        ros::Duration(0.5).sleep();
    }
    return 0;
}
