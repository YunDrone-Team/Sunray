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

#include "ros_msg_utils.h"

using namespace std;
#define TRA_WINDOW 2000
sunray_msgs::UAVState uav_state;
sunray_msgs::UAVControlCMD uav_cmd;
sunray_msgs::UAVSetup setup;
sunray_msgs::UAVState uav_control_state;
std::vector<geometry_msgs::PoseStamped> posehistory_vector_;

// 如果要使用地面站PrometheusGround控制，需要将此值改为true，否则改为false
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

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "uav_command_pub");
    ros::NodeHandle nh("~");

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
    ros::Publisher setup_pub = nh.advertise<sunray_msgs::UAVSetup>(topic_prefix + "/sunray/setup",1);

    // 用于控制器测试的类，功能例如：生成圆形轨迹，8字轨迹等
    //  Controller_Test Controller_Test;
    //  Controller_Test.printf_param();

    int CMD = 0;
    float state_desired[4];
    bool yaw_rate = false;

    uav_cmd.header.stamp = ros::Time::now();
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
    uav_cmd.cmd_id = 0;

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
            cout << GREEN << "101 for arm or dis arm, 102 for switch sunray control(offboard), 103 for takeoff, 104 for hover, 105 for land,  100 for enable or disable yawRate(now ";
            if (yaw_rate)
            {
                cout << "enable)" << TAIL << endl;
            }
            else
            {
                cout << "disable)" << TAIL << endl;
            }
            cout << GREEN << "Please choose the CMD: 1 for Move(XYZ_POS), 2 for Move(XY_VEL_Z_POS), \
3 for Move(XYZ_VEL), 4 for Move(XYZ_POS_BODY), 5 for Move(XYZ_VEL_BODY), 6 for Move(XY_VEL_Z_POS_BODY)" << TAIL << endl;
            cin >> CMD;
        }

        switch (CMD)
        {
        case 1:
            cout << "Move for XYZ_POS in END frame, Pls input the desired position and yaw (or yaw rate) angle" << endl;
            cout << "desired state: --- x [m] " << endl;
            cin >> state_desired[0];
            cout << "desired state: --- y [m]" << endl;
            cin >> state_desired[1];
            cout << "desired state: --- z [m]" << endl;
            cin >> state_desired[2];
            cout << "desired state: --- yaw [deg]:" << endl;
            cin >> state_desired[3];
            state_desired[3] = state_desired[3] / 180.0 * M_PI;

            uav_cmd.header.stamp = ros::Time::now();
            uav_cmd.cmd = sunray_msgs::UAVControlCMD::XYZ_POS;
            uav_cmd.desired_pos[0] = state_desired[0];
            uav_cmd.desired_pos[1] = state_desired[1];
            uav_cmd.desired_pos[2] = state_desired[2];
            uav_cmd.desired_yaw = state_desired[3];
            uav_cmd.enable_yawRate = yaw_rate;
            uav_cmd.cmd_id = uav_cmd.cmd_id + 1;
            uav_command_pub.publish(uav_cmd);
            cout << "pos_des [X Y Z] : " << state_desired[0] << " [ m ] " << state_desired[1] << " [ m ] " << state_desired[2] << " [ m ] " << endl;
            cout << "yaw_des : " << state_desired[3] / M_PI * 180.0 << " [ deg ] " << endl;
            break;
        case 2:
            cout << "Move for XY_VEL_Z_POS in END frame, Pls input the desired position and yaw (or yaw rate) angle" << endl;
            cout << "desired state: --- x [m/s] " << endl;
            cin >> state_desired[0];
            cout << "desired state: --- y [m/s]" << endl;
            cin >> state_desired[1];
            cout << "desired state: --- z [m]" << endl;
            cin >> state_desired[2];
            cout << "desired state: --- yaw [deg]:" << endl;
            cin >> state_desired[3];
            state_desired[4] = yaw_rate;
            state_desired[3] = state_desired[3] / 180.0 * M_PI;

            uav_cmd.header.stamp = ros::Time::now();
            uav_cmd.cmd = sunray_msgs::UAVControlCMD::XY_VEL_Z_POS;
            uav_cmd.desired_vel[0] = state_desired[0];
            uav_cmd.desired_vel[1] = state_desired[1];
            uav_cmd.desired_vel[2] = 0.0;
            uav_cmd.desired_pos[0] = 0.0;
            uav_cmd.desired_pos[1] = 0.0;
            uav_cmd.desired_pos[2] = state_desired[2];
            uav_cmd.desired_yaw = state_desired[3];
            uav_cmd.enable_yawRate = state_desired[4];
            uav_cmd.cmd_id = uav_cmd.cmd_id + 1;
            uav_command_pub.publish(uav_cmd);
            cout << "vel_des [X Y] : " << state_desired[0] << " [ m/s ] " << state_desired[1] << " [ m/s ] " << " pos_des [Z] : " << state_desired[2] << " [ m ] " << endl;
            cout << "yaw_des : " << state_desired[3] / M_PI * 180.0 << " [ deg ] " << endl;
            break;
        case 3:
            cout << "Move for XYZ_VEL in END frame, Pls input the desired position and yaw (or yaw rate) angle" << endl;
            cout << "desired state: --- x [m/s] " << endl;
            cin >> state_desired[0];
            cout << "desired state: --- y [m/s]" << endl;
            cin >> state_desired[1];
            cout << "desired state: --- z [m/s]" << endl;
            cin >> state_desired[2];
            cout << "desired state: --- yaw [deg]:" << endl;
            cin >> state_desired[3];
            state_desired[4] = yaw_rate;
            state_desired[3] = state_desired[3] / 180.0 * M_PI;

            uav_cmd.header.stamp = ros::Time::now();
            uav_cmd.cmd = sunray_msgs::UAVControlCMD::XYZ_VEL;
            uav_cmd.desired_vel[0] = state_desired[0];
            uav_cmd.desired_vel[1] = state_desired[1];
            uav_cmd.desired_vel[2] = state_desired[2];
            uav_cmd.desired_pos[0] = 0.0;
            uav_cmd.desired_pos[1] = 0.0;
            uav_cmd.desired_pos[2] = 0.0;
            uav_cmd.desired_yaw = state_desired[3];
            uav_cmd.enable_yawRate = state_desired[4];
            uav_cmd.cmd_id = uav_cmd.cmd_id + 1;
            uav_command_pub.publish(uav_cmd);
            cout << "vel_des [X Y Z] : " << state_desired[0] << " [ m/s ] " << state_desired[1] << " [ m/s ] " << state_desired[2] << " [ m/s ] " << endl;
            cout << "yaw_des : " << state_desired[3] / M_PI * 180.0 << " [ deg ] " << endl;
            break;
        case 4:
            cout << "Move for XYZ_POS in BODY frame, Pls input the desired position and yaw (or yaw rate) angle" << endl;
            cout << "desired state: --- x [m] " << endl;
            cin >> state_desired[0];
            cout << "desired state: --- y [m]" << endl;
            cin >> state_desired[1];
            cout << "desired state: --- z [m]" << endl;
            cin >> state_desired[2];
            cout << "desired state: --- yaw [deg]:" << endl;
            cin >> state_desired[3];
            state_desired[4] = yaw_rate;
            state_desired[3] = state_desired[3] / 180.0 * M_PI;

            uav_cmd.header.stamp = ros::Time::now();
            uav_cmd.cmd = sunray_msgs::UAVControlCMD::XYZ_POS_BODY;
            uav_cmd.desired_pos[0] = state_desired[0];
            uav_cmd.desired_pos[1] = state_desired[1];
            uav_cmd.desired_pos[2] = state_desired[2];
            uav_cmd.desired_yaw = state_desired[3];
            uav_cmd.enable_yawRate = state_desired[4];
            uav_cmd.cmd_id = uav_cmd.cmd_id + 1;
            uav_command_pub.publish(uav_cmd);
            cout << "pos_des [X Y Z] : " << state_desired[0] << " [ m ] " << state_desired[1] << " [ m ] " << state_desired[2] << " [ m ] " << endl;
            cout << "yaw_des : " << state_desired[3] / M_PI * 180.0 << " [ deg ] " << endl;
            break;
        case 5:
            cout << "Move for XYZ_VEL in BODY frame, Pls input the desired position and yaw (or yaw rate) angle" << endl;
            cout << "desired state: --- x [m/s] " << endl;
            cin >> state_desired[0];
            cout << "desired state: --- y [m/s]" << endl;
            cin >> state_desired[1];
            cout << "desired state: --- z [m/s]" << endl;
            cin >> state_desired[2];
            cout << "desired state: --- yaw [deg]:" << endl;
            cin >> state_desired[3];
            state_desired[4] = yaw_rate;
            state_desired[3] = state_desired[3] / 180.0 * M_PI;

            uav_cmd.header.stamp = ros::Time::now();
            uav_cmd.cmd = sunray_msgs::UAVControlCMD::XYZ_VEL_BODY;
            uav_cmd.desired_vel[0] = state_desired[0];
            uav_cmd.desired_vel[1] = state_desired[1];
            uav_cmd.desired_vel[2] = state_desired[2];
            uav_cmd.desired_pos[0] = 0.0;
            uav_cmd.desired_pos[1] = 0.0;
            uav_cmd.desired_pos[2] = 0.0;
            uav_cmd.desired_yaw = state_desired[3];
            uav_cmd.enable_yawRate = state_desired[4];
            uav_cmd.cmd_id = uav_cmd.cmd_id + 1;
            uav_command_pub.publish(uav_cmd);
            cout << "vel_des [X Y Z] : " << state_desired[0] << " [ m/s ] " << state_desired[1] << " [ m/s ] " << state_desired[2] << " [ m/s ] " << endl;
            cout << "yaw_des : " << state_desired[3] / M_PI * 180.0 << " [ deg ] " << endl;
            break;
        case 6:
            cout << "Move for XY_VEL_Z_POS in BODY frame, Pls input the desired position and yaw (or yaw rate) angle" << endl;
            cout << "desired state: --- x [m/s] " << endl;
            cin >> state_desired[0];
            cout << "desired state: --- y [m/s]" << endl;
            cin >> state_desired[1];
            cout << "desired state: --- z [m]" << endl;
            cin >> state_desired[2];
            cout << "desired state: --- yaw [deg]:" << endl;
            cin >> state_desired[3];
            state_desired[4] = yaw_rate;
            state_desired[3] = state_desired[3] / 180.0 * M_PI;

            uav_cmd.header.stamp = ros::Time::now();
            uav_cmd.cmd = sunray_msgs::UAVControlCMD::XY_VEL_Z_POS_BODY;
            uav_cmd.desired_vel[0] = state_desired[0];
            uav_cmd.desired_vel[1] = state_desired[1];
            uav_cmd.desired_vel[2] = 0.0;
            uav_cmd.desired_pos[0] = 0.0;
            uav_cmd.desired_pos[1] = 0.0;
            uav_cmd.desired_pos[2] = state_desired[2];
            uav_cmd.desired_yaw = state_desired[3];
            uav_cmd.enable_yawRate = state_desired[4];
            uav_cmd.cmd_id = uav_cmd.cmd_id + 1;
            uav_command_pub.publish(uav_cmd);
            cout << "vel_des [X Y] : " << state_desired[0] << " [ m/s ] " << state_desired[1] << " [ m/s ] " << " pos_des [Z] : " << state_desired[2] << " [ m ] " << endl;
            cout << "yaw_des : " << state_desired[3] / M_PI * 180.0 << " [ deg ] " << endl;
            break;
        case 100:
        {
            yaw_rate = !yaw_rate;
            break;
        }
        case 101:
        {
            int arming;
            std::cout << "Please select Operation 1 arm 0 disarm" << std::endl;
            std::cin >> arming;
            if (arming == 1)
            {
                setup.cmd = 1;
                setup_pub.publish(setup);
            } 
            else if(arming == 0)
            {
                setup.cmd = 0;
                setup_pub.publish(setup);
            }
            else {
                std::cout << "input error" << std::endl;
            }
            break;
        }
        case 102:
        {
            setup.cmd = 4;
            setup.control_state = "CMD_CONTROL";
            setup_pub.publish(setup);
            break;
        }
        case 103:
            uav_cmd.cmd = sunray_msgs::UAVControlCMD::Takeoff;
            uav_cmd.cmd_id = uav_cmd.cmd_id + 1;
            uav_command_pub.publish(uav_cmd);
            cout << "Takeoff. " << endl;
            break;
        case 104:
            uav_cmd.cmd = sunray_msgs::UAVControlCMD::Hover;
            uav_cmd.cmd_id = uav_cmd.cmd_id + 1;
            uav_command_pub.publish(uav_cmd);
            cout << "Hover. " << endl;
            break;
        case 105:
            uav_cmd.cmd = sunray_msgs::UAVControlCMD::Land;
            uav_cmd.cmd_id = uav_cmd.cmd_id + 1;
            uav_command_pub.publish(uav_cmd);
            cout << "Land. " << endl;
            break;
        break;
        }

        ros::Duration(0.5).sleep();
    }
    return 0;
}
