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
std_msgs::Int32 uav_control_state;
std::vector<geometry_msgs::PoseStamped> posehistory_vector_;

// 如果要使用地面站PrometheusGround控制，需要将此值改为true，否则改为false
bool is_ground_station_control = false;
bool flag = false;

void uav_state_cb(const sunray_msgs::UAVState::ConstPtr &msg)
{
    uav_state = *msg;
}

void uav_control_state_cb(const std_msgs::Int32::ConstPtr &msg)
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
    nh.param("uav_id", uav_id, 1);
    nh.param("sim_mode", sim_mode, true);
    nh.getParam("/communication_bridge/trajectory_ground_control",is_ground_station_control);

    string uav_name = "/uav" + std::to_string(uav_id);

    //【订阅】状态信息
    ros::Subscriber uav_state_sub = nh.subscribe<sunray_msgs::UAVState>(uav_name + "/sunray/state", 1, uav_state_cb);

    //【订阅】无人机控制信息
    ros::Subscriber uav_contorl_state_sub = nh.subscribe<std_msgs::Int32>(uav_name + "/sunray/control_state", 1, uav_control_state_cb);

    //【发布】UAVCommand
    // ros::Publisher ref_trajectory_pub = nh.advertise<sunray_msgs::Path>(uav_name + "/sunray/reference_trajectory", 10);

    //【发布】UAVCommand
    ros::Publisher uav_command_pub = nh.advertise<sunray_msgs::UAVControlCMD>(uav_name + "/sunray/uav_control_cmd", 1);

    //用于控制器测试的类，功能例如：生成圆形轨迹，8字轨迹等
    // Controller_Test Controller_Test;
    // Controller_Test.printf_param();

    int CMD = 0;
    float state_desired[4];

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

        if (uav_control_state.data != 2)
        {
            cout << YELLOW << "Please switch to COMMAND_CONTROL mode first" << TAIL << endl;
        }

        if (!is_ground_station_control)
        {
            cout << GREEN << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>UAV Terminal Control<<<<<<<<<<<<<<<<<<<<<<<<< " << TAIL << endl;
            cout << GREEN << "Please choose the CMD: 1 for Takeoff, 2 for Current_Pos_Hover, 3 for Land, 4 for Move(XYZ_POS), 5 for Move(XY_VEL_Z_POS), \
6 for Move(XYZ_VEL), 7 for Move(XYZ_POS_BODY), 8 for Move(XYZ_VEL_BODY), 9 for Move(XY_VEL_Z_POS_BODY), \
10 for TRAJECTORY, 11 for Move(XYZ_ATT), 12 for Move(LAT_LON_ALT) ..." << TAIL << endl;
            cin >> CMD;
        }
        else
        {
            CMD = 5;
        }

        switch (CMD)
        {
        case 1:
            uav_cmd.cmd = sunray_msgs::UAVControlCMD::Takeoff;
            uav_cmd.cmd_id = uav_cmd.cmd_id + 1;
            uav_command_pub.publish(uav_cmd);
            cout << "Takeoff. " << endl;
            break;
        case 2:
            uav_cmd.cmd = sunray_msgs::UAVControlCMD::Hover;
            uav_cmd.cmd_id = uav_cmd.cmd_id + 1;
            uav_command_pub.publish(uav_cmd);
            cout << "Hover. " << endl;
            break;
        case 3:
            uav_cmd.cmd = sunray_msgs::UAVControlCMD::Land;
            uav_cmd.cmd_id = uav_cmd.cmd_id + 1;
            uav_command_pub.publish(uav_cmd);
            cout << "Land. " << endl;
            break;
        case 4:
            cout << "Move for XYZ_POS in END frame, Pls input the desired position and yaw (or yaw rate) angle" << endl;
            cout << "desired state: --- x [m] " << endl;
            cin >> state_desired[0];
            cout << "desired state: --- y [m]" << endl;
            cin >> state_desired[1];
            cout << "desired state: --- z [m]" << endl;
            cin >> state_desired[2];
            cout << "desired state: --- yaw [deg]:" << endl;
            cin >> state_desired[3];
            cout << "enable yaw rate: --- 0 yaw  1 yaw rate" << endl;
            cin >> state_desired[4];
            state_desired[3] = state_desired[3] / 180.0 * M_PI;

            uav_cmd.header.stamp = ros::Time::now();
            uav_cmd.cmd = sunray_msgs::UAVControlCMD::XYZ_POS;
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
            cout << "Move for XY_VEL_Z_POS in END frame, Pls input the desired position and yaw (or yaw rate) angle" << endl;
            cout << "desired state: --- x [m/s] " << endl;
            cin >> state_desired[0];
            cout << "desired state: --- y [m/s]" << endl;
            cin >> state_desired[1];
            cout << "desired state: --- z [m]" << endl;
            cin >> state_desired[2];
            cout << "desired state: --- yaw [deg]:" << endl;
            cin >> state_desired[3];
            cout << "enable yaw rate: --- 0 yaw  1 yaw rate" << endl;
            cin >> state_desired[4];
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
            cout << "vel_des [X Y] : " << state_desired[0] << " [ m/s ] " << state_desired[1] << " [ m/s ] " <<" pos_des [Z] : "<< state_desired[2] << " [ m ] " << endl;
            cout << "yaw_des : " << state_desired[3] / M_PI * 180.0 << " [ deg ] " << endl;
            break;
        case 6:
            cout << "Move for XYZ_VEL in END frame, Pls input the desired position and yaw (or yaw rate) angle" << endl;
            cout << "desired state: --- x [m/s] " << endl;
            cin >> state_desired[0];
            cout << "desired state: --- y [m/s]" << endl;
            cin >> state_desired[1];
            cout << "desired state: --- z [m/s]" << endl;
            cin >> state_desired[2];
            cout << "desired state: --- yaw [deg]:" << endl;
            cin >> state_desired[3];
            cout << "enable yaw rate: --- 0 yaw  1 yaw rate" << endl;
            cin >> state_desired[4];
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
        case 7:
            cout << "Move for XYZ_POS in BODY frame, Pls input the desired position and yaw (or yaw rate) angle" << endl;
            cout << "desired state: --- x [m] " << endl;
            cin >> state_desired[0];
            cout << "desired state: --- y [m]" << endl;
            cin >> state_desired[1];
            cout << "desired state: --- z [m]" << endl;
            cin >> state_desired[2];
            cout << "desired state: --- yaw [deg]:" << endl;
            cin >> state_desired[3];
            cout << "enable yaw rate: --- 0 yaw  1 yaw rate" << endl;
            cin >> state_desired[4];
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
        case 8:
            cout << "Move for XYZ_VEL in BODY frame, Pls input the desired position and yaw (or yaw rate) angle" << endl;
            cout << "desired state: --- x [m/s] " << endl;
            cin >> state_desired[0];
            cout << "desired state: --- y [m/s]" << endl;
            cin >> state_desired[1];
            cout << "desired state: --- z [m/s]" << endl;
            cin >> state_desired[2];
            cout << "desired state: --- yaw [deg]:" << endl;
            cin >> state_desired[3];
            cout << "enable yaw rate: --- 0 yaw  1 yaw rate" << endl;
            cin >> state_desired[4];
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
        case 9:
            cout << "Move for XY_VEL_Z_POS in BODY frame, Pls input the desired position and yaw (or yaw rate) angle" << endl;
            cout << "desired state: --- x [m/s] " << endl;
            cin >> state_desired[0];
            cout << "desired state: --- y [m/s]" << endl;
            cin >> state_desired[1];
            cout << "desired state: --- z [m]" << endl;
            cin >> state_desired[2];
            cout << "desired state: --- yaw [deg]:" << endl;
            cin >> state_desired[3];
            cout << "enable yaw rate: --- 0 yaw  1 yaw rate" << endl;
            cin >> state_desired[4];
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
            cout << "vel_des [X Y] : " << state_desired[0] << " [ m/s ] " << state_desired[1] << " [ m/s ] " <<" pos_des [Z] : "<< state_desired[2] << " [ m ] " << endl;
            cout << "yaw_des : " << state_desired[3] / M_PI * 180.0 << " [ deg ] " << endl;
            break;
        case 10:

            // static int Trjectory_mode;
            // static int trajectory_total_time;
            // if (!is_ground_station_control)
            // {
            //     cout << "For safety, please move the drone near to the trajectory start point firstly!!!" << endl;
            //     cout << "Please choose the trajectory type: 0 for Circle, 1 for Eight Shape, 2 for Step, 3 for Line" << endl;
            //     cin >> Trjectory_mode;
            //     cout << "Input the trajectory_total_time:" << endl;
            //     cin >> trajectory_total_time;
            // }
            // else
            // {
            //     while (1)
            //     {
            //         nh.getParam("/communication_bridge/trajectory_flag", flag);
            //         if (flag){
            //             break;
            //         }
            //         usleep(10);
            //     }
            //     nh.getParam("/communication_bridge/trajectory_mode", Trjectory_mode);
            //     nh.getParam("/communication_bridge/trajectory_time", trajectory_total_time);
            //     nh.setParam("/communication_bridge/trajectory_flag",false);
            // }

            // time_trajectory = 0.0;

            // while (time_trajectory < trajectory_total_time)
            // {

            //     if (Trjectory_mode == 0)
            //     {
            //         agent_command = Controller_Test.Circle_trajectory_generation(time_trajectory);
            //     }
            //     else if (Trjectory_mode == 1)
            //     {
            //         agent_command = Controller_Test.Eight_trajectory_generation(time_trajectory);
            //     }
            //     else if (Trjectory_mode == 2)
            //     {
            //         agent_command = Controller_Test.Step_trajectory_generation(time_trajectory);
            //     }
            //     else if (Trjectory_mode == 3)
            //     {
            //         agent_command = Controller_Test.Line_trajectory_generation(time_trajectory);
            //     }

            //     uav_cmd.header.stamp = ros::Time::now();
            //     uav_cmd.Agent_CMD = sunray_msgs::UAVControlCMD::Move;
            //     uav_cmd.Move_mode = sunray_msgs::UAVControlCMD::TRAJECTORY;
            //     uav_command_pub.publish(agent_command);

            //     time_trajectory = time_trajectory + 0.01;
            //     cout << "Trajectory tracking: " << time_trajectory << " / " << trajectory_total_time << " [ s ]" << endl;

            //     geometry_msgs::PoseStamped reference_pose;

            //     reference_pose.header.stamp = ros::Time::now();
            //     reference_pose.header.frame_id = "world";

            //     reference_pose.pose.position.x = uav_cmd.position_ref[0];
            //     reference_pose.pose.position.y = uav_cmd.position_ref[1];
            //     reference_pose.pose.position.z = uav_cmd.position_ref[2];

            //     posehistory_vector_.insert(posehistory_vector_.begin(), reference_pose);
            //     if (posehistory_vector_.size() > TRA_WINDOW)
            //     {
            //         posehistory_vector_.pop_back();
            //     }

            //     nav_msgs::Path reference_trajectory;
            //     reference_trajectory.header.stamp = ros::Time::now();
            //     reference_trajectory.header.frame_id = "world";
            //     reference_trajectory.poses = posehistory_vector_;
            //     ref_trajectory_pub.publish(reference_trajectory);

            //     ros::Duration(0.01).sleep();
            // }
            break;
        }

        ros::Duration(0.5).sleep();
    }
    return 0;
}
