#include <ros/ros.h>
#include "ros_msg_utils.h"
#include "printf_utils.h"
#include <sunray_msgs/TargetMsg.h>
#include <sunray_msgs/TargetsInFrameMsg.h>
#include <Eigen/Dense>
#include "utils.hpp"

using namespace std;
using namespace Eigen;

geometry_msgs::PoseStamped current_pose;
sunray_msgs::UAVControlCMD uav_cmd;
geometry_msgs::PoseStamped target_pose;
sunray_msgs::UAVSetup setup;

float target_yaw;
bool stop_flag{false};

// 全局变量存储无人机和车的坐标和朝向
double uav_x, uav_y, uav_z, uav_yaw, x_rel, y_rel, z_rel, yaw_rel, roll_rel, pitch_rel;
double ang;
bool sub_flag{false};
bool tag_flag{false};
double x_vel = 0.0;
double y_vel = 0.0;
double z_vel = 0.0;
double yaw = 0.0;

MovingAverageFilter x_filter(5);
MovingAverageFilter y_filter(5);
MovingAverageFilter z_filter(5);
MovingAverageFilter ang_filter(5);
ros::Time last_time{0};

double get_roll_pitch(double yaw, double &roll, double &pitch)
{
    //  将 yaw 转换到 0 到 360 度之间
    yaw = int(yaw);
    if (-180 < yaw && yaw < 0)
    {
        yaw += 360; // 将负角度转换为正角度
    }
    if (0 <= yaw && yaw < 45)
    {
        return roll;
    }
    else if (45 <= yaw && yaw < 135)
    {
        if (-180 < pitch && pitch < 0)
        {
            return -(pitch + 180);
        }
        return 180 - pitch;
    }
    else if (135 <= yaw && yaw < 225)
    {
        return -roll;
    }
    else if (225 <= yaw && yaw < 315)
    {
        if (-180 < pitch && pitch < 0)
        {
            return (pitch + 180);
        }
        return pitch - 180;
    }
    else if (315 <= yaw && yaw < 360)
    {
        return roll;
    }
}

void stop_tutorial_cb(const std_msgs::Empty::ConstPtr &msg)
{
    stop_flag = true;
}

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
void tagCallback(const sunray_msgs::TargetsInFrameMsg::ConstPtr &msg)
{

    if (msg->targets.size() > 0)
    {
        tag_flag = true;
        last_time = ros::Time::now();
        x_rel = x_filter.filter(msg->targets[0].px);
        y_rel = y_filter.filter(-msg->targets[0].py);
        z_rel = z_filter.filter(msg->targets[0].pz);
        yaw_rel = msg->targets[0].yaw;
        roll_rel = msg->targets[0].roll;
        pitch_rel = msg->targets[0].pitch;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "auto_land_node");
    ros::NodeHandle nh("~");

    ros::Rate rate(10.0);

    int uav_id;
    bool auto_takeoff = false;
    string uav_name, target_tpoic_name;
    bool sim_mode, flag_printf;
    nh.param<bool>("sim_mode", sim_mode, true);
    nh.param<bool>("flag_printf", flag_printf, true);
    nh.param<bool>("auto_takeoff", auto_takeoff, false);
    // 【参数】无人机编号
    nh.param<int>("uav_id", uav_id, 1);
    // 【参数】无人机名称
    nh.param<string>("uav_name", uav_name, "uav");

    double k_p_xy, k_p_z, k_p_yaw, max_vel, max_vel_z, max_yaw;
    double hight;
    nh.param<double>("k_p_xy", k_p_xy, 1.2);
    nh.param<double>("k_p_z", k_p_z, 0.5);
    nh.param<double>("k_p_yaw", k_p_yaw, 0.04);
    nh.param<double>("max_vel", max_vel, 0.5);
    nh.param<double>("max_vel_z", max_vel_z, 0.2);
    nh.param<double>("max_yaw", max_yaw, 0.4);
    nh.param<double>("hight", hight, 8.0);
    
    uav_name = uav_name + to_string(uav_id);
    string topic_prefix = "/" + uav_name;
    // 【订阅】目标点位置
    ros::Subscriber target_pos_sub = nh.subscribe<sunray_msgs::TargetsInFrameMsg>(topic_prefix + "/sunray_detect/qrcode_detection_ros", 1, tagCallback);
    // 【订阅】任务结束
    ros::Subscriber stop_tutorial_sub = nh.subscribe<std_msgs::Empty>(topic_prefix + "/sunray/stop_tutorial", 1, stop_tutorial_cb);

    // 【发布】无人机控制指令 （本节点 -> sunray_control_node）
    ros::Publisher control_cmd_pub = nh.advertise<sunray_msgs::UAVControlCMD>(topic_prefix + "/sunray/uav_control_cmd", 1);
    // 【发布】无人机设置指令（本节点 -> sunray_control_node）
    ros::Publisher uav_setup_pub = nh.advertise<sunray_msgs::UAVSetup>(topic_prefix + "/sunray/setup", 1);

    ros::Subscriber pose_sub = nh.subscribe(topic_prefix + "/mavros/local_position/pose", 10, pose_cb);

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

    if(auto_takeoff)
    {
        ros::Duration(10).sleep();

        ros::Duration(0.5).sleep();
        // 解锁
        cout << "arm" << endl;
        setup.cmd = 1;
        uav_setup_pub.publish(setup);
        ros::Duration(1.5).sleep();

        // 切换到指令控制模式
        cout << "switch CMD_CONTROL" << endl;
        setup.cmd = 4;
        setup.control_mode = "CMD_CONTROL";
        uav_setup_pub.publish(setup);
        ros::Duration(1.0).sleep();

        // 起飞
        cout << "takeoff" << endl;
        uav_cmd.cmd = 100;
        control_cmd_pub.publish(uav_cmd);
        ros::Duration(5).sleep();

        cout << "rising" << endl;
        uav_cmd.cmd = 1;
        uav_cmd.desired_pos[0] = 0.0;
        uav_cmd.desired_pos[1] = 0.0;
        uav_cmd.desired_pos[2] = hight;
        control_cmd_pub.publish(uav_cmd);
        ros::Duration(2).sleep();
    }


    ros::Duration(0.5).sleep();
    ros::spinOnce();

    double x_vel = 0.0;
    double y_vel = 0.0;
    double z_vel = 0.0;
    double yaw = 0.0;

    geometry_msgs::PoseStamped pose;
    ros::spinOnce();
    ros::Duration(0.5).sleep();
    while (!sub_flag)
    {
        cout << "waiting for pose" << endl;
        ros::spinOnce();
        ros::Duration(1).sleep();
    }
    while (!tag_flag)
    {
        cout << "waiting for tag" << endl;
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
            cout << "land" << endl;
            uav_cmd.cmd = sunray_msgs::UAVControlCMD::Land;
            control_cmd_pub.publish(uav_cmd);
            break;
        }
        ang = get_roll_pitch(yaw_rel, roll_rel, pitch_rel);
        ang = ang_filter.filter(ang);
        if (((ros::Time::now() - last_time).toSec()) < 0.5)
        {
            landing_point_num += 1;

            if (landing_point_num > 10)
            {
                if (ang < 2 && ang > -2)
                {
                    ang = 0;
                }

                // if (abs(ang) > 30)
                // {
                //     max_yaw = 0.5;
                // }
                // else if (abs(ang) < 10){
                //     max_yaw = 0.05;
                // }

                x_vel = (z_rel - 2) * k_p_xy;
                y_vel = y_rel * k_p_xy;
                z_vel = x_rel * k_p_z;

                x_vel = min(max(x_vel, -max_vel), max_vel);
                y_vel = min(max(y_vel, -max_vel), max_vel);
                z_vel = min(max(z_vel, -max_vel_z), max_vel_z);
                ang = min(max(ang, -max_yaw), max_yaw);
                yaw = ang / 180.0 * M_PI; // 转为弧度制

                uav_cmd.header.stamp = ros::Time::now();
                uav_cmd.cmd = sunray_msgs::UAVControlCMD::XyzVelYawBody;
                uav_cmd.desired_vel[0] = x_vel;
                uav_cmd.desired_vel[1] = y_vel;
                uav_cmd.desired_vel[2] = z_vel;
                uav_cmd.desired_yaw = yaw;

                cout << "x_rel: " << z_rel << " y_rel: " << y_rel << " z_rel: " << x_rel << " yaw_rel: " << ang << endl;
                cout << "x_vel: " << x_vel << " y_vel: " << y_vel << " z_vel: " << z_vel << " yaw: " << yaw << endl;
                control_cmd_pub.publish(uav_cmd);
            }
            continue;
        }
        if ((ros::Time::now() - last_time).toSec() > 10)
        {
            cout << "降落点丢失超时，直接降落" << endl;
            landing_point_num = 0;
            uav_cmd.header.stamp = ros::Time::now();
            uav_cmd.cmd = sunray_msgs::UAVControlCMD::Land;
            control_cmd_pub.publish(uav_cmd);
            break;
        }
        if ((ros::Time::now() - last_time).toSec() > 1)
        {
            cout << "降落点丢失，进入悬停" << endl;
            uav_cmd.header.stamp = ros::Time::now();
            uav_cmd.cmd = sunray_msgs::UAVControlCMD::Hover;
            control_cmd_pub.publish(uav_cmd);
            continue;
        }
    }
}
