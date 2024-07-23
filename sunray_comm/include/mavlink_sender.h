#ifndef MAVLINK_SENDER_H
#define MAVLINK_SENDER_H

//头文件
#include <ros/ros.h>
#include <iostream>
#include <Eigen/Eigen>
#include <signal.h>
#include <stdio.h>
#include <cstdlib>
#include <unistd.h>
#include <cmath>
#include <string.h>
#include <inttypes.h>
#include <fstream>
#include <signal.h>
#include <time.h>
#include <sys/time.h>
#include <pthread.h> // This uses POSIX Threads
#include <unistd.h>  // UNIX standard function definitions

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Bool.h>


#include <sunray_msgs/Heartbeat.h>
#include <sunray_msgs/Attitude.h>
#include <sunray_msgs/LocalPositionNED.h>
#include <sunray_msgs/ArmCmd.h>
#include <sunray_msgs/AttitudeSetpoint.h>
#include <sunray_msgs/GlobalPositionSetpoint.h>
#include <sunray_msgs/LocalPositionSetpoint.h>
#include <sunray_msgs/VisionPositionEstimate.h>
#include <sunray_msgs/SetMode.h>

#include "common/mavlink.h"
#include "msg_status.h"
#include "printf_utils.h"
#include "enum_utils.h"
#include "generic_port.h"
#include "serial_port.h"
#include "udp_port.h"

using namespace std;

class MavlinkSender
{
    public:
        //构造函数
        MavlinkSender(){};

		int	system_id; // system id
		int	component_id; // component id

        // 初始化函数
        void init(ros::NodeHandle& nh, Generic_Port *port);
        // 打印debug信息
        void printf_debug_info();

    private:        
        // 通信端口
        Generic_Port *port;

        // 无人机ID
        int uav_id;
        // 无人机名字
        string uav_name{""};

        // 发送消息状态（监测用）
        MSG_STATUS msg_status_heartbeat;
        MSG_STATUS msg_status_vision_pose;
        MSG_STATUS msg_status_command_long;


        // 【定时器】 发送heartbeat
        ros::Timer timer_heartbeat;
        ros::Timer timer_test;

        ros::Publisher heartbeat_pub;

        ros::Subscriber arming_cmd_sub;
        ros::Subscriber set_mode_cmd_sub;
        ros::Subscriber emergency_kill_cmd_sub;
        ros::Subscriber reboot_cmd_sub;
        ros::Subscriber setpoint_raw_attitude_sub;
        ros::Subscriber setpoint_raw_local_sub;
        ros::Subscriber setpoint_raw_global_sub;


        ros::Subscriber vision_pose_sub;

        ros::ServiceServer arming_srv;
        // ros::ServiceClient set_mode_client;
        // ros::ServiceClient emergency_client;
        // ros::ServiceClient reboot_client;
        void timer_test_cb(const ros::TimerEvent& e);

        void timer_heartbeat_cb(const ros::TimerEvent& e);
        void arming_cmd_cb(const std_msgs::Bool::ConstPtr &msg);
        void set_mode_cmd_cb(const sunray_msgs::SetMode::ConstPtr &msg);
        void emergency_kill_cmd_cb(const std_msgs::Bool::ConstPtr &msg);
        void reboot_cmd_cb(const std_msgs::Bool::ConstPtr &msg);
        void setpoint_raw_attitude_cb(const sunray_msgs::AttitudeSetpoint::ConstPtr &msg);
        void setpoint_raw_local_cb(const sunray_msgs::LocalPositionSetpoint::ConstPtr &msg);
        void setpoint_raw_global_cb(const sunray_msgs::GlobalPositionSetpoint::ConstPtr &msg);
        void vision_pose_cb(const sunray_msgs::VisionPositionEstimate::ConstPtr &msg);
        bool arming_cb(sunray_msgs::ArmCmd::Request &req, sunray_msgs::ArmCmd::Response &res);
};

void MavlinkSender::init(ros::NodeHandle& nh, Generic_Port *port)
{
    nh.param<string>("uav_name", uav_name, "none");
    nh.param<int>("uav_id", uav_id, 0);

    string topic_prefix = "/"  + uav_name + std::to_string(uav_id);

    this->port = port;

    system_id    = 1; // system id
    component_id = 1; // component id

    // 【定时器】 发送heartbeat - 1Hz
    timer_heartbeat = nh.createTimer(ros::Duration(1), &MavlinkSender::timer_heartbeat_cb, this);

    // 【定时器】 发送heartbeat - 1Hz
    timer_test = nh.createTimer(ros::Duration(0.02), &MavlinkSender::timer_test_cb, this);


    // 【发布】无人机当前状态 - HEARTBEAT #0
    heartbeat_pub = nh.advertise<sunray_msgs::Heartbeat>(topic_prefix + "/sunray/heartbeat", 1);

    // 【服务】解锁/上锁 - COMMAND_LONG ( #76 )
    arming_srv = nh.advertiseService(topic_prefix + "/sunray/arm_cmd", &MavlinkSender::arming_cb, this);

    // 【订阅】解锁/上锁 - 封装为：COMMAND_LONG ( #76 )
    arming_cmd_sub = nh.subscribe<std_msgs::Bool>(topic_prefix + "/sunray/arm_cmd", 1, &MavlinkSender::arming_cmd_cb, this);
    // 【订阅】修改系统模式 - 封装为：COMMAND_LONG ( #76 )
    set_mode_cmd_sub = nh.subscribe<sunray_msgs::SetMode>(topic_prefix + "/sunray/set_mode_cmd", 1, &MavlinkSender::set_mode_cmd_cb, this);
    // 【订阅】紧急上锁服务(KILL) - 封装为：COMMAND_LONG ( #76 )
    emergency_kill_cmd_sub = nh.subscribe<std_msgs::Bool>(topic_prefix + "/sunray/emergency_kill_cmd", 1, &MavlinkSender::emergency_kill_cmd_cb, this);
    // 【订阅】重启PX4飞控 - 封装为：COMMAND_LONG ( #76 )
    reboot_cmd_sub = nh.subscribe<std_msgs::Bool>(topic_prefix + "/sunray/reboot_cmd", 1, &MavlinkSender::reboot_cmd_cb, this);

    // 【订阅】姿态期望值 - 封装为：SET_ATTITUDE_TARGET ( #82 )
    setpoint_raw_attitude_sub = nh.subscribe<sunray_msgs::AttitudeSetpoint>(topic_prefix + "/sunray/setpoint_raw/attitude", 1, &MavlinkSender::setpoint_raw_attitude_cb, this);
    // 【订阅】位置/速度/加速度期望值 - SET_POSITION_TARGET_LOCAL_NED ( #84 )
    setpoint_raw_local_sub = nh.subscribe<sunray_msgs::LocalPositionSetpoint>(topic_prefix + "/sunray/setpoint_raw/local", 1, &MavlinkSender::setpoint_raw_local_cb, this);
    // 【订阅】经纬度以及高度位置 - 封装为：SET_POSITION_TARGET_GLOBAL_INT ( #86 )
    setpoint_raw_global_sub = nh.subscribe<sunray_msgs::GlobalPositionSetpoint>(topic_prefix + "/sunray/setpoint_raw/global", 1, &MavlinkSender::setpoint_raw_global_cb, this);
    // 【订阅】视觉估计的位置 - 封装为：VISION_POSITION_ESTIMATE #102
    vision_pose_sub = nh.subscribe<sunray_msgs::VisionPositionEstimate>(topic_prefix + "/sunray/vision_position_estimate", 100, &MavlinkSender::vision_pose_cb, this);

}

void MavlinkSender::printf_debug_info()
{
    cout << BLUE << ">>>>>>>>>>>>>>>>>>>>>> UAV [" << uav_id << "] Mavlink Sender <<<<<<<<<<<<<<<<<<<<<<" << TAIL << endl;
    cout << BLUE << "uav_name : " << uav_name + std::to_string(uav_id) <<  TAIL << endl;
    //固定的浮点显示
    cout.setf(ios::fixed);
    // setprecision(n) 设显示小数精度为n位
    cout << setprecision(2);
    //左对齐
    cout.setf(ios::left);
    // 强制显示小数点
    cout.setf(ios::showpoint);
    // 强制显示符号
    cout.setf(ios::showpos);    

    // heartbeat
    cout << BLUE << "Send heartbeat: ";
    cout << "Msg id: [ "<< MAVLINK_MSG_ID_HEARTBEAT << " ], ";
    cout << "Msg frequency: [ "<< msg_status_heartbeat.get_send_msg_frequency() << " Hz ], ";
    cout << "Total num: [ "<< msg_status_heartbeat.get_send_msg_count() <<" ]." <<  TAIL << endl;

    cout << BLUE << "Send vision_pose: ";
    cout << "Msg id: [ "<< MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE << " ], ";
    cout << "Msg frequency: [ "<< msg_status_vision_pose.get_send_msg_frequency() << " Hz ], ";
    cout << "Total num: [ "<< msg_status_vision_pose.get_send_msg_count() <<" ]." <<  TAIL << endl;

}

void MavlinkSender::timer_test_cb(const ros::TimerEvent& e)
{

    mavlink_vision_position_estimate_t mavlink_vision_position_estimate = { 0 };
    mavlink_vision_position_estimate.x = 1;
    mavlink_vision_position_estimate.y = 2;
    mavlink_vision_position_estimate.z = 3;
    mavlink_vision_position_estimate.roll = 0.0;
    mavlink_vision_position_estimate.pitch = 0.0;
    mavlink_vision_position_estimate.yaw = 0.0;

    mavlink_message_t message;
    mavlink_msg_vision_position_estimate_encode(system_id, 240, &message, &mavlink_vision_position_estimate);
    // 发送
    int len = port->write_message(message);

    // log
    msg_status_vision_pose.send_new_msg(ros::Time::now());
}

void MavlinkSender::timer_heartbeat_cb(const ros::TimerEvent& e)
{
    mavlink_heartbeat_t mavlink_heartbeat = { 0 };
    mavlink_heartbeat.custom_mode = 1;
    mavlink_heartbeat.type = 1;
    mavlink_heartbeat.autopilot = 1;
    mavlink_heartbeat.base_mode = 1;
    mavlink_heartbeat.system_status = 1;
    mavlink_heartbeat.mavlink_version = 1;

    mavlink_message_t message;
    mavlink_msg_heartbeat_encode(system_id, component_id, &message, &mavlink_heartbeat);
    // 发送
    int len = port->write_message(message);

    // log
    msg_status_heartbeat.send_new_msg(ros::Time::now());
}

void MavlinkSender::setpoint_raw_attitude_cb(const sunray_msgs::AttitudeSetpoint::ConstPtr &msg)
{
    mavlink_attitude_target_t mavlink_attitude_target = { 0 };
    mavlink_attitude_target.time_boot_ms = msg->header.stamp.toNSec() / 1000000;
    mavlink_attitude_target.type_mask = msg->type_mask;
    mavlink_attitude_target.q[0] = msg->orientation.w;
    mavlink_attitude_target.q[1] = msg->orientation.x;
    mavlink_attitude_target.q[2] = msg->orientation.y;
    mavlink_attitude_target.q[3] = msg->orientation.z;
    mavlink_attitude_target.thrust = msg->thrust;
    mavlink_attitude_target.body_roll_rate = msg->body_rate.x;
    mavlink_attitude_target.body_pitch_rate = msg->body_rate.y;
    mavlink_attitude_target.body_yaw_rate = msg->body_rate.z;

    mavlink_message_t message;
    mavlink_msg_attitude_target_encode(system_id, component_id, &message, &mavlink_attitude_target);
    // 发送
    int len = port->write_message(message);
}

void MavlinkSender::setpoint_raw_local_cb(const sunray_msgs::LocalPositionSetpoint::ConstPtr &msg)
{
    mavlink_set_position_target_local_ned_t mavlink_set_position_target_local_ned = { 0 };
    mavlink_set_position_target_local_ned.time_boot_ms = msg->header.stamp.toNSec() / 1000000;
    mavlink_set_position_target_local_ned.coordinate_frame = msg->coordinate_frame;
    mavlink_set_position_target_local_ned.type_mask = msg->type_mask;
    mavlink_set_position_target_local_ned.x = msg->position.x;
    mavlink_set_position_target_local_ned.y = msg->position.y;
    mavlink_set_position_target_local_ned.z = msg->position.z;
    mavlink_set_position_target_local_ned.vx = msg->velocity.x;
    mavlink_set_position_target_local_ned.vy = msg->velocity.y;
    mavlink_set_position_target_local_ned.vz = msg->velocity.z;
    mavlink_set_position_target_local_ned.afx = msg->acceleration_or_force.x;
    mavlink_set_position_target_local_ned.afy = msg->acceleration_or_force.y;
    mavlink_set_position_target_local_ned.afz = msg->acceleration_or_force.z;
    mavlink_set_position_target_local_ned.yaw = msg->yaw;
    mavlink_set_position_target_local_ned.yaw_rate = msg->yaw_rate;

    mavlink_message_t message;
    mavlink_msg_set_position_target_local_ned_encode(system_id, component_id, &message, &mavlink_set_position_target_local_ned);
    // 发送
    int len = port->write_message(message);
}

void MavlinkSender::setpoint_raw_global_cb(const sunray_msgs::GlobalPositionSetpoint::ConstPtr &msg)
{
    //该话题支持三个坐标系:
    //FRAME_GLOBAL_INT 高度数据为海拔高度
    //FRAME_GLOBAL_REL_ALT 高度数据为相对起始位置的高度,HOME点的高度为0
    //FRAME_GLOBAL_TERRAIN_ALT 具有 AGL 高度的全球 (WGS84) 坐标系（在航路点坐标处）。第一个值/x：以度为单位的纬度，第二个值/y：以度为单位的经度，第三个值/z：以米为单位的正高度，0 表示地形模型中的地平面。
    //在仿真中测试使用后,FRAME_GLOBAL_REL_ALT使用比较方便推荐使用该坐标系
    //https://mavlink.io/en/messages/common.html#MAV_FRAME_GLOBAL_INT  坐标系的详细介绍
    mavlink_set_position_target_global_int_t mavlink_set_position_target_global_int = { 0 };
    mavlink_set_position_target_global_int.time_boot_ms = msg->header.stamp.toNSec() / 1000000;
    mavlink_set_position_target_global_int.coordinate_frame = msg->coordinate_frame;
    mavlink_set_position_target_global_int.type_mask = msg->type_mask;
    mavlink_set_position_target_global_int.lat_int = msg->latitude;
    mavlink_set_position_target_global_int.lon_int = msg->longitude;
    mavlink_set_position_target_global_int.alt = msg->altitude;
    mavlink_set_position_target_global_int.vx = msg->velocity.x;
    mavlink_set_position_target_global_int.vy = msg->velocity.y;
    mavlink_set_position_target_global_int.vz = msg->velocity.z;
    mavlink_set_position_target_global_int.afx = msg->acceleration_or_force.x;
    mavlink_set_position_target_global_int.afy = msg->acceleration_or_force.y;
    mavlink_set_position_target_global_int.afz = msg->acceleration_or_force.z;
    mavlink_set_position_target_global_int.yaw = msg->yaw;
    mavlink_set_position_target_global_int.yaw_rate = msg->yaw_rate;

    mavlink_message_t message;
    mavlink_msg_set_position_target_global_int_encode(system_id, component_id, &message, &mavlink_set_position_target_global_int);
    // 发送
    int len = port->write_message(message);
}

void MavlinkSender::arming_cmd_cb(const std_msgs::Bool::ConstPtr &msg)
{
    mavlink_command_long_t command_long = { 0 };
    command_long.target_system = system_id;
    command_long.target_component = component_id;
    command_long.command = MAV_CMD::MAV_CMD_COMPONENT_ARM_DISARM;
    command_long.confirmation = 1;
    command_long.param1 = msg->data;    // 1: arm, 0: disarm
    command_long.param2 = 0;    
    command_long.param3 = 0;    
    command_long.param4 = 0;    
    command_long.param5 = 0;    
    command_long.param6 = 0;    
    command_long.param7 = 0;    

    if(msg->data)
    {
        cout << GREEN_IN_WHITE << " Arm CMD send!" << TAIL << endl;
    }
    else
    {
        cout << GREEN_IN_WHITE << " Disarm CMD send!" << TAIL << endl;
    }

    mavlink_message_t message;
    mavlink_msg_command_long_encode(system_id, 240, &message, &command_long);
    // 发送
    int len = port->write_message(message);
}

void MavlinkSender::set_mode_cmd_cb(const sunray_msgs::SetMode::ConstPtr &msg)
{
    mavlink_set_mode_t set_mode = { 0 };
    set_mode.target_system = system_id;
    set_mode.base_mode = 1;
    uint32_t custom_mode = 0;
    cmode_from_str(msg->custom_mode, custom_mode);
    set_mode.custom_mode = custom_mode;

    cout << GREEN_IN_WHITE << " Set Mode: [ "<< msg->custom_mode << "]" << TAIL << endl;

    mavlink_message_t message;
    mavlink_msg_set_mode_encode(system_id, 240, &message, &set_mode);
    // 发送
    int len = port->write_message(message);
}

void MavlinkSender::emergency_kill_cmd_cb(const std_msgs::Bool::ConstPtr &msg)
{
    // TODO
}

void MavlinkSender::reboot_cmd_cb(const std_msgs::Bool::ConstPtr &msg)
{
    if(!msg->data)
    {
        cout << RED_IN_WHITE << "Autopilot Reboot failed: wrong value!" << TAIL << endl;
        return;
    }

    mavlink_command_long_t command_long = { 0 };
    command_long.target_system = system_id;
    command_long.target_component = component_id;
    command_long.command = MAV_CMD::MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN;
    command_long.confirmation = 1;
    command_long.param1 = 1.0;    // 1: reboot, 0: do nothing
    command_long.param2 = 0;    
    command_long.param3 = 0;    
    command_long.param4 = 0;    
    command_long.param5 = 0;    
    command_long.param6 = 0;    
    command_long.param7 = 0;    
    cout << GREEN_IN_WHITE << "Autopilot Reboot!" << TAIL << endl;

    mavlink_message_t message;
    mavlink_msg_command_long_encode(system_id, 240, &message, &command_long);
    // 发送
    int len = port->write_message(message);
}



bool MavlinkSender::arming_cb(sunray_msgs::ArmCmd::Request &req, sunray_msgs::ArmCmd::Response &res)
{
    // TODO
}

void MavlinkSender::vision_pose_cb(const sunray_msgs::VisionPositionEstimate::ConstPtr &msg)
{
    mavlink_vision_position_estimate_t mavlink_vision_position_estimate = { 0 };
    mavlink_vision_position_estimate.x = msg->position[0];
    mavlink_vision_position_estimate.y = msg->position[1];
    mavlink_vision_position_estimate.z = msg->position[2];
    mavlink_vision_position_estimate.roll = msg->attitude[0];
    mavlink_vision_position_estimate.pitch = msg->attitude[1];
    mavlink_vision_position_estimate.yaw = msg->attitude[2];

    mavlink_message_t message;
    mavlink_msg_vision_position_estimate_encode(system_id, component_id, &message, &mavlink_vision_position_estimate);
    // 发送
    int len = port->write_message(message);

    // log
    msg_status_vision_pose.send_new_msg(ros::Time::now());
}

#endif
