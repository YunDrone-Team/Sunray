#ifndef MAVLINK_RECEIVER_H
#define MAVLINK_RECEIVER_H

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
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>


#include <sunray_msgs/Heartbeat.h>
#include <sunray_msgs/SysStatus.h>
#include <sunray_msgs/RCChannels.h>

#include <sunray_msgs/Attitude.h>
#include <sunray_msgs/AttitudeQuaternion.h>
#include <sunray_msgs/AttitudeTarget.h>
#include <sunray_msgs/PositionTargetLocalNED.h>


#include <sunray_msgs/LocalPositionNED.h>

#include "common/mavlink.h"
#include "msg_status.h"
#include "printf_utils.h"
#include "enum_utils.h"
#include "generic_port.h"
#include "serial_port.h"
#include "udp_port.h"
#include "px4_custom_mode.h"
#include "get_mode.h"
#include "math_utils.h"

using namespace std;

class MavlinkReceiver
{
    public:
        //构造函数
        MavlinkReceiver(){};

		int	system_id; // system id
		int	component_id; // component id

        // 初始化函数
        void init(ros::NodeHandle& nh, Generic_Port *port);
        // 打印debug信息
        void printf_debug_info();
        // 设置Mavlink消息接收频率（FMT生效，PX4待测试）
        void set_mavlink_msg_frequency(int msg_id, int msg_rate);
        // 消息处理函数
        void handle_msg(const mavlink_message_t* msg);
        // HEARTBEAT #0
        void handle_heartbeat(const mavlink_message_t* msg);
        // SYS_STATUS #1
        void handle_sys_status(const mavlink_message_t* msg);
        // GPS_RAW_INT #24
        void handle_gps_raw_int(const mavlink_message_t* msg);
        // ATTITUDE #30
        void handle_attitude(const mavlink_message_t* msg);
        // ATTITUDE_QUATERNION #31
        void handle_attitude_quaternion(const mavlink_message_t* msg);
        // LOCAL_POSITION_NED #32
        void handle_local_position_ned(const mavlink_message_t* msg);
        // GLOBAL_POSITION_INT #33
        void handle_global_position_int(const mavlink_message_t* msg);
        // RC_CHANNELS_RAW #35
        void handle_rc_channels_raw(const mavlink_message_t* msg);
        // RC_CHANNELS #65
        void handle_rc_channels(const mavlink_message_t* msg);
        // ATTITUDE_TARGET #83
        void handle_attitude_target(const mavlink_message_t* msg);
        // POSITION_TARGET_LOCAL_NED #85 
        void handle_position_target(const mavlink_message_t* msg);
        // HIGHRES_IMU #105 
        void handle_highres_imu(const mavlink_message_t* msg);
        // DISTANCE_SENSOR #132
        void handle_distance_sensor(const mavlink_message_t* msg);
    private:        
        // 通信端口
        Generic_Port *port;

        // 无人机ID
        int uav_id;
        // 无人机名字
        string uav_name{""};
        string px4_or_fmt{""};

        // 接收消息状态（监测用）
        MSG_STATUS msg_status_heartbeat;
        MSG_STATUS msg_status_sys_status;
        MSG_STATUS msg_status_gps_raw_init;
        MSG_STATUS msg_status_attitude;
        MSG_STATUS msg_status_attitude_quaterion;
        MSG_STATUS msg_status_local_position_ned;
        MSG_STATUS msg_status_global_position_init;
        MSG_STATUS msg_status_rc_channels_raw;
        MSG_STATUS msg_status_rc_channels;
        MSG_STATUS msg_status_attitude_target;
        MSG_STATUS msg_status_position_target;
        MSG_STATUS msg_status_highres_imu;
        MSG_STATUS msg_status_distance_sensor;
        

        // 最新消息存储
        struct LAST_MSG
        {
            sunray_msgs::Heartbeat heartbeat;
            sunray_msgs::SysStatus sys_status;
            sunray_msgs::Attitude attitude;
            sunray_msgs::AttitudeQuaternion attitude_quat;
            sunray_msgs::RCChannels rc_channels;
            sunray_msgs::AttitudeTarget attitude_target;
            sunray_msgs::LocalPositionNED local_position_ned;
            sunray_msgs::PositionTargetLocalNED position_target;
            sensor_msgs::Imu highers_imu_ned;
        };
        LAST_MSG last_msg;

        ros::Publisher heartbeat_pub;
        ros::Publisher sys_status_pub;
        ros::Publisher gps_raw_init_pub;
        ros::Publisher attitude_pub;
        ros::Publisher attitude_quaterion_pub;
        ros::Publisher local_position_ned_pub;
        ros::Publisher global_position_init_pub;
        ros::Publisher rc_channels_raw_pub;
        ros::Publisher rc_channels_pub;
        ros::Publisher attitude_target_pub;
        ros::Publisher position_target_pub;
        ros::Publisher highres_imu_pub;
        ros::Publisher distance_sensor_pub;
};

void MavlinkReceiver::init(ros::NodeHandle& nh, Generic_Port *port)
{
    nh.param<string>("uav_name", uav_name, "none");
    nh.param<int>("uav_id", uav_id, 0);
    nh.param<string>("px4_or_fmt", px4_or_fmt, "px4");

    string topic_name = "/"  + uav_name + std::to_string(uav_id);

    this->port = port;

    system_id    = 1; // system id
    component_id = 1; // component id

    // 【发布】无人机当前状态 - HEARTBEAT #0
    heartbeat_pub = nh.advertise<sunray_msgs::Heartbeat>(topic_name + "/sunray/heartbeat", 1);
    // 【发布】无人机电池状态 - SYS_STATUS #1
    sys_status_pub = nh.advertise<sunray_msgs::SysStatus>(topic_name + "/sunray/sys_status", 1);
    // 【发布】GPS状态 - GPS_RAW_INT #24
    gps_raw_init_pub = nh.advertise<geometry_msgs::Point>(topic_name + "/sunray/gps_raw_init", 10);
    // 【发布】无人机当前欧拉角 - ATTITUDE #30 (2选1都可)
    attitude_pub = nh.advertise<sunray_msgs::Attitude>(topic_name + "/sunray/attitude", 1);
    // 【发布】无人机当前欧拉角 - ATTITUDE_QUATERNION #31 (2选1都可)
    attitude_quaterion_pub = nh.advertise<sunray_msgs::AttitudeQuaternion>(topic_name + "/sunray/attitude_quaterion", 1);
    // 【发布】无人机当前位置 - LOCAL_POSITION_NED #32
    local_position_ned_pub = nh.advertise<sunray_msgs::LocalPositionNED>(topic_name + "/sunray/local_position_ned", 1);
    // 【发布】无人机当前经纬度 - GLOBAL_POSITION_INT #33
    global_position_init_pub = nh.advertise<geometry_msgs::Point>(topic_name + "/sunray/global_position_init", 1);
    //【发布】遥控器信息 - RC_CHANNELS_RAW #35 or RC_CHANNELS #65
    rc_channels_raw_pub = nh.advertise<geometry_msgs::Point>(topic_name + "/sunray/rc_channels_raw", 1);    
    //【发布】遥控器信息 - RC_CHANNELS_RAW #35 or RC_CHANNELS #65
    rc_channels_pub = nh.advertise<sunray_msgs::RCChannels>(topic_name + "/sunray/rc_channels", 1);  
    //【发布】无人机的姿态设定值 - ATTITUDE_TARGET #83
    attitude_target_pub = nh.advertise<sunray_msgs::AttitudeTarget>(topic_name + "/sunray/attitude_target", 1);
    //【发布】无人机的位置/速度/加速度设定值 - POSITION_TARGET_LOCAL_NED #85 
    position_target_pub = nh.advertise<sunray_msgs::PositionTargetLocalNED>(topic_name + "/sunray/position_target", 1);
    // 【发布】无人机原始imu信息 - HIGHRES_IMU #105
    highres_imu_pub = nh.advertise<sensor_msgs::Imu>(topic_name + "/sunray/highres_imu_ned", 1);
    // 【发布】无人机定高雷达数据 - DISTANCE_SENSOR #132
    distance_sensor_pub = nh.advertise<sensor_msgs::Range>(topic_name + "/sunray/distance_sensor", 1);

}

// 设置Mavlink消息接收频率（FMT生效，PX4待测试）
void MavlinkReceiver::set_mavlink_msg_frequency(int msg_id, int msg_rate)
{
    // 初始化消息
    mavlink_request_data_stream_t mavlink_request_data_stream = { 0 };
    mavlink_request_data_stream.target_system = system_id;
    mavlink_request_data_stream.target_component = component_id;
    mavlink_request_data_stream.req_stream_id = msg_id;
    mavlink_request_data_stream.req_message_rate = msg_rate;
    mavlink_request_data_stream.start_stop = true;      //false代表停止发送

    // 发送消息
    mavlink_message_t message;
    mavlink_msg_request_data_stream_encode(system_id, component_id, &message, &mavlink_request_data_stream);
    int len = port->write_message(message);
    usleep(10);


    cout << BLUE << "Set MAVLINK ID [" << msg_id << "] rate to "<< msg_rate << "Hz successfully!" << TAIL << endl;
}

void MavlinkReceiver::handle_msg(const mavlink_message_t* msg)
{
    // Handle Message ID
    switch (msg->msgid)
    {
        case MAVLINK_MSG_ID_HEARTBEAT:
        {
            handle_heartbeat(msg);
            break;
        }
        case MAVLINK_MSG_ID_SYS_STATUS:
        {
            handle_sys_status(msg);
            break;
        }
        case MAVLINK_MSG_ID_GPS_RAW_INT:
        {
            handle_gps_raw_int(msg);
            break;
        }
        case MAVLINK_MSG_ID_ATTITUDE:
        {
            handle_attitude(msg);
            break;
        }
        case MAVLINK_MSG_ID_ATTITUDE_QUATERNION:
        {
            handle_attitude_quaternion(msg);
            break;
        }
        case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
        {
            handle_local_position_ned(msg);
            break;
        }
        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
        {
            handle_global_position_int(msg);
            break;
        }
        case MAVLINK_MSG_ID_RC_CHANNELS_RAW:
        {
            handle_rc_channels_raw(msg);
            break;
        }
        case MAVLINK_MSG_ID_RC_CHANNELS:
        {
            handle_rc_channels(msg);
            break;
        }
        case MAVLINK_MSG_ID_ATTITUDE_TARGET:
        {
            handle_attitude_target(msg);
            break;
        }
        case MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED:
        {
            handle_position_target(msg);
            break;
        }
        case MAVLINK_MSG_ID_HIGHRES_IMU:
        {
            // handle_highres_imu(msg);
            break;
        }
        case MAVLINK_MSG_ID_DISTANCE_SENSOR:
        {
            handle_distance_sensor(msg);
            break;
        }
        default:
        {
            // cout << RED << "Got msg with ID: " << int(msg->msgid) << TAIL << endl;
            break;
        }
    }
}

void MavlinkReceiver::handle_heartbeat(const mavlink_message_t* msg)
{
    mavlink_heartbeat_t mavlink_heartbeat = { 0 };
    mavlink_msg_heartbeat_decode(msg, &mavlink_heartbeat);

    sunray_msgs::Heartbeat heartbeat;
    heartbeat.header.stamp = ros::Time::now();
    heartbeat.mav_type = int(mavlink_heartbeat.type);
    heartbeat.autopilot = int(mavlink_heartbeat.autopilot);
    heartbeat.base_mode = int(mavlink_heartbeat.base_mode);
    heartbeat.custom_mode = uint32_t(mavlink_heartbeat.custom_mode);
    // 合适设置为false？ 如果丢失心跳包一段时间，则发布false
    heartbeat.connected = true;
    heartbeat.armed =  !!(mavlink_heartbeat.base_mode & MAV_MODE_FLAG::MAV_MODE_FLAG_SAFETY_ARMED);
    heartbeat.mode = get_mode(mavlink_heartbeat.base_mode, mavlink_heartbeat.custom_mode);
    heartbeat_pub.publish(heartbeat);

    // 估算接收频率
    msg_status_heartbeat.recv_new_msg(ros::Time::now());

    // 存储最新的消息
    last_msg.heartbeat = heartbeat;

    // cout << GREEN << "Got mavlink_heartbeat:" << TAIL << endl;
    // cout << GREEN << "--> type      : " << int(mavlink_heartbeat.type) << TAIL << endl;
    // cout << GREEN << "--> autopilot : " << int(mavlink_heartbeat.autopilot) << TAIL << endl;
    // cout << GREEN << "--> base_mode : " << int(mavlink_heartbeat.base_mode) << TAIL << endl;
    // cout << GREEN << "--> custom_mode : " << int(mavlink_heartbeat.custom_mode) << TAIL << endl;
    // cout << GREEN << "--> system_status : " << int(mavlink_heartbeat.system_status) << TAIL << endl;
    // cout << GREEN << "--> mavlink_version : " << int(mavlink_heartbeat.mavlink_version) << TAIL << endl;
}
// SYS_STATUS #1
void MavlinkReceiver::handle_sys_status(const mavlink_message_t* msg)
{
    mavlink_sys_status_t mavlink_sys_status = { 0 };
    mavlink_msg_sys_status_decode(msg, &mavlink_sys_status);

    sunray_msgs::SysStatus sys_status;
    sys_status.header.stamp = ros::Time::now();
    sys_status.battery_voltage = mavlink_sys_status.voltage_battery / 1000.0f;
    sys_status.battery_percetage = mavlink_sys_status.battery_remaining / 100.0f;
    sys_status_pub.publish(sys_status);

    // 估算接收频率
    msg_status_sys_status.recv_new_msg(ros::Time::now());
    // 存储最新的消息
    last_msg.sys_status = sys_status;
}
// GPS_RAW_INT #24
void MavlinkReceiver::handle_gps_raw_int(const mavlink_message_t* msg)
{
    geometry_msgs::Point sys_status;
    gps_raw_init_pub.publish(sys_status);
    // 估算接收频率
    msg_status_gps_raw_init.recv_new_msg(ros::Time::now());
    // cout << GREEN << "Got GPS_RAW_INT:" << TAIL << endl;

}
// ATTITUDE #30
void MavlinkReceiver::handle_attitude(const mavlink_message_t* msg)
{
    mavlink_attitude_t mavlink_attitude = { 0 };
    mavlink_msg_attitude_decode(msg, &mavlink_attitude);
    sunray_msgs::Attitude attitude;
    attitude.header.stamp = ros::Time::now();
    attitude.time_boot_ms = float(mavlink_attitude.time_boot_ms);
    attitude.attitude[0] = float(mavlink_attitude.roll);
    attitude.attitude[1] = float(mavlink_attitude.pitch);
    attitude.attitude[2] = float(mavlink_attitude.yaw);
    // 欧拉角转四元数
    Eigen::Vector3d euler;
    euler[0] = attitude.attitude[0];
    euler[1] = attitude.attitude[1];
    euler[2] = attitude.attitude[2];
    attitude.attitude_q = geo_quaternion_from_rpy(euler);
    attitude.attitude_rate[0] = float(mavlink_attitude.rollspeed);
    attitude.attitude_rate[1] = float(mavlink_attitude.pitchspeed);
    attitude.attitude_rate[2] = float(mavlink_attitude.yawspeed);
    attitude_pub.publish(attitude);

    // 估算接收频率
    msg_status_attitude.recv_new_msg(attitude.header.stamp);
    // 存储最新的消息
    last_msg.attitude = attitude;
}

// ATTITUDE_QUATERNION #31
void MavlinkReceiver::handle_attitude_quaternion(const mavlink_message_t* msg)
{
    mavlink_attitude_quaternion_t mavlink_attitude_quaternion = { 0 };
    mavlink_msg_attitude_quaternion_decode(msg, &mavlink_attitude_quaternion);

    sunray_msgs::AttitudeQuaternion attitude_quat;
    attitude_quat.header.stamp = ros::Time::now();
    attitude_quat.time_boot_ms = float(mavlink_attitude_quaternion.time_boot_ms);
    attitude_quat.attitude_q.w = float(mavlink_attitude_quaternion.q1);
    attitude_quat.attitude_q.x = float(mavlink_attitude_quaternion.q2);
    attitude_quat.attitude_q.y = float(mavlink_attitude_quaternion.q3);
    attitude_quat.attitude_q.z = float(mavlink_attitude_quaternion.q4);
    // 四元数转欧拉角
    Eigen::Vector3d euler;
    euler = geo_quaternion_to_euler(attitude_quat.attitude_q);
    attitude_quat.attitude[0] = euler[0];
    attitude_quat.attitude[1] = euler[1];
    attitude_quat.attitude[2] = euler[2];
    attitude_quat.attitude_rate[0] = float(mavlink_attitude_quaternion.rollspeed);
    attitude_quat.attitude_rate[1] = float(mavlink_attitude_quaternion.pitchspeed);
    attitude_quat.attitude_rate[2] = float(mavlink_attitude_quaternion.yawspeed);

    attitude_quaterion_pub.publish(attitude_quat);

    // 估算接收频率
    msg_status_attitude_quaterion.recv_new_msg(ros::Time::now());
    // 存储最新的消息
    last_msg.attitude_quat = attitude_quat;
}
// LOCAL_POSITION_NED #32
void MavlinkReceiver::handle_local_position_ned(const mavlink_message_t* msg)
{
    mavlink_local_position_ned_t mavlink_local_position_ned = { 0 };
    mavlink_msg_local_position_ned_decode(msg, &mavlink_local_position_ned);

    sunray_msgs::LocalPositionNED local_position_ned;
    local_position_ned.header.stamp = ros::Time::now();
    local_position_ned.time_boot_ms = float(mavlink_local_position_ned.time_boot_ms);
    local_position_ned.position[0] = float(mavlink_local_position_ned.x);
    local_position_ned.position[1] = float(mavlink_local_position_ned.y);
    local_position_ned.position[2] = float(mavlink_local_position_ned.z);
    local_position_ned.velocity[0] = float(mavlink_local_position_ned.vx);
    local_position_ned.velocity[1] = float(mavlink_local_position_ned.vy);
    local_position_ned.velocity[2] = float(mavlink_local_position_ned.vz);
    local_position_ned_pub.publish(local_position_ned);

    // 估算接收频率
    msg_status_local_position_ned.recv_new_msg(local_position_ned.header.stamp);
    // 存储最新的消息
    last_msg.local_position_ned = local_position_ned;
}

// GLOBAL_POSITION_INT #33
void MavlinkReceiver::handle_global_position_int(const mavlink_message_t* msg)
{
    geometry_msgs::Point sys_status;
    global_position_init_pub.publish(sys_status);

    // 估算接收频率
    msg_status_global_position_init.recv_new_msg(ros::Time::now());

    // cout << GREEN << "Got GLOBAL_POSITION_INT:" << TAIL << endl;

}

// RC_CHANNELS_RAW #35
void MavlinkReceiver::handle_rc_channels_raw(const mavlink_message_t* msg)
{
    
    geometry_msgs::Point sys_status;
    rc_channels_raw_pub.publish(sys_status);

    // 估算接收频率
    msg_status_rc_channels_raw.recv_new_msg(ros::Time::now());
    // cout << GREEN << "Got RC_CHANNELS_RAW:" << TAIL << endl;
}

// RC_CHANNELS #65
void MavlinkReceiver::handle_rc_channels(const mavlink_message_t* msg)
{
    mavlink_rc_channels_t mavlink_rc_channels = { 0 };
    mavlink_msg_rc_channels_decode(msg, &mavlink_rc_channels);

    sunray_msgs::RCChannels rc_channels;
    rc_channels.header.stamp = ros::Time::now();
    rc_channels.time_boot_ms = float(mavlink_rc_channels.time_boot_ms);
    rc_channels.channels[0] = float(mavlink_rc_channels.chan1_raw);
    rc_channels.channels[1] = float(mavlink_rc_channels.chan2_raw);
    rc_channels.channels[2] = float(mavlink_rc_channels.chan3_raw);
    rc_channels.channels[3] = float(mavlink_rc_channels.chan4_raw);
    rc_channels.channels[4] = float(mavlink_rc_channels.chan5_raw);
    rc_channels.channels[5] = float(mavlink_rc_channels.chan6_raw);
    rc_channels.channels[6] = float(mavlink_rc_channels.chan7_raw);
    rc_channels.channels[7] = float(mavlink_rc_channels.chan8_raw);
    rc_channels.channels[8] = float(mavlink_rc_channels.chan9_raw);
    rc_channels.channels[9] = float(mavlink_rc_channels.chan10_raw);
    rc_channels.channels[10] = float(mavlink_rc_channels.chan11_raw);
    rc_channels.channels[11] = float(mavlink_rc_channels.chan12_raw);
    rc_channels.channels[12] = float(mavlink_rc_channels.chan13_raw);
    rc_channels.channels[13] = float(mavlink_rc_channels.chan14_raw);
    rc_channels.channels[14] = float(mavlink_rc_channels.chan15_raw);
    rc_channels.channels[15] = float(mavlink_rc_channels.chan16_raw);
    rc_channels.channels[16] = float(mavlink_rc_channels.chan17_raw);
    rc_channels.channels[17] = float(mavlink_rc_channels.chan18_raw);
    rc_channels_pub.publish(rc_channels);
    // 估算接收频率
    msg_status_rc_channels.recv_new_msg(ros::Time::now());
    // 存储最新的消息
    last_msg.rc_channels = rc_channels;
}

// ATTITUDE_TARGET #83
void MavlinkReceiver::handle_attitude_target(const mavlink_message_t* msg)
{
    mavlink_attitude_target_t mavlink_attitude_target = { 0 };
    mavlink_msg_attitude_target_decode(msg, &mavlink_attitude_target);

    sunray_msgs::AttitudeTarget attitude_target;
    attitude_target.header.stamp = ros::Time::now();
    attitude_target.time_boot_ms = float(mavlink_attitude_target.time_boot_ms);
    attitude_target.header.stamp = ros::Time::now();
    attitude_target.attitude_q.w = float(mavlink_attitude_target.q[0]);
    attitude_target.attitude_q.x = float(mavlink_attitude_target.q[1]);
    attitude_target.attitude_q.y = float(mavlink_attitude_target.q[2]);
    attitude_target.attitude_q.z = float(mavlink_attitude_target.q[3]);
    // 四元数转欧拉角
    Eigen::Vector3d euler;
    euler = geo_quaternion_to_euler(attitude_target.attitude_q);
    attitude_target.attitude[0] = euler[0];
    attitude_target.attitude[1] = euler[1];
    attitude_target.attitude[2] = euler[2];
    attitude_target.attitude_rate[0] = float(mavlink_attitude_target.body_roll_rate);
    attitude_target.attitude_rate[1] = float(mavlink_attitude_target.body_pitch_rate);
    attitude_target.attitude_rate[2] = float(mavlink_attitude_target.body_yaw_rate);
    attitude_target_pub.publish(attitude_target);

    // 估算接收频率
    msg_status_attitude_target.recv_new_msg(ros::Time::now());
    // 存储最新的消息
    last_msg.attitude_target = attitude_target;
}

// POSITION_TARGET_LOCAL_NED #85
void MavlinkReceiver::handle_position_target(const mavlink_message_t* msg)
{
    mavlink_position_target_local_ned_t mavlink_position_target_local_ned = { 0 };
    mavlink_msg_position_target_local_ned_decode(msg, &mavlink_position_target_local_ned);

    sunray_msgs::PositionTargetLocalNED position_target;
    position_target.header.stamp = ros::Time::now();
    position_target.time_boot_ms = float(mavlink_position_target_local_ned.time_boot_ms);
    position_target.position[0] = float(mavlink_position_target_local_ned.x);
    position_target.position[1] = float(mavlink_position_target_local_ned.y);
    position_target.position[2] = float(mavlink_position_target_local_ned.z);
    position_target.velocity[0] = float(mavlink_position_target_local_ned.vx);
    position_target.velocity[1] = float(mavlink_position_target_local_ned.vy);
    position_target.velocity[2] = float(mavlink_position_target_local_ned.vz);
    position_target.accel[0] = float(mavlink_position_target_local_ned.afx);
    position_target.accel[1] = float(mavlink_position_target_local_ned.afy);
    position_target.accel[2] = float(mavlink_position_target_local_ned.afz);
    position_target.yaw = float(mavlink_position_target_local_ned.yaw);
    position_target.yaw_rate = float(mavlink_position_target_local_ned.yaw_rate);
    position_target_pub.publish(position_target);

    // 估算接收频率
    msg_status_position_target.recv_new_msg(ros::Time::now());
    // 存储最新的消息
    last_msg.position_target = position_target;
}

// HIGHRES_IMU #105
void MavlinkReceiver::handle_highres_imu(const mavlink_message_t* msg)
{
    mavlink_highres_imu_t mavlink_highres_imu = { 0 };
    mavlink_msg_highres_imu_decode(msg, &mavlink_highres_imu);

    sensor_msgs::Imu highers_imu_ned;
    sensor_msgs::Imu highers_imu_enu;
    highers_imu_ned.header.stamp = ros::Time::now();
    highers_imu_ned.linear_acceleration.x = mavlink_highres_imu.xacc;
    highers_imu_ned.linear_acceleration.y = mavlink_highres_imu.yacc;
    highers_imu_ned.linear_acceleration.z = mavlink_highres_imu.zacc;
    highers_imu_ned.angular_velocity.x = mavlink_highres_imu.xgyro;
    highers_imu_ned.angular_velocity.y = mavlink_highres_imu.ygyro;
    highers_imu_ned.angular_velocity.z = mavlink_highres_imu.zgyro;
    highres_imu_pub.publish(highers_imu_ned);

    // 估算接收频率
    msg_status_highres_imu.recv_new_msg(ros::Time::now());
    // 存储最新的消息
    last_msg.highers_imu_ned = highers_imu_ned;
}

// DISTANCE_SENSOR #132
void MavlinkReceiver::handle_distance_sensor(const mavlink_message_t* msg)
{
    sensor_msgs::Range sys_status;
    distance_sensor_pub.publish(sys_status);

    // 估算接收频率
    msg_status_distance_sensor.recv_new_msg(ros::Time::now());
    // cout << GREEN << "Got DISTANCE_SENSOR:" << TAIL << endl;

}

void MavlinkReceiver::printf_debug_info()
{
    cout << BLUE << ">>>>>>>>>>>>>>>>>>>>>> UAV [" << uav_id << "] Mavlink Receiver <<<<<<<<<<<<<<<<<<<<<<" << TAIL << endl;
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
    // cout << GREEN << "Get sys_status with           : " << msg_frequency.sys_status << " [Hz]." <<  TAIL << endl;
    // cout << GREEN << "Get gps_raw_init with         : " << msg_frequency.gps_raw_init << " [Hz]." <<  TAIL << endl;
    // cout << GREEN << "Get attitude_quaterion with   : " << msg_frequency.attitude_quaterion << " [Hz]." <<  TAIL << endl;
    // cout << GREEN << "Get global_position_init with : " << msg_frequency.global_position_init << " [Hz]." <<  TAIL << endl;
    // cout << GREEN << "Get rc_channels_raw with      : " << msg_frequency.rc_channels_raw << " [Hz]." <<  TAIL << endl;
    // cout << GREEN << "Get rc_channels with          : " << msg_frequency.rc_channels << " [Hz]." <<  TAIL << endl;
    // cout << GREEN << "Get attitude_target with      : " << msg_frequency.attitude_target << " [Hz]." <<  TAIL << endl;
    // cout << GREEN << "Get position_target with      : " << msg_frequency.position_target << " [Hz]." <<  TAIL << endl;
    // cout << GREEN << "Get highres_imu with          : " << msg_frequency.highres_imu << " [Hz]." <<  TAIL << endl;
    // cout << GREEN << "Get distance_sensor with      : " << msg_frequency.distance_sensor << " [Hz]." <<  TAIL << endl;

    // HEARTBEAT
    cout << BLUE << "Get HEARTBEAT: ";
    cout << "Msg id: [ "<< MAVLINK_MSG_ID_HEARTBEAT << " ], ";
    cout << "Msg frequency: [ "<< msg_status_heartbeat.get_recv_msg_frequency() << " Hz ], ";
    cout << "Total num: [ "<< msg_status_heartbeat.get_recv_msg_count() <<" ]." <<  TAIL << endl;
    if(last_msg.heartbeat.mav_type == MAV_TYPE_QUADROTOR)
    {
        cout << GREEN << "--> mav_type :  MAV_TYPE_QUADROTOR " << TAIL << endl;
    }else
    {
        cout << GREEN << "--> mav_type :  unknown MAV_TYPE " << TAIL << endl;
    }
    if(last_msg.heartbeat.autopilot == MAV_AUTOPILOT_PX4)
    {
        cout << GREEN << "--> autopilot:  MAV_AUTOPILOT_PX4 " << TAIL << endl;
    }else
    {
        cout << GREEN << "--> autopilot:  unknown AUTOPILOT " << TAIL << endl;
    }
    cout << GREEN << "--> connected: " << int(last_msg.heartbeat.connected) << " " << TAIL << endl;
    cout << GREEN << "--> armed    : " << int(last_msg.heartbeat.armed) << " " << TAIL << endl;
    cout << GREEN << "--> mode     : " << last_msg.heartbeat.mode << " " << TAIL << endl;
    // SYS_STATUS
    cout << BLUE << "Get SYS_STATUS: ";
    cout << "Msg id: [ "<< MAVLINK_MSG_ID_SYS_STATUS << " ], ";
    cout << "Msg frequency: [ "<< msg_status_sys_status.get_recv_msg_frequency() << " Hz ], ";
    cout << "Total num: [ "<< msg_status_sys_status.get_recv_msg_count() <<" ]." <<  TAIL << endl;
    // cout << GREEN << "--> battery_voltage   : " << last_msg.sys_status.battery_voltage << " [V] " << TAIL << endl;
    // cout << GREEN << "--> battery_percetage : " << last_msg.sys_status.battery_percetage << " [%] " << TAIL << endl;
    // GPS_RAW_INT
    // cout << BLUE << "Get GPS_RAW_INT: ";
    // cout << "Msg id: [ "<< MAVLINK_MSG_ID_GPS_RAW_INT << " ], ";
    // cout << "Msg frequency: [ "<< msg_status_gps_raw_init.get_recv_msg_frequency() << " Hz ], ";
    // cout << "Total num: [ "<< msg_status_gps_raw_init.get_recv_msg_count() <<" ]." <<  TAIL << endl;
    // ATTITUDE
    cout << BLUE << "Get ATTITUDE: ";
    cout << "Msg id: [ "<< MAVLINK_MSG_ID_ATTITUDE << " ], ";
    cout << "Msg frequency: [ "<< msg_status_attitude.get_recv_msg_frequency() << " Hz ], ";
    cout << "Total num: [ "<< msg_status_attitude.get_recv_msg_count() <<" ]." <<  TAIL << endl;
    // cout << GREEN << "--> time_boot_ms : " << last_msg.attitude.time_boot_ms << " [ms] " << TAIL << endl;
    // cout << GREEN << "--> roll         : " << last_msg.attitude.attitude[0]/M_PI*180.0 << " [deg] " << TAIL << endl;
    // cout << GREEN << "--> pitch        : " << last_msg.attitude.attitude[1]/M_PI*180.0 << " [deg] " << TAIL << endl;
    // cout << GREEN << "--> yaw          : " << last_msg.attitude.attitude[2]/M_PI*180.0 << " [deg] " << TAIL << endl;
    // cout << GREEN << "--> quat[w,x,y,z]:[" << last_msg.attitude.attitude_q.w <<", "<< last_msg.attitude.attitude_q.x <<", "<< last_msg.attitude.attitude_q.y <<", "<< last_msg.attitude.attitude_q.z << " ] " << TAIL << endl;   
    // cout << GREEN << "--> rollspeed    : " << last_msg.attitude.attitude_rate[0]/M_PI*180.0 << " [deg/s] " << TAIL << endl;
    // cout << GREEN << "--> pitchspeed   : " << last_msg.attitude.attitude_rate[1]/M_PI*180.0 << " [deg/s] " << TAIL << endl;
    // cout << GREEN << "--> yawspeed     : " << last_msg.attitude.attitude_rate[2]/M_PI*180.0 << " [deg/s] " << TAIL << endl;
    // ATTITUDE_QUATERNION
    // cout << BLUE << "Get ATTITUDE_QUATERNION: ";
    // cout << "Msg id: [ "<< MAVLINK_MSG_ID_ATTITUDE_QUATERNION << " ], ";
    // cout << "Msg frequency: [ "<< msg_status_attitude_quaterion.get_recv_msg_frequency() << " Hz ], ";
    // cout << "Total num: [ "<< msg_status_attitude_quaterion.get_recv_msg_count() <<" ]." <<  TAIL << endl;
    // cout << GREEN << "--> time_boot_ms : " << last_msg.attitude_quat.time_boot_ms << " [ms] " << TAIL << endl;
    // cout << GREEN << "--> roll         : " << last_msg.attitude_quat.attitude[0]/M_PI*180.0 << " [deg] " << TAIL << endl;
    // cout << GREEN << "--> pitch        : " << last_msg.attitude_quat.attitude[1]/M_PI*180.0 << " [deg] " << TAIL << endl;
    // cout << GREEN << "--> yaw          : " << last_msg.attitude_quat.attitude[2]/M_PI*180.0 << " [deg] " << TAIL << endl;
    // cout << GREEN << "--> quat[w,x,y,z]:[" << last_msg.attitude_quat.attitude_q.w <<", "<< last_msg.attitude_quat.attitude_q.x <<", "<< last_msg.attitude_quat.attitude_q.y <<", "<< last_msg.attitude_quat.attitude_q.z << " ] " << TAIL << endl;   
    // cout << GREEN << "--> rollspeed    : " << last_msg.attitude_quat.attitude_rate[0]/M_PI*180.0 << " [deg/s] " << TAIL << endl;
    // cout << GREEN << "--> pitchspeed   : " << last_msg.attitude_quat.attitude_rate[1]/M_PI*180.0 << " [deg/s] " << TAIL << endl;
    // cout << GREEN << "--> yawspeed     : " << last_msg.attitude_quat.attitude_rate[2]/M_PI*180.0 << " [deg/s] " << TAIL << endl;   
    // LOCAL_POSITION_NED
    cout << BLUE << "Get local_position_ned: ";
    cout << "Msg id: [ "<< MAVLINK_MSG_ID_LOCAL_POSITION_NED << " ], ";
    cout << "Msg frequency: [ "<< msg_status_local_position_ned.get_recv_msg_frequency() << " Hz ], ";
    cout << "Total num: [ "<< msg_status_local_position_ned.get_recv_msg_count() <<" ]." <<  TAIL << endl;
    cout << GREEN << "--> time_boot_ms         : " << last_msg.local_position_ned.time_boot_ms << " [ms] " << TAIL << endl;
    cout << GREEN << "--> local_position_ned_x : " << last_msg.local_position_ned.position[0] << " [m] " << TAIL << endl;
    cout << GREEN << "--> local_position_ned_y : " << last_msg.local_position_ned.position[1] << " [m] " << TAIL << endl;
    cout << GREEN << "--> local_position_ned_z : " << last_msg.local_position_ned.position[2] << " [m] " << TAIL << endl;
    cout << GREEN << "--> local_position_ned_vx: " << last_msg.local_position_ned.velocity[0] << " [m/s] " << TAIL << endl;
    cout << GREEN << "--> local_position_ned_vy: " << last_msg.local_position_ned.velocity[1] << " [m/s] " << TAIL << endl;
    cout << GREEN << "--> local_position_ned_vz: " << last_msg.local_position_ned.velocity[2] << " [m/s] " << TAIL << endl;
    // GLOBAL_POSITION_INT
    cout << BLUE << "Get GLOBAL_POSITION_INT: ";
    cout << "Msg id: [ "<< MAVLINK_MSG_ID_GLOBAL_POSITION_INT << " ], ";
    cout << "Msg frequency: [ "<< msg_status_global_position_init.get_recv_msg_frequency() << " Hz ], ";
    cout << "Total num: [ "<< msg_status_global_position_init.get_recv_msg_count() <<" ]." <<  TAIL << endl;
    // RC_CHANNELS_RAW
    cout << BLUE << "Get RC_CHANNELS_RAW: ";
    cout << "Msg id: [ "<< MAVLINK_MSG_ID_RC_CHANNELS_RAW << " ], ";
    cout << "Msg frequency: [ "<< msg_status_rc_channels_raw.get_recv_msg_frequency() << " Hz ], ";
    cout << "Total num: [ "<< msg_status_rc_channels_raw.get_recv_msg_count() <<" ]." <<  TAIL << endl;
    // RC_CHANNELS
    cout << BLUE << "Get RC_CHANNELS: ";
    cout << "Msg id: [ "<< MAVLINK_MSG_ID_RC_CHANNELS << " ], ";
    cout << "Msg frequency: [ "<< msg_status_rc_channels.get_recv_msg_frequency() << " Hz ], ";
    cout << "Total num: [ "<< msg_status_rc_channels.get_recv_msg_count() <<" ]." <<  TAIL << endl;
    // cout << GREEN << "--> time_boot_ms : " << last_msg.rc_channels.time_boot_ms << " [ms] " << TAIL << endl;
    // cout << GREEN << "--> channel[0]   : " << last_msg.rc_channels.channels[0] << " " << TAIL << endl;
    // cout << GREEN << "--> channel[1]   : " << last_msg.rc_channels.channels[1] << " " << TAIL << endl;
    // cout << GREEN << "--> channel[2]   : " << last_msg.rc_channels.channels[2] << " " << TAIL << endl;
    // cout << GREEN << "--> channel[3]   : " << last_msg.rc_channels.channels[3] << " " << TAIL << endl;
    // cout << GREEN << "--> channel[4]   : " << last_msg.rc_channels.channels[4] << " " << TAIL << endl;
    // cout << GREEN << "--> channel[5]   : " << last_msg.rc_channels.channels[5] << " " << TAIL << endl;
    // cout << GREEN << "--> channel[6]   : " << last_msg.rc_channels.channels[6] << " " << TAIL << endl;
    // ATTITUDE_TARGET
    cout << BLUE << "Get ATTITUDE_TARGET: ";
    cout << "Msg id: [ "<< MAVLINK_MSG_ID_ATTITUDE_TARGET << " ], ";
    cout << "Msg frequency: [ "<< msg_status_attitude_target.get_recv_msg_frequency() << " Hz ], ";
    cout << "Total num: [ "<< msg_status_attitude_target.get_recv_msg_count() <<" ]." <<  TAIL << endl;
    // cout << GREEN << "--> time_boot_ms : " << last_msg.attitude_target.time_boot_ms << " [ms] " << TAIL << endl;
    // cout << GREEN << "--> roll_target        : " << last_msg.attitude_target.attitude[0]/M_PI*180.0 << " [deg] " << TAIL << endl;
    // cout << GREEN << "--> pitch_target       : " << last_msg.attitude_target.attitude[1]/M_PI*180.0 << " [deg] " << TAIL << endl;
    // cout << GREEN << "--> yaw_target         : " << last_msg.attitude_target.attitude[2]/M_PI*180.0 << " [deg] " << TAIL << endl;
    // cout << GREEN << "--> q_target[w,x,y,z]  :[" << last_msg.attitude_target.attitude_q.w <<", "<< last_msg.attitude_target.attitude_q.x <<", "<< last_msg.attitude_target.attitude_q.y <<", "<< last_msg.attitude_target.attitude_q.z << " ] " << TAIL << endl;   
    // cout << GREEN << "--> rollspeed_target   : " << last_msg.attitude_target.attitude_rate[0]/M_PI*180.0 << " [deg/s] " << TAIL << endl;
    // cout << GREEN << "--> pitchspeed_target  : " << last_msg.attitude_target.attitude_rate[1]/M_PI*180.0 << " [deg/s] " << TAIL << endl;
    // cout << GREEN << "--> yawspeed_target    : " << last_msg.attitude_target.attitude_rate[2]/M_PI*180.0 << " [deg/s] " << TAIL << endl;
    // POSITION_TARGET_LOCAL_NED
    cout << BLUE << "Get POSITION_TARGET_LOCAL_NED: ";
    cout << "Msg id: [ "<< MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED << " ], ";
    cout << "Msg frequency: [ "<< msg_status_position_target.get_recv_msg_frequency() << " Hz ], ";
    cout << "Total num: [ "<< msg_status_position_target.get_recv_msg_count() <<" ]." <<  TAIL << endl;
    // cout << GREEN << "--> time_boot_ms         : " << last_msg.position_target.time_boot_ms << " [ms] " << TAIL << endl;
    // cout << GREEN << "--> target_x      : " << last_msg.position_target.position[0] << " [m] " << TAIL << endl;
    // cout << GREEN << "--> target_y      : " << last_msg.position_target.position[1] << " [m] " << TAIL << endl;
    // cout << GREEN << "--> target_z      : " << last_msg.position_target.position[2] << " [m] " << TAIL << endl;
    // cout << GREEN << "--> target_vx     : " << last_msg.position_target.velocity[0] << " [m/s] " << TAIL << endl;
    // cout << GREEN << "--> target_vy     : " << last_msg.position_target.velocity[1] << " [m/s] " << TAIL << endl;
    // cout << GREEN << "--> target_vz     : " << last_msg.position_target.velocity[2] << " [m/s] " << TAIL << endl;
    // cout << GREEN << "--> target_yaw    : " << last_msg.position_target.yaw/M_PI*180.0 << " [deg] " << TAIL << endl;
    // cout << GREEN << "--> target_yaw_rate: " << last_msg.position_target.yaw_rate/M_PI*180.0 << " [deg/s] " << TAIL << endl;
    // HIGHRES_IMU
    cout << BLUE << "Get HIGHRES_IMU: ";
    cout << "Msg id: [ "<< MAVLINK_MSG_ID_HIGHRES_IMU << " ], ";
    cout << "Msg frequency: [ "<< msg_status_highres_imu.get_recv_msg_frequency() << " Hz ], ";
    cout << "Total num: [ "<< msg_status_highres_imu.get_recv_msg_count() <<" ]." <<  TAIL << endl;
    // DISTANCE_SENSOR
    // cout << BLUE << "Get DISTANCE_SENSOR: ";
    // cout << "Msg id: [ "<< MAVLINK_MSG_ID_DISTANCE_SENSOR << " ], ";
    // cout << "Msg frequency: [ "<< msg_status_distance_sensor.get_recv_msg_frequency() << " Hz ], ";
    // cout << "Total num: [ "<< msg_status_distance_sensor.get_recv_msg_count() <<" ]." <<  TAIL << endl;
}
#endif
