#ifndef UAV_CONTROL_H
#define UAV_CONTROL_H

#include <ros/ros.h>
#include <iostream>
#include <Eigen/Eigen>
#include <signal.h>

#include "printf_utils.h"
#include "rc_input.h"
#include "math_utils.h"
#include "geometry_utils.h"
#include "ros_msg_utils.h"

using namespace std;

class UAVControl
{
    public:
        // 构造函数
        UAVControl(){};
        // 初始化函数
        void init(ros::NodeHandle& nh);
        // 主循环函数
        void mainloop();

        // 基本变量
        int pos_controller;
        bool enable_external_control;
        bool sim_mode;
        bool quick_land;
        float Takeoff_height; // 默认起飞高度
        float Disarm_height;  // 自动上锁高度
        float Land_speed;     // 降落速度
        bool set_landing_des;
        bool check_off;       // offboard进入标志

        // 无人机状态量
        Eigen::Vector3d uav_pos;     // 无人机位置
        Eigen::Vector3d uav_vel;     // 无人机速度
        Eigen::Quaterniond uav_quat; // 无人机四元数
        double uav_yaw;

        // 辅助点
        Eigen::Vector3d Takeoff_position;
        double Takeoff_yaw;
        Eigen::Vector3d Hover_position;
        double Hover_yaw;

        // 目标设定值
        Eigen::Vector3d pos_des;
        Eigen::Vector3d global_pos_des;
        Eigen::Vector3d vel_des;
        Eigen::Vector3d acc_des;
        Eigen::Vector4d u_att;      // 期望姿态角（rad）+期望油门（0-1）
        double yaw_des;
        double yaw_rate_des;

        sunray_msgs::TextInfo text_info;

        ros::Time last_set_hover_pose_time;


    private:
        string node_name;               // 节点名称
        string uav_name{""};            // 无人机名称
        int uav_id;                     // 无人机编号

        sunray_msgs::UAVControlCMD control_cmd;    // 外部控制指令
        sunray_msgs::UAVControlCMD last_control_cmd;    // 外部控制指令

        sunray_msgs::UAVState uav_state;        // 飞控状态
        sunray_msgs::UAVState uav_state_last;        // 飞控状态

        // 地理围栏
        struct geo_fence
        {
            float x_min;
            float x_max;
            float y_min;
            float y_max;
            float z_min;
            float z_max;
        };
        geo_fence uav_geo_fence;

        enum Control_Mode
        {
            INIT = 0,               // 初始模式      
            RC_CONTROL = 1,         // 遥控器控制模式
            CMD_CONTROL = 2,         // 外部指令控制模式
            LAND_CONTROL = 3        // 降落
        };
        Control_Mode control_mode;
        Control_Mode last_control_mode;

        struct Desired_State
        {
            Eigen::Vector3d pos;
            Eigen::Vector3d vel;
            Eigen::Vector3d acc;
            Eigen::Vector3d att;
            double yaw;
            double yaw_rate;
            double thrust;
            Eigen::Quaterniond q;
            Eigen::Vector3d global_pos;
        };
        Desired_State desired_state;


        // PX4中的位置设定值（用于验证控制指令是否正确发送）
        Eigen::Vector3d px4_pos_target;
        // PX4中的速度设定值（用于验证控制指令是否正确发送）
        Eigen::Vector3d px4_vel_target;
        // PX4中的加速度设定值（用于验证控制指令是否正确发送）
        Eigen::Vector3d px4_acc_target;
        // PX4中的姿态设定值（用于验证控制指令是否正确发送）
        Eigen::Vector3d px4_att_target;
        Eigen::Vector3d px4_rates_target;
        // PX4中的推力设定值（用于验证控制指令是否正确发送）
        float px4_thrust_target;

        // 订阅话题
        ros::Subscriber uav_state_sub;
        ros::Subscriber control_cmd_sub;
        ros::Subscriber px4_rc_sub;
        ros::Subscriber uav_setup_sub;

        // 发布话题
        ros::Publisher setpoint_raw_local_pub;
        ros::Publisher setpoint_raw_attitude_pub;
        ros::Publisher setpoint_raw_global_pub;
        ros::Publisher control_mode_pub;

        RC_Input rc_input;

        // 服务
        ros::ServiceClient px4_arming_client;
        ros::ServiceClient px4_set_mode_client;
        ros::ServiceClient px4_reboot_client;
        ros::ServiceClient px4_emergency_client;


        int safety_check();

        // 根据遥控器指令设置期望值
        void get_desired_state_from_rc();
        // 根据外部指令设置期望值
        void get_desired_state_from_cmd();

        void control_cmd_cb(const sunray_msgs::UAVControlCMD::ConstPtr &msg);
        void uav_state_cb(const sunray_msgs::UAVState::ConstPtr &msg);
        void uav_setup_cb(const sunray_msgs::UAVSetup::ConstPtr &msg);
        void px4_rc_cb(const mavros_msgs::RCIn::ConstPtr &msg);

        void send_local_pos_setpoint(const Eigen::Vector3d &pos_sp, double yaw_sp, bool enable_rate=false);
        void send_local_vel_setpoint(const Eigen::Vector3d &vel_sp, float yaw_sp, bool enable_rate=false);
        void send_attitude_setpoint(const Eigen::Vector3d &att_sp, double thrust_sp);
        void send_global_pos_setpoint(const Eigen::Vector3d &global_pos_sp, float yaw_sp);
        void send_vel_xy_pos_z_setpoint(const Eigen::Vector3d &pos_sp, const Eigen::Vector3d &vel_sp, float yaw_sp, bool enable_rate=false);
        void send_pos_vel_xyz_setpoint(const Eigen::Vector3d &pos_sp, const Eigen::Vector3d &vel_sp, float yaw_sp, bool enable_rate=false);

        int  check_failsafe();

        void set_px4_mode_func(string mode);
        void rotation_yaw(double yaw_angle, float body_frame[2], float enu_frame[2]);
        void printf_param();
        void set_hover_pose_with_odom();
        void set_hover_pose_with_rc();
        void arm_disarm_func(bool on_or_off);
        void enable_emergency_func();
        void reboot_PX4();


};
#endif