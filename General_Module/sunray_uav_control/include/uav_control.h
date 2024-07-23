#ifndef UAV_CONTROL_H
#define UAV_CONTROL_H

#include <ros/ros.h>
#include <iostream>
#include <Eigen/Eigen>

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
        // 是否仿真模式
        bool sim_mode;
        // 是否快速降落
        bool quick_land;
        // 默认起飞高度
        float Takeoff_height; 
        // 默认上锁高度
        float Disarm_height;  
        // 降落模式中的降落速度
        float Land_speed;    
        // 是否设置降落期望点
        bool set_landing_des;
        // offboard进入标志
        bool check_off;       

        // 无人机状态量
        Eigen::Vector3d uav_pos;     // 无人机位置
        Eigen::Vector3d uav_vel;     // 无人机速度
        Eigen::Quaterniond uav_quat; // 无人机四元数
        double uav_yaw;              // 无人机偏航角

        // 无人机起飞位置
        Eigen::Vector3d Takeoff_position;
        // 无人机偏航角
        double Takeoff_yaw;
        // 无人机当前悬停点（Hover模式）
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

        // 文字信息
        sunray_msgs::TextInfo text_info;

        // 
        ros::Time last_set_hover_pose_time;

    private:
        // 节点名称
        string node_name;      
        // 无人机名称         
        string uav_name{""}; 
        // 无人机编号           
        int uav_id;                     
        // 遥控器输入
        RC_Input rc_input;

        // 外部控制指令
        sunray_msgs::UAVControlCMD control_cmd;    
        sunray_msgs::UAVControlCMD last_control_cmd;    

        // 无人机状态
        sunray_msgs::UAVState uav_state;        
        sunray_msgs::UAVState uav_state_last;        

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

        // 无人机控制模式
        enum Control_Mode
        {
            INIT = 0,               // 初始模式      
            RC_CONTROL = 1,         // 遥控器控制模式
            CMD_CONTROL = 2,         // 外部指令控制模式
            LAND_CONTROL = 3        // 降落
        };
        Control_Mode control_mode;
        Control_Mode last_control_mode;

        // 无人机期望状态
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
        // 服务
        ros::ServiceClient px4_arming_client;
        ros::ServiceClient px4_set_mode_client;
        ros::ServiceClient px4_reboot_client;
        ros::ServiceClient px4_emergency_client;

        // 安全检查
        int safety_check();
        // 根据遥控器指令设置期望值
        void get_desired_state_from_rc();
        // 根据外部指令设置期望值
        void get_desired_state_from_cmd();
        // 回调函数
        void control_cmd_cb(const sunray_msgs::UAVControlCMD::ConstPtr &msg);
        void uav_state_cb(const sunray_msgs::UAVState::ConstPtr &msg);
        void uav_setup_cb(const sunray_msgs::UAVSetup::ConstPtr &msg);
        void px4_rc_cb(const mavros_msgs::RCIn::ConstPtr &msg);
        // 发送底层控制指令函数
        void send_local_pos_setpoint(const Eigen::Vector3d &pos_sp, double yaw_sp, bool enable_rate=false);
        void send_local_vel_setpoint(const Eigen::Vector3d &vel_sp, float yaw_sp, bool enable_rate=false);
        void send_attitude_setpoint(const Eigen::Vector3d &att_sp, double thrust_sp);
        void send_global_pos_setpoint(const Eigen::Vector3d &global_pos_sp, float yaw_sp);
        void send_vel_xy_pos_z_setpoint(const Eigen::Vector3d &pos_sp, const Eigen::Vector3d &vel_sp, float yaw_sp, bool enable_rate=false);
        void send_pos_vel_xyz_setpoint(const Eigen::Vector3d &pos_sp, const Eigen::Vector3d &vel_sp, float yaw_sp, bool enable_rate=false);
        void set_px4_mode_func(string mode);
        void arm_disarm_func(bool on_or_off);
        void enable_emergency_func();
        void reboot_PX4();
        // 辅助函数
        void rotation_yaw(double yaw_angle, float body_frame[2], float enu_frame[2]);
        void printf_param();
        void set_hover_pose_with_odom();
        void set_hover_pose_with_rc();
};
#endif