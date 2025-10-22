#ifndef AGENT_SIM_H
#define AGENT_SIM_H

#include "ros_msg_utils.h"

using namespace std;

class AGENT_SIM
{
    public:
        // 构造函数
        AGENT_SIM(){};
        // 初始化函数
        void init(ros::NodeHandle& nh);
        // 主循环函数
        void mainloop();

    private:
        // 节点名称
        string node_name;   
        // 智能体名称 = 智能体类型+ID号
        string agent_name;
        // 智能体的固定高度 - 通过参数配置
        float agent_height;

        // 订阅话题
        ros::Subscriber agent_cmd_sub;
        // 发布话题
        ros::Publisher mocap_pos_pub;
        // 定时器
        ros::Timer fake_odom_timer;

        double dt = 0.01;

        // 位置控制模式枚举
        enum POS_TYPE
        {
            IDLE,
            XYZ_POS,
            XYZ_VEL,
            XY_VEL_Z_POS,
            XYZ_POS_VEL
        };

        struct AGENT_INFO
        {
            // 智能体编号 - 通过参数配置
            int agent_id;  

            // 模拟位姿(中间量)
            Eigen::Vector3d fake_pos, fake_vel, fake_acc, fake_thrust, fake_euler;
            geometry_msgs::Quaternion fake_quat;
            Eigen::Quaterniond fake_quat2;   
            Eigen::Matrix3d fake_Rb;
            bool get_fake_odom;
            double agent_yaw;
            // 模拟位姿（最终要发布的量）
            geometry_msgs::PoseStamped fake_mocap;
            
            // 是否收到位置控制指令
            bool get_local_cmd;
            // 原始的位置控制指令
            mavros_msgs::PositionTarget local_cmd;
            // 位置控制模式
            POS_TYPE pos_control_type;
            // 位置控制的期望值
            Eigen::Vector3d pos_sp, vel_sp, acc_sp, euler_sp;
            double throttle_sp;
            // 模拟控制器中间量
            Eigen::Vector3d thrust_body_sp, thrust_enu_sp;
            geometry_msgs::Quaternion quat_sp;
            Eigen::Quaterniond quat2_sp;   
            Eigen::Matrix3d Rb_sp;

            // 控制参数
            float quad_mass;
            float gravity;
            float hover_per;
            float tilt_angle_max;
            float k_pos, k_vel;  
        };
        AGENT_INFO agent;

        void agent_cmd_cb(const mavros_msgs::PositionTarget::ConstPtr& msg);
        geometry_msgs::Quaternion ros_quaternion_from_rpy(double roll, double pitch, double yaw);
        void update_agent_pos(const ros::TimerEvent &e);
        void fake_pos_control();
};
#endif