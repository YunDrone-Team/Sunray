#ifndef UGV_CONTROL_H
#define UGV_CONTROL_H

#include <ros/ros.h>
#include <iostream>
#include <Eigen/Eigen>
#include <vector>

#include "printf_utils.h"
#include "ros_msg_utils.h"


#include "Astar.h"
#include "map_generator.h"

using namespace std;

#define TRA_WINDOW 50
#define ODOM_TIMEOUT 0.35

class UGV_CONTROL
{
    public:
        // 构造函数
        UGV_CONTROL() {};
        // 初始化函数
        void init(ros::NodeHandle &nh);
        // 主循环函数
        void mainloop();

    private:
        // 节点名称
        string node_name;    
        int ugv_id;                                    // 编号 - 通过参数配置
        int pose_source;                               // 位置来源（1：代表动捕、2代表gazebo odom） - 通过参数配置
        string ugv_name;                               // 名称
        string topic_prefix;                           // 话题前缀
        string vel_topic;                               // 速度话题
        string odom_topic;
        bool flag_printf;                              // 是否打印 - 通过参数配置
        bool goal_set;                                 // 是否有规划目标点
        bool enable_rviz;
        bool enable_astar;
        float resolution;                              // 地图分辨率
        float inflate;                                 // 地图膨胀系数
        double desired_yaw{0.0};                       // 当前期望偏航角（来自外部控制指令赋值）
        sunray_msgs::UGVState ugv_state;               // 当前状态
        sunray_msgs::UGVState ugv_state_last;          // 上一时刻状态
        sunray_msgs::UGVControlCMD current_ugv_cmd;    // 当前控制指令
        ros::Time get_odom_time{0};                    // 获得上一帧定位数据的时间（用于检查定位数据获取是否超时）
        ros::Time get_battery_time{0};
        geometry_msgs::Point desired_position;         // 当前期望位置+偏航角（来自外部控制指令赋值）
        geometry_msgs::Twist desired_vel;              // 当前期望速度
        geometry_msgs::PoseStamped planner_goal;       // 规划目标点
        nav_msgs::Path planner_path;

        // 以下为辅助变量
        std_msgs::ColorRGBA led_color;
        std_msgs::String text_info;
        // 轨迹容器,用于rviz显示
        vector<geometry_msgs::PoseStamped> pos_vector;  

        Astar astar;                    // A*算法
        MapGenerator map_gen;           // 地图生成器
        std::vector<GridLocation> astar_path; // A*路径

        struct control_param // 控制参数 - 通过参数配置
        {
            float Kp_xy;
            float Kp_yaw;
            float max_vel_xy;
            float max_vel_yaw;
        };
        control_param ugv_control_param;

        struct geo_fence // 地理围栏 - 通过参数配置
        {
            float max_x;
            float min_x;
            float max_y;
            float min_y;
        };
        geo_fence ugv_geo_fence;

        // 订阅话题
        ros::Subscriber mocap_pos_sub;
        ros::Subscriber mocap_vel_sub;
        ros::Subscriber gazebo_odom_sub;
        ros::Subscriber viobot_odom_sub;
        ros::Subscriber ugv_cmd_sub;
        ros::Subscriber battery_sub;
        ros::Subscriber planner_goal_sub;

        // 发布话题
        ros::Publisher ugv_cmd_vel_pub;
        ros::Publisher ugv_state_pub;
        ros::Publisher ugv_mesh_pub;
        ros::Publisher ugv_trajectory_pub;
        ros::Publisher text_info_pub;
        ros::Publisher goal_point_pub;
        ros::Publisher vel_rviz_pub;
        ros::Publisher astar_path_pub;

        // 定时器
        ros::Timer timer_state_pub;
        ros::Timer timer_rivz;
        ros::Timer timer_debug;
        ros::Timer timer_rivz2;
        ros::Timer timer_update_map;
        ros::Timer timer_update_astar;

        void mocap_pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);
        void mocap_vel_cb(const geometry_msgs::TwistStamped::ConstPtr &msg);
        void odom_cb(const nav_msgs::Odometry::ConstPtr &msg);
        void ugv_cmd_cb(const sunray_msgs::UGVControlCMD::ConstPtr &msg);
        void goal_point_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);
        void battery_cb(const std_msgs::Float32::ConstPtr &msg);
        void timercb_state(const ros::TimerEvent &e);
        void timercb_rviz(const ros::TimerEvent &e);
        void timercb_debug(const ros::TimerEvent &e);
        void timercb_update_astar(const ros::TimerEvent &e);
        void printf_param();
        bool check_geo_fence();
        geometry_msgs::Twist enu_to_body(double vel_x, double vel_y);
        void pos_control(double x_ref, double y_ref, double yaw_ref);
        void pos_control_diff(double x_ref, double y_ref, double yaw_ref);
        void path_control();
        float constrain_function(float data, float Max, float Min);
        Eigen::Vector3d quaternion_to_euler(const Eigen::Quaterniond &q);
        double get_yaw_error(double yaw_ref, double yaw_now);

        void rotation_yaw(double yaw_angle, float body_frame[2], float enu_frame[2]);
        void setup_rviz_color();
};
#endif