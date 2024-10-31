/*
无人机搜索策略头文件
分为横向搜索和纵向搜索
*/
#ifndef UAV_SEARCH_STRATEGY_H
#define UAV_SEARCH_STRATEGY_H

#include <ros/ros.h>
#include <ros/console.h>
#include <Eigen/Eigen>
#include <sstream>
#include <iostream>
#include <cmath>
#include <boost/thread.hpp> 
#include <numeric>
#include <string>

#include "ros_msg_utils.h"
#include <std_msgs/Int16.h>
#include <sunray_msgs/detection.h>
#include <sunray_msgs/targetPos.h>
#include <sunray_msgs/front_detect.h>

#include "printf_utils.h"
#include <list>
#include <unordered_map>
#include <sensor_msgs/Range.h>
#include <tf/tf.h>

#include "uav_pid.h"

#include <map>

#define MOCAP_TIMEOUT 0.35                 
#define GAZEBO_TIMEOUT 0.1 
#define VINS_TIMEOUT 0.35


class uav_search_strategy
{
private:
    std::string uav_name{""};            // 无人机名称
    int uav_id;                     // 无人机编号
    float square_width;               // 飞行区域的宽
    float square_height;              // 飞行区域的高
    float step;                       // 横向栅格步长
    float height_step;                 // 纵向栅格步长
    float back_step;                    // 后退的步长
    float Takeoff_height;              //起飞高度
    float State1_height;                // 第一阶段高度---> 横向搜索高度
    float State2_height;                // 第二阶段的高度------> 纵向搜索高度
    int uav_group;
    bool land = 0;
    int external_source;            // 外部定位数据来源
    std::string node_name;               // 节点名称
    int uav_state;

    std::string vision_source{""};      // 无人机定位来源 VO or VIO 

public:
    // 【发布者】 
    ros::Publisher setup_pub_;
    ros::Publisher cmd_pub_;
    ros::Publisher stage_pub_;
    ros::Publisher yolov8_detect_pose_pub_; // yolov8 识别到的靶标坐标话题发布
    ros::Publisher uav_search_state_pub_; // 无人机搜索状态，1为搜索完成（切为手动），0为还在搜索，

    ros::Publisher uav_true_pose_pub_; // 无人机真实位置发布（在自己坐标系）
    ros::Publisher uav_init_pose_pub_; // 无人机开始搜索时的坐标

    

    // 【订阅者】
    ros::Subscriber boxes_sub_;
    ros::Subscriber vins_sub_;
    ros::Subscriber gazebo_sub_;
    ros::Subscriber mocap_pos_sub_;
    ros::Subscriber mocap_vel_sub_;
    ros::Subscriber send_sub_;
    ros::Subscriber lidar_height_sub_;
    ros::Subscriber px4_velocity_sub; // 飞机的当前速度
    ros::Subscriber yolov8_detect_id_sub_; // yolov8 识别到的靶标类型话题订阅
    ros::Subscriber uav_search_state_all_sub_; // 其他无人机搜索状态，1为搜索完成，0为还在搜索
    ros::Subscriber uav_rc_in_sub; // 无人机遥控输入
    ros::Subscriber uav_targetPose_sub; // 无人机target_pose订阅，对应数字1，2，3的坐标
    ros::Subscriber uav_takeoff_sub_; // 无人机起飞指令订阅



    ros::Timer pub_trajectory_;

    
    sunray_msgs::UAVSetup setup_;
    sunray_msgs::UAVControlCMD cmd_;
    sunray_msgs::front_detect yolov8_detect_pose_; // yolov8 识别到的靶标坐标，其中列表索引号对应数字类别，其中的坐标对应数字的坐标
    
    std_msgs::Int16 stage;
    std_msgs::Int16 uav_search_state; // 无人机搜索状态，1为搜索完成，0为还在搜索



    Eigen::Vector3d input;
    Eigen::Vector3d pos_;
    Eigen::Vector3d vel_;
    Eigen::Vector3d vins_pose; // vins的世界坐标
    Eigen::Vector3d vins_rpy; // vins的旋转角
    Eigen::Vector3d vel_from_autopilot; // 飞机当前速度
    Eigen::Vector3d init_pose; // uav的开始搜索时的初始世界坐标

    std::vector<Eigen::Vector3d> boxes_poses_; // 接收到的boxes坐标容器
    std::vector<Eigen::Vector3d> edges_poses_; //边界点坐标，提前根据航行区域设置好，该变量可以根据launch文件设置 w,h 
    std::vector<Eigen::Vector3d> target_poses_; // uav需要飞行的目标点容器，uav只需要按照该变量飞航迹点即可
    std::vector<Eigen::Vector3d> box_pose_buff; // 接收到满足条件的buff
    

    int buff_len;
    std::vector<std::vector<Eigen::Vector3d>> filter_pose_buff; // 滤波用的buff
    std::vector<std::vector<Eigen::Vector3d>> pose_buff_mean;
    

    std::list<int> window; // 存储最近接收到的数据
    std::unordered_map<int, int> counts; // 计数每个数字出现的次数
    std::list<Eigen::Vector3d> positions; // 存储对应的坐标

    std::string search_strategy; // 搜索策略：h_search or v_search
    
    
    
    int mode_ = 0;
    int yolo_detect_timer = 0;                     // yolov8 识别靶标时的延时器
    int send = 0; // 接收握手信号
    int state = 0;
    int box_sort_flag = 0; // box排序标志位
    int timer_cnt = 0; // 计时器
    int16_t yolov8_detect_id = 0;
    int uav_search_state_all = 0;
    int target_num = 0;
    int takeoff = 0; // 起飞指令

    bool timer = 0; // 1s定时器标志位
    bool velocity_ctrl_mode; // 控制模式
    bool enable_hight_lidar; // 使用高度雷达
    bool uav_ready_go = false; // uav准备搜索标志位
    // 是否启用外部姿态估计
    bool use_external_attitude;

    
    velocity_ctrl vel_ctrl; // 速度控制结构体
    double last_time;// 上一次时间
    float lidar_height; // 高度测距
    float height_up;// 飞机飞行高度的上限
    float service_height; // 飞机前往搜索区域的纵向步长
    float total_width; //整个场地的宽度

    // 电子围栏
    float x_min, x_max, y_min, y_max, z_min, z_max;
    
    // 无人机纵向搜索轮数
    int v_search_rounds = 0;

    /* ************************挑战杯专用接口***********************************/ 
    
    bool NeedBack = false; // 是否需要返程（如果转完yaw角之后向前推进，到达终点后如果需要返程，那么就置为true，否则置为flase，注：如果需要返程，则飞机会向前推进square_width/2的距离，然后再返回） 

    bool IsFindTargetLand = false; // 是否需要找到靶标后立即降落（false：飞机会搜索完成之后才降落；true：飞机找到之后立即降落）

    int YawDirection = -1; // 旋转yaw角的方向，-1：向右旋转；1：向左旋转

    int rc_cmd = 0; // 遥控器指令 1：解锁，起飞 2：搜索

    /* ************************挑战杯专用接口***********************************/ 


    ros::Time get_mocap_stamp{0};
    ros::Time get_gazebo_stamp{0};
    ros::Time get_vins_stamp{0};

    boost::thread run_thread_;

    uav_search_strategy(ros::NodeHandle &nh_);

    void boxes_cb(const sunray_msgs::front_detect::ConstPtr & msg);
    void send_cb(const std_msgs::Int16::ConstPtr & msg);
    void vins_cb(const nav_msgs::Odometry::ConstPtr& msg);
    void gazebo_cb(const nav_msgs::Odometry::ConstPtr& msg);
    void mocap_pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void mocap_vel_cb(const geometry_msgs::TwistStamped::ConstPtr &msg);
    void local_vel_ned_cb(const geometry_msgs::TwistStamped::ConstPtr &msg);
    void lidar_height_cb(const sensor_msgs::Range::ConstPtr& msg);
    void detect_id_cb(const std_msgs::Int16::ConstPtr& msg);
    void pub_trajectory_cb(const ros::TimerEvent& e);
    void uav_search_state_all_cb(const std_msgs::Int16::ConstPtr & msg);
    void pc4_rc_in_cb(const mavros_msgs::RCIn::ConstPtr & msg);
    void uav_targetPose_cb(const sunray_msgs::targetPos::ConstPtr & msg);
    void takeoff_cb(const std_msgs::Int16::ConstPtr & msg);

    Eigen::Vector3d calculateAveragePosition(int id);
    bool is_to_target_pose(Eigen::Vector3d current, Eigen::Vector3d target, double distance_th = 0.2 , bool choice = 0);
    bool is_out_fence(Eigen::Vector3d current);
    bool check_timeout();

    void h_search();
    void v_search();
    void v_center_search();
    void v_center_search_new();

    void pose_buff_clear();

    void ready_go(); // 搜索开始前的服务函数

    void run();
    void start();


    void print_info();



};

#endif /* UAV_SEARCH_STRATEGY_H */