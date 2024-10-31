#ifndef VISION_POSE_H
#define VISION_POSE_H

// 头文件
#include <ros/ros.h>

#include "math_utils.h"
#include "printf_utils.h"
#include "ros_msg_utils.h"
#include "utils.h"

using namespace std;

// 宏定义
#define TRA_WINDOW 40                // 发布轨迹长度
#define MOCAP_TIMEOUT 0.35                 
#define GAZEBO_TIMEOUT 0.3                 
#define T265_TIMEOUT 0.3
#define UWB_TIMEOUT 0.1
#define GPS_TIMEOUT 1.0
#define VINS_TIMEOUT 0.35
#define VIOBOT_TIMEOUT 0.35

class VISION_POSE
{
    public:
        // 构造函数
        VISION_POSE(){};
        // 初始化函数
        void init(ros::NodeHandle& nh);
        // 打印debug信息函数
        void printf_debug_info();

    private:
        // 节点名称
        string node_name; 
        // 无人机名称              
        string uav_name{""};          
        // 无人机编号  
        int uav_id;                     
        // 外部定位数据来源
        int external_source;         
        // 激光定位数据
        double range_hight;   
        // vision_pose消息
        geometry_msgs::PoseStamped vision_pose;
        // 无人机状态信息
        sunray_msgs::UAVState uav_state;
        // 无人机轨迹容器,用于rviz显示
        std::vector<geometry_msgs::PoseStamped> uav_pos_vector;    

        MovingAverageFilter* hight_filter;
        struct uav_pose
        {
            ros::Time get_time{0};
            Eigen::Vector3d pos;          // 位置
            Eigen::Vector3d vel;          // 速度
            Eigen::Vector3d att;          // 姿态 - 欧拉角
            geometry_msgs::Quaternion q;  // 姿态 - 四元数
        };
        // 从飞控中获取的位姿
        uav_pose uav_pose_px4;
        // 从外部获取的位姿
        uav_pose uav_pose_external;

        // 外部定位数据是否异常
        bool vision_pose_error{false};  
        bool enable_rangeSensor{false};
        // 误差 - vision_pose与PX4回传的状态量之间的误差
        Eigen::Vector3d pos_error;
        Eigen::Vector3d vel_error;
        Eigen::Vector3d att_error;
        double yaw_error;

        // 订阅话题
        ros::Subscriber mocap_pos_sub;              // 通过vrpn从动捕系统获取定位信息
        ros::Subscriber mocap_vel_sub;              // 通过vrpn从动捕系统获取定位信息
        ros::Subscriber sim_pose_sub;               // 通过gazebo仿真插件获取真值
        ros::Subscriber viobot_sub;                 // 通过VIOBOT获取定位信息
        ros::Subscriber gazebo_sub;                 // gazebo仿真
        ros::Subscriber t265_sub;                   // t265
        ros::Subscriber vins_sub;                   // 通过VINS算法获取定位信息
        ros::Subscriber range_sub;                   // 激光定高
        ros::Subscriber px4_position_sub;           // 从PX4订阅的本地位置信息
        ros::Subscriber px4_velocity_sub;
        ros::Subscriber px4_attitude_sub;           // 从PX4订阅的姿态信息
        ros::Subscriber px4_state_sub;
        ros::Subscriber px4_battery_sub;
        // 发布话题
        ros::Publisher vision_pose_pub;             // 将vision_pose消息发布至PX4
        ros::Publisher uav_state_pub;               // 
        ros::Publisher uav_odom_pub;               // 
        ros::Publisher uav_trajectory_pub;  
        ros::Publisher uav_mesh_pub;  
        // 定时器
        ros::Timer timer_px4_vision_pub;            // 定时发布vision_pose消息
        ros::Timer timer_rviz_pub;            // 定时发布rviz显示消息

        // 回调函数
        void px4_state_cb(const mavros_msgs::State::ConstPtr& msg);
        void px4_battery_cb(const sensor_msgs::BatteryState::ConstPtr& msg);
        void mocap_pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);
        void mocap_vel_cb(const geometry_msgs::TwistStamped::ConstPtr &msg);
        void gazebo_cb(const nav_msgs::Odometry::ConstPtr &msg);
        void viobot_cb(const nav_msgs::Odometry::ConstPtr &msg);
        void vins_cb(const nav_msgs::Odometry::ConstPtr &msg);
        void t265_cb(const nav_msgs::Odometry::ConstPtr &msg);
        void range_cb(const sensor_msgs::Range::ConstPtr &msg);
        void local_position_ned_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);
        void local_vel_ned_cb(const geometry_msgs::TwistStamped::ConstPtr &msg);
        void attitude_cb(const sensor_msgs::Imu::ConstPtr &msg);
        void timercb_pub_vision_pose(const ros::TimerEvent &e);
        void timercb_rviz(const ros::TimerEvent &e);
        bool check_timeout();
};

void VISION_POSE::init(ros::NodeHandle& nh)
{
    node_name = ros::this_node::getName();
    // 【参数】无人机编号
    nh.param<int>("uav_id", uav_id, 1);
    // 【参数】无人机名称
    nh.param<string>("uav_name", uav_name, "uav");
    uav_name = uav_name + std::to_string(uav_id);
    // 【参数】外部定位数据来源
    nh.param<int>("external_source", external_source, sunray_msgs::ExternalOdom::GAZEBO);
    bool flag_printf;
    bool enable_rviz;
    nh.param<bool>("flag_printf", flag_printf, false);
    nh.param<bool>("enable_rviz", enable_rviz, false);
    nh.param<bool>("enable_rangeSensor", enable_rangeSensor, false);
    
    string topic_prefix = "/" + uav_name;
    // 【订阅】无人机模式 - 飞控 -> 本节点
    px4_state_sub = nh.subscribe<mavros_msgs::State>(topic_prefix + "/mavros/state", 1,  &VISION_POSE::px4_state_cb, this);
    // 【订阅】无人机电池状态 - 飞控 -> 本节点
    px4_battery_sub = nh.subscribe<sensor_msgs::BatteryState>(topic_prefix + "/mavros/battery", 1,  &VISION_POSE::px4_battery_cb, this);
    if(flag_printf)
    {
        // 【订阅】无人机当前位置（对比用） - 飞控 -> 本节点
        px4_position_sub = nh.subscribe<geometry_msgs::PoseStamped>(topic_prefix + "/mavros/local_position/pose", 1, &VISION_POSE::local_position_ned_cb, this);
        // 【订阅】无人机当前速度 坐标系:ENU系 (PX4 -> 本节点)
        px4_velocity_sub = nh.subscribe<geometry_msgs::TwistStamped>(topic_prefix + "/mavros/local_position/velocity_local", 1, &VISION_POSE::local_vel_ned_cb, this);
        // 【订阅】无人机当前欧拉角（对比用） - 飞控 -> 本节点
        px4_attitude_sub = nh.subscribe<sensor_msgs::Imu>(topic_prefix + "/mavros/imu/data", 1, &VISION_POSE::attitude_cb, this);
    }
    
    // 根据设定的定位来源订阅不同的定位数据
    if (external_source == sunray_msgs::ExternalOdom::MOCAP)
    {
        // 【订阅】mocap pose
        mocap_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node_" + std::to_string(uav_id) + topic_prefix + "/pose", 1, &VISION_POSE::mocap_pos_cb, this);
        // 【订阅】mocap Twist
        mocap_vel_sub = nh.subscribe<geometry_msgs::TwistStamped>("/vrpn_client_node_" + std::to_string(uav_id) + topic_prefix + "/twist", 1, &VISION_POSE::mocap_vel_cb, this);
    }
    else if (external_source == sunray_msgs::ExternalOdom::VINS)
    {
        // 【订阅】VINS估计位置
        vins_sub = nh.subscribe<nav_msgs::Odometry>("/vins_estimator/imu_propagate", 1, &VISION_POSE::vins_cb, this);
    }
    else if (external_source == sunray_msgs::ExternalOdom::VIOBOT)
    {
        // 【订阅】VIOBOT估计位置
        viobot_sub = nh.subscribe<nav_msgs::Odometry>("/pr_loop/odometry_rect", 1, &VISION_POSE::viobot_cb, this);
    }
    else if (external_source == sunray_msgs::ExternalOdom::GAZEBO)
    {
        // 【订阅】gazebo仿真真值
        gazebo_sub = nh.subscribe<nav_msgs::Odometry>(topic_prefix + "/sunray/gazebo_pose", 1, &VISION_POSE::gazebo_cb, this);
    }
    else if (external_source == sunray_msgs::ExternalOdom::T265)
    {
        // 【订阅】T265
        t265_sub = nh.subscribe<nav_msgs::Odometry>("/camera/odom/sample", 1, &VISION_POSE::t265_cb, this);
    }
    else
    {
        cout << RED << node_name << ": wrong external_source param, no external location information input!" << TAIL << endl;
    }
    // 【发布】无人机位置和偏航角，传输至PX4_EKF2模块用于位置姿态估计 坐标系:ENU系 - 本节点 -> 飞控
    vision_pose_pub = nh.advertise<geometry_msgs::PoseStamped>(topic_prefix + "/mavros/vision_pose/pose", 1);
    // 【发布】无人机综合状态信息 - 本节点 -> 其他控制&任务节点
    uav_state_pub = nh.advertise<sunray_msgs::UAVState>(topic_prefix + "/sunray/uav_state", 1);

    // 【定时器】使用外部定位设备时，需要定时发送vision信息至飞控,并保证一定频率 - 本节点 -> 飞控
    timer_px4_vision_pub = nh.createTimer(ros::Duration(0.02), &VISION_POSE::timercb_pub_vision_pose, this);

    if(enable_rviz)
    {
        // 【发布】无人机里程计,主要用于RVIZ显示
        uav_odom_pub = nh.advertise<nav_msgs::Odometry>(topic_prefix + "/sunray/uav_odom", 1);
        // 【发布】无人机运动轨迹,主要用于RVIZ显示
        uav_trajectory_pub = nh.advertise<nav_msgs::Path>(topic_prefix + "/sunray/uav_trajectory", 1);
        // 【发布】无人机位置(带图标),用于RVIZ显示
        uav_mesh_pub = nh.advertise<visualization_msgs::Marker>(topic_prefix + "/sunray/uav_mesh", 1);
        // 【定时器】定时发布 rviz显示,保证1Hz以上
        timer_rviz_pub = nh.createTimer(ros::Duration(0.1), &VISION_POSE::timercb_rviz, this);
    }

    if(enable_rangeSensor)
    {
        range_sub = nh.subscribe<sensor_msgs::Range>(topic_prefix + "/mavros/distance_sensor/hrlv_ez4_pub", 1, &VISION_POSE::range_cb, this);
        hight_filter = new MovingAverageFilter(5);
        range_hight = 0;
    }

    // 初始化
    uav_state.header.frame_id = uav_name;
    uav_state.header.stamp = ros::Time::now();
    uav_state.uav_id = uav_id;
    uav_state.connected = false;
    uav_state.armed = false;
    uav_state.mode = "";
    uav_state.location_source = external_source;
    uav_state.odom_valid = false;
    uav_state.position = {0};
    uav_state.velocity = {0};
    uav_state.attitude = {0};
    uav_state.attitude_q.x = 0;
    uav_state.attitude_q.y = 0;
    uav_state.attitude_q.z = 0;
    uav_state.attitude_q.w = 1;
    uav_state.pos_setpoint = {0};
    uav_state.vel_setpoint = {0};
    uav_state.att_setpoint = {0};
    uav_state.attitude_rate = {0};
    uav_state.battery_state = 0;
    uav_state.battery_percetage = 0;

    uav_pose_external.pos << 0, 0, 0;
    uav_pose_external.vel << 0, 0, 0;
    uav_pose_external.att << 0, 0, 0;

    uav_pose_px4.pos << 999, 999, 999;
    uav_pose_px4.vel << 999, 999, 999;
    uav_pose_px4.att << 999, 999, 999;

    

    cout << GREEN << node_name << " init! " << TAIL << endl;
}

void VISION_POSE::timercb_rviz(const ros::TimerEvent &e)
{
    if(!uav_state.odom_valid)
    {
        return;
    }
    nav_msgs::Odometry uav_odom;
    // 发布无人机当前odometry(有些节点需要Odometry这个数据类型)
    uav_odom.header.stamp = ros::Time::now();
    uav_odom.header.frame_id = "world";
    uav_odom.child_frame_id = "base_link";
    uav_odom.pose.pose.position.x = uav_state.position[0];
    uav_odom.pose.pose.position.y = uav_state.position[1];
    uav_odom.pose.pose.position.z = uav_state.position[2];
    if(enable_rangeSensor)
    {
        uav_odom.pose.pose.position.z = range_hight;
    }
    // 导航算法规定 高度不能小于0
    if (uav_odom.pose.pose.position.z <= 0)
    {
        uav_odom.pose.pose.position.z = 0.01;
    }
    uav_odom.pose.pose.orientation = uav_state.attitude_q;
    uav_odom.twist.twist.linear.x = uav_state.velocity[0];
    uav_odom.twist.twist.linear.y = uav_state.velocity[1];
    uav_odom.twist.twist.linear.z = uav_state.velocity[2];
    uav_odom_pub.publish(uav_odom);


    // 发布无人机运动轨迹，用于rviz显示
    geometry_msgs::PoseStamped uav_pos;
    uav_pos.header.stamp = ros::Time::now();
    uav_pos.header.frame_id = "world";
    uav_pos.pose.position.x = uav_state.position[0];
    uav_pos.pose.position.y = uav_state.position[1];
    uav_pos.pose.position.z = uav_state.position[2];
    uav_pos.pose.orientation = uav_state.attitude_q;
    uav_pos_vector.insert(uav_pos_vector.begin(), uav_pos);
    if (uav_pos_vector.size() > TRA_WINDOW)
    {
        uav_pos_vector.pop_back();
    }
    nav_msgs::Path uav_trajectory;
    uav_trajectory.header.stamp = ros::Time::now();
    uav_trajectory.header.frame_id = "world";
    uav_trajectory.poses = uav_pos_vector;
    uav_trajectory_pub.publish(uav_trajectory);

    // 发布无人机marker
    visualization_msgs::Marker meshROS;
    meshROS.header.frame_id = "world";
    meshROS.header.stamp = ros::Time::now();
    meshROS.ns = "mesh";
    meshROS.id = 0;
    meshROS.type = visualization_msgs::Marker::MESH_RESOURCE;
    meshROS.action = visualization_msgs::Marker::ADD;
    meshROS.pose.position.x = uav_state.position[0];
    meshROS.pose.position.y = uav_state.position[1];
    meshROS.pose.position.z = uav_state.position[2];
    meshROS.pose.orientation.w = uav_state.attitude_q.w;
    meshROS.pose.orientation.x = uav_state.attitude_q.x;
    meshROS.pose.orientation.y = uav_state.attitude_q.y;
    meshROS.pose.orientation.z = uav_state.attitude_q.z;
    meshROS.scale.x = 1.0;
    meshROS.scale.y = 1.0;
    meshROS.scale.z = 1.0;
    meshROS.color.a = 1.0;
    meshROS.color.r = 1.0;
    meshROS.color.g = 0.0;
    meshROS.color.b = 0.0;
    meshROS.mesh_use_embedded_materials = false;
    meshROS.mesh_resource = std::string("package://sunray_uav_control/meshes/uav.mesh");
    uav_mesh_pub.publish(meshROS);

    // 发布TF用于RVIZ显示（用于lidar）
    static tf2_ros::TransformBroadcaster broadcaster;
    geometry_msgs::TransformStamped tfs;
    //  |----头设置
    tfs.header.frame_id = "world";       //相对于世界坐标系
    tfs.header.stamp = ros::Time::now(); //时间戳
    //  |----坐标系 ID
    tfs.child_frame_id = uav_name + "/lidar_link"; //子坐标系，无人机的坐标系
    // tfs.child_frame_id = "/lidar_link"; //子坐标系，无人机的坐标系
    //  |----坐标系相对信息设置  偏移量  无人机相对于世界坐标系的坐标
    tfs.transform.translation.x = uav_state.position[0];
    tfs.transform.translation.y = uav_state.position[1];
    tfs.transform.translation.z = uav_state.position[2];
    //  |--------- 四元数设置
    tfs.transform.rotation = uav_state.attitude_q;
    //  |--------- 广播器发布数据
    broadcaster.sendTransform(tfs);

    //q_orig  是原姿态转换的tf的四元数
    //q_rot   旋转四元数
    //q_new   旋转后的姿态四元数
    tf2::Quaternion q_orig, q_rot, q_new;

    // commanded_pose.pose.orientation  这个比如说 是 订阅的别的节点的topic 是一个  姿态的 msg 四元数
    //通过tf2::convert()  转换成 tf 的四元数
    tf2::convert(tfs.transform.rotation , q_orig);

    // 设置 绕 x 轴 旋转180度
    double r=-1.57, p=0, y=-1.57;  
    q_rot.setRPY(r, p, y);//求得 tf 的旋转四元数

    q_new = q_orig*q_rot;  // 通过 姿态的四元数 乘以旋转的四元数 即为 旋转 后的  四元数
    q_new.normalize(); // 归一化

    //  将 旋转后的 tf 四元数 转换 为 msg 四元数
    tf2::convert(q_new, tfs.transform.rotation);
    tfs.child_frame_id = uav_name + "/camera_link"; //子坐标系，无人机的坐标系
    //  |--------- 广播器发布数据
    broadcaster.sendTransform(tfs);

}

void VISION_POSE::timercb_pub_vision_pose(const ros::TimerEvent &e)
{
    bool odom_valid;
    odom_valid = check_timeout();

    vision_pose.header.stamp = ros::Time::now();
    vision_pose.pose.position.x = uav_pose_external.pos[0];
    vision_pose.pose.position.y = uav_pose_external.pos[1];
    vision_pose.pose.position.z = uav_pose_external.pos[2];
    vision_pose.pose.orientation = uav_pose_external.q;
    if(enable_rangeSensor)
    {
        vision_pose.pose.position.z = range_hight;
    }
    vision_pose_pub.publish(vision_pose);
    
    uav_state.uav_id = uav_id;
    uav_state.odom_valid = odom_valid;
    uav_state.header.stamp = ros::Time::now();
    uav_state.position[0] = vision_pose.pose.position.x;
    uav_state.position[1] = vision_pose.pose.position.y;
    uav_state.position[2] = vision_pose.pose.position.z;
    uav_state.velocity[0] = uav_pose_external.vel[0];
    uav_state.velocity[1] = uav_pose_external.vel[1];
    uav_state.velocity[2] = uav_pose_external.vel[2];
    uav_state.attitude[0] = uav_pose_external.att[0];
    uav_state.attitude[1] = uav_pose_external.att[1];
    uav_state.attitude[2] = uav_pose_external.att[2];
    uav_state.attitude_q = uav_pose_external.q;
    uav_state_pub.publish(uav_state);

    // 位置误差
    pos_error = uav_pose_external.pos - uav_pose_px4.pos;
    // 速度误差
    vel_error = uav_pose_external.vel - uav_pose_px4.vel;
    // 姿态角误差
    att_error = uav_pose_external.att - uav_pose_px4.att;
    // 偏航角误差
    yaw_error = uav_pose_external.att[2] - uav_pose_px4.att[2];
}

void VISION_POSE::printf_debug_info()
{
    cout << BLUE << ">>>>>>>>>>>>>>>>>>>>>> UAV [" << uav_id << "] VISION_POSE <<<<<<<<<<<<<<<<<<<<<<" << TAIL << endl;
    cout << BLUE << "uav_name : " << uav_name <<  TAIL << endl;
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

    // 打印外部输入的原始数据
    switch (external_source)
    {
    case sunray_msgs::ExternalOdom::MOCAP:
        cout << GREEN << "External Odom: [ MOCAP ] " << TAIL << endl;
        break;
    case sunray_msgs::ExternalOdom::VIOBOT:
        cout << GREEN << "External Odom: [ VIOBOT ] " << TAIL << endl;
        break;
    case sunray_msgs::ExternalOdom::GAZEBO:
        cout << GREEN << "External Odom: [ GAZEBO ] " << TAIL << endl;
        break;
    case sunray_msgs::ExternalOdom::VINS:
        cout << GREEN << "External Odom: [ VINS ] " << TAIL << endl;
        break;
    }
    // 打印 无人机状态
    if (uav_state.connected == true)
    {
        cout << GREEN << "PX4 Status:  [ Connected ] ";
    }
    else
    {
        cout << RED << "PX4 Status:[ Unconnected ] ";
    }
    //是否上锁
    if (uav_state.armed == true)
    {
        cout << GREEN << "[  Armed   ] ";
    }
    else
    {
        cout << RED << "[ DisArmed ] ";
    }

    cout << "[ " << uav_state.mode << " ] " << TAIL << endl;

    // 打印外部输入的原始数据
    switch (uav_state.location_source)
    {
    case sunray_msgs::ExternalOdom::MOCAP:
        cout << GREEN << "External Odom: [ MOCAP ] ";
        break;
    case sunray_msgs::ExternalOdom::VIOBOT:
        cout << GREEN << "External Odom: [ VIOBOT ] ";
        break;
    case sunray_msgs::ExternalOdom::GAZEBO:
        cout << GREEN << "External Odom: [ GAZEBO ] ";
        break;
    case sunray_msgs::ExternalOdom::VINS:
        cout << GREEN << "External Odom: [ VINS ] ";
        break;
    }

    if (uav_state.odom_valid)
    {
        cout << GREEN << " Odom Status : [ Valid ] " << TAIL << endl;
    }
    else
    {
        cout << RED   << " Odom Status : [ Invalid ] " << TAIL << endl;
    }
    cout << GREEN << "Battery Voltage : " << uav_state.battery_state << " [V] "
         << " Battery Percent : " << uav_state.battery_percetage << " [%] "<< TAIL << endl;


    // 打印发布的Vision Pose数据
    cout << BLUE << "Pose Send to Autopilot [NED]: " << TAIL << endl;
    cout << GREEN << "Pos [X Y Z] : " << uav_pose_external.pos[0] << " [ m ] " << uav_pose_external.pos[1] << " [ m ] " << uav_pose_external.pos[2] << " [ m ] " << TAIL << endl;
    cout << GREEN << "Vel [X Y Z] : " << uav_pose_external.vel[0] << " [m/s] " << uav_pose_external.vel[1] << " [m/s] " << uav_pose_external.vel[2] << " [m/s] " << TAIL << endl;
    cout << GREEN << "Att [R P Y] : " << uav_pose_external.att[0] * 180 / M_PI << " [deg] " << uav_pose_external.att[1] * 180 / M_PI << " [deg] " << uav_pose_external.att[2] * 180 / M_PI << " [deg] " << TAIL << endl;

    // 打印飞控回传的数据
    cout << BLUE << "Pose from Autopilot [NED]: " << TAIL << endl;
    cout << GREEN << "Pos [X Y Z] : " << uav_pose_px4.pos[0] << " [ m ] " << uav_pose_px4.pos[1] << " [ m ] " << uav_pose_px4.pos[2] << " [ m ] " << TAIL << endl;
    cout << GREEN << "Vel [X Y Z] : " << uav_pose_px4.vel[0] << " [m/s] " << uav_pose_px4.vel[1] << " [m/s] " << uav_pose_px4.vel[2] << " [m/s] " << TAIL << endl;
    cout << GREEN << "Att [R P Y] : " << uav_pose_px4.att[0] * 180 / M_PI << " [deg] " << uav_pose_px4.att[1] * 180 / M_PI << " [deg] " << uav_pose_px4.att[2] * 180 / M_PI << " [deg] " << TAIL << endl;

    // 打印计算得到的差值
    cout << BLUE << "Pose Error [NED]: " << TAIL << endl;
    cout << GREEN << "Pos [X Y Z] : " << pos_error[0] << " [ m ] " << pos_error[1] << " [ m ] " << pos_error[2] << " [ m ] " << TAIL << endl;
    cout << GREEN << "Vel [X Y Z] : " << vel_error[0] << " [m/s] " << vel_error[1] << " [m/s] " << vel_error[2] << " [m/s] " << TAIL << endl;
    cout << GREEN << "Att [R P Y] : " << att_error[0] * 180 / M_PI << " [deg] " << att_error[1] * 180 / M_PI << " [deg] " << att_error[2] * 180 / M_PI << " [deg] " << TAIL << endl;

}

bool VISION_POSE::check_timeout()
{
    if (external_source == sunray_msgs::ExternalOdom::MOCAP)
    {
        if((ros::Time::now() - uav_pose_external.get_time).toSec()>MOCAP_TIMEOUT){
            cout << RED << "Odom Timeut: [ MOCAP ] " << TAIL << endl;
            return false;
        }
    }
    else if (external_source == sunray_msgs::ExternalOdom::VINS)
    {
        if((ros::Time::now() - uav_pose_external.get_time).toSec()>VINS_TIMEOUT){
            cout << RED << "Odom Timeut: [ VINS ] " << TAIL << endl;
            return false;
        }
    }
    else if (external_source == sunray_msgs::ExternalOdom::VIOBOT)
    {
        if((ros::Time::now() - uav_pose_external.get_time).toSec()>VIOBOT_TIMEOUT){
            cout << RED << "Odom Timeut: [ VIOBOT ] " << TAIL << endl;
            return false;
        }
    }
    else if (external_source == sunray_msgs::ExternalOdom::GAZEBO)
    {
        if((ros::Time::now() - uav_pose_external.get_time).toSec()>GAZEBO_TIMEOUT){
            cout << RED << "Odom Timeut: [ GAZEBO ] " << TAIL << endl;
            return false;
        }
    }
    return true;
}

void VISION_POSE::mocap_pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    uav_pose_external.get_time = ros::Time::now(); // 记录时间戳，防止超时
    tf2::Quaternion quaternion;
    tf2::fromMsg(msg->pose.orientation, quaternion);
    double roll, pitch, yaw;
    tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
    uav_pose_external.pos = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    uav_pose_external.att = Eigen::Vector3d(roll, pitch, yaw);
    uav_pose_external.q = msg->pose.orientation;
}

void VISION_POSE::mocap_vel_cb(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    uav_pose_external.vel = Eigen::Vector3d(msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z);
}

void VISION_POSE::vins_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
    uav_pose_external.get_time = ros::Time::now(); // 记录时间戳，防止超时
    geometry_msgs::Pose pose = msg->pose.pose;

    // Convert the pose from RFU to FLU coordinate system
    tf::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    // Swap y and z axes to convert from RFU to FLU
    double temp = pose.position.y;
    pose.position.y = -pose.position.x;
    pose.position.x = temp;

    // Update the orientation accordingly
    roll = roll + M_PI/2;
    q.setRPY(pitch, -roll, yaw);
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();

    uav_pose_external.pos = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
    uav_pose_external.q = pose.orientation;
    uav_pose_external.att = Eigen::Vector3d(pitch, -roll, yaw);
}

void VISION_POSE::t265_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
    uav_pose_external.get_time = ros::Time::now(); // 记录时间戳，防止超时
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg->pose.pose.orientation, quat);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    uav_pose_external.pos = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    uav_pose_external.vel = Eigen::Vector3d(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
    uav_pose_external.att = Eigen::Vector3d(roll, pitch, yaw);
    uav_pose_external.q = msg->pose.pose.orientation;
}

void VISION_POSE::gazebo_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
    uav_pose_external.get_time = ros::Time::now(); // 记录时间戳，防止超时
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg->pose.pose.orientation, quat);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    uav_pose_external.pos = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    uav_pose_external.vel = Eigen::Vector3d(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
    uav_pose_external.att = Eigen::Vector3d(roll, pitch, yaw);
    uav_pose_external.q = msg->pose.pose.orientation;
}

void VISION_POSE::viobot_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
    uav_pose_external.get_time = ros::Time::now(); // 记录时间戳，防止超时
    nav_msgs::Odometry viobot_raw;
    // 读取VIOBOT原始数据
    viobot_raw = *msg;

    // 坐标转换
    // VIOBOT坐标系定义为：算法初始化时，左目相机位置为原点；X轴沿着左目相机朝向设备前方，Y轴朝左，Z轴朝上；

    // 位置
    vision_pose.pose.position.x = viobot_raw.pose.pose.position.x;
    vision_pose.pose.position.y = -viobot_raw.pose.pose.position.y;
    vision_pose.pose.position.z = -viobot_raw.pose.pose.position.z;
    // 速度
    uav_pose_external.pos = Eigen::Vector3d(viobot_raw.pose.pose.position.x, -viobot_raw.pose.pose.position.y, -viobot_raw.pose.pose.position.z);
    uav_pose_external.vel = Eigen::Vector3d(viobot_raw.twist.twist.linear.x, -viobot_raw.twist.twist.linear.y, -viobot_raw.twist.twist.linear.z);
    // // 姿态
    // // VIOBOT四元数转欧拉角（VIOBOT系）
    // Eigen::Quaterniond q_viobot = Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    // Eigen::Vector3d euler_viobot = quaternion_to_euler(q_viobot);
    // // 欧拉角方向变化（VIOBOT系 -> 飞控 -> 本节点系）
    // vision_pose.attitude[0] = euler_viobot[0];
    // vision_pose.attitude[1] = -euler_viobot[1];
    // vision_pose.attitude[2] = -euler_viobot[2];
    // // 欧拉角转四元数（飞控 -> 本节点系）
    // Eigen::Vector3d euler_ned;
    // euler_ned[0] = vision_pose.attitude[0];
    // euler_ned[1] = vision_pose.attitude[1];
    // euler_ned[2] = vision_pose.attitude[2];
    // Eigen::Quaterniond q_vision_pose = quaternion_from_rpy(euler_ned);
    // vision_pose.attitude_q.x = q_vision_pose.x();
    // vision_pose.attitude_q.y = q_vision_pose.y();
    // vision_pose.attitude_q.z = q_vision_pose.z();
    // vision_pose.attitude_q.w = q_vision_pose.w();
}

void VISION_POSE::px4_state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    uav_state.connected = msg->connected;
    uav_state.mode = msg->mode.c_str();
    uav_state.armed = msg->armed;
}

void VISION_POSE::px4_battery_cb(const sensor_msgs::BatteryState::ConstPtr& msg)
{
    uav_state.battery_state = msg->voltage;
    uav_state.battery_percetage = msg->percentage * 100;
}

void VISION_POSE::local_position_ned_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    uav_pose_px4.pos = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
}

void VISION_POSE::local_vel_ned_cb(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    uav_pose_px4.vel = Eigen::Vector3d(msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z);
}

void VISION_POSE::attitude_cb(const sensor_msgs::Imu::ConstPtr &msg)
{
    tf2::Quaternion quaternion;
    tf2::fromMsg(msg->orientation, quaternion);

    // 转换为欧拉角
    tf2::Matrix3x3 mat(quaternion);
    double roll, pitch, yaw;
    mat.getRPY(roll, pitch, yaw);

    uav_pose_px4.att = Eigen::Vector3d(roll, pitch, yaw);
    uav_pose_px4.q = msg->orientation;
}

void VISION_POSE::range_cb(const sensor_msgs::Range::ConstPtr &msg)
{
    range_hight = hight_filter->filter(msg->range);
    std::cout << "height = " << range_hight << std::endl;
}

#endif
