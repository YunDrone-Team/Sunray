/*
本程序功能：
    1.根据外部定位来源参数，订阅外部定位数据（如动捕、SLAM等），并进行坐标转换处理
    2.检查外部定位数据超时、跳变、异常情况，进行定位有效判断
    3.use_vision_pose{true}:将外部定位数据通过~/mavros/vision_pose/pose话题转发到PX4中，用于给无人机做位姿估计
    4.use_vision_pose{false}:将外部定位数据通过mavlink发送到PX4中，用于给无人机做位姿估计
*/

#ifndef EXTERNALPOSITION_H
#define EXTERNALPOSITION_H

#include "ros_msg_utils.h"
#include "MovingAverageFilter.h"
#include "mavlink_control.h"

#define ODOM_TIMEOUT 0.3
#define DISTANCE_SENSOR_TIMEOUT 0.3
#define AVERAGE_COUNT 200

class ExternalPosition
{
public:
    ExternalPosition() {};

    // 参数
    int uav_id;                         // 无人机ID
    std::string uav_name;               // 无人机名字
    bool enable_external_fusion{false}; // 是否使能外部定位源
    bool use_vision_pose{true};         // true:使用vision_pose话题至PX4，false:直接使用Mavlink发送外部定位数据到PX4
    bool enable_range_sensor{false};
    std::string uart_name;
    int baudrate;
    std::string source_topic{""}; // 外部定位数据来源话题

    // VIOBOT相关参数
    bool tilted;                                         // 是否倾斜放置
    std::vector<double> ax_values, ay_values, az_values; // 加速度容器
    Eigen::Quaterniond eigen_q_rot;                      // 四元数 eigen
    tf2::Quaternion q_rot;                               // 旋转四元数
    double rot_roll, rot_pitch, rot_yaw;                 // 初始Viobot放置的角度
    bool calculation_done{false};                        // 偏转角计算是否完成

    bool get_new_external_pos{false}; // 是否收到新的外部定位数据

    sunray_msgs::ExternalOdom external_odom; // 声明一个自定义话题 - sunray_msgs::ExternalOdom
    sensor_msgs::Range distance_sensor;      // 距离传感器原始数据

    Eigen::Vector3d px4_local_pos; // PX4本地位置
    double px4_yaw;                // PX4本地偏航角

    void init(ros::NodeHandle &nh, int external_source);
    void PosCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void VelCallback(const geometry_msgs::TwistStamped::ConstPtr &msg);
    void OdomCallback(const nav_msgs::Odometry::ConstPtr &msg);
    void viobot_imuCallback(const sensor_msgs::Imu::ConstPtr &msg);
    void viobot_odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
    void viobot_algoStatusCallback(const sunray_msgs::algo_status::ConstPtr &msg);
    void px4_distance_callback(const sensor_msgs::Range::ConstPtr &msg);
    void timer_send_external_pos_cb(const ros::TimerEvent &event); // 定时器更新和发布
    mavlink_odometry_t get_mavlink_msg();
    void px4_pose_callback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void px4_att_callback(const sensor_msgs::Imu::ConstPtr &msg);

    sunray_msgs::ExternalOdom GetExternalOdom()
    {
        return external_odom;
    }

private:
    // ROS话题订阅句柄
    ros::Subscriber odom_sub;
    ros::Subscriber pos_sub;
    ros::Subscriber vel_sub;
    ros::Subscriber range_sub;
    ros::Subscriber viobot_imu_sub;
    ros::Subscriber viobot_odom_sub;
    ros::Subscriber viobot_algo_status_sub;
    ros::Subscriber px4_pose_sub, px4_att_sub;

    // ROS话题发布句柄
    ros::Publisher viobot_algo_ctrl_pub;
    ros::Publisher viobot_state_pub;
    ros::Publisher vision_pose_pub;
    ros::Publisher viobot_odom_pub;

    // 定时器句柄
    ros::Timer timer_send_external_pos;

    // 数据平滑滤波器（效果不好，未使用）
    MovingAverageFilter moving_average_filter;
};

// 初始化函数
void ExternalPosition::init(ros::NodeHandle &nh, int external_source = 0)
{
    // 初始化参数
    nh.param<int>("uav_id", uav_id, 1);
    nh.param<std::string>("uav_name", uav_name, "uav");
    uav_name = "/" + uav_name + std::to_string(uav_id);
    nh.param<string>("position_topic", source_topic, "/uav1/sunray/gazebo_pose"); // 【参数】外部定位数据来源
    nh.param<bool>("enable_range_sensor", enable_range_sensor, false);            // 【参数】是否使用距离传感器数据
    nh.param<bool>("use_vision_pose", use_vision_pose, true);                     // 【参数】是否使用vision_pose话题至PX4，false:直接使用Mavlink发送外部定位数据到PX4

    // 根据外部定位数据来源，订阅不同的话题
    switch (external_source)
    {
    // 定位源：nav_msgs::Odometry类型的话题
    case sunray_msgs::ExternalOdom::ODOM:
        odom_sub = nh.subscribe<nav_msgs::Odometry>(source_topic, 10, &ExternalPosition::OdomCallback, this);
        enable_external_fusion = true;
        break;
    // 定位源：geometry_msgs::PoseStamped类型的话题
    case sunray_msgs::ExternalOdom::POSE:
        pos_sub = nh.subscribe<geometry_msgs::PoseStamped>(source_topic, 10, &ExternalPosition::PosCallback, this);
        enable_external_fusion = true;
        break;
    // 定位源：GAZEBO仿真时使用Gazebo插件提供的位姿真值
    case sunray_msgs::ExternalOdom::GAZEBO:
        // GAZEBO属于外部定位源，默认开启外部定位融合
        enable_external_fusion = true;
        // 根据uav_id，订阅不同的gazebo_pose话题
        source_topic = uav_name + "/sunray/gazebo_pose";
        odom_sub = nh.subscribe<nav_msgs::Odometry>(source_topic, 10, &ExternalPosition::OdomCallback, this);
        break;
    // 定位源：动作捕捉系统
    case sunray_msgs::ExternalOdom::MOCAP:
        // MOCAP属于外部定位源，默认开启外部定位融合
        enable_external_fusion = true;
        // 【订阅】动捕的定位数据(坐标系:动捕系统惯性系) vrpn_client_node -> 本节点
        pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node_" + std::to_string(uav_id) + uav_name + "/pose", 1, &ExternalPosition::PosCallback, this);
        // 【订阅】动捕的定位数据(坐标系:动捕系统惯性系) vrpn_client_node -> 本节点 （此处只是订阅，实际没有使用该速度）
        vel_sub = nh.subscribe<geometry_msgs::TwistStamped>("/vrpn_client_node_" + std::to_string(uav_id) + uav_name + "/twist", 1, &ExternalPosition::VelCallback, this);
        break;
    // 定位源：VIOBOT
    case sunray_msgs::ExternalOdom::VIOBOT:
        // VIOBOT属于外部定位源，默认开启外部定位融合
        nh.param<bool>("tilted", tilted, false); // 【参数】Viobot是否倾斜放置
        enable_external_fusion = true;
        // 【订阅】VIOBOT算法的IMU数据 - VIOBOT算法程序 -> 本节点
        viobot_imu_sub = nh.subscribe<sensor_msgs::Imu>("/baton/imu", 10, &ExternalPosition::viobot_imuCallback, this);
        // 【订阅】VIOBOT算法的里程计数据 - VIOBOT算法程序 -> 本节点
        viobot_odom_sub = nh.subscribe<nav_msgs::Odometry>("/baton/stereo3/odometry", 10, &ExternalPosition::viobot_odomCallback, this);
        viobot_odom_pub = nh.advertise<nav_msgs::Odometry>(uav_name + "/sunray/viobot/odom", 10);
        // 【订阅】VIOBOT算法状态 - VIOBOT算法程序 -> 本节点
        viobot_algo_status_sub = nh.subscribe<sunray_msgs::algo_status>("/baton/algo_status", 2, &ExternalPosition::viobot_algoStatusCallback, this);
        // 【发布】控制VIOBOT算法启动与停止 - 本节点 -> VIOBOT算法程序
        viobot_algo_ctrl_pub = nh.advertise<sunray_msgs::algo_ctrl>("/baton/stereo3_ctrl", 2);
        break;
    // 定位源：Gps模式下，无需开启外部定位
    case sunray_msgs::ExternalOdom::GPS:
        // GPS不属于外部定位源，不开启外部定位融合
        enable_external_fusion = false;
        break;
    // 定位源：RTK模式下，无需开启外部定位
    case sunray_msgs::ExternalOdom::RTK:
        // RTK不属于外部定位源，不开启外部定位融合
        enable_external_fusion = false;
        break;
    // 定位源：VINS-Fusion
    case sunray_msgs::ExternalOdom::VINS:
        source_topic = uav_name + "/vins_estimator/odometry";
        enable_external_fusion = true;
        break;
    default:
        enable_external_fusion = false;
        Logger::print_color(int(LogColor::red), LOG_BOLD, "Unknown external position source type - [", external_source, "]");
        break;
    }

    // 无人机Z轴高度是否单独订阅定高激光雷达数据
    if (enable_range_sensor)
    {
        // 【订阅】无人机上的激光定高原始数据
        range_sub = nh.subscribe<sensor_msgs::Range>(uav_name + "/mavros/distance_sensor/hrlv_ez4_pub", 1, &ExternalPosition::px4_distance_callback, this);
    }

    // 如果使能了外部定位融合，则选择一个方式发送外部定位数据至PX4
    if (enable_external_fusion)
    {
        // 【订阅】PX4中的无人机位置，用于判断是否融合成功
        px4_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(uav_name + "/mavros/local_position/pose", 10, &ExternalPosition::px4_pose_callback, this);
        px4_att_sub = nh.subscribe<sensor_msgs::Imu>(uav_name + "/mavros/imu/data", 10, &ExternalPosition::px4_att_callback, this);
        // 如果使能了use_vision_pose，则通过mavros/vision_pose/pose话题定时更新及发布外部定位数据
        if (use_vision_pose)
        {
            // 【发布】mavros/vision_pose/pose - 本节点 -> mavros
            vision_pose_pub = nh.advertise<geometry_msgs::PoseStamped>(uav_name + "/mavros/vision_pose/pose", 10);
        }
        else
        {
            nh.param<string>("uart_name", uart_name, "/dev/ttyS0");
            nh.param<int>("baudrate", baudrate, 115200);
            // mavlink通信初始化
            mavlink_init(uart_name.c_str(), baudrate);
            // mavlink通信线程
            mavlink_send_odometry_thread(); // 先保存再在另外的线程里面发送
        }

        // 【定时器】当PX4需要外部定位输入时，定时更新和发布
        timer_send_external_pos = nh.createTimer(ros::Duration(0.01), &ExternalPosition::timer_send_external_pos_cb, this);
    }

    // 初始化外部定位状态
    external_odom.header.stamp = ros::Time::now();
    external_odom.external_source = external_source;
    external_odom.odom_valid = false;
    external_odom.fusion_success = false;
    external_odom.position[0] = NAN;
    external_odom.position[1] = NAN;
    external_odom.position[2] = NAN;
    external_odom.velocity[0] = NAN;
    external_odom.velocity[1] = NAN;
    external_odom.velocity[2] = NAN;
    external_odom.attitude[0] = NAN;
    external_odom.attitude[1] = NAN;
    external_odom.attitude[2] = NAN;
    external_odom.attitude_q.x = 0;
    external_odom.attitude_q.y = 0;
    external_odom.attitude_q.z = 0;
    external_odom.attitude_q.w = 1;
    external_odom.vio_start = false;
    external_odom.algo_status = "disable";
}

// 定时器回调函数
void ExternalPosition::timer_send_external_pos_cb(const ros::TimerEvent &event)
{
    if (!enable_external_fusion)
    {
        external_odom.odom_valid = true;
        return;
    }

    if (!get_new_external_pos)
    {
        // 如果没有收到新的外部定位数据，则不进行后续处理
        return;
    }
    get_new_external_pos = false;

    // 1.判断odom_valid：根据当前时刻和上一个收到外部定位的时间戳来判断是否持续收到外部定位数据信号
    bool odom_timeout = (ros::Time::now() - external_odom.header.stamp).toSec() > ODOM_TIMEOUT;
    external_odom.odom_valid = !odom_timeout;

    if (enable_range_sensor)
    {
        bool distance_timeout = (ros::Time::now() - distance_sensor.header.stamp).toSec() > DISTANCE_SENSOR_TIMEOUT;
        external_odom.odom_valid = external_odom.odom_valid && !distance_timeout;
    }

    // 外部定位失效时，停止发布外部定位数据（因为发了也是错的，还不如不发让飞控端了解到已经丢失数据了）
    if (!external_odom.odom_valid)
    {
        return;
    }

    // 2.判断fusion_success：根据外部定位数据与PX4回传的（XYZ+YAW）判断是否融合成功
    static Eigen::Vector3d err_external_px4;
    // 计算差值
    err_external_px4[0] = external_odom.position[0] - px4_local_pos[0];
    err_external_px4[1] = external_odom.position[1] - px4_local_pos[1];
    err_external_px4[2] = external_odom.position[2] - px4_local_pos[2];
    external_odom.fusion_success = true;

    if (abs(err_external_px4[0]) > 0.05 ||
        abs(err_external_px4[1]) > 0.05 ||
        abs(err_external_px4[2]) > 0.05 ||
        abs(external_odom.attitude[2] - px4_yaw) / M_PI * 180.0 > 5.0) // deg
    {
        external_odom.fusion_success = false;
    }

    // 3.将外部定位数据发送到PX4
    if (use_vision_pose)
    {

        // 将外部定位数据赋值到vision_pose，并发布至PX4（PX4接收并处理该消息需要修改EKF2参数，从而使能EKF2模块融合VISION数据）
        // 发布的ROS话题为~/mavros/vision_pose/pose，对应的MAVLINK消息为VISION_POSITION_ESTIMATE(#102)
        // 注意：该话题需要无人机的XYZ+YAW数据，坐标系方向为FLU（右手系：前-左-上，也可以理解为ENU，只是在无GPS环境，前方就代表E方向）
        static geometry_msgs::PoseStamped vision_pose;
        vision_pose.header.stamp = ros::Time::now();
        vision_pose.pose.position.x = external_odom.position[0];
        vision_pose.pose.position.y = external_odom.position[1];
        vision_pose.pose.position.z = external_odom.position[2];
        vision_pose.pose.orientation.x = external_odom.attitude_q.x;
        vision_pose.pose.orientation.y = external_odom.attitude_q.y;
        vision_pose.pose.orientation.z = external_odom.attitude_q.z;
        vision_pose.pose.orientation.w = external_odom.attitude_q.w;
        vision_pose_pub.publish(vision_pose);
    }
    else
    {
        // 通过线程定时发送（25Hz）
        mavlink_save_odometry(get_mavlink_msg());
        // 直接发送，随到随发
        // mavlink_send_odometry(get_mavlink_msg());
    }
}

void ExternalPosition::OdomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    get_new_external_pos = true;
    // 四元素转rpy
    tf2::Quaternion quaternion;
    tf2::fromMsg(msg->pose.pose.orientation, quaternion);
    double roll, pitch, yaw;
    tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);

    // external_odom赋值
    external_odom.header.stamp = ros::Time::now();
    external_odom.position[0] = msg->pose.pose.position.x;
    external_odom.position[1] = msg->pose.pose.position.y;
    external_odom.position[2] = msg->pose.pose.position.z;
    external_odom.velocity[0] = msg->twist.twist.linear.x;
    external_odom.velocity[1] = msg->twist.twist.linear.y;
    external_odom.velocity[2] = msg->twist.twist.linear.z;
    external_odom.attitude_q.x = msg->pose.pose.orientation.x;
    external_odom.attitude_q.y = msg->pose.pose.orientation.y;
    external_odom.attitude_q.z = msg->pose.pose.orientation.z;
    external_odom.attitude_q.w = msg->pose.pose.orientation.w;
    external_odom.attitude[0] = roll;
    external_odom.attitude[1] = pitch;
    external_odom.attitude[2] = yaw;
}

void ExternalPosition::PosCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    get_new_external_pos = true;
    // 四元素转rpy
    tf2::Quaternion quaternion;
    tf2::fromMsg(msg->pose.orientation, quaternion);
    double roll, pitch, yaw;
    tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
    external_odom.header.stamp = ros::Time::now();
    external_odom.position[0] = msg->pose.position.x;
    external_odom.position[1] = msg->pose.position.y;
    external_odom.position[2] = msg->pose.position.z;
    external_odom.attitude_q.x = msg->pose.orientation.x;
    external_odom.attitude_q.y = msg->pose.orientation.y;
    external_odom.attitude_q.z = msg->pose.orientation.z;
    external_odom.attitude_q.w = msg->pose.orientation.w;
    external_odom.attitude[0] = roll;
    external_odom.attitude[1] = pitch;
    external_odom.attitude[2] = yaw;
}

void ExternalPosition::VelCallback(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    external_odom.velocity[0] = msg->twist.linear.x;
    external_odom.velocity[1] = msg->twist.linear.y;
    external_odom.velocity[2] = msg->twist.linear.z;
}

void ExternalPosition::viobot_imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
    if (calculation_done)
    {
        viobot_imu_sub.shutdown(); // 注销订阅者
        return;
    }

    // 存储加速度计数据
    ax_values.push_back(msg->linear_acceleration.z);
    ay_values.push_back(-msg->linear_acceleration.x);
    az_values.push_back(msg->linear_acceleration.y);

    // 当收集到足够的样本时计算平均值和角度
    if (ax_values.size() >= AVERAGE_COUNT)
    {
        // 计算平均值
        double sum_ax = 0, sum_ay = 0, sum_az = 0;
        for (size_t i = 0; i < ax_values.size(); ++i)
        {
            sum_ax += ax_values[i];
            sum_ay += ay_values[i];
            sum_az += az_values[i];
        }

        double avg_ax = sum_ax / ax_values.size();
        double avg_ay = sum_ay / ay_values.size();
        double avg_az = sum_az / az_values.size();

        // 计算旋转矩阵和欧拉角
        Eigen::Vector3d vectorBefore(avg_ax, avg_ay, avg_az);
        vectorBefore.normalize();
        Eigen::Vector3d vectorAfter(0, 0, -1);
        eigen_q_rot = Eigen::Quaterniond::FromTwoVectors(vectorBefore, vectorAfter);

        // 赋值
        q_rot = tf2::Quaternion(eigen_q_rot.x(), eigen_q_rot.y(), eigen_q_rot.z(), eigen_q_rot.w());
        tf2::Matrix3x3(q_rot).getRPY(rot_roll, rot_pitch, rot_yaw);

        // 打印结果
        ROS_INFO("=== IMU Tilt Calculation Results ===");
        ROS_INFO("Collected %zu samples", ax_values.size());
        ROS_INFO("Roll (around X-axis):  %.2f degrees", rot_roll * 180.0 / M_PI);
        ROS_INFO("Pitch (around Y-axis): %.2f degrees", rot_pitch * 180.0 / M_PI);
        ROS_INFO("Yaw (around Z-axis): %.2f degrees", rot_yaw * 180.0 / M_PI);
        ROS_INFO("=== IMU Tilt Calculation End ===");
        calculation_done = true;
    }
}

void ExternalPosition::viobot_odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    if (!calculation_done)
        return;

    get_new_external_pos = true;

    // 位置信息
    Eigen::Vector3d p = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);

    tf2::Quaternion q;
    q.setW(msg->pose.pose.orientation.w);
    q.setX(msg->pose.pose.orientation.x);
    q.setY(msg->pose.pose.orientation.y);
    q.setZ(msg->pose.pose.orientation.z);

    // 先把数据处理成相机的FLU（相对于ENU）
    /*
        这个坐标系其实就是FLU相对于ENU的位置，ENU作为全局坐标系，其实在程序开启的时候就确定了，如果他和FLU重合，
        说明此时E指定的就是程序开启时F的指向（机头方向，也就是前方）
        旋转矩阵：
        左乘是变换父坐标系
        右乘是在原本的坐标系下旋转

        以下做的处理是把相机body姿态转为FLU
    */
    // 绕 Z 轴旋转 90°
    tf2::Quaternion q_z;
    q_z.setRPY(0, 0, M_PI / 2); // M_PI/2 = 90°

    // 绕 Y 轴旋转 -90°
    tf2::Quaternion q_y;
    q_y.setRPY(0, -M_PI / 2, 0); // -M_PI/2 = -90°

    // 组合旋转（顺序：先 q_z，再 q_y）
    q = q * q_z * q_y;

    // 处理初始倾角(重力对齐)
    if (tilted)
    {
        // q = q * q_rot;
        // q = q_rot.inverse() * q;
        q = q * q_rot.inverse();
    }

    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    // external_odom赋值
    external_odom.header.stamp = ros::Time::now();
    external_odom.position[0] = p.x();
    external_odom.position[1] = p.y();
    external_odom.position[2] = p.z();
    external_odom.velocity[0] = NAN;
    external_odom.velocity[1] = NAN;
    external_odom.velocity[2] = NAN;
    external_odom.attitude_q.x = q.x();
    external_odom.attitude_q.y = q.y();
    external_odom.attitude_q.z = q.z();
    external_odom.attitude_q.w = q.w();
    external_odom.attitude[0] = roll;
    external_odom.attitude[1] = pitch;
    external_odom.attitude[2] = yaw;

        // 发布里程计消息
    nav_msgs::Odometry odom_msg;
    odom_msg.header = external_odom.header;
    odom_msg.pose.pose.position.x = p.x();
    odom_msg.pose.pose.position.y = p.y();
    odom_msg.pose.pose.position.z = p.z();
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();
    viobot_odom_pub.publish(odom_msg);
}

void ExternalPosition::viobot_algoStatusCallback(const sunray_msgs::algo_status::ConstPtr &msg)
{
    external_odom.algo_status = msg->algo_status;

    if (msg->algo_status == "stereo3_initializing" || msg->algo_status == "stereo3_running")
    {
        external_odom.vio_start = true;
    }
    else
    {
        external_odom.vio_start = false;
    }

    if (external_odom.vio_start == false)
    {
        static sunray_msgs::algo_ctrl algo_set;
        algo_set.algo_enable = true;
        viobot_algo_ctrl_pub.publish(algo_set);
    }
}

// 回调函数：接收PX4距离传感器原始数据
void ExternalPosition::px4_distance_callback(const sensor_msgs::Range::ConstPtr &msg)
{
    distance_sensor = *msg;
}

mavlink_odometry_t ExternalPosition::get_mavlink_msg()
{
    static mavlink_odometry_t mavlink_odom;

    memset(&mavlink_odom, 0, sizeof(mavlink_odometry_t));

    mavlink_odom.frame_id = MAV_FRAME_LOCAL_FRD;
    mavlink_odom.child_frame_id = MAV_FRAME_LOCAL_FRD;
    mavlink_odom.estimator_type = MAV_ESTIMATOR_TYPE_VISION;
    mavlink_odom.time_usec = external_odom.header.stamp.toSec() * 1000;

    // 将里程计相对于ENU的位置转换为相对于NED的位置
    mavlink_odom.x = external_odom.position[1];
    mavlink_odom.y = external_odom.position[0];
    mavlink_odom.z = -external_odom.position[2];

    Eigen::Quaterniond mav_q(external_odom.attitude_q.w, external_odom.attitude_q.x, external_odom.attitude_q.y, external_odom.attitude_q.z);

    // ql是参考坐标系的旋转矩阵：ENU转换到NED，需要绕原始x坐标轴旋转180，再绕原始y坐标轴旋转90
    Eigen::Quaterniond ql(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));
    // qr是自身坐标系的旋转矩阵：FLU转换到FRD，需要绕原始x坐标轴旋转180
    Eigen::Quaterniond qr(Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));

    // 先左乘换参考坐标系，再右乘变换自身坐标系
    mav_q = ql * mav_q * qr;

    // printf("Original Euler Angles: Roll: %f, Pitch: %f, Yaw: %f\n", roll * 180 / M_PI, pitch * 180 / M_PI, yaw * 180 / M_PI);
    // q.setRPY(roll, -pitch, -yaw);

    mavlink_odom.q[0] = mav_q.w();
    mavlink_odom.q[1] = mav_q.x();
    mavlink_odom.q[2] = mav_q.y();
    mavlink_odom.q[3] = mav_q.z();

    mavlink_odom.vx = external_odom.velocity[0];
    mavlink_odom.vy = external_odom.velocity[1];
    mavlink_odom.vz = external_odom.velocity[2];

    mavlink_odom.rollspeed = NAN;
    mavlink_odom.pitchspeed = NAN;
    mavlink_odom.yawspeed = NAN;

    for (int i = 0; i < 21; i++)
    {
        mavlink_odom.pose_covariance[i] = NAN;
        mavlink_odom.velocity_covariance[i] = NAN;
    }

    return mavlink_odom;
}

// 回调函数：PX4中的无人机位置
void ExternalPosition::px4_pose_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    px4_local_pos[0] = msg->pose.position.x;
    px4_local_pos[1] = msg->pose.position.y;
    px4_local_pos[2] = msg->pose.position.z;
}

// 回调函数：PX4中的无人机姿态
void ExternalPosition::px4_att_callback(const sensor_msgs::Imu::ConstPtr &msg)
{
    // 转为rpy
    tf::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    px4_yaw = yaw;
}
#endif