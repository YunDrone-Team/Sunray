/*
本程序功能：
    1.根据外部定位来源参数，订阅外部定位数据（如动捕、SLAM等），并进行坐标转换处理
    2.检查外部定位数据超时、跳变、异常情况，进行定位有效判断
    3.将外部定位数据通过~/mavros/vision_pose/pose话题转发到PX4中，用于给无人机做位姿估计
    4.将外部定位数据打包成一个话题发布，~/sunray/external_odom_state
    5.订阅PX4相关话题（多个），打包为一个自定义消息话题PX4State发布，~/sunray/px4_state
    6.发布相关无人机位置、轨迹、mesh等话题用于rviz显示

添加自定义外部定位数据：
    1.参考相关程序在 【include/ExternalPosition.h】 中实现自己的解析类
    2.在source_map中添加对应的定位名称
*/

#include "externalFusion.h"
#include <signal.h>

using namespace std;
using namespace sunray_logger;

ExternalFusion::ExternalFusion()
{
    source_map[sunray_msgs::ExternalOdom::ODOM] = "ODOM";
    source_map[sunray_msgs::ExternalOdom::POSE] = "POSE";
    source_map[sunray_msgs::ExternalOdom::GAZEBO] = "GAZEBO";
    source_map[sunray_msgs::ExternalOdom::MOCAP] = "MOCAP";
    source_map[sunray_msgs::ExternalOdom::VIOBOT] = "VIOBOT";
    source_map[sunray_msgs::ExternalOdom::GPS] = "GPS";
    source_map[sunray_msgs::ExternalOdom::RTK] = "RTK";
    source_map[sunray_msgs::ExternalOdom::VINS] = "VINS";
}

void ExternalFusion::init(ros::NodeHandle &nh)
{
    nh_ = nh;
    node_name = ros::this_node::getName();                                              // 【参数】节点名称
    nh.param<int>("uav_id", uav_id, 1);                                                 // 【参数】无人机编号
    nh.param<int>("external_source", external_source, sunray_msgs::ExternalOdom::ODOM); // 【参数】外部定位数据来源
    nh.param<string>("uav_name", uav_name, "uav");                                      // 【参数】无人机名称
    nh.param<string>("position_topic", source_topic, "/uav1/sunray/gazebo_pose");       // 【参数】外部定位数据来源
    nh.param<bool>("enable_rviz", enable_rviz, false);                                  // 【参数】是否发布到rviz
    nh.param<bool>("enable_range_sensor", enable_range_sensor, false);                  // 【参数】是否使用距离传感器数据
    // nh.param<bool>("enable_rangeSensor", enable_rangeSensor, false);           // 【参数】是否使用rangeSensor

    // 初始化外部定位数据解析类
    external_position.init(nh, source_topic, external_source);

    uav_name = "/" + uav_name + std::to_string(uav_id);
    // 【订阅】无人机PX4模式 - 飞控 -> mavros -> 本节点
    px4_state_sub = nh.subscribe<mavros_msgs::State>(uav_name + "/mavros/state", 10, &ExternalFusion::px4_state_callback, this);
    // 【订阅】无人机电池状态 - 飞控 -> mavros -> 本节点
    px4_battery_sub = nh.subscribe<sensor_msgs::BatteryState>(uav_name + "/mavros/battery", 10, &ExternalFusion::px4_battery_callback, this);
    // 【订阅】PX4中的无人机位置（坐标系:ENU系） - 飞控 -> mavros -> 本节点
    px4_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(uav_name + "/mavros/local_position/pose", 10, &ExternalFusion::px4_pose_callback, this);
    // 【订阅】PX4中的无人机速度（坐标系:ENU系） - 飞控 -> mavros -> 本节点
    px4_vel_sub = nh.subscribe<geometry_msgs::TwistStamped>(uav_name + "/mavros/local_position/velocity_local", 10, &ExternalFusion::px4_vel_callback, this);
    // 【订阅】PX4中的无人机欧拉角 - 飞控 -> mavros -> 本节点
    px4_att_sub = nh.subscribe<sensor_msgs::Imu>(uav_name + "/mavros/imu/data", 10, &ExternalFusion::px4_att_callback, this);
    // 【订阅】无人机GPS卫星数量 - 飞控 -> mavros -> 本节点
    px4_gps_satellites_sub = nh.subscribe<std_msgs::UInt32>(uav_name + "/mavros/global_position/raw/satellites", 10, &ExternalFusion::px4_gps_satellites_callback, this);
    // 【订阅】无人机GPS状态 - 飞控 -> mavros -> 本节点
    px4_gps_state_sub = nh.subscribe<sensor_msgs::NavSatFix>(uav_name + "/mavros/global_position/global", 10, &ExternalFusion::px4_gps_state_callback, this);
    // 【订阅】无人机GPS经纬度 - 飞控 -> mavros -> 本节点
    px4_gps_raw_sub = nh.subscribe<mavros_msgs::GPSRAW>(uav_name + "/mavros/gpsstatus/gps1/raw", 10, &ExternalFusion::px4_gps_raw_callback, this);
    // 【订阅】PX4中无人机的位置/速度/加速度设定值 - 飞控 -> mavros -> 本节点 （用于检验控制指令是否被PX4执行）
    px4_pos_target_sub = nh.subscribe<mavros_msgs::PositionTarget>(uav_name + "/mavros/setpoint_raw/target_local", 1, &ExternalFusion::px4_pos_target_callback, this);
    // 【订阅】PX4中无人机的姿态设定值 - 飞控 -> mavros -> 本节点 （用于检验控制指令是否被PX4执行）
    px4_att_target_sub = nh.subscribe<mavros_msgs::AttitudeTarget>(uav_name + "/mavros/setpoint_raw/target_attitude", 1, &ExternalFusion::px4_att_target_callback, this);
    // 【订阅】无人机上的激光定高原始数据
    px4_distance_sub = nh.subscribe<sensor_msgs::Range>(uav_name + "/mavros/distance_sensor/hrlv_ez4_pub", 1, &ExternalFusion::px4_distance_callback, this);
    // 如果使能了RVIZ，则发布RVIZ中显示的话题
    if (enable_rviz)
    {
        // 【发布】无人机里程计 - 本节点 -> RVIZ
        uav_odom_pub = nh.advertise<nav_msgs::Odometry>(uav_name + "/sunray/uav_odom", 1);
        // 【发布】无人机运动轨迹 - 本节点 -> RVIZ
        uav_trajectory_pub = nh.advertise<nav_msgs::Path>(uav_name + "/sunray/uav_trajectory", 1);
        // 【发布】无人机MESH图标 - 本节点 -> RVIZ
        uav_mesh_pub = nh.advertise<visualization_msgs::Marker>(uav_name + "/sunray/uav_mesh", 1);
        // 【定时器】RVIZ相关话题定时发布  - 本节点 -> RVIZ
        timer_rviz_pub = nh.createTimer(ros::Duration(0.1), &ExternalFusion::timer_rviz, this);
    }

    // 【发布】mavros/vision_pose/pose - 本节点 -> mavros
    vision_pose_pub = nh.advertise<geometry_msgs::PoseStamped>(uav_name + "/mavros/vision_pose/pose", 10);
    // 【发布】无人机状态 - 本节点 -> uav_control_node
    px4_state_pub = nh.advertise<sunray_msgs::PX4State>(uav_name + "/sunray/px4_state", 10);

    // 【定时器】任务 检查超时等任务以及发布PX4_STATE状态
    timer_pub_px4_state = nh.createTimer(ros::Duration(0.05), &ExternalFusion::timer_pub_px4_state_cb, this);
    // 【定时器】定时更新和发布到mavros/vision_pose/pose
    timer_pub_vision_pose = nh.createTimer(ros::Duration(0.02), &ExternalFusion::timer_pub_vision_pose_cb, this);

    // 无人机状态初始化
    px4_state.connected = false;
    px4_state.armed = false;
    px4_state.battery_state = 0;
    px4_state.battery_percentage = 0;
    px4_state.mode = "UNKNOWN";

    Logger::info("external fusion node init");
}

// 定时器回调函数
void ExternalFusion::timer_pub_vision_pose_cb(const ros::TimerEvent &event)
{
    external_odom = external_position.external_odom;

    if (!external_odom.odom_valid)
    {
        // 加一个打印
        return;
    }

    if (external_source != sunray_msgs::ExternalOdom::GPS && external_source != sunray_msgs::ExternalOdom::RTK)
    {
        // 将外部定位数据赋值到vision_pose，并发布至PX4
        vision_pose.header.stamp = ros::Time::now();
        vision_pose.pose.position.x = external_odom.position[0];
        vision_pose.pose.position.y = external_odom.position[1];
        vision_pose.pose.position.z = external_odom.position[2];
        if (enable_range_sensor)
        {
            vision_pose.pose.position.z = distance_sensor.range;
        }
        vision_pose.pose.orientation.x = external_odom.attitude_q.x;
        vision_pose.pose.orientation.y = external_odom.attitude_q.y;
        vision_pose.pose.orientation.z = external_odom.attitude_q.z;
        vision_pose.pose.orientation.w = external_odom.attitude_q.w;
        vision_pose_pub.publish(vision_pose);
    }
}

// 定时器回调函数
void ExternalFusion::timer_pub_px4_state_cb(const ros::TimerEvent &event)
{
    // 检查mavros连接是否正常
    if ((ros::Time::now() - px4_state_time).toSec() > 1.0)
    {
        px4_state.connected = false;
    }

    // 发布PX4State
    px4_state.external_odom = external_odom;
    px4_state.header.stamp = ros::Time::now();
    px4_state_pub.publish(px4_state);

    if (external_source != sunray_msgs::ExternalOdom::GPS && external_source != sunray_msgs::ExternalOdom::RTK)
    {
        // 检查超时
        if (!external_odom.odom_valid)
        {
            // Logger::warning("Warning: The external position is timeout!", external_position->position_state.timeout_count);
            err_msg.insert(2);
        }
        Eigen::Vector3d err_external_px4;
        // 计算差值
        err_external_px4[0] = external_odom.position[0] - px4_state.position[0];
        err_external_px4[1] = external_odom.position[1] - px4_state.position[1];
        err_external_px4[2] = external_odom.position[2] - px4_state.position[2];

        // 如果误差状态的绝对值大于阈值，则打印警告信息   只检查位置和偏航角
        if (abs(err_external_px4[0]) > 0.1 ||
            abs(err_external_px4[1]) > 0.1 ||
            abs(err_external_px4[2]) > 0.1)
        // abs(err_state.yaw) > 10) // deg
        {
            // Logger::warning("Warning: The error between external state and px4 state is too large!");
            err_msg.insert(1);
        }
    }
    if (enable_range_sensor)
    {
        if ((ros::Time::now() - distance_sensor.header.stamp).toSec() > 1.0)
        {
            err_msg.insert(3);
        }
    }
}

// 定时器回调函数，用于发布无人机当前轨迹等
void ExternalFusion::timer_rviz(const ros::TimerEvent &e)
{
    // 如果无人机的odom的状态无效，则停止发布
    if (!external_odom.odom_valid)
    {
        return;
    }

    // 发布无人机里程计，用于rviz显示
    nav_msgs::Odometry uav_odom;
    uav_odom.header.stamp = ros::Time::now();
    uav_odom.header.frame_id = "world";
    uav_odom.child_frame_id = "base_link";
    uav_odom.pose.pose.position.x = px4_state.position[0];
    uav_odom.pose.pose.position.y = px4_state.position[1];
    uav_odom.pose.pose.position.z = px4_state.position[2];
    // 导航算法规定 高度不能小于0
    if (uav_odom.pose.pose.position.z <= 0)
    {
        uav_odom.pose.pose.position.z = 0.01;
    }
    uav_odom.pose.pose.orientation = px4_state.attitude_q;
    uav_odom.twist.twist.linear.x = px4_state.velocity[0];
    uav_odom.twist.twist.linear.y = px4_state.velocity[1];
    uav_odom.twist.twist.linear.z = px4_state.velocity[2];
    uav_odom_pub.publish(uav_odom);

    // 发布无人机运动轨迹，用于rviz显示
    geometry_msgs::PoseStamped uav_pos;
    uav_pos.header.stamp = ros::Time::now();
    uav_pos.header.frame_id = "world";
    uav_pos.pose.position.x = px4_state.position[0];
    uav_pos.pose.position.y = px4_state.position[1];
    uav_pos.pose.position.z = px4_state.position[2];
    uav_pos.pose.orientation = px4_state.attitude_q;
    uav_pos_vector.insert(uav_pos_vector.begin(), uav_pos);
    // 轨迹滑窗
    if (uav_pos_vector.size() > 40)
    {
        uav_pos_vector.pop_back();
    }
    nav_msgs::Path uav_trajectory;
    uav_trajectory.header.stamp = ros::Time::now();
    uav_trajectory.header.frame_id = "world";
    uav_trajectory.poses = uav_pos_vector;
    uav_trajectory_pub.publish(uav_trajectory);

    // 发布无人机MESH，用于rviz显示
    visualization_msgs::Marker rviz_mesh;
    rviz_mesh.header.frame_id = "world";
    rviz_mesh.header.stamp = ros::Time::now();
    rviz_mesh.ns = "mesh";
    rviz_mesh.id = 0;
    rviz_mesh.type = visualization_msgs::Marker::MESH_RESOURCE;
    rviz_mesh.action = visualization_msgs::Marker::ADD;
    rviz_mesh.pose.position.x = px4_state.position[0];
    rviz_mesh.pose.position.y = px4_state.position[1];
    rviz_mesh.pose.position.z = px4_state.position[2];
    rviz_mesh.pose.orientation = px4_state.attitude_q;
    rviz_mesh.scale.x = 1.0;
    rviz_mesh.scale.y = 1.0;
    rviz_mesh.scale.z = 1.0;
    rviz_mesh.color.a = 1.0;
    // 根据uav_id生成对应的颜色
    rviz_mesh.color.r = static_cast<float>((uav_id * 123) % 256) / 255.0;
    rviz_mesh.color.g = static_cast<float>((uav_id * 456) % 256) / 255.0;
    rviz_mesh.color.b = static_cast<float>((uav_id * 789) % 256) / 255.0;
    rviz_mesh.mesh_use_embedded_materials = false;
    rviz_mesh.mesh_resource = std::string("package://sunray_uav_control/meshes/uav.mesh");
    uav_mesh_pub.publish(rviz_mesh);

    // // 发布TF用于RVIZ显示（用于lidar）
    // static tf2_ros::TransformBroadcaster broadcaster;
    // geometry_msgs::TransformStamped tfs;
    // //  |----头设置
    // tfs.header.frame_id = "world";       // 相对于世界坐标系
    // tfs.header.stamp = ros::Time::now(); // 时间戳
    // //  |----坐标系 ID
    // tfs.child_frame_id = uav_prefix + "/lidar_link"; // 子坐标系，无人机的坐标系
    // // tfs.child_frame_id = "/lidar_link"; //子坐标系，无人机的坐标系
    // //  |----坐标系相对信息设置  偏移量  无人机相对于世界坐标系的坐标
    // tfs.transform.translation.x = external_odom.position[0];
    // tfs.transform.translation.y = external_odom.position[1];
    // tfs.transform.translation.z = external_odom.position[2];
    // //  |--------- 四元数设置
    // tfs.transform.rotation.x = external_state.att_x;
    // tfs.transform.rotation.y = external_state.att_y;
    // tfs.transform.rotation.z = external_state.att_z;
    // tfs.transform.rotation.w = external_state.att_w;
    // //  |--------- 广播器发布数据
    // broadcaster.sendTransform(tfs);

    // // q_orig  是原姿态转换的tf的四元数
    // // q_rot   旋转四元数
    // // q_new   旋转后的姿态四元数
    // tf2::Quaternion q_orig, q_rot, q_new;

    // // commanded_pose.pose.orientation  这个比如说 是 订阅的别的节点的topic 是一个  姿态的 msg 四元数
    // // 通过tf2::convert()  转换成 tf 的四元数
    // tf2::convert(tfs.transform.rotation, q_orig);

    // // 设置 绕 x 轴 旋转180度
    // double r = -1.57, p = 0, y = -1.57;
    // q_rot.setRPY(r, p, y); // 求得 tf 的旋转四元数

    // q_new = q_orig * q_rot; // 通过 姿态的四元数 乘以旋转的四元数 即为 旋转 后的  四元数
    // q_new.normalize();      // 归一化

    // //  将 旋转后的 tf 四元数 转换 为 msg 四元数
    // tf2::convert(q_new, tfs.transform.rotation);
    // tfs.child_frame_id = uav_name + "/camera_link"; // 子坐标系，无人机的坐标系
    // //  |--------- 广播器发布数据
    // broadcaster.sendTransform(tfs);
}

// 回调函数：PX4中的无人机位置
void ExternalFusion::px4_pose_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    px4_state.position[0] = msg->pose.position.x;
    px4_state.position[1] = msg->pose.position.y;
    px4_state.position[2] = msg->pose.position.z;
}

// 回调函数：PX4中的无人机速度
void ExternalFusion::px4_vel_callback(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    px4_state.velocity[0] = msg->twist.linear.x;
    px4_state.velocity[1] = msg->twist.linear.y;
    px4_state.velocity[2] = msg->twist.linear.z;
}

// 回调函数：PX4状态
void ExternalFusion::px4_state_callback(const mavros_msgs::State::ConstPtr &msg)
{
    px4_state_time = ros::Time::now();
    px4_state.connected = msg->connected;
    px4_state.armed = msg->armed;
    px4_state.mode = msg->mode;
}

// 回调函数：PX4电池
void ExternalFusion::px4_battery_callback(const sensor_msgs::BatteryState::ConstPtr &msg)
{
    px4_state.battery_state = msg->voltage;
    px4_state.battery_percentage = msg->percentage * 100;
}

// 回调函数：PX4中的无人机姿态
void ExternalFusion::px4_att_callback(const sensor_msgs::Imu::ConstPtr &msg)
{
    px4_state.attitude_q.x = msg->orientation.x;
    px4_state.attitude_q.y = msg->orientation.y;
    px4_state.attitude_q.z = msg->orientation.z;
    px4_state.attitude_q.w = msg->orientation.w;

    // 转为rpy
    tf::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    px4_state.attitude[0] = roll;
    px4_state.attitude[1] = pitch;
    px4_state.attitude[2] = yaw;
}

// 无人机卫星数量回调函数
void ExternalFusion::px4_gps_satellites_callback(const std_msgs::UInt32::ConstPtr &msg)
{
    px4_state.satellites = msg->data;
}

// 无人机卫星状态回调函数
void ExternalFusion::px4_gps_state_callback(const sensor_msgs::NavSatFix::ConstPtr &msg)
{
    px4_state.gps_status = msg->status.status;
    px4_state.gps_service = msg->status.service;
}

// 无人机gps原始数据回调函数
void ExternalFusion::px4_gps_raw_callback(const mavros_msgs::GPSRAW::ConstPtr &msg)
{
    px4_state.latitude = msg->lat;
    px4_state.longitude = msg->lon;
    px4_state.altitude = msg->alt;
}

// 回调函数：接收PX4的姿态设定值
void ExternalFusion::px4_att_target_callback(const mavros_msgs::AttitudeTarget::ConstPtr &msg)
{
    px4_state.q_setpoint.x = msg->orientation.x;
    px4_state.q_setpoint.y = msg->orientation.y;
    px4_state.q_setpoint.z = msg->orientation.z;
    px4_state.q_setpoint.w = msg->orientation.w;

    // 转为rpy
    tf::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    px4_state.att_setpoint[0] = roll;
    px4_state.att_setpoint[1] = pitch;
    px4_state.att_setpoint[2] = yaw;

    // px4_rates_target = Eigen::Vector3d(msg->body_rate.x, msg->body_rate.y, msg->body_rate.z);
    px4_state.thrust_setpoint = msg->thrust;
}

// 回调函数：接收PX4位置设定值
void ExternalFusion::px4_pos_target_callback(const mavros_msgs::PositionTarget::ConstPtr &msg)
{
    px4_state.pos_setpoint[0] = msg->position.x;
    px4_state.pos_setpoint[1] = msg->position.y;
    px4_state.pos_setpoint[2] = msg->position.z;
    px4_state.vel_setpoint[0] = msg->velocity.x;
    px4_state.vel_setpoint[1] = msg->velocity.y;
    px4_state.vel_setpoint[2] = msg->velocity.z;
}

// 回调函数：接收PX4距离传感器原始数据
void ExternalFusion::px4_distance_callback(const sensor_msgs::Range::ConstPtr &msg)
{
    distance_sensor = *msg;
}

// 打印状态
void ExternalFusion::show_px4_state()
{
    Logger::print_color(int(LogColor::blue), LOG_BOLD, ">>>>>>>>>>>>>>", uav_name, "<<<<<<<<<<<<<<<");
    if (px4_state.connected)
    {
        Logger::print_color(int(LogColor::green), "CONNECTED:", "TRUE");
        if (px4_state.armed)
            Logger::print_color(int(LogColor::green), "MODE:", px4_state.mode, LOG_GREEN, "ARMED");
        else
            Logger::print_color(int(LogColor::green), "MODE:", px4_state.mode, LOG_RED, "DISARMED");
        Logger::print_color(int(LogColor::green), "BATTERY:", px4_state.battery_state, "[V]", px4_state.battery_percentage, "[%]");
    }
    else
        Logger::print_color(int(LogColor::red), "CONNECTED:", "FALSE");

    if (external_source == sunray_msgs::ExternalOdom::GPS || external_source == sunray_msgs::ExternalOdom::RTK)
    {
        Logger::print_color(int(LogColor::green), "GPS STATUS:", px4_state.gps_status, "SERVICE:", px4_state.gps_service);
        Logger::print_color(int(LogColor::green), "GPS SATS:", px4_state.satellites);
        Logger::print_color(int(LogColor::green), "GPS POS[lat lon alt]:", int(px4_state.latitude), int(px4_state.longitude), int(px4_state.altitude));
        Logger::print_color(int(LogColor::blue), "EXTERNAL GLOBAL POS(receive)");
    }
    else
    {
        Logger::print_color(int(LogColor::blue), "EXTERNAL POS(send)");
    }
    // Logger::print_color(int(LogColor::blue), "EXTERNAL POS(send)");
    if (enable_range_sensor)
    {
        Logger::print_color(int(LogColor::green), "POS[X Y Z]:",
                            external_odom.position[0],
                            external_odom.position[1],
                            distance_sensor.range,
                            "[m]");
    }
    else
    {
        Logger::print_color(int(LogColor::green), "POS[X Y Z]:",
                            external_odom.position[0],
                            external_odom.position[1],
                            external_odom.position[2],
                            "[m]");
    }
    Logger::print_color(int(LogColor::green), "VEL[X Y Z]:",
                        external_odom.velocity[0],
                        external_odom.velocity[1],
                        external_odom.velocity[2],
                        "[m/s]");
    Logger::print_color(int(LogColor::green), "ATT[X Y Z]:",
                        external_odom.attitude[0] / M_PI * 180,
                        external_odom.attitude[1] / M_PI * 180,
                        external_odom.attitude[2] / M_PI * 180,
                        "[deg]");

    if (px4_state.connected)
    {
        if (external_source != sunray_msgs::ExternalOdom::GPS && external_source != sunray_msgs::ExternalOdom::RTK)
        {
            Logger::print_color(int(LogColor::blue), "PX4 POS(receive)");
            Logger::print_color(int(LogColor::green), "POS[X Y Z]:",
                                px4_state.position[0],
                                px4_state.position[1],
                                px4_state.position[2],
                                "[m]");
            Logger::print_color(int(LogColor::green), "VEL[X Y Z]:",
                                px4_state.velocity[0],
                                px4_state.velocity[1],
                                px4_state.velocity[2],
                                "[m/s]");
            Logger::print_color(int(LogColor::green), "ATT[X Y Z]:",
                                px4_state.attitude[0] / M_PI * 180,
                                px4_state.attitude[1] / M_PI * 180,
                                px4_state.attitude[2] / M_PI * 180,
                                "[deg]");

            Logger::print_color(int(LogColor::blue), "ERR POS(send - receive)");
            Logger::print_color(int(LogColor::green), "POS[X Y Z]:",
                                external_odom.position[0] - px4_state.position[0],
                                external_odom.position[1] - px4_state.position[1],
                                external_odom.position[2] - px4_state.position[2],
                                "[m]");
            Logger::print_color(int(LogColor::green), "VEL[X Y Z]:",
                                external_odom.velocity[0] - px4_state.velocity[0],
                                external_odom.velocity[1] - px4_state.velocity[1],
                                external_odom.velocity[2] - px4_state.velocity[2],
                                "[m/s]");
            Logger::print_color(int(LogColor::green), "ATT[X Y Z]:",
                                (external_odom.attitude[0] - px4_state.attitude[0]) / M_PI * 180,
                                (external_odom.attitude[1] - px4_state.attitude[1]) / M_PI * 180,
                                (external_odom.attitude[2] - px4_state.attitude[2]) / M_PI * 180,
                                "[deg]");
        }
    }
    // 打印报错信息
    for (int msg : err_msg)
    {
        Logger::warning(ERR_MSG[msg]);
    }
    err_msg.clear();
}
