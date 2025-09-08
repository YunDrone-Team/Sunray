/*
本程序功能：
    1.通过ExternalPosition发布外部定位数据
    2.订阅PX4相关话题（多个）及外部定位信息，打包为一个自定义消息话题PX4State发布，~/sunray/px4_state
    3.发布相关无人机位置、轨迹、mesh等话题用于rviz显示(包括TF转换)

添加自定义外部定位数据：
    1.参考相关程序在 【include/ExternalPosition.h】 中实现自己的解析类
*/

#include "externalFusion.h"

void ExternalFusion::init(ros::NodeHandle &nh)
{
    node_name = ros::this_node::getName();                                              // 【参数】节点名称
    nh.param<int>("uav_id", uav_id, 1);                                                 // 【参数】无人机编号
    nh.param<int>("external_source", external_source, sunray_msgs::ExternalOdom::ODOM); // 【参数】外部定位数据来源
    nh.param<string>("uav_name", uav_name, "uav");                                      // 【参数】无人机名称
    // 无人机名字 = 无人机名字前缀 + 无人机ID
    uav_name = "/" + uav_name + std::to_string(uav_id);

    // 【重要外部类】初始化外部定位数据解析类(输入：外部定位数据来源类型 - external_source)
    ext_pos.init(nh, external_source);

    // 【订阅】【重要话题】无人机PX4模式 - 飞控 -> mavros -> 本节点
    px4_state_sub = nh.subscribe<mavros_msgs::State>(uav_name + "/mavros/state", 10, &ExternalFusion::px4_state_callback, this);
    // 【订阅】无人机PX4状态（是否降落） - 飞控 -> mavros -> 本节点
    px4_extended_state_sub = nh.subscribe<mavros_msgs::ExtendedState>(uav_name + "/mavros/extended_state", 10, &ExternalFusion::px4_extended_state_callback, this);
    // 【订阅】无人机电池状态 - 飞控 -> mavros -> 本节点
    px4_battery_sub = nh.subscribe<sensor_msgs::BatteryState>(uav_name + "/mavros/battery", 10, &ExternalFusion::px4_battery_callback, this);
    // 【订阅】【重要话题】PX4中的无人机位置（坐标系:ENU系） - 飞控 -> mavros -> 本节点
    px4_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(uav_name + "/mavros/local_position/pose", 10, &ExternalFusion::px4_pose_callback, this);
    // 【订阅】【重要话题】PX4中的无人机速度（坐标系:ENU系） - 飞控 -> mavros -> 本节点
    px4_vel_sub = nh.subscribe<geometry_msgs::TwistStamped>(uav_name + "/mavros/local_position/velocity_local", 10, &ExternalFusion::px4_vel_callback, this);
    // 【订阅】【重要话题】PX4中的无人机欧拉角 - 飞控 -> mavros -> 本节点
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

    // 【发布】【重要话题】PX4无人机综合状态 - 本节点 -> uav_control_node
    px4_state_pub = nh.advertise<sunray_msgs::PX4State>(uav_name + "/sunray/px4_state", 1);
    // 【发布】无人机里程计 - 本节点 -> 其他需要odom接口的节点/RVIZ
    uav_odom_pub = nh.advertise<nav_msgs::Odometry>(uav_name + "/sunray/uav_odom", 1);
    // 【发布】无人机运动轨迹 - 本节点 -> RVIZ
    uav_trajectory_pub = nh.advertise<nav_msgs::Path>(uav_name + "/sunray/uav_trajectory", 1);
    // 【发布】无人机MESH图标 - 本节点 -> RVIZ
    uav_mesh_pub = nh.advertise<visualization_msgs::Marker>(uav_name + "/sunray/uav_mesh", 1);

    // 【定时器】【重要定时器】检查超时等任务以及发布PX4_STATE状态
    timer_pub_px4_state = nh.createTimer(ros::Duration(0.01), &ExternalFusion::timer_pub_px4_state_cb, this);
    // 【定时器】RVIZ相关话题定时发布  - 本节点 -> RVIZ
    timer_rviz_pub = nh.createTimer(ros::Duration(0.1), &ExternalFusion::timer_rviz, this);

    // PX4无人机状态 - 初始化
    px4_state.header.stamp = ros::Time::now();
    px4_state.connected = false;
    px4_state.armed = false;
    px4_state.mode = "none";
    px4_state.battery_state = 0;
    px4_state.battery_percentage = 0;
    px4_state.external_odom = ext_pos.external_odom;
    px4_state.position[0] = -0.01;
    px4_state.position[1] = -0.01;
    px4_state.position[2] = -0.01;
    px4_state.velocity[0] = 0.0;
    px4_state.velocity[1] = 0.0;
    px4_state.velocity[2] = 0.0;
    px4_state.attitude[0] = 0.0;
    px4_state.attitude[1] = 0.0;
    px4_state.attitude[2] = 0.0;
    px4_state.attitude_q.x = 0;
    px4_state.attitude_q.y = 0;
    px4_state.attitude_q.z = 0;
    px4_state.attitude_q.w = 1;
    px4_state.attitude_rate[0] = 0.0;
    px4_state.attitude_rate[1] = 0.0;
    px4_state.attitude_rate[2] = 0.0;
    px4_state.satellites = -1;
    px4_state.gps_status = -1;
    px4_state.gps_service = -1;
    px4_state.latitude = -1;
    px4_state.longitude = -1;
    px4_state.altitude = -1;
    px4_state.pos_setpoint[0] = 0.0;
    px4_state.pos_setpoint[1] = 0.0;
    px4_state.pos_setpoint[2] = 0.0;
    px4_state.vel_setpoint[0] = 0.0;
    px4_state.vel_setpoint[1] = 0.0;
    px4_state.vel_setpoint[2] = 0.0;
    px4_state.att_setpoint[0] = 0.0;
    px4_state.att_setpoint[1] = 0.0;
    px4_state.att_setpoint[2] = 0.0;
    px4_state.q_setpoint.x = 0;
    px4_state.q_setpoint.y = 0;
    px4_state.q_setpoint.z = 0;
    px4_state.q_setpoint.w = 1;
    px4_state.thrust_setpoint = 0.0;

    Logger::info("external fusion node init");
}

// 定时器回调函数
void ExternalFusion::timer_pub_px4_state_cb(const ros::TimerEvent &event)
{
    // 检查mavros连接是否正常
    if ((ros::Time::now() - px4_state_time).toSec() > PX4_TIMEOUT)
    {
        px4_state.connected = false;
    }

    // 更新无人机时间戳
    px4_state.header.stamp = ros::Time::now();
    // external_odom来自external_position类
    px4_state.external_odom = ext_pos.external_odom;
    // 发布PX4State话题
    px4_state_pub.publish(px4_state);

    // 发布无人机odom格式数据，用于需要odom话题类型的节点
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
}

// 定时器回调函数，用于发布无人机当前轨迹等
void ExternalFusion::timer_rviz(const ros::TimerEvent &e)
{
    // 如果无人机的odom的状态无效，则停止发布
    if (!ext_pos.external_odom.odom_valid)
    {
        return;
    }

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
    if (uav_pos_vector.size() > TRAJECTORY_WINDOW)
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

    // 发布TF用于RVIZ显示（用于sensor显示）
    static tf2_ros::TransformBroadcaster broadcaster;
    geometry_msgs::TransformStamped tfs;
    //  世界坐标系
    tfs.header.frame_id = "world";    
    tfs.header.stamp = ros::Time::now();
    //  局部坐标系（如传感器坐标系）
    tfs.child_frame_id = uav_name + "/base_link"; 
    //  坐标系相对信息设置，局部坐标系相对于世界坐标系的坐标（偏移量）
    tfs.transform.translation.x = px4_state.position[0];
    tfs.transform.translation.y = px4_state.position[1];
    tfs.transform.translation.z = px4_state.position[2];
    tfs.transform.rotation.x = px4_state.attitude_q.x;
    tfs.transform.rotation.y = px4_state.attitude_q.y;
    tfs.transform.rotation.z = px4_state.attitude_q.z;
    tfs.transform.rotation.w = px4_state.attitude_q.w;
    broadcaster.sendTransform(tfs);

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

// 回调函数：PX4状态
void ExternalFusion::px4_extended_state_callback(const mavros_msgs::ExtendedState::ConstPtr &msg)
{
    px4_state.landed_state = msg->landed_state;
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

// 打印状态
void ExternalFusion::show_px4_state()
{
    Logger::print_color(int(LogColor::cyan), ">>>>>>>>>>>>>>>> 无人机状态节点 - [", uav_name, "] <<<<<<<<<<<<<<<<<");

    // 基本信息 - 连接状态、飞控模式、电池状态
    Logger::print_color(int(LogColor::cyan), "-------- 飞控状态 - [~/sunray/px4_state]");
    
    Logger::print_color(int(LogColor::green), "飞控连接: [ 已连接 ]  电池状态:", px4_state.battery_state, "[V]", px4_state.battery_percentage, "[%]");

    if (px4_state.armed)
    {
        if(px4_state.landed_state == sunray_msgs::PX4State::LANDED_STATE_ON_GROUND)
        {
            Logger::print_color(int(LogColor::green), "飞控状态: [ 已解锁 ]", LOG_GREEN, "[", px4_state.mode, "]", LOG_GREEN, "[ 未起飞 ]");   
        }else
        {
            Logger::print_color(int(LogColor::green), "飞控状态: [ 已解锁 ]", LOG_GREEN, "[", px4_state.mode, "]", LOG_GREEN, "[ 已起飞 ]");   
        }
    }
    else
    {
        if(px4_state.landed_state == sunray_msgs::PX4State::LANDED_STATE_ON_GROUND)
        {
            Logger::print_color(int(LogColor::red), "飞控状态: [ 未解锁 ]", LOG_GREEN, "[ ", px4_state.mode, " ]", LOG_GREEN, "[ 未起飞 ]");   
        }else
        {
            Logger::print_color(int(LogColor::red), "飞控状态: [ 未解锁 ]", LOG_GREEN, "[ ", px4_state.mode, " ]", LOG_GREEN, "[ 已起飞 ]");   
        }
    }

    // 无人机的位置和姿态
    if (external_source != sunray_msgs::ExternalOdom::GPS && external_source != sunray_msgs::ExternalOdom::RTK)
    {
        // 无GPS模式的情况：本地位置
        Logger::print_color(int(LogColor::green), "无人机位置[X Y Z]:",
                            px4_state.position[0],
                            px4_state.position[1],
                            px4_state.position[2],
                            "[ m ]");
        Logger::print_color(int(LogColor::green), "无人机速度[X Y Z]:",
                            px4_state.velocity[0],
                            px4_state.velocity[1],
                            px4_state.velocity[2],
                            "[m/s]");
        Logger::print_color(int(LogColor::green), "无人机姿态[X Y Z]:",
                            px4_state.attitude[0] / M_PI * 180,
                            px4_state.attitude[1] / M_PI * 180,
                            px4_state.attitude[2] / M_PI * 180,
                            "[deg]");
    }else
    {
        // GPS模式的情况：经纬度（全局位置）
        Logger::print_color(int(LogColor::blue), "GPS Status");
        Logger::print_color(int(LogColor::green), "GPS STATUS:", px4_state.gps_status, "SERVICE:", px4_state.gps_service);
        Logger::print_color(int(LogColor::green), "GPS SATS:", px4_state.satellites);
        Logger::print_color(int(LogColor::green), "GPS POS[lat lon alt]:", int(px4_state.latitude), int(px4_state.longitude), int(px4_state.altitude));
        // todo global position
    }

    // 期望位置和姿态
    Logger::print_color(int(LogColor::green), "位置期望值[X Y Z]:",
                        px4_state.pos_setpoint[0],
                        px4_state.pos_setpoint[1],
                        px4_state.pos_setpoint[2],
                        "[ m ]");
    Logger::print_color(int(LogColor::green), "速度期望值[X Y Z]:",
                        px4_state.vel_setpoint[0],
                        px4_state.vel_setpoint[1],
                        px4_state.vel_setpoint[2],
                        "[m/s]");
    Logger::print_color(int(LogColor::green), "姿态期望值[X Y Z]:",
                        px4_state.att_setpoint[0] / M_PI * 180,
                        px4_state.att_setpoint[1] / M_PI * 180,
                        px4_state.att_setpoint[2] / M_PI * 180,
                        "[deg]",
                        "推力期望值 :", px4_state.thrust_setpoint * 100, "[ % ]");


    // 外部定位信息
    Logger::print_color(int(LogColor::cyan), "-------- 外部定位状态");

    switch (px4_state.external_odom.external_source)
    {
        case sunray_msgs::ExternalOdom::ODOM:
            Logger::print_color(int(LogColor::green), "外部定位源: [ ODOM ]");
            break;
        case sunray_msgs::ExternalOdom::POSE:
            Logger::print_color(int(LogColor::green), "外部定位源: [ POSE ]");
            break;
        case sunray_msgs::ExternalOdom::GAZEBO:
            Logger::print_color(int(LogColor::green), "外部定位源: [ GAZEBO ]");
            break;
        case sunray_msgs::ExternalOdom::MOCAP:
            Logger::print_color(int(LogColor::green), "外部定位源: [ MOCAP ]");
            break;
        case sunray_msgs::ExternalOdom::VIOBOT:
            Logger::print_color(int(LogColor::green), "外部定位源: [ VIOBOT ]", " VIO算法是否开启: ", ext_pos.external_odom.vio_start == true ? "[ True ]" : "[ False ]", "算法状态: [ ", ext_pos.external_odom.algo_status, " ]");
            break;
        case sunray_msgs::ExternalOdom::GPS:
            Logger::print_color(int(LogColor::green), "外部定位源: [ GPS ]");
            break;
        default:
            Logger::print_color(int(LogColor::red), "外部定位源: [ UNKNOWN ]");
            break;
    }

    if(ext_pos.enable_external_fusion)
    {
        if (ext_pos.external_odom.odom_valid)
        {
            if (ext_pos.external_odom.fusion_success)
            {
                Logger::print_color(int(LogColor::green), "定位源状态: [ 有效 ]", "飞控融合状态: [ 成功 ]");
            }
            else
            {
                Logger::print_color(int(LogColor::green), "定位源状态: [ 有效 ]", "飞控融合成功: [ 失败 ]");
            }
        }
        else
        {
            Logger::print_color(int(LogColor::red), "定位源状态: [ 失效 ]");
        }

        Logger::print_color(int(LogColor::green), "外部里程计位置[X Y Z]:",
                            ext_pos.external_odom.position[0],
                            ext_pos.external_odom.position[1],
                            ext_pos.external_odom.position[2],
                            "[ m ]");
        Logger::print_color(int(LogColor::green), "外部里程计速度[X Y Z]:",
                            ext_pos.external_odom.velocity[0],
                            ext_pos.external_odom.velocity[1],
                            ext_pos.external_odom.velocity[2],
                            "[m/s]");
        Logger::print_color(int(LogColor::green), "外部里程计姿态[X Y Z]:",
                            ext_pos.external_odom.attitude[0] / M_PI * 180,
                            ext_pos.external_odom.attitude[1] / M_PI * 180,
                            ext_pos.external_odom.attitude[2] / M_PI * 180,
                            "[deg]");

        Logger::print_color(int(LogColor::green), "飞控融合位置误差[X Y Z]:",
                            ext_pos.external_odom.position[0] - px4_state.position[0],
                            ext_pos.external_odom.position[1] - px4_state.position[1],
                            ext_pos.external_odom.position[2] - px4_state.position[2],
                            "[m]");
        Logger::print_color(int(LogColor::green), "飞控融合速度误差[X Y Z]:",
                            ext_pos.external_odom.velocity[0] - px4_state.velocity[0],
                            ext_pos.external_odom.velocity[1] - px4_state.velocity[1],
                            ext_pos.external_odom.velocity[2] - px4_state.velocity[2],
                            "[m/s]");
        Logger::print_color(int(LogColor::green), "飞控融合偏航误差[ YAW ]:",
                            (ext_pos.external_odom.attitude[2] - px4_state.attitude[2]) / M_PI * 180,
                            "[deg]");
    }
    else
    {
        Logger::print_color(int(LogColor::green), "外部定位源: [未启用]");
    }

    // 控制误差打印
    Logger::print_color(int(LogColor::white_bg_green), ">>>>> 控制误差");

    if (external_source != sunray_msgs::ExternalOdom::GPS && external_source != sunray_msgs::ExternalOdom::RTK)
    {
        static Eigen::Vector3d pos_control_error; // 控制误差（位置）
        pos_control_error[0] = px4_state.pos_setpoint[0] - px4_state.position[0];
        pos_control_error[1] = px4_state.pos_setpoint[1] - px4_state.position[1];
        pos_control_error[2] = px4_state.pos_setpoint[2] - px4_state.position[2];

        // 无GPS模式的情况
        Logger::print_color(int(LogColor::green), "位置控制误差[X Y Z norm]:",
                            pos_control_error[0],
                            pos_control_error[1],
                            pos_control_error[2],
                            pos_control_error.norm(),
                            "[ m ]");
        Logger::print_color(int(LogColor::green), "速度控制误差[X Y Z]:",
                            px4_state.vel_setpoint[0] - px4_state.velocity[0],
                            px4_state.vel_setpoint[1] - px4_state.velocity[1],
                            px4_state.vel_setpoint[2] - px4_state.velocity[2],
                            "[m/s]");
        Logger::print_color(int(LogColor::green), "姿态控制误差[X Y Z]:",
                            px4_state.att_setpoint[0] / M_PI * 180 - px4_state.attitude[0] / M_PI * 180,
                            px4_state.att_setpoint[1] / M_PI * 180 - px4_state.attitude[1] / M_PI * 180,
                            px4_state.att_setpoint[2] / M_PI * 180 - px4_state.attitude[2] / M_PI * 180,
                            "[deg]");
    }
    Logger::print_color(int(LogColor::cyan), "---------------------------------------------------------");
}
