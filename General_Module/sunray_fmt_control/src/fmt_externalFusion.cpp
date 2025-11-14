/*
本程序功能：
    1.根据外部定位来源参数，订阅外部定位数据（如动捕、SLAM等），并进行坐标转换处理
    2.检查外部定位数据超时、跳变、异常情况，进行定位有效判断
    3.将外部定位数据通过~/mavros/vision_pose/pose话题转发到PX4中，用于给无人机做位姿估计
    4.订阅FMT相关话题（多个）及外部定位信息，打包为一个自定义消息话题PX4State发布，~/sunray/px4_state
    5.通过服务设置FMT数据流

添加自定义外部定位数据：
    1.参考相关程序在 【include/fmt_externalFusion.h】 中实现自己的解析类
*/

#include "FMTexternalFusion.h"

FMTExternalFusion::FMTExternalFusion()
{
    // 初始化外部定位数据源映射
    source_map[sunray_msgs::ExternalOdom::ODOM] = "ODOM";
    source_map[sunray_msgs::ExternalOdom::POSE] = "POSE";
    source_map[sunray_msgs::ExternalOdom::GAZEBO] = "GAZEBO";
    source_map[sunray_msgs::ExternalOdom::MOCAP] = "MOCAP";
    source_map[sunray_msgs::ExternalOdom::VIOBOT] = "VIOBOT";
    source_map[sunray_msgs::ExternalOdom::GPS] = "GPS";
    source_map[sunray_msgs::ExternalOdom::RTK] = "RTK";
    source_map[sunray_msgs::ExternalOdom::VINS] = "VINS";
}

// 设置FMT数据流
bool FMTExternalFusion::set_fmt_stream_rate(uint8_t stream_id, uint16_t message_rate, bool on_off)
{
    mavros_msgs::StreamRate srv;
    srv.request.stream_id = stream_id;
    srv.request.message_rate = message_rate;
    srv.request.on_off = on_off;

    if (fmt_stream_rate_client.call(srv)) {
        ROS_INFO("Set stream rate for ID %d to %d Hz", stream_id, message_rate);
        return true;
    } else {
        ROS_ERROR("Failed to set stream rate for ID %d", stream_id);
        return false;
    }
}

void FMTExternalFusion::init(ros::NodeHandle &nh)
{
    std::string node_name = ros::this_node::getName(); // 【参数】节点名称
    nh.param<int>("uav_id", uav_id, 1);                // 【参数】无人机编号
    nh.param<std::string>("uav_name", uav_name, "uav"); // 【参数】无人机名称
    nh.param<int>("external_source", external_source, sunray_msgs::ExternalOdom::ODOM); // 【参数】外部定位数据来源
    
    std::string source_topic;
    nh.param<std::string>("position_topic", source_topic, "/uav1/sunray/gazebo_pose"); // 【参数】外部定位数据话题
    
    // 无人机名字 = 无人机名字前缀 + 无人机ID
    uav_name = "/" + uav_name + std::to_string(uav_id);

    // 初始化外部定位数据解析类
    bool enable_range_sensor = false;

    // GPS或RTK定位时，不需要发布vision_pose
    if (external_source == sunray_msgs::ExternalOdom::GPS || external_source == sunray_msgs::ExternalOdom::RTK)
    {
        enable_vision_pose = false;
    }

    ext_pos.init(nh, external_source, source_topic, enable_range_sensor);

    // 【订阅】FMT状态
    fmt_state_sub = nh.subscribe<mavros_msgs::State>(uav_name + "/mavros/state", 10, &FMTExternalFusion::fmt_state_callback, this);
    // 【订阅】FMT扩展状态
    fmt_extended_state_sub = nh.subscribe<mavros_msgs::ExtendedState>(uav_name + "/mavros/extended_state", 10, &FMTExternalFusion::fmt_extended_state_callback, this);
    // 【订阅】FMT电池状态
    fmt_battery_sub = nh.subscribe<sensor_msgs::BatteryState>(uav_name + "/mavros/battery", 10, &FMTExternalFusion::fmt_battery_callback, this);
    // 【订阅】FMT本地位置
    fmt_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(uav_name + "/mavros/local_position/pose", 10, &FMTExternalFusion::fmt_pose_callback, this);
    // 【订阅】FMT本地速度
    fmt_vel_sub = nh.subscribe<geometry_msgs::TwistStamped>(uav_name + "/mavros/local_position/velocity_local", 10, &FMTExternalFusion::fmt_vel_callback, this);
    // 【订阅】FMT姿态
    fmt_att_sub = nh.subscribe<sensor_msgs::Imu>(uav_name + "/mavros/imu/data", 10, &FMTExternalFusion::fmt_att_callback, this);
    // 【订阅】FMT GPS卫星数量
    fmt_gps_satellites_sub = nh.subscribe<std_msgs::UInt32>(uav_name + "/mavros/global_position/raw/satellites", 10, &FMTExternalFusion::fmt_gps_satellites_callback, this);
    // 【订阅】FMT GPS状态
    fmt_gps_state_sub = nh.subscribe<sensor_msgs::NavSatFix>(uav_name + "/mavros/global_position/global", 10, &FMTExternalFusion::fmt_gps_state_callback, this);
    // 【订阅】FMT GPS原始数据
    fmt_gps_raw_sub = nh.subscribe<mavros_msgs::GPSRAW>(uav_name + "/mavros/gpsstatus/gps1/raw", 10, &FMTExternalFusion::fmt_gps_raw_callback, this);
    // 【订阅】FMT位置设定值
    fmt_pos_target_sub = nh.subscribe<mavros_msgs::PositionTarget>(uav_name + "/mavros/setpoint_raw/target_local", 1, &FMTExternalFusion::fmt_pos_target_callback, this);
    // 【订阅】FMT姿态设定值
    fmt_att_target_sub = nh.subscribe<mavros_msgs::AttitudeTarget>(uav_name + "/mavros/setpoint_raw/target_attitude", 1, &FMTExternalFusion::fmt_att_target_callback, this);
    // 【发布】无人机里程计
    uav_odom_pub = nh.advertise<nav_msgs::Odometry>(uav_name + "/sunray/uav_odom", 10);
    // 【发布】vision_pose
    vision_pose_pub = nh.advertise<geometry_msgs::PoseStamped>(uav_name + "/mavros/vision_pose/pose", 10); 
    // 【发布】FMT综合状态
    fmt_state_pub = nh.advertise<sunray_msgs::PX4State>(uav_name + "/sunray/px4_state", 10);
    // 【服务】FMT数据流设置
    fmt_stream_rate_client = nh.serviceClient<mavros_msgs::StreamRate>(uav_name + "/mavros/set_stream_rate");

    // 【发布】无人机轨迹，用于仿真环境下的可视化
    uav_trajectory_pub = nh.advertise<nav_msgs::Path>(uav_name + "/sunray/uav_trajectory", 1);
    // 【发布】无人机MESH图标，用于仿真环境下的可视化
    uav_mesh_pub = nh.advertise<visualization_msgs::Marker>(uav_name + "/sunray/uav_mesh", 1);

    // 【定时器】当FMT需要外部定位输入时，定时更新和发布到mavros/vision_pose/pose
    if (enable_vision_pose)
    {
        timer_pub_vision_pose = nh.createTimer(ros::Duration(0.01), &FMTExternalFusion::timer_pub_vision_pose_cb, this);
    }
    
    // 设置FMT数据流
    ros::Duration(1.0).sleep(); // 等待服务可用
    //请求电池状态 (BATTERY_STATUS) 频率 1Hz
    set_fmt_stream_rate(147, 1, true);
    // 请求高精度IMU数据 (HIGHRES_IMU) 频率 100Hz
    set_fmt_stream_rate(105, 100, true);
    //请求无人机的位置/速度/加速度设定值 (POSITION_TARGET_LOCAL_NED) 频率 20Hz
    set_fmt_stream_rate(85, 20, true);
    // 请求无人机的姿态设定值 (ATTITUDE_TARGET) 频率 20Hz
    set_fmt_stream_rate(83, 20, true);
    //请求imu的姿态解算数据 (ATTITUDE) 频率 100Hz
    set_fmt_stream_rate(30, 100, true);
    // 请求本地位置和速度数据 (LOCAL_POSITION_NED) 频率 50Hz
    set_fmt_stream_rate(32, 50, true);
    // 请求全局位置数据 (GLOBAL_POSITION_INT) 频率 10Hz
    set_fmt_stream_rate(33, 10, true);
    // 请求GPS原始数据 (GPS_RAW_INT) 频率 5Hz
    set_fmt_stream_rate(24, 5, true);

    ROS_INFO("数据流请求完成，开始监听话题...");

    // 定时器任务 - 发布FMT状态
    timer_pub_fmt_state = nh.createTimer(ros::Duration(0.01), &FMTExternalFusion::timer_pub_fmt_state_cb, this);
    // 定时器任务 - RVIZ可视化发布
    time_rviz_pub = nh.createTimer(ros::Duration(0.1), &FMTExternalFusion::timer_rviz, this);

    // FMT无人机状态 - 初始化
    init_fmt_state();

    Logger::info("FMT external fusion node init for %s", uav_name.c_str());
}

void FMTExternalFusion::init_fmt_state()
{
    fmt_state.header.stamp = ros::Time::now();
    fmt_state.connected = false;
    fmt_state.armed = false;
    fmt_state.mode = "none";
    fmt_state.battery_state = 0;
    fmt_state.battery_percentage = 0;
    fmt_state.external_odom = ext_pos.external_odom;
    fmt_state.position[0] = -0.01;
    fmt_state.position[1] = -0.01;
    fmt_state.position[2] = -0.01;
    fmt_state.velocity[0] = 0.0;
    fmt_state.velocity[1] = 0.0;
    fmt_state.velocity[2] = 0.0;
    fmt_state.attitude[0] = 0.0;
    fmt_state.attitude[1] = 0.0;
    fmt_state.attitude[2] = 0.0;
    fmt_state.attitude_q.x = 0;
    fmt_state.attitude_q.y = 0;
    fmt_state.attitude_q.z = 0;
    fmt_state.attitude_q.w = 1;
    fmt_state.attitude_rate[0] = 0.0;
    fmt_state.attitude_rate[1] = 0.0;
    fmt_state.attitude_rate[2] = 0.0;
    fmt_state.satellites = -1;
    fmt_state.gps_status = -1;
    fmt_state.gps_service = -1;
    fmt_state.latitude = -1;
    fmt_state.longitude = -1;
    fmt_state.altitude = -1;
    fmt_state.pos_setpoint[0] = 0.0;
    fmt_state.pos_setpoint[1] = 0.0;
    fmt_state.pos_setpoint[2] = 0.0;
    fmt_state.vel_setpoint[0] = 0.0;
    fmt_state.vel_setpoint[1] = 0.0;
    fmt_state.vel_setpoint[2] = 0.0;
    fmt_state.att_setpoint[0] = 0.0;
    fmt_state.att_setpoint[1] = 0.0;
    fmt_state.att_setpoint[2] = 0.0;
    fmt_state.q_setpoint.x = 0;
    fmt_state.q_setpoint.y = 0;
    fmt_state.q_setpoint.z = 0;
    fmt_state.q_setpoint.w = 1;
    fmt_state.thrust_setpoint = 0.0;
}

// 定时器回调函数
void FMTExternalFusion::timer_pub_vision_pose_cb(const ros::TimerEvent &event)
{
    // 外部定位失效时，停止发布vision_pose（因为发了也是错的，还不如不发让飞控端了解到已经丢失数据了）
    if (!ext_pos.external_odom.odom_valid)
    {
        // TODO: 加一个打印
        return;
    }

    // 位置转换: FLU -> RFU
    // x_rfu = y_flu (左变右，需要取反)
    // y_rfu = x_flu (前保持不变)
    // z_rfu = z_flu (上保持不变)
    double x_rfu = -ext_pos.external_odom.position[1]; // FLU的y取反得到RFU的x
    double y_rfu = ext_pos.external_odom.position[0];  // FLU的x变为RFU的y
    double z_rfu = ext_pos.external_odom.position[2];  // FLU的z保持不变

    // 姿态转换: FLU -> RFU
    // 需要将FLU的姿态旋转90度绕Z轴，使前方向从X轴变为Y轴
    tf2::Quaternion q_rfu(
        ext_pos.external_odom.attitude_q.x,
        ext_pos.external_odom.attitude_q.y,
        ext_pos.external_odom.attitude_q.z,
        ext_pos.external_odom.attitude_q.w
    );

    // 将转换后的数据发布到FMT
    vision_pose.header.stamp = ros::Time::now();
    vision_pose.header.frame_id = "map"; // 确保使用正确的坐标系
    vision_pose.pose.position.x = x_rfu;
    vision_pose.pose.position.y = y_rfu;
    vision_pose.pose.position.z = z_rfu;
    vision_pose.pose.orientation.x = q_rfu.x();
    vision_pose.pose.orientation.y = q_rfu.y();
    vision_pose.pose.orientation.z = q_rfu.z();
    vision_pose.pose.orientation.w = q_rfu.w();
    vision_pose_pub.publish(vision_pose);
}

// 定时器回调函数：发布FMT状态
void FMTExternalFusion::timer_pub_fmt_state_cb(const ros::TimerEvent &event)
{
    // 检查FMT连接是否正常
    if ((ros::Time::now() - fmt_state_time).toSec() > 1.0) // 1秒超时
    {
        fmt_state.connected = false;
    }

        
    // 更新时间戳并发布FMT状态
    fmt_state.header.stamp = ros::Time::now();
    // external_odom来自external_position类
    fmt_state.external_odom = ext_pos.external_odom;
    //发布状态
    fmt_state_pub.publish(fmt_state);

    // 发布无人机odom格式数据，用于需要odom话题类型的节点
    nav_msgs::Odometry uav_odom;
    uav_odom.header.stamp = ros::Time::now();
    uav_odom.header.frame_id = "world";
    uav_odom.child_frame_id = "base_link";
    uav_odom.pose.pose.position.x = fmt_state.position[0];
    uav_odom.pose.pose.position.y = fmt_state.position[1];
    uav_odom.pose.pose.position.z = fmt_state.position[2];
    // 导航算法规定 高度不能小于0
    if (uav_odom.pose.pose.position.z <= 0)
    {
        uav_odom.pose.pose.position.z = 0.01;
    }
    uav_odom.pose.pose.orientation = fmt_state.attitude_q;
    uav_odom.twist.twist.linear.x = fmt_state.velocity[0];
    uav_odom.twist.twist.linear.y = fmt_state.velocity[1];
    uav_odom.twist.twist.linear.z = fmt_state.velocity[2];
    uav_odom_pub.publish(uav_odom);

    // 打印debug信息
    if (external_source != sunray_msgs::ExternalOdom::GPS && external_source != sunray_msgs::ExternalOdom::RTK)
    {
        // 检查超时
        if (!ext_pos.external_odom.odom_valid)
        {
            err_msg.insert(2);
        }
        Eigen::Vector3d err_external_fmt;
        // 计算差值
        err_external_fmt[0] = ext_pos.external_odom.position[0] - fmt_state.position[0];
        err_external_fmt[1] = ext_pos.external_odom.position[1] - fmt_state.position[1];
        err_external_fmt[2] = ext_pos.external_odom.position[2] - fmt_state.position[2];

        // 如果误差状态的绝对值大于阈值，则打印警告信息   只检查位置和偏航角
        if (abs(err_external_fmt[0]) > 0.1 ||
            abs(err_external_fmt[1]) > 0.1 ||
            abs(err_external_fmt[2]) > 0.1)
        // abs(err_state.yaw) > 10) // deg
        {
            err_msg.insert(1);
        }
    }
}

void FMTExternalFusion::timer_rviz(const ros::TimerEvent &event)
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
    uav_pos.pose.position.x = fmt_state.position[0];
    uav_pos.pose.position.y = fmt_state.position[1];
    uav_pos.pose.position.z = fmt_state.position[2];
    uav_pos.pose.orientation = fmt_state.attitude_q;
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
    rviz_mesh.pose.position.x = fmt_state.position[0];
    rviz_mesh.pose.position.y = fmt_state.position[1];
    rviz_mesh.pose.position.z = fmt_state.position[2];
    rviz_mesh.pose.orientation = fmt_state.attitude_q;
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
    tfs.transform.translation.x = fmt_state.position[0];
    tfs.transform.translation.y = fmt_state.position[1];
    tfs.transform.translation.z = fmt_state.position[2];
    tfs.transform.rotation.x = fmt_state.attitude_q.x;
    tfs.transform.rotation.y = fmt_state.attitude_q.y;
    tfs.transform.rotation.z = fmt_state.attitude_q.z;
    tfs.transform.rotation.w = fmt_state.attitude_q.w;
    broadcaster.sendTransform(tfs);
}

// 回调函数：FMT状态
void FMTExternalFusion::fmt_state_callback(const mavros_msgs::State::ConstPtr &msg)
{
    fmt_state_time = ros::Time::now();
    fmt_state.connected = msg->connected;
    fmt_state.armed = msg->armed;
    fmt_state.mode = msg->mode;
}

// 回调函数：FMT扩展状态
void FMTExternalFusion::fmt_extended_state_callback(const mavros_msgs::ExtendedState::ConstPtr &msg)
{
    fmt_state.landed_state = msg->landed_state;
}

// 回调函数：FMT电池状态
void FMTExternalFusion::fmt_battery_callback(const sensor_msgs::BatteryState::ConstPtr &msg)
{
    fmt_state.battery_state = msg->voltage;
    fmt_state.battery_percentage = msg->percentage * 100;
}

// 回调函数：FMT本地位置
void FMTExternalFusion::fmt_pose_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    fmt_state.position[0] = msg->pose.position.x;
    fmt_state.position[1] = msg->pose.position.y;
    fmt_state.position[2] = msg->pose.position.z;
}

// 回调函数：FMT本地速度
void FMTExternalFusion::fmt_vel_callback(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    fmt_state.velocity[0] = msg->twist.linear.x;
    fmt_state.velocity[1] = msg->twist.linear.y;
    fmt_state.velocity[2] = msg->twist.linear.z;
}

// 回调函数：FMT姿态
void FMTExternalFusion::fmt_att_callback(const sensor_msgs::Imu::ConstPtr &msg)
{
    fmt_state.attitude_q.x = msg->orientation.x;
    fmt_state.attitude_q.y = msg->orientation.y;
    fmt_state.attitude_q.z = msg->orientation.z;
    fmt_state.attitude_q.w = msg->orientation.w;

    // 转为rpy
    tf::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    fmt_state.attitude[0] = roll;
    fmt_state.attitude[1] = pitch;
    fmt_state.attitude[2] = yaw;
}

// 回调函数：FMT GPS卫星数量
void FMTExternalFusion::fmt_gps_satellites_callback(const std_msgs::UInt32::ConstPtr &msg)
{
    fmt_state.satellites = msg->data;
}

// 回调函数：FMT GPS状态
void FMTExternalFusion::fmt_gps_state_callback(const sensor_msgs::NavSatFix::ConstPtr &msg)
{
    fmt_state.gps_status = msg->status.status;
    fmt_state.gps_service = msg->status.service;
}

// 回调函数：FMT GPS原始数据
void FMTExternalFusion::fmt_gps_raw_callback(const mavros_msgs::GPSRAW::ConstPtr &msg)
{
    fmt_state.latitude = msg->lat;
    fmt_state.longitude = msg->lon;
    fmt_state.altitude = msg->alt;
}

// 回调函数：FMT位置设定值
void FMTExternalFusion::fmt_pos_target_callback(const mavros_msgs::PositionTarget::ConstPtr &msg)
{
    fmt_state.pos_setpoint[0] = msg->position.x;
    fmt_state.pos_setpoint[1] = msg->position.y;
    fmt_state.pos_setpoint[2] = msg->position.z;
    fmt_state.vel_setpoint[0] = msg->velocity.x;
    fmt_state.vel_setpoint[1] = msg->velocity.y;
    fmt_state.vel_setpoint[2] = msg->velocity.z;
}

// 回调函数：FMT姿态设定值
void FMTExternalFusion::fmt_att_target_callback(const mavros_msgs::AttitudeTarget::ConstPtr &msg)
{
    fmt_state.q_setpoint.x = msg->orientation.x;
    fmt_state.q_setpoint.y = msg->orientation.y;
    fmt_state.q_setpoint.z = msg->orientation.z;
    fmt_state.q_setpoint.w = msg->orientation.w;

    // 转为rpy
    tf::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    fmt_state.att_setpoint[0] = roll;
    fmt_state.att_setpoint[1] = pitch;
    fmt_state.att_setpoint[2] = yaw;

    fmt_state.thrust_setpoint = msg->thrust;
}

// 打印FMT状态
void FMTExternalFusion::show_fmt_state()
{
    Logger::print_color(int(LogColor::white_bg_blue), ">>>>>>>>>>>>>>>> fmt_external_fusion_node - [", uav_name, "] <<<<<<<<<<<<<<<<<");

    if(!fmt_state.connected)
    {
        Logger::print_color(int(LogColor::red), "FMT FCU:", "[ UNCONNECTED ]");
        Logger::print_color(int(LogColor::red), "Wait for FMT FCU connection...");
        return;
    }

    // 基本信息 - 连接状态、飞控模式、电池状态
    Logger::print_color(int(LogColor::white_bg_green), ">>>>> TOPIC: ~/sunray/px4_state (Sunray get from FMT via Mavros)");
    Logger::print_color(int(LogColor::green), "FMT FCU  : [ CONNECTED ]  BATTERY:", fmt_state.battery_state, "[V]", fmt_state.battery_percentage, "[%]");

    if (fmt_state.armed)
    {
        if(fmt_state.landed_state == 1)
        {
            Logger::print_color(int(LogColor::green), "FMT STATE: [ ARMED ]", LOG_GREEN, "[", fmt_state.mode, "]", LOG_GREEN, "[ ON_GROUND ]");   
        }else
        {
            Logger::print_color(int(LogColor::green), "FMT STATE: [ ARMED ]", LOG_GREEN, "[", fmt_state.mode, "]", LOG_GREEN, "[ IN_AIR ]");   
        }
    }
    else
    {
        if(fmt_state.landed_state == 1)
        {
            Logger::print_color(int(LogColor::red), "FMT STATE: [ DISARMED ]", LOG_GREEN, "[ ", fmt_state.mode, " ]", LOG_GREEN, "[ ON_GROUND ]");   
        }else
        {
            Logger::print_color(int(LogColor::red), "FMT STATE: [ DISARMED ]", LOG_GREEN, "[ ", fmt_state.mode, " ]", LOG_GREEN, "[ IN_AIR ]");   
        }
    }

    // 位置和姿态
    if (external_source != sunray_msgs::ExternalOdom::GPS && external_source != sunray_msgs::ExternalOdom::RTK)
    {
        // 无GPS模式的情况
        Logger::print_color(int(LogColor::blue), "FMT Local Position & Attitude:");
        Logger::print_color(int(LogColor::green), "POS_UAV [X Y Z]:",
                            fmt_state.position[0],
                            fmt_state.position[1],
                            fmt_state.position[2],
                            "[ m ]");
        Logger::print_color(int(LogColor::green), "VEL_UAV [X Y Z]:",
                            fmt_state.velocity[0],
                            fmt_state.velocity[1],
                            fmt_state.velocity[2],
                            "[m/s]");
        Logger::print_color(int(LogColor::green), "ATT_UAV [X Y Z]:",
                            fmt_state.attitude[0] / M_PI * 180,
                            fmt_state.attitude[1] / M_PI * 180,
                            fmt_state.attitude[2] / M_PI * 180,
                            "[deg]");
        Logger::print_color(int(LogColor::blue), "FMT Local Position Setpoint & Attitude Setpoint:");
        Logger::print_color(int(LogColor::green), "POS_SP [X Y Z]:",
                            fmt_state.pos_setpoint[0],
                            fmt_state.pos_setpoint[1],
                            fmt_state.pos_setpoint[2],
                            "[ m ]");
        Logger::print_color(int(LogColor::green), "VEL_SP [X Y Z]:",
                            fmt_state.vel_setpoint[0],
                            fmt_state.vel_setpoint[1],
                            fmt_state.vel_setpoint[2],
                            "[m/s]");   
        Logger::print_color(int(LogColor::green), "ATT_SP [X Y Z]:",
                            fmt_state.att_setpoint[0] / M_PI * 180,
                            fmt_state.att_setpoint[1] / M_PI * 180,
                            fmt_state.att_setpoint[2] / M_PI * 180,
                            "[deg]");       
        Logger::print_color(int(LogColor::green), "THRUST_SP :", fmt_state.thrust_setpoint*100, "[ % ]");
    }else
    {
        Logger::print_color(int(LogColor::blue), "GPS Status");
        Logger::print_color(int(LogColor::green), "GPS STATUS:", fmt_state.gps_status, "SERVICE:", fmt_state.gps_service);
        Logger::print_color(int(LogColor::green), "GPS SATS:", fmt_state.satellites);
        Logger::print_color(int(LogColor::green), "GPS POS[lat lon alt]:", int(fmt_state.latitude), int(fmt_state.longitude), int(fmt_state.altitude));
    }

    // 外部定位信息
    Logger::print_color(int(LogColor::white_bg_green), ">>>>> External Position Source:");

    switch (fmt_state.external_odom.external_source)
    {
        case sunray_msgs::ExternalOdom::ODOM:
            Logger::print_color(int(LogColor::green), "external_source: [ ODOM ]");
            break;
        case sunray_msgs::ExternalOdom::POSE:   
            Logger::print_color(int(LogColor::green), "external_source: [ POSE ]");
            break;
        case sunray_msgs::ExternalOdom::GAZEBO:
            Logger::print_color(int(LogColor::green), "external_source: [ GAZEBO ]");
            break;
        case sunray_msgs::ExternalOdom::MOCAP:
            Logger::print_color(int(LogColor::green), "external_source: [ MOCAP ]");
            break;
        case sunray_msgs::ExternalOdom::VIOBOT:
            Logger::print_color(int(LogColor::green), "external_source: [ VIOBOT ]");
            break;
        case sunray_msgs::ExternalOdom::GPS:
            Logger::print_color(int(LogColor::green), "external_source: [ GPS ]");
            break;
        default:
            Logger::print_color(int(LogColor::red), "external_source: [ UNKNOWN ]");
            break;
    }

    if (ext_pos.external_odom.odom_valid)
    {
        Logger::print_color(int(LogColor::green), "external_odom: [ VALID ]");
    }
    else
    {
        Logger::print_color(int(LogColor::red), "external_odom: [ INVALID ]");
    }

    Logger::print_color(int(LogColor::blue), "External Position Data:");

    Logger::print_color(int(LogColor::green), "EXT_POS [X Y Z]:",
                        ext_pos.external_odom.position[0],
                        ext_pos.external_odom.position[1],
                        ext_pos.external_odom.position[2],
                        "[ m ]");
    Logger::print_color(int(LogColor::green), "EXT_VEL [X Y Z]:",
                        ext_pos.external_odom.velocity[0],
                        ext_pos.external_odom.velocity[1],
                        ext_pos.external_odom.velocity[2],
                        "[m/s]");
    Logger::print_color(int(LogColor::green), "EXT_ATT [X Y Z]:",
                        ext_pos.external_odom.attitude[0] / M_PI * 180,
                        ext_pos.external_odom.attitude[1] / M_PI * 180,
                        ext_pos.external_odom.attitude[2] / M_PI * 180,
                        "[deg]");
}

