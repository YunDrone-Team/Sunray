#include "externalFusion.h"
#include "printf_format.h"
#include <signal.h>

using namespace std;
using namespace sunray_logger;

ExternalFusion::ExternalFusion()
{
    source_map[ODOM] = "ODOM";
    source_map[POSE] = "POSE";
    source_map[GAZEBO] = "GAZEBO";
    source_map[MOCAP] = "MOCAP";
    source_map[VIOBOT] = "VIOBOT";
    source_map[GPS] = "GPS";
    source_map[RTK] = "RTK";
    source_map[VINS] = "VINS";
}
void ExternalFusion::init(ros::NodeHandle &nh)
{
    nh_ = nh;
    double source_timeout, jump_threshold, pub_rate, task_rate;
    bool flag_printf, enable_rviz, check_jump;

    node_name = ros::this_node::getName();                                        // 【参数】节点名称
    nh.param<int>("uav_id", uav_id, 1);                                           // 【参数】无人机编号
    nh.param<int>("external_source", external_source, ODOM);                      // 【参数】外部定位数据来源
    nh.param<string>("uav_name", uav_name, "uav");                                // 【参数】无人机名称
    nh.param<string>("position_topic", source_topic, "/uav1/sunray/gazebo_pose"); // 【参数】外部定位数据来源
    nh.param<double>("source_timeout", source_timeout, 0.35);                     // 【参数】外部定位数据来源
    nh.param<double>("jump_threshold", jump_threshold, 0.1);                      // 【参数】外部定位数据跳变阈值
    nh.param<double>("pub_rate", pub_rate, 0.02);                                 // 【参数】发布到mavros的频率
    nh.param<double>("task_rate", task_rate, 0.05);                               // 【参数】定时任务的频率
    nh.param<bool>("check_jump", check_jump, false);                              // 【参数】是否检查外部定位数据跳变
    nh.param<bool>("listen_uav", listen_uav_state, true);                         // 【参数】是否监听无人机状态
    nh.param<bool>("flag_printf", flag_printf, true);                             // 【参数】是否打印日志
    nh.param<bool>("enable_rviz", enable_rviz, false);                            // 【参数】是否发布到rviz
    // nh.param<bool>("enable_rangeSensor", enable_rangeSensor, false);               // 【参数】是否使用rangeSensor

    uav_prefix = uav_name + std::to_string(uav_id);
    string topic_prefix = "/" + uav_prefix;

    if (source_map.find(external_source) != source_map.end())
    {
        external_position = factory.create(source_map[external_source]);
        if (source_map[external_source] == "MOCAP")
        {
            external_position->init(uav_id, uav_name, source_topic);
        }
        else
        {
            external_position->init(uav_id, uav_name, source_topic);
        }
        external_position->bindTopic(nh);
        Logger::info("external source: [", source_map[external_source], "]");
    }
    else
    {
        Logger::error("external source not found,external source id: [ ", external_source, " ]");
    }

    if (listen_uav_state || flag_printf)
    {
        px4_state_sub = nh.subscribe<mavros_msgs::State>(topic_prefix + "/mavros/state", 10, &ExternalFusion::px4_state_callback, this);
        px4_battery_sub = nh.subscribe<sensor_msgs::BatteryState>(topic_prefix + "/mavros/battery", 10, &ExternalFusion::px4_battery_callback, this);
        px4_odom_sub = nh.subscribe<nav_msgs::Odometry>(topic_prefix + "/mavros/local_position/odom", 10, &ExternalFusion::px4_odom_callback, this);
        px4_att_sub = nh.subscribe<sensor_msgs::Imu>(topic_prefix + "/mavros/imu/data", 10, &ExternalFusion::px4_att_callback, this);
    }

    if (enable_rviz)
    {
        // 【发布】无人机里程计,主要用于RVIZ显示
        uav_odom_pub = nh.advertise<nav_msgs::Odometry>(topic_prefix + "/sunray/uav_odom", 1);
        // 【发布】无人机运动轨迹,主要用于RVIZ显示
        uav_trajectory_pub = nh.advertise<nav_msgs::Path>(topic_prefix + "/sunray/uav_trajectory", 1);
        // 【发布】无人机位置(带图标),用于RVIZ显示
        uav_mesh_pub = nh.advertise<visualization_msgs::Marker>(topic_prefix + "/sunray/uav_mesh", 1);
        // 【定时器】定时发布 rviz显示,保证1Hz以上
        timer_rviz_pub = nh.createTimer(ros::Duration(0.1), &ExternalFusion::timer_rviz, this);
    }
    
    odom_state_pub = nh.advertise<std_msgs::Bool>(topic_prefix + "/sunray/odom_state", 10);
    // 定时任务
    timer_task = nh.createTimer(ros::Duration(0.2), &ExternalFusion::timer_callback, this);

    // 初始化变量
    px4_state.connected = false;
    px4_state.armed = false;
    px4_state.battery = 0;
    px4_state.battery_percentage = 0;
    px4_state.mode = "UNKNOWN";
    Logger::info("external fusion node init");
}

void ExternalFusion::px4_odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    px4_state.position_state.pos_x = msg->pose.pose.position.x;
    px4_state.position_state.pos_y = msg->pose.pose.position.y;
    px4_state.position_state.pos_z = msg->pose.pose.position.z;
    px4_state.position_state.vel_x = msg->twist.twist.linear.x;
    px4_state.position_state.vel_y = msg->twist.twist.linear.y;
    px4_state.position_state.vel_z = msg->twist.twist.linear.z;
}

void ExternalFusion::px4_state_callback(const mavros_msgs::State::ConstPtr &msg)
{
    px4_state.connected = msg->connected;
    px4_state.armed = msg->armed;
    px4_state.mode = msg->mode;
}

void ExternalFusion::px4_battery_callback(const sensor_msgs::BatteryState::ConstPtr &msg)
{
    px4_state.battery = msg->voltage;
    px4_state.battery_percentage = msg->percentage * 100;
}

void ExternalFusion::px4_att_callback(const sensor_msgs::Imu::ConstPtr &msg)
{
    px4_state.position_state.att_x = msg->orientation.x;
    px4_state.position_state.att_y = msg->orientation.y;
    px4_state.position_state.att_z = msg->orientation.z;
    px4_state.position_state.att_w = msg->orientation.w;

    // 转为rpy
    tf::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    px4_state.position_state.roll = roll;
    px4_state.position_state.pitch = pitch;
    px4_state.position_state.yaw = yaw;
}

void ExternalFusion::show_px4_state()
{
    Logger::print_color(int(LogColor::blue), LOG_BOLD, ">>>>>>>>>>>>>>", uav_prefix, "<<<<<<<<<<<<<<<");
    if (px4_state.connected)
    {
        Logger::print_color(int(LogColor::green), "CONNECTED:", "TRUE");
        if (px4_state.armed)
            Logger::print_color(int(LogColor::green), "MODE:", px4_state.mode, LOG_GREEN, "ARMED");
        else
            Logger::print_color(int(LogColor::green), "MODE:", px4_state.mode, LOG_RED, "DISARMED");
        Logger::print_color(int(LogColor::green), "BATTERY:", px4_state.battery, "[V]", px4_state.battery_percentage, "[%]");
    }
    else
        Logger::print_color(int(LogColor::red), "CONNECTED:", "FALSE");

    Logger::print_color(int(LogColor::blue), "EXTERNAL POS(send)");
    Logger::print_color(int(LogColor::green), "POS[X Y Z]:",
                        external_state.pos_x,
                        external_state.pos_y,
                        external_state.pos_z,
                        "[m]");
    Logger::print_color(int(LogColor::green), "VEL[X Y Z]:",
                        external_state.vel_x,
                        external_state.vel_y,
                        external_state.vel_z,
                        "[m/s]");
    Logger::print_color(int(LogColor::green), "ATT[X Y Z]:",
                        external_state.roll/M_PI*180,
                        external_state.pitch/M_PI*180,
                        external_state.yaw/M_PI*180,
                        "[deg]");

    if (px4_state.connected)
    {
        Logger::print_color(int(LogColor::blue), "PX4 POS(receive)");
        Logger::print_color(int(LogColor::green), "POS[X Y Z]:",
                            px4_state.position_state.pos_x,
                            px4_state.position_state.pos_y,
                            px4_state.position_state.pos_z,
                            "[m]");
        Logger::print_color(int(LogColor::green), "VEL[X Y Z]:",
                            px4_state.position_state.vel_x,
                            px4_state.position_state.vel_y,
                            px4_state.position_state.vel_z,
                            "[m/s]");
        Logger::print_color(int(LogColor::green), "ATT[X Y Z]:",
                            px4_state.position_state.roll/M_PI*180,
                            px4_state.position_state.pitch/M_PI*180,
                            px4_state.position_state.yaw/M_PI*180,
                            "[deg]");

        Logger::print_color(int(LogColor::blue), "ERR POS(send - receive)");
        Logger::print_color(int(LogColor::green), "POS[X Y Z]:",
                            err_state.pos_x,
                            err_state.pos_y,
                            err_state.pos_z,
                            "[m]");
        Logger::print_color(int(LogColor::green), "VEL[X Y Z]:",
                            err_state.vel_x,
                            err_state.vel_y,
                            err_state.vel_z,
                            "[m/s]");
        Logger::print_color(int(LogColor::green), "ATT[X Y Z]:",
                            err_state.roll/M_PI*180,
                            err_state.pitch/M_PI*180,
                            err_state.yaw/M_PI*180,
                            "[deg]");
    }
}

// 定时器回调函数
void ExternalFusion::timer_callback(const ros::TimerEvent &event)
{
    external_state.pos_x = external_position->position_state.px;
    external_state.pos_y = external_position->position_state.py;
    external_state.pos_z = external_position->position_state.pz;
    external_state.vel_x = external_position->position_state.vx;
    external_state.vel_y = external_position->position_state.vy;
    external_state.vel_z = external_position->position_state.vz;
    external_state.att_x = external_position->position_state.roll;
    external_state.roll = external_position->position_state.pitch;
    external_state.yaw = external_position->position_state.yaw;
    external_state.att_w = external_position->position_state.qw;
    external_state.att_x = external_position->position_state.qx;
    external_state.att_y = external_position->position_state.qy;
    external_state.att_z = external_position->position_state.qz;


    err_state.pos_x = external_state.pos_x - px4_state.position_state.pos_x;
    err_state.pos_y = external_state.pos_y - px4_state.position_state.pos_y;
    err_state.pos_z = external_state.pos_z - px4_state.position_state.pos_z;
    err_state.vel_x = external_state.vel_x - px4_state.position_state.vel_x;
    err_state.vel_y = external_state.vel_y - px4_state.position_state.vel_y;
    err_state.vel_z = external_state.vel_z - px4_state.position_state.vel_z;
    err_state.roll = external_state.roll - px4_state.position_state.roll;
    err_state.pitch = external_state.pitch - px4_state.position_state.pitch;
    err_state.yaw = external_state.yaw - px4_state.position_state.yaw;

    // 检查超时
    if (!external_position->position_state.valid)
    {
        Logger::warning("Warning: The external position is timeout!", external_position->position_state.timeout_count);
    }

    // 如果误差状态的绝对值大于阈值，则打印警告信息   只检查位置和偏航角
    if (abs(err_state.pos_x) > 0.1 ||
        abs(err_state.pos_y) > 0.1 ||
        abs(err_state.pos_z) > 0.1)
    // abs(err_state.yaw) > 10) // deg
    {
        Logger::warning("Warning: The error between external state and px4 state is too large!");
    }

    std_msgs::Bool msg;
    msg.data = external_position->position_state.valid;
    odom_state_pub.publish(msg);
}

void ExternalFusion::timer_rviz(const ros::TimerEvent &e)
{
    if (!external_position->position_state.valid)
    {
        return;
    }
    nav_msgs::Odometry uav_odom;
    // 发布无人机当前odometry(有些节点需要Odometry这个数据类型)
    uav_odom.header.stamp = ros::Time::now();
    uav_odom.header.frame_id = "world";
    uav_odom.child_frame_id = "base_link";
    uav_odom.pose.pose.position.x = external_state.pos_x;
    uav_odom.pose.pose.position.y = external_state.pos_y;
    uav_odom.pose.pose.position.z = external_state.pos_z;
    // 导航算法规定 高度不能小于0
    if (uav_odom.pose.pose.position.z <= 0)
    {
        uav_odom.pose.pose.position.z = 0.01;
    }
    uav_odom.pose.pose.orientation.x = external_state.att_x;
    uav_odom.pose.pose.orientation.y = external_state.att_y;
    uav_odom.pose.pose.orientation.z = external_state.att_z;
    uav_odom.pose.pose.orientation.w = external_state.att_w;
    uav_odom.twist.twist.linear.x = external_state.vel_x;
    uav_odom.twist.twist.linear.y = external_state.vel_y;
    uav_odom.twist.twist.linear.z = external_state.vel_z;
    uav_odom_pub.publish(uav_odom);

    // 发布无人机运动轨迹，用于rviz显示
    geometry_msgs::PoseStamped uav_pos;
    uav_pos.header.stamp = ros::Time::now();
    uav_pos.header.frame_id = "world";
    uav_pos.pose.position.x = external_state.pos_x;
    uav_pos.pose.position.y = external_state.pos_y;
    uav_pos.pose.position.z = external_state.pos_z;
    uav_pos.pose.orientation.x = external_state.att_x;
    uav_pos.pose.orientation.y = external_state.att_y;
    uav_pos.pose.orientation.z = external_state.att_z;
    uav_pos.pose.orientation.w = external_state.att_w;
    uav_pos_vector.insert(uav_pos_vector.begin(), uav_pos);
    if (uav_pos_vector.size() > 40)
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
    meshROS.pose.position.x = external_state.pos_x;
    meshROS.pose.position.y = external_state.pos_y;
    meshROS.pose.position.z = external_state.pos_z;
    meshROS.pose.orientation.w = external_state.att_w;
    meshROS.pose.orientation.x = external_state.att_x;
    meshROS.pose.orientation.y = external_state.att_y;
    meshROS.pose.orientation.z = external_state.att_z;
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
    tfs.header.frame_id = "world";       // 相对于世界坐标系
    tfs.header.stamp = ros::Time::now(); // 时间戳
    //  |----坐标系 ID
    tfs.child_frame_id = uav_name + "/lidar_link"; // 子坐标系，无人机的坐标系
    // tfs.child_frame_id = "/lidar_link"; //子坐标系，无人机的坐标系
    //  |----坐标系相对信息设置  偏移量  无人机相对于世界坐标系的坐标
    tfs.transform.translation.x = external_state.pos_x;
    tfs.transform.translation.y = external_state.pos_y;
    tfs.transform.translation.z = external_state.pos_z;
    //  |--------- 四元数设置
    tfs.transform.rotation.x = external_state.att_x;
    tfs.transform.rotation.y = external_state.att_y;
    tfs.transform.rotation.z = external_state.att_z;
    tfs.transform.rotation.w = external_state.att_w;
    //  |--------- 广播器发布数据
    broadcaster.sendTransform(tfs);

    // q_orig  是原姿态转换的tf的四元数
    // q_rot   旋转四元数
    // q_new   旋转后的姿态四元数
    tf2::Quaternion q_orig, q_rot, q_new;

    // commanded_pose.pose.orientation  这个比如说 是 订阅的别的节点的topic 是一个  姿态的 msg 四元数
    // 通过tf2::convert()  转换成 tf 的四元数
    tf2::convert(tfs.transform.rotation, q_orig);

    // 设置 绕 x 轴 旋转180度
    double r = -1.57, p = 0, y = -1.57;
    q_rot.setRPY(r, p, y); // 求得 tf 的旋转四元数

    q_new = q_orig * q_rot; // 通过 姿态的四元数 乘以旋转的四元数 即为 旋转 后的  四元数
    q_new.normalize();      // 归一化

    //  将 旋转后的 tf 四元数 转换 为 msg 四元数
    tf2::convert(q_new, tfs.transform.rotation);
    tfs.child_frame_id = uav_name + "/camera_link"; // 子坐标系，无人机的坐标系
    //  |--------- 广播器发布数据
    broadcaster.sendTransform(tfs);
}

void mySigintHandler(int sig)
{
    ROS_INFO("[external_fusion_node] exit...");
    ros::shutdown();
    exit(0);
}

int main(int argc, char **argv)
{

    Logger::init_default();
    Logger::setPrintLevel(false);
    Logger::setPrintTime(false);
    Logger::setPrintToFile(false);
    Logger::setFilename("/home/yundrone/Sunray/General_Module/sunray_uav_control/test/log.txt");

    ros::init(argc, argv, "external_fusion");
    ros::NodeHandle nh("~");
    ros::Rate rate(50);

    signal(SIGINT, mySigintHandler);
    ExternalFusion external_fusion;
    external_fusion.init(nh);
    ros::Time now = ros::Time::now();
    while (ros::ok)
    {
        ros::spinOnce();
        if( ros::Time::now() - now > ros::Duration(1.0))
        {
            external_fusion.show_px4_state();
            now = ros::Time::now();
        }
        
        rate.sleep();
    }

    return 0;
}