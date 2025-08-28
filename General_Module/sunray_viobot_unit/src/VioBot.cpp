#include "VioBot.h"

Viobot::Viobot()
{
    // 初始化相关参数
    is_viobot_start = false;
    algo_set.algo_enable = false;
    algo_set.algo_reboot = false;
    algo_set.algo_reset = false;
    odom_timeout = false;
    calculation_done = false;
}

void Viobot::init(ros::NodeHandle &nh)
{
    // 读取参数
    nh.param<string>("uart_name", uart_name, "/dev/ttyS0");
    nh.param<int>("baudrate", baudrate, 115200);
    nh.param<bool>("viobot_tilted", tilted, true);

    // 初始化话题和定时器
    imu_sub_ = nh.subscribe("/baton/imu", 10, &Viobot::imuCallback, this);
    odom_sub_ = nh.subscribe("/baton/stereo3/odometry", 2, &Viobot::odomCallback, this);
    algo_status_sub_ = nh.subscribe<sunray_viobot_unit::algo_status>("/baton/algo_status", 2, &Viobot::algoStatusCallback, this);

    algo_ctrl_pub_ = nh.advertise<sunray_viobot_unit::algo_ctrl>("/baton/stereo3_ctrl", 2);
    viobot_state_pub_ = nh.advertise<sunray_msgs::ViobotState>("/sunray_viobot/ViobotState", 10);

    check_timeout_timer_ = nh.createTimer(ros::Duration(0.05), &Viobot::timerCheckTimeoutCallback, this);
}

void Viobot::imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
    if (calculation_done)
    {
        imu_sub_.shutdown(); // 注销订阅者
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

void Viobot::odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    if (!calculation_done)
        return;

    memset(&mavlink_odom, 0, sizeof(mavlink_odometry_t));

    mavlink_odom.frame_id = MAV_FRAME_LOCAL_FLU;
    mavlink_odom.child_frame_id = MAV_FRAME_LOCAL_FLU;
    mavlink_odom.estimator_type = MAV_ESTIMATOR_TYPE_VISION;
    mavlink_odom.time_usec = msg->header.stamp.toSec() * 1000;

    Eigen::Vector3d p = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    mavlink_odom.x = p.x();
    mavlink_odom.y = p.y();
    mavlink_odom.z = p.z();

    tf2::Quaternion q;
    q.setW(msg->pose.pose.orientation.w);
    q.setX(msg->pose.pose.orientation.x);
    q.setY(msg->pose.pose.orientation.y);
    q.setZ(msg->pose.pose.orientation.z);

    // 先把数据处理成FLU
    // 绕 Z 轴旋转 90°
    tf2::Quaternion q_z;
    q_z.setRPY(0, 0, M_PI / 2); // M_PI/2 = 90°

    // 绕 Y 轴旋转 -90°
    tf2::Quaternion q_y;
    q_y.setRPY(0, -M_PI / 2, 0); // -M_PI/2 = -90°

    // 组合旋转（顺序：先 q_z，再 q_y）
    q = q * q_z * q_y;

    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    // // 打印欧拉角
    // printf("Original Euler Angles: Roll: %f, Pitch: %f, Yaw: %f\n", roll * 180 / M_PI, pitch * 180 / M_PI, yaw * 180 / M_PI);

    // 处理成FRD
    q.setRPY(roll, -pitch, -yaw);

    // 处理初始倾角(重力对齐)
    if (tilted)
    {
        q = q * q_rot;
    }

    // // 打印出目前处理后的里程计姿态
    // double n_roll, n_pitch, n_yaw;
    // tf2::Matrix3x3(q).getRPY(n_roll, n_pitch, n_yaw);
    // printf("Odometry Euler Angles: Roll: %f, Pitch: %f, Yaw: %f\n", n_roll * 180 / M_PI, n_pitch * 180 / M_PI, n_yaw * 180 / M_PI);

    mavlink_odom.q[0] = q.w();
    mavlink_odom.q[1] = q.x();
    mavlink_odom.q[2] = q.y();
    mavlink_odom.q[3] = q.z();

    // mavlink_odom.vx = msg->twist.twist.linear.z;
    // mavlink_odom.vy = -msg->twist.twist.linear.x;
    // mavlink_odom.vz = -msg->twist.twist.linear.y;

    // TODO: 处理速度融合问题
    mavlink_odom.vx = NAN;
    mavlink_odom.vy = NAN;
    mavlink_odom.vz = NAN;

    mavlink_odom.rollspeed = NAN;
    mavlink_odom.pitchspeed = NAN;
    mavlink_odom.yawspeed = NAN;

    for (int i = 0; i < 21; i++)
    {

        mavlink_odom.pose_covariance[i] = NAN;
        mavlink_odom.velocity_covariance[i] = NAN;
    }

    // 把数据复制到viobot_state
    viobot_state.position[0] = mavlink_odom.x;
    viobot_state.position[1] = mavlink_odom.y;
    viobot_state.position[2] = mavlink_odom.z;

    viobot_state.velocity[0] = mavlink_odom.vx;
    viobot_state.velocity[1] = mavlink_odom.vy;
    viobot_state.velocity[2] = mavlink_odom.vz;

    viobot_state.attitude_q.w = mavlink_odom.q[0];
    viobot_state.attitude_q.x = mavlink_odom.q[1];
    viobot_state.attitude_q.y = mavlink_odom.q[2];
    viobot_state.attitude_q.z = mavlink_odom.q[3];

    viobot_state.header.stamp = ros::Time::now();
}

void Viobot::algoStatusCallback(const sunray_viobot_unit::algo_status::ConstPtr &msg)
{
    viobot_state.algo_status = msg->algo_status;
    if (msg->algo_status == "ready")
    {
        is_viobot_start = false;
    }
    else if (msg->algo_status == "stereo3_initializing" || msg->algo_status == "stereo3_running")
    {
        // 算法初始化
        is_viobot_start = true;
    }

    viobot_state.vio_start = is_viobot_start;

    if (is_viobot_start == false) // 如果没有开启算法，自动开启
    {
        algo_set.algo_enable = true;
        algo_ctrl_pub_.publish(algo_set);
    }
}

void Viobot::timerCheckTimeoutCallback(const ros::TimerEvent &event)
{
    odom_timeout = (ros::Time::now() - viobot_state.header.stamp).toSec() > ODOM_TIMEOUT;
    viobot_state.odom_valid = !odom_timeout;

    viobot_state_pub_.publish(viobot_state);
}
