#include "VioBot.h"

Viobot::Viobot()
{
    // ��ʼ����ز���
    is_viobot_start = false;
    algo_set.algo_enable = false;
    algo_set.algo_reboot = false;
    algo_set.algo_reset = false;
    odom_timeout = false;
    calculation_done = false;
}

void Viobot::init(ros::NodeHandle &nh)
{
    // ��ȡ����
    nh.param<string>("uart_name", uart_name, "/dev/ttyS0");
    nh.param<int>("baudrate", baudrate, 115200);
    nh.param<bool>("viobot_tilted", tilted, false);

    // ��ʼ������Ͷ�ʱ��
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
        imu_sub_.shutdown(); // ע��������
        return;
    }

    // �洢���ٶȼ�����
    ax_values.push_back(msg->linear_acceleration.z);
    ay_values.push_back(-msg->linear_acceleration.x);
    az_values.push_back(msg->linear_acceleration.y);

    // ���ռ����㹻������ʱ����ƽ��ֵ�ͽǶ�
    if (ax_values.size() >= AVERAGE_COUNT)
    {
        // ����ƽ��ֵ
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

        // ������ת�����ŷ����
        Eigen::Vector3d vectorBefore(avg_ax, avg_ay, avg_az);
        vectorBefore.normalize();
        Eigen::Vector3d vectorAfter(0, 0, -1);
        eigen_q_rot = Eigen::Quaterniond::FromTwoVectors(vectorBefore, vectorAfter);

        // ��ֵ
        q_rot = tf2::Quaternion(eigen_q_rot.x(), eigen_q_rot.y(), eigen_q_rot.z(), eigen_q_rot.w());
        tf2::Matrix3x3(q_rot).getRPY(rot_roll, rot_pitch, rot_yaw);

        // ��ӡ���
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

    mavlink_odom.frame_id = MAV_FRAME_LOCAL_FRD;
    mavlink_odom.child_frame_id = MAV_FRAME_LOCAL_FRD;
    mavlink_odom.estimator_type = MAV_ESTIMATOR_TYPE_VISION;
    mavlink_odom.time_usec = msg->header.stamp.toSec() * 1000;

    Eigen::Vector3d p = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    mavlink_odom.x = p.y();
    mavlink_odom.y = p.x();
    mavlink_odom.z = -p.z();

    tf2::Quaternion q;
    q.setW(msg->pose.pose.orientation.w);
    q.setX(msg->pose.pose.orientation.x);
    q.setY(msg->pose.pose.orientation.y);
    q.setZ(msg->pose.pose.orientation.z);

    // �Ȱ����ݴ�����
    // �� Z ����ת 90��
    tf2::Quaternion q_z;
    q_z.setRPY(0, 0, M_PI / 2); // M_PI/2 = 90��

    // �� Y ����ת -90��
    tf2::Quaternion q_y;
    q_y.setRPY(0, -M_PI / 2, 0); // -M_PI/2 = -90��

    // �����ת��˳���� q_z���� q_y��
    q = q * q_z * q_y;

    // if (tilted)
    // {
    //     q = q * q_rot;
    // }

    // double n_roll, n_pitch, n_yaw;
    // tf2::Matrix3x3(q).getRPY(n_roll, n_pitch, n_yaw);
    // printf("Odometry Euler Angles: Roll: %f, Pitch: %f, Yaw: %f\n", n_roll * 180 / M_PI, n_pitch * 180 / M_PI, n_yaw * 180 / M_PI);

    Eigen::Quaterniond mav_q(q.getW(), q.getX(), q.getY(), q.getZ());
    Eigen::Quaterniond ql(Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));
    Eigen::Quaterniond qr(Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));

    mav_q = ql * mav_q *  qr;

    mavlink_odom.q[0] = mav_q.w();
    mavlink_odom.q[1] = mav_q.x();
    mavlink_odom.q[2] = mav_q.y();
    mavlink_odom.q[3] = mav_q.z();

    // mavlink_odom.vx = msg->twist.twist.linear.z;
    // mavlink_odom.vy = -msg->twist.twist.linear.x;
    // mavlink_odom.vz = -msg->twist.twist.linear.y;

    // TODO: �����ٶ��ں�����
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

    // �����ݸ��Ƶ�viobot_state
    viobot_state.position[0] = p.x();
    viobot_state.position[1] = p.y();
    viobot_state.position[2] = p.z();

    viobot_state.velocity[0] = mavlink_odom.vx;
    viobot_state.velocity[1] = mavlink_odom.vy;
    viobot_state.velocity[2] = mavlink_odom.vz;

    viobot_state.attitude_q.w = q.getW();
    viobot_state.attitude_q.x = q.getX();
    viobot_state.attitude_q.y = q.getY();
    viobot_state.attitude_q.z = q.getZ();

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
        // �㷨��ʼ��
        is_viobot_start = true;
    }

    viobot_state.vio_start = is_viobot_start;

    if (is_viobot_start == false) // ���û�п����㷨���Զ�����
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


// void Viobot::odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
// {
//     if (!calculation_done)
//         return;

//     memset(&mavlink_odom, 0, sizeof(mavlink_odometry_t));

//     mavlink_odom.frame_id = MAV_FRAME_LOCAL_FLU ;
//     mavlink_odom.child_frame_id = MAV_FRAME_LOCAL_FLU;
//     mavlink_odom.estimator_type = MAV_ESTIMATOR_TYPE_VISION;
//     mavlink_odom.time_usec = msg->header.stamp.toSec() * 1000;

//     Eigen::Vector3d p = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
//     mavlink_odom.x = p.x();
//     mavlink_odom.y = p.y();
//     mavlink_odom.z = p.z();

//     tf2::Quaternion q;
//     q.setW(msg->pose.pose.orientation.w);
//     q.setX(msg->pose.pose.orientation.x);
//     q.setY(msg->pose.pose.orientation.y);
//     q.setZ(msg->pose.pose.orientation.z);

//     // �Ȱ����ݴ�����FLU
//     // �� Z ����ת 90��
//     tf2::Quaternion q_z;
//     q_z.setRPY(0, 0, M_PI / 2); // M_PI/2 = 90��

//     // �� Y ����ת -90��
//     tf2::Quaternion q_y;
//     q_y.setRPY(0, -M_PI / 2, 0); // -M_PI/2 = -90��

//     // �����ת��˳���� q_z���� q_y��
//     q = q * q_z * q_y;

//     double roll, pitch, yaw;
//     tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
//     // // ��ӡŷ����
//     // printf("Original Euler Angles: Roll: %f, Pitch: %f, Yaw: %f\n", roll * 180 / M_PI, pitch * 180 / M_PI, yaw * 180 / M_PI);

//     // ������FRD
//     q.setRPY(roll, -pitch, -yaw);

//     // ������ʼ���(��������)
//     if (tilted)
//     {
//         q = q * q_rot;
//     }

//     // // ��ӡ��Ŀǰ���������̼���̬
//     // double n_roll, n_pitch, n_yaw;
//     // tf2::Matrix3x3(q).getRPY(n_roll, n_pitch, n_yaw);
//     // printf("Odometry Euler Angles: Roll: %f, Pitch: %f, Yaw: %f\n", n_roll * 180 / M_PI, n_pitch * 180 / M_PI, n_yaw * 180 / M_PI);

//     mavlink_odom.q[0] = q.w();
//     mavlink_odom.q[1] = q.x();
//     mavlink_odom.q[2] = q.y();
//     mavlink_odom.q[3] = q.z();

//     // mavlink_odom.vx = msg->twist.twist.linear.z;
//     // mavlink_odom.vy = -msg->twist.twist.linear.x;
//     // mavlink_odom.vz = -msg->twist.twist.linear.y;

//     // TODO: �����ٶ��ں�����
//     mavlink_odom.vx = NAN;
//     mavlink_odom.vy = NAN;
//     mavlink_odom.vz = NAN;

//     mavlink_odom.rollspeed = NAN;
//     mavlink_odom.pitchspeed = NAN;
//     mavlink_odom.yawspeed = NAN;

//     for (int i = 0; i < 21; i++)
//     {

//         mavlink_odom.pose_covariance[i] = NAN;
//         mavlink_odom.velocity_covariance[i] = NAN;
//     }

//     // �����ݸ��Ƶ�viobot_state
//     viobot_state.position[0] = mavlink_odom.x;
//     viobot_state.position[1] = mavlink_odom.y;
//     viobot_state.position[2] = mavlink_odom.z;

//     viobot_state.velocity[0] = mavlink_odom.vx;
//     viobot_state.velocity[1] = mavlink_odom.vy;
//     viobot_state.velocity[2] = mavlink_odom.vz;

//     viobot_state.attitude_q.w = mavlink_odom.q[0];
//     viobot_state.attitude_q.x = mavlink_odom.q[1];
//     viobot_state.attitude_q.y = mavlink_odom.q[2];
//     viobot_state.attitude_q.z = mavlink_odom.q[3];

//     viobot_state.header.stamp = ros::Time::now();
// }
