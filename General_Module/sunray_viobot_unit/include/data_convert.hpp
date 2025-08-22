#ifndef DATA_CONVERT_HPP
#define DATA_CONVERT_HPP

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <common/mavlink.h>
#include <Eigen/Eigen>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

static void odometry_msg_convert(const nav_msgs::OdometryPtr &msg, mavlink_odometry_t &mavlink_odom)
{
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

    // 加上转
    //  绕 Z 轴旋转 90°
    tf2::Quaternion q_z;
    q_z.setRPY(0, 0, M_PI / 2); // M_PI/2 = 90°

    // 绕 Y 轴旋转 -90°
    tf2::Quaternion q_y;
    q_y.setRPY(0, -M_PI / 2, 0); // -M_PI/2 = -90°

    // 组合旋转（顺序：先 q_z，再 q_y）
    q = q * q_z * q_y;

    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    // 打印欧拉角
    // printf("Original Euler Angles: Roll: %f, Pitch: %f, Yaw: %f\n", roll * 180 / M_PI, pitch * 180 / M_PI, yaw * 180 / M_PI);

    q.setRPY(0, 0, -yaw);

    double n_roll, n_pitch, n_yaw;
    tf2::Matrix3x3(q).getRPY(n_roll, n_pitch, n_yaw);
    // printf("Odometry Euler Angles: Roll: %f, Pitch: %f, Yaw: %f\n", n_roll * 180 / M_PI, n_pitch * 180 / M_PI, n_yaw * 180 / M_PI);

    mavlink_odom.q[0] = q.w();
    mavlink_odom.q[1] = q.x();
    mavlink_odom.q[2] = q.y();
    mavlink_odom.q[3] = q.z();

    // mavlink_odom.vx = msg->twist.twist.linear.z;
    // mavlink_odom.vy = -msg->twist.twist.linear.x;
    // mavlink_odom.vz = -msg->twist.twist.linear.y;

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
}

#endif