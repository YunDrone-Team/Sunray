#ifndef DATA_CONVERT_HPP
#define DATA_CONVERT_HPP

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <common/mavlink.h>

static void odometry_msg_convert(const nav_msgs::OdometryPtr &msg,mavlink_odometry_t &mavlink_odom){
    mavlink_odom.time_usec = msg->header.stamp.toNSec() * 1000; 
    mavlink_odom.x = msg->pose.pose.position.x;
    mavlink_odom.y = msg->pose.pose.position.y;
    mavlink_odom.z = msg->pose.pose.position.z;
    mavlink_odom.q[0] = msg->pose.pose.orientation.w;
    mavlink_odom.q[1] = msg->pose.pose.orientation.x;
    mavlink_odom.q[2] = msg->pose.pose.orientation.y;   
    mavlink_odom.q[3] = msg->pose.pose.orientation.z;
    mavlink_odom.vx = msg->twist.twist.linear.x;
    mavlink_odom.vy = msg->twist.twist.linear.y;
    mavlink_odom.vz = msg->twist.twist.linear.z;
    mavlink_odom.rollspeed = msg->twist.twist.angular.x;
    mavlink_odom.pitchspeed = msg->twist.twist.angular.y;
    mavlink_odom.yawspeed = msg->twist.twist.angular.z;
}
    

#endif 