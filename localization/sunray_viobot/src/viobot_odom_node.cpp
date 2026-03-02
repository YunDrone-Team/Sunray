#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <signal.h>
#include "sunray_log.hpp"

ros::Publisher viobot_odom_pub;

// 中断信号
void MySigintHandler(int sig) {

    ros::shutdown();
}

void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg) {

    nav_msgs::Odometry odom_msg = *msg;

    tf2::Quaternion q;
    q.setW(msg->pose.pose.orientation.w);
    q.setX(msg->pose.pose.orientation.x);
    q.setY(msg->pose.pose.orientation.y);
    q.setZ(msg->pose.pose.orientation.z);

    // 绕 Z 轴旋转 90°
    tf2::Quaternion q_z;
    q_z.setRPY(0, 0, M_PI / 2);

    // 绕 Y 轴旋转 -90°
    tf2::Quaternion q_y;
    q_y.setRPY(0, -M_PI / 2, 0);

    // 组合旋转（顺序：先 q_z，再 q_y）
    q = q * q_z * q_y;

    // 发布里程计消息
    odom_msg.pose.pose.position.x = msg->pose.pose.position.x;
    odom_msg.pose.pose.position.y = msg->pose.pose.position.y;
    odom_msg.pose.pose.position.z = msg->pose.pose.position.z;
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();
    viobot_odom_pub.publish(odom_msg);
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "viobot_odom_node");

    ros::NodeHandle nh;

    // 中断信号注册
    signal(SIGINT, MySigintHandler);

    int uav_id = 0;
    nh.param<int>("uav_id", uav_id, 1);
    std::string odom_pub_topic = "/uav" + std::to_string(uav_id) + "/sunray/odometry";
    viobot_odom_pub = nh.advertise<nav_msgs::Odometry>(odom_pub_topic, 10);

    std::string odom_sub_topic = "/baton/stereo3/odometry";
    ros::Subscriber viobot_odom_sub = nh.subscribe(odom_sub_topic, 10, OdomCallback);

    SUNRAY_INFO("Subscribe odometry topic: {}", odom_sub_topic);
    SUNRAY_INFO("Publish odometry topic: {}", odom_pub_topic);

    ros::spin();
    return 0;
}
