#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <signal.h>
#include "sunray_log.hpp"

typedef message_filters::sync_policies::ExactTime<geometry_msgs::PoseStamped, geometry_msgs::TwistStamped> SyncPolicy;

ros::Publisher mocap_odom_pub;

// 中断信号
void MySigintHandler(int sig) {

    ros::shutdown();
}

void SyncCallback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg,
                  const geometry_msgs::TwistStamped::ConstPtr& twist_msg) {

    nav_msgs::Odometry odom_msg;

    odom_msg.header.stamp = pose_msg->header.stamp;
    odom_msg.header.frame_id = "world";
    odom_msg.child_frame_id = "base_link";

    // Pose
    odom_msg.pose.pose.position.x = pose_msg->pose.position.x;
    odom_msg.pose.pose.position.y = pose_msg->pose.position.y;
    odom_msg.pose.pose.position.z = pose_msg->pose.position.z;
    odom_msg.pose.pose.orientation.x = pose_msg->pose.orientation.x;
    odom_msg.pose.pose.orientation.y = pose_msg->pose.orientation.y;
    odom_msg.pose.pose.orientation.z = pose_msg->pose.orientation.z;
    odom_msg.pose.pose.orientation.w = pose_msg->pose.orientation.w;

    // Twist
    odom_msg.twist.twist.linear.x = twist_msg->twist.linear.x;
    odom_msg.twist.twist.linear.y = twist_msg->twist.linear.y;
    odom_msg.twist.twist.linear.z = twist_msg->twist.linear.z;
    odom_msg.twist.twist.angular.x = twist_msg->twist.angular.x;
    odom_msg.twist.twist.angular.y = twist_msg->twist.angular.y;
    odom_msg.twist.twist.angular.z = twist_msg->twist.angular.z;

    mocap_odom_pub.publish(odom_msg);
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "mocap_odom_node");

    ros::NodeHandle nh;

    // 中断信号注册
    signal(SIGINT, MySigintHandler);

    int uav_id = 0;
    nh.param<int>("uav_id", uav_id, 1);

    std::string odom_pub_topic = "/uav" + std::to_string(uav_id) + "/sunray/odometry";
    mocap_odom_pub = nh.advertise<nav_msgs::Odometry>(odom_pub_topic, 10);

    std::string pose_sub_topic = "/vrpn_client_node/uav" + std::to_string(uav_id) + "/pose";
    std::string twist_sub_topic = "/vrpn_client_node/uav" + std::to_string(uav_id) + "/twist";
    message_filters::Subscriber<geometry_msgs::PoseStamped> mocap_pose_sub(nh, pose_sub_topic, 10);
    message_filters::Subscriber<geometry_msgs::TwistStamped> mocap_twist_sub(nh, twist_sub_topic, 10);

    message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), mocap_pose_sub, mocap_twist_sub);
    sync.registerCallback(boost::bind(&SyncCallback, _1, _2));

    SUNRAY_INFO("Subscribe pose topic: {}", pose_sub_topic);
    SUNRAY_INFO("Subscribe twist topic: {}", twist_sub_topic);
    SUNRAY_INFO("Publish odometry topic: {}", odom_pub_topic);

    ros::spin();
    return 0;
}
