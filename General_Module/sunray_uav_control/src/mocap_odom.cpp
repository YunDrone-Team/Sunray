#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>

void callback(const geometry_msgs::PoseStamped::ConstPtr& pose, 
              const geometry_msgs::TwistStamped::ConstPtr& twist,  ros::Publisher &odom_pub) {

    nav_msgs::Odometry odom_msg;
        
    // 设置header
    odom_msg.header.stamp = pose->header.stamp;
    odom_msg.header.frame_id = pose->header.frame_id;
    odom_msg.child_frame_id = "base_link";
    
    // 填充pose和twist信息
    odom_msg.pose.pose = pose->pose;
    odom_msg.twist.twist = twist->twist;
    
    // 发布融合后的消息
    odom_pub.publish(odom_msg);
}

int main(int argc, char** argv) {
    
    ros::init(argc, argv, "mocap_odom_node");
    ros::NodeHandle nh("~");
    
    // 读取参数
    std::string pose_topic, twist_topic, odom_topic;
    int queue_size = 10;
    
    nh.param("pose_topic", pose_topic, std::string("/vrpn_client_node_2/uav2/pose"));
    nh.param("twist_topic", twist_topic, std::string("/vrpn_client_node_2/uav2/twist"));
    nh.param("odom_topic", odom_topic, std::string("/mocap/odometry"));
    nh.param("queue_size", queue_size, 1000);
    
    // 创建发布者
     ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>(odom_topic, 10);
    
    // 创建订阅器
    message_filters::Subscriber<geometry_msgs::PoseStamped> pose_sub(nh, pose_topic, queue_size);
    message_filters::Subscriber<geometry_msgs::TwistStamped> twist_sub(nh, twist_topic, queue_size);
    typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped, geometry_msgs::TwistStamped> SyncPolicy;
    message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(queue_size), pose_sub, twist_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2, boost::ref(odom_pub)));
    
    ros::spin();
    return 0;
}