#ifndef EXTERNALPOSITION_H
#define EXTERNALPOSITION_H

#include "ros_msg_utils.h"

class ExternalPosition
{
public:
    ExternalPosition()
    {
    }

    // 声明一个自定义话题 - sunray_msgs::ExternalOdom
    sunray_msgs::ExternalOdom external_odom;

    void init(ros::NodeHandle &nh, std::string source_topic = "Odometry", int external_source = 0)
    {
        // 初始化参数
        nh.param<int>("uav_id", uav_id, 1);
        nh.param<std::string>("uav_name", uav_name, "uav");
        source_topic_name = source_topic;

        std::string topic_prefix = "/" + uav_name + std::to_string(uav_id);

        // 【发布】外部定位状态 - 本节点 -> uav_control_node
        odom_state_pub = nh.advertise<sunray_msgs::ExternalOdom>(topic_prefix + "/sunray/external_odom_state", 10);
        timer_pub_odom_state = nh.createTimer(ros::Duration(0.05), &ExternalPosition::timerCallback, this);
        
        // 初始化外部定位状态
        external_odom.header.stamp = ros::Time::now();
        external_odom.external_source = external_source;
        external_odom.position[0] = -0.01;
        external_odom.position[1] = -0.01;
        external_odom.position[2] = -0.01;
        external_odom.velocity[0] = 0.0;
        external_odom.velocity[1] = 0.0;
        external_odom.velocity[2] = 0.0;
        external_odom.attitude_q.x = 0;
        external_odom.attitude_q.y = 0;
        external_odom.attitude_q.z = 0;
        external_odom.attitude_q.w = 1;
        external_odom.attitude[0] = 0.0;
        external_odom.attitude[1] = 0.0;
        external_odom.attitude[2] = 0.0;

        switch (external_source)
        {
        case sunray_msgs::ExternalOdom::ODOM:
            odom_sub = nh.subscribe<nav_msgs::Odometry>(source_topic_name, 10, &ExternalPosition::OdomCallback, this);
            break;
        case sunray_msgs::ExternalOdom::POSE:
            pos_sub = nh.subscribe<geometry_msgs::PoseStamped>(source_topic_name, 10, &ExternalPosition::PosCallback, this);
            break;
        case sunray_msgs::ExternalOdom::GAZEBO:
            source_topic = topic_prefix + "/sunray/gazebo_pose";
            odom_sub = nh.subscribe<nav_msgs::Odometry>(source_topic_name, 10, &ExternalPosition::OdomCallback, this);
            break;
        case sunray_msgs::ExternalOdom::MOCAP:
            // 【订阅】动捕的定位数据(坐标系:动捕系统惯性系) vrpn -> 本节点
            pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node_" + std::to_string(uav_id) + topic_prefix + "/pose", 1, &ExternalPosition::PosCallback, this);
            // 【订阅】动捕的定位数据(坐标系:动捕系统惯性系) vrpn -> 本节点
            vel_sub = nh.subscribe<geometry_msgs::TwistStamped>("/vrpn_client_node_" + std::to_string(uav_id) + topic_prefix + "/twist", 1, &ExternalPosition::VelCallback, this);
            break;
        case sunray_msgs::ExternalOdom::GPS:
            source_topic = topic_prefix + "/mavros/global_position/local";
            odom_sub = nh.subscribe<nav_msgs::Odometry>(source_topic_name, 10, &ExternalPosition::OdomCallback, this);
            break;
        default:
            // Logger::error("Unknown external position source type: ", type);
            break;
        }
    }

    // 实现外部定位源话题回调函数
    void OdomCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        // 四元素转rpy
        tf2::Quaternion quaternion;
        tf2::fromMsg(msg->pose.pose.orientation, quaternion);
        double roll, pitch, yaw;
        tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
        external_odom.header.stamp = ros::Time::now();
        external_odom.position[0] = msg->pose.pose.position.x;
        external_odom.position[1] = msg->pose.pose.position.y;
        external_odom.position[2] = msg->pose.pose.position.z;
        external_odom.velocity[0] = msg->twist.twist.linear.x;
        external_odom.velocity[1] = msg->twist.twist.linear.y;
        external_odom.velocity[2] = msg->twist.twist.linear.z;
        external_odom.attitude_q.x = msg->pose.pose.orientation.x;
        external_odom.attitude_q.y = msg->pose.pose.orientation.y;
        external_odom.attitude_q.z = msg->pose.pose.orientation.z;
        external_odom.attitude_q.w = msg->pose.pose.orientation.w;
        external_odom.attitude[0] = roll;
        external_odom.attitude[1] = pitch;
        external_odom.attitude[2] = yaw;
    }

    // 实现外部定位源话题回调函数
    void PosCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        // 四元素转rpy
        tf2::Quaternion quaternion;
        tf2::fromMsg(msg->pose.orientation, quaternion);
        double roll, pitch, yaw;
        tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
        external_odom.header.stamp = ros::Time::now();
        external_odom.position[0] = msg->pose.position.x;
        external_odom.position[1] = msg->pose.position.y;
        external_odom.position[2] = msg->pose.position.z;
        external_odom.attitude_q.x = msg->pose.orientation.x;
        external_odom.attitude_q.y = msg->pose.orientation.y;
        external_odom.attitude_q.z = msg->pose.orientation.z;
        external_odom.attitude_q.w = msg->pose.orientation.w;
        external_odom.attitude[0] = roll;
        external_odom.attitude[1] = pitch;
        external_odom.attitude[2] = yaw;
    }

    // Gps模式下，实际接收的是mavros/global_position/local
    void GpsOdomCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        // 四元素转rpy
        tf2::Quaternion quaternion;
        tf2::fromMsg(msg->pose.pose.orientation, quaternion);
        double roll, pitch, yaw;
        tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
        external_odom.header.stamp = ros::Time::now();
        external_odom.position[0] = msg->pose.pose.position.x;
        external_odom.position[1] = msg->pose.pose.position.y;
        external_odom.position[2] = msg->pose.pose.position.z;
        external_odom.velocity[0] = msg->twist.twist.linear.x;
        external_odom.velocity[1] = msg->twist.twist.linear.y;
        external_odom.velocity[2] = msg->twist.twist.linear.z;
        external_odom.attitude_q.x = msg->pose.pose.orientation.x;
        external_odom.attitude_q.y = msg->pose.pose.orientation.y;
        external_odom.attitude_q.z = msg->pose.pose.orientation.z;
        external_odom.attitude_q.w = msg->pose.pose.orientation.w;
        external_odom.attitude[0] = roll;
        external_odom.attitude[1] = pitch;
        external_odom.attitude[2] = yaw;
    }

    void VelCallback(const geometry_msgs::TwistStamped::ConstPtr &msg)
    {
        external_odom.velocity[0] = msg->twist.linear.x;
        external_odom.velocity[1] = msg->twist.linear.y;
        external_odom.velocity[2] = msg->twist.linear.z;
    }

    void timerCallback(const ros::TimerEvent &event)
    {
        bool is_timeout = (ros::Time::now() - external_odom.header.stamp).toSec() > 1.0;
        external_odom.odom_valid = !is_timeout;
        odom_state_pub.publish(external_odom);
    }

    sunray_msgs::ExternalOdom GetExternalOdom()
    {
        return external_odom;
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber odom_sub;
    ros::Subscriber pos_sub;
    ros::Subscriber vel_sub;
    ros::Publisher odom_state_pub; // 【发布】发布定位状态
    ros::Timer timer_pub_odom_state;
    int uav_id;
    std::string uav_name;
    std::string source_topic_name;
};

#endif // EXTERNALPOSITION_H// 实现外部定位源话题回调函数