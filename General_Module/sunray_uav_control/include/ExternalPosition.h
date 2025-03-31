#ifndef EXTERNALPOSITION_H
#define EXTERNALPOSITION_H

#include "ros_msg_utils.h"

struct PoseState
{
    ros::Time stamp;
    double pos_x = 0.0;
    double pos_y = 0.0;
    double pos_z = 0.0;
    double vel_x = 0.0;
    double vel_y = 0.0;
    double vel_z = 0.0;
    double att_x = 0.0;
    double att_y = 0.0;
    double att_z = 0.0;
    double att_w = 0.0;
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
};

class ExternalPosition
{
public:
    ExternalPosition()
    {
    }

    void init(ros::NodeHandle &nh, std::string souce_topic = "Odometry", int type = 0)
    {
        // 初始化参数
        nh.param<int>("uav_id", uav_id, 1);
        nh.param<std::string>("uav_name", uav_name, "uav");
        source_topic_name = souce_topic;

        std::string topic_prefix = "/" + uav_name + std::to_string(uav_id);
        std::string vision_topic = topic_prefix + "/mavros/vision_pose/pose";

        position_state.pos_x = -0.01;
        position_state.pos_y = -0.01;
        position_state.pos_z = -0.01;
        position_state.att_x = 0;
        position_state.att_y = 0;
        position_state.att_z = 0;
        position_state.att_w = 1;
        position_state.roll = 0.0;
        position_state.pitch = 0.0;
        position_state.yaw = 0.0;

        switch (type)
        {
        case 0:
            odom_sub = nh.subscribe<nav_msgs::Odometry>(source_topic_name, 10, &ExternalPosition::OdomCallback, this);
            break;
        case 1:
            pos_sub = nh.subscribe<geometry_msgs::PoseStamped>(source_topic_name, 10, &ExternalPosition::PosCallback, this);
            break;
        case 2:
            souce_topic = topic_prefix + "/sunray/gazebo_pose";
            odom_sub = nh.subscribe<nav_msgs::Odometry>(source_topic_name, 10, &ExternalPosition::OdomCallback, this);
            break;
        case 3:
            // 【订阅】动捕的定位数据(坐标系:动捕系统惯性系) vrpn -> 本节点
            pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node_" + std::to_string(uav_id) + topic_prefix + "/pose", 1, &ExternalPosition::PosCallback, this);
            // 【订阅】动捕的定位数据(坐标系:动捕系统惯性系) vrpn -> 本节点
            vel_sub = nh.subscribe<geometry_msgs::TwistStamped>("/vrpn_client_node_" + std::to_string(uav_id) + topic_prefix + "/twist", 1, &ExternalPosition::VelCallback, this);
            break;
        case 5:
            souce_topic = topic_prefix + "/mavros/global_position/local";
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
        position_state.stamp = ros::Time::now();
        position_state.pos_x = msg->pose.pose.position.x;
        position_state.pos_y = msg->pose.pose.position.y;
        position_state.pos_z = msg->pose.pose.position.z;
        position_state.vel_x = msg->twist.twist.linear.x;
        position_state.vel_y = msg->twist.twist.linear.y;
        position_state.vel_z = msg->twist.twist.linear.z;
        position_state.att_x = msg->pose.pose.orientation.x;
        position_state.att_y = msg->pose.pose.orientation.y;
        position_state.att_z = msg->pose.pose.orientation.z;
        position_state.att_w = msg->pose.pose.orientation.w;
        position_state.roll = roll;
        position_state.pitch = pitch;
        position_state.yaw = yaw;
    }

    // 实现外部定位源话题回调函数
    void PosCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        // 四元素转rpy
        tf2::Quaternion quaternion;
        tf2::fromMsg(msg->pose.orientation, quaternion);
        double roll, pitch, yaw;
        tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
        position_state.stamp = ros::Time::now();
        position_state.pos_x = msg->pose.position.x;
        position_state.pos_y = msg->pose.position.y;
        position_state.pos_z = msg->pose.position.z;
        position_state.att_x = msg->pose.orientation.x;
        position_state.att_y = msg->pose.orientation.y;
        position_state.att_z = msg->pose.orientation.z;
        position_state.att_w = msg->pose.orientation.w;
        position_state.roll = roll;
        position_state.pitch = pitch;
        position_state.yaw = yaw;
    }

    // Gps模式下，实际接收的是mavros/global_position/local
    void GpsOdomCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        // 四元素转rpy
        tf2::Quaternion quaternion;
        tf2::fromMsg(msg->pose.pose.orientation, quaternion);
        double roll, pitch, yaw;
        tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
        position_state.stamp = ros::Time::now();
        position_state.pos_x = msg->pose.pose.position.x;
        position_state.pos_y = msg->pose.pose.position.y;
        position_state.pos_z = msg->pose.pose.position.z;
        position_state.vel_x = msg->twist.twist.linear.x;
        position_state.vel_y = msg->twist.twist.linear.y;
        position_state.vel_z = msg->twist.twist.linear.z;
        position_state.att_x = msg->pose.pose.orientation.x;
        position_state.att_y = msg->pose.pose.orientation.y;
        position_state.att_z = msg->pose.pose.orientation.z;
        position_state.att_w = msg->pose.pose.orientation.w;
        position_state.roll = roll;
        position_state.pitch = pitch;
        position_state.yaw = yaw;
    }

    void VelCallback(const geometry_msgs::TwistStamped::ConstPtr &msg)
    {
        position_state.vel_x = msg->twist.linear.x;
        position_state.vel_y = msg->twist.linear.y;
        position_state.vel_z = msg->twist.linear.z;
    }

    PoseState GetPositionState()
    {
        return position_state;
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber odom_sub;
    ros::Subscriber pos_sub;
    ros::Subscriber vel_sub;
    int uav_id;
    std::string uav_name;
    std::string source_topic_name;
    PoseState position_state;
};

#endif // EXTERNALPOSITION_H// 实现外部定位源话题回调函数