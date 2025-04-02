#ifndef EXTERNALPOSITION_H
#define EXTERNALPOSITION_H

#include "ros_msg_utils.h"

// 超时检测 定时调用改函数
bool ExternalFusion::timeoutCheck()
{
    timeout_counter = (ros::Time::now() - vision_pose.header.stamp).toSec();
    if (timeout_counter > source_timeout)
    {
        is_timeout = true;
    }
    else
    {
        is_timeout = false;
    }

    if ((external_source == GPS || external_source == RTK))
    {
        if (px4_state.gps_status == -1)
        {
            is_timeout = false;
        }
        else
        {
            is_timeout = true;
        }
    }
    return is_timeout;
}

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

    // 声明一个自定义话题 - sunray_msgs::ExternalOdom
    sunray_msgs::ExternalOdom external_odom;

    void init(ros::NodeHandle &nh, std::string souce_topic = "Odometry", int type = 0)
    {
        // 初始化参数
        nh.param<int>("uav_id", uav_id, 1);
        nh.param<std::string>("uav_name", uav_name, "uav");
        source_topic_name = souce_topic;

        std::string topic_prefix = "/" + uav_name + std::to_string(uav_id);

        // 【发布】外部定位状态 - 本节点 -> uav_control_node
        odom_state_pub = nh.advertise<sunray_msgs::ExternalOdom>(uav_name + "/sunray/external_odom_state", 10);

    // 发布外部定位状态 - 移除这部分到另外一个类
    external_odom.header.stamp = ros::Time::now();
    external_odom.external_source = external_source;
    external_odom.odom_valid = !is_timeout;
    external_odom.position[0] = external_state.pos_x;
    external_odom.position[1] = external_state.pos_y;
    external_odom.position[2] = external_state.pos_z;
    external_odom.velocity[0] = external_state.vel_x;
    external_odom.velocity[1] = external_state.vel_y;
    external_odom.velocity[2] = external_state.vel_z;
    external_odom.attitude_q.w = external_state.att_w;
    external_odom.attitude_q.x = external_state.att_x;
    external_odom.attitude_q.y = external_state.att_y;
    external_odom.attitude_q.z = external_state.att_z;
    odom_state_pub.publish(external_odom);

        // 增加外部定位数据超时检查

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
        case sunray_msgs::ExternalOdom::ODOM:
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

    sunray_msgs::ExternalOdom GetPositionState()
    {
        return external_odom;
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber odom_sub;
    ros::Subscriber pos_sub;
    ros::Subscriber vel_sub;
    int uav_id;
    std::string uav_name;
    std::string source_topic_name;

};

#endif // EXTERNALPOSITION_H// 实现外部定位源话题回调函数