#ifndef EXTERNALPOSITION_H
#define EXTERNALPOSITION_H

#include "ros_msg_utils.h"

// 滑动平均滤波器
class MovingAverageFilter
{
public:
    MovingAverageFilter(int size = 5)
    {
        this->size = size;
        this->data = new double[size];
        this->sum = 0;
        this->count = 0;
        this->index = 0;
    }

    ~MovingAverageFilter()
    {
        delete[] data;
    }

    void setSize(int size)
    {
        this->size = size;
        delete[] data;
        this->data = new double[size];
        this->sum = 0;
        this->count = 0;
        this->index = 0;
    }

    void addData(double value)
    {

        if (count < size)
        {
            sum += value;
            data[count++] = value;
        }
        else
        {
            sum -= data[index];
            sum += value;
            data[index] = value;
            index = (index + 1) % size;
        }
    }

    double getAverage()
    {
        return sum / count;
    }

    double filter(double value)
    {
        addData(value);
        return getAverage();
    }

private:
    int size;
    double *data;
    double sum;
    int count;
    int index;
};

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
        case sunray_msgs::ExternalOdom::VIOBOT:
            moving_average_filter.setSize(1);
            source_topic = "/baton_mini/odometry";
            odom_sub = nh.subscribe<nav_msgs::Odometry>(source_topic, 10, &ExternalPosition::viobotCallback, this);
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

    void viobotCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        // 四元素转rpy
        tf2::Quaternion quaternion;
        tf2::fromMsg(msg->pose.pose.orientation, quaternion);
        double roll, pitch, yaw;
        external_odom.header.stamp = ros::Time::now();
        external_odom.position[0] = msg->pose.pose.position.x;
        external_odom.position[1] = msg->pose.pose.position.y;
        external_odom.position[2] = msg->pose.pose.position.z;
        // external_odom.position[0] = moving_average_filter.filter(msg->pose.pose.position.x);
        // external_odom.position[1] = moving_average_filter.filter(msg->pose.pose.position.y);
        // external_odom.position[2] = moving_average_filter.filter(msg->pose.pose.position.z);
        external_odom.velocity[0] = msg->twist.twist.linear.x;
        external_odom.velocity[1] = msg->twist.twist.linear.y;
        external_odom.velocity[2] = msg->twist.twist.linear.z;

        tf2::Quaternion q;
        q.setW(msg->pose.pose.orientation.w);
        q.setX(msg->pose.pose.orientation.x);
        q.setY(msg->pose.pose.orientation.y);
        q.setZ(msg->pose.pose.orientation.z);
        // 绕 Z 轴旋转 90°
        tf2::Quaternion q_z;
        q_z.setRPY(0, 0, M_PI / 2); // M_PI/2 = 90°

        // 绕 Y 轴旋转 -90°
        tf2::Quaternion q_y;
        q_y.setRPY(0, -M_PI / 2, 0); // -M_PI/2 = -90°

        // 组合旋转（顺序：先 q_z，再 q_y）
        q = q * q_z * q_y;

        // 转欧拉角
        tf2::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);
        // std::cout << "roll: " << roll * 180 / M_PI << " pitch: " << pitch * 180 / M_PI << " yaw: " << yaw * 180 / M_PI << std::endl;

        // external_odom.attitude_q.x = msg->pose.pose.orientation.x;
        // external_odom.attitude_q.y = msg->pose.pose.orientation.y;
        // external_odom.attitude_q.z = msg->pose.pose.orientation.z;
        // external_odom.attitude_q.w = msg->pose.pose.orientation.w;

        external_odom.attitude_q.x = q.getX();
        external_odom.attitude_q.y = q.getY();
        external_odom.attitude_q.z = q.getZ();
        external_odom.attitude_q.w = q.getW();
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
    MovingAverageFilter moving_average_filter;
};

#endif // EXTERNALPOSITION_H// 实现外部定位源话题回调函数