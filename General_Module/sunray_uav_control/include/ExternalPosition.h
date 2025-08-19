#ifndef EXTERNALPOSITION_H
#define EXTERNALPOSITION_H

#include "ros_msg_utils.h"
#include "printf_format.h"
#include <sunray_msgs/ViobotState.h>
#include <sunray_viobot_unit/algo_ctrl.h>
#include <sunray_viobot_unit/algo_status.h>

using namespace sunray_logger;

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

#define ODOM_TIMEOUT 0.3
#define DISTANCE_SENSOR_TIMEOUT 0.3

class ExternalPosition
{
public:
    ExternalPosition()
    {
    }

    sunray_msgs::ExternalOdom external_odom; // 声明一个自定义话题 - sunray_msgs::ExternalOdom
    sensor_msgs::Range distance_sensor;      // 距离传感器原始数据

    std::string algo_status;
    bool is_viobot_start;

    void init(ros::NodeHandle &nh, int external_source = 0, std::string source_topic_name = "Odometry", bool range_sensor = false)
    {
        // 初始化参数
        nh.param<int>("uav_id", uav_id, 1);
        nh.param<std::string>("uav_name", uav_name, "uav");
        uav_name = "/" + uav_name + std::to_string(uav_id);

        enable_range_sensor = range_sensor;

        // 定时检查外部定位数据是否超时
        timer_check_timeout = nh.createTimer(ros::Duration(0.05), &ExternalPosition::timer_check_timeout_cb, this);

        // 初始化外部定位状态
        external_odom.header.stamp = ros::Time::now();
        external_odom.external_source = external_source;
        external_odom.odom_valid = false;
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

        // 初始化viobot相关状态
        is_viobot_start = false;
        algo_set.algo_enable = false;
        algo_set.algo_reboot = false;
        algo_set.algo_reset = false;

        switch (external_source)
        {
        case sunray_msgs::ExternalOdom::ODOM:
            odom_sub = nh.subscribe<nav_msgs::Odometry>(source_topic_name, 10, &ExternalPosition::OdomCallback, this);
            break;
        case sunray_msgs::ExternalOdom::POSE:
            pos_sub = nh.subscribe<geometry_msgs::PoseStamped>(source_topic_name, 10, &ExternalPosition::PosCallback, this);
            break;
        case sunray_msgs::ExternalOdom::GAZEBO:
            source_topic_name = uav_name + "/sunray/gazebo_pose";
            odom_sub = nh.subscribe<nav_msgs::Odometry>(source_topic_name, 10, &ExternalPosition::OdomCallback, this);
            break;
        case sunray_msgs::ExternalOdom::MOCAP:
            // 【订阅】动捕的定位数据(坐标系:动捕系统惯性系) vrpn -> 本节点
            pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node_" + std::to_string(uav_id) + uav_name + "/pose", 1, &ExternalPosition::PosCallback, this);
            // 【订阅】动捕的定位数据(坐标系:动捕系统惯性系) vrpn -> 本节点
            vel_sub = nh.subscribe<geometry_msgs::TwistStamped>("/vrpn_client_node_" + std::to_string(uav_id) + uav_name + "/twist", 1, &ExternalPosition::VelCallback, this);
            break;
        case sunray_msgs::ExternalOdom::VIOBOT:
            // 【订阅】viobot的mavlink直通程序 -> 本节点
            source_topic_name = "/sunray_viobot/ViobotState";
            odom_sub = nh.subscribe<sunray_msgs::ViobotState>(source_topic_name, 10, &ExternalPosition::viobotCallback, this);
            // 【订阅】viobot/ROS_interfaces -> 本节点
            algo_status_sub = nh.subscribe<sunray_viobot_unit::algo_status>("/baton/algo_status", 2, &ExternalPosition::status_callback, this);
            // 【发布】发布算法启动话题
            pub_stereo3_ctrl = nh.advertise<sunray_viobot_unit::algo_ctrl>("/baton/stereo3_ctrl", 2);
            break;
        default:
            Logger::print_color(int(LogColor::red), LOG_BOLD, "Unknown external position source type - [", external_source, "]");
            break;
        }

        if (enable_range_sensor)
        {
            // 【订阅】无人机上的激光定高原始数据
            range_sub = nh.subscribe<sensor_msgs::Range>(uav_name + "/mavros/distance_sensor/hrlv_ez4_pub", 1, &ExternalPosition::px4_distance_callback, this);
        }
    }

    // 回调函数：接收PX4距离传感器原始数据
    void px4_distance_callback(const sensor_msgs::Range::ConstPtr &msg)
    {
        distance_sensor = *msg;
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

    void viobotCallback(const sunray_msgs::ViobotState::ConstPtr &msg)
    {
        // 四元素转rpy
        tf2::Quaternion quaternion;
        tf2::fromMsg(msg->attitude_q, quaternion);
        double roll, pitch, yaw;
        tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
        external_odom.header.stamp = ros::Time::now();
        external_odom.position[0] = msg->position[0];
        external_odom.position[1] = msg->position[1];
        external_odom.position[2] = msg->position[2];
        external_odom.velocity[0] = msg->velocity[0];
        external_odom.velocity[1] = msg->velocity[0];
        external_odom.velocity[2] = msg->velocity[0];
        external_odom.attitude_q.x = msg->attitude_q.x;
        external_odom.attitude_q.y = msg->attitude_q.y;
        external_odom.attitude_q.z = msg->attitude_q.z;
        external_odom.attitude_q.w = msg->attitude_q.w;
        external_odom.attitude[0] = roll;
        external_odom.attitude[1] = pitch;
        external_odom.attitude[2] = yaw;
    }

    void status_callback(const sunray_viobot_unit::algo_status::ConstPtr &msg)
    {
        algo_status = msg->algo_status;
        if (msg->algo_status == "ready")
        {
            is_viobot_start = false;
        }
        else if (msg->algo_status == "stereo3_initializing" || msg->algo_status == "stereo3_running")
        {
            // 算法初始化
            is_viobot_start = true;
        }

        if (is_viobot_start == false) // 如果没有开启算法，自动开启
        {
            algo_set.algo_enable = true;
            pub_stereo3_ctrl.publish(algo_set);
        }
    }

    void VelCallback(const geometry_msgs::TwistStamped::ConstPtr &msg)
    {
        external_odom.velocity[0] = msg->twist.linear.x;
        external_odom.velocity[1] = msg->twist.linear.y;
        external_odom.velocity[2] = msg->twist.linear.z;
    }

    void timer_check_timeout_cb(const ros::TimerEvent &event)
    {
        odom_timeout = (ros::Time::now() - external_odom.header.stamp).toSec() > ODOM_TIMEOUT;
        external_odom.odom_valid = !odom_timeout;

        if (enable_range_sensor)
        {
            distance_timeout = (ros::Time::now() - distance_sensor.header.stamp).toSec() > DISTANCE_SENSOR_TIMEOUT;
        }
    }

    sunray_msgs::ExternalOdom GetExternalOdom()
    {
        return external_odom;
    }

private:
    ros::Subscriber odom_sub;
    ros::Subscriber pos_sub;
    ros::Subscriber vel_sub;
    ros::Subscriber range_sub;
    ros::Timer timer_check_timeout;
    bool odom_timeout;
    bool distance_timeout;
    bool enable_range_sensor;
    int uav_id;
    std::string uav_name;

    ros::Subscriber algo_status_sub;
    ros::Publisher pub_stereo3_ctrl;
    sunray_viobot_unit::algo_ctrl algo_set;
    // MovingAverageFilter moving_average_filter;
};

#endif // EXTERNALPOSITION_H// 实现外部定位源话题回调函数