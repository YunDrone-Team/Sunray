#include "ExternalPosition.h"

class GPSExternalPosition : public ExternalPosition
{
public:
    GPSExternalPosition()
    {
    }

    void init(int id = 1, std::string name = "uav", std::string souce_topic = "Odometry") override
    {
        // 初始化参数
        uav_id = id;
        uav_name = name;
        source_topic_name = souce_topic;

        std::string topic_prefix = "/" + uav_name + std::to_string(uav_id);

        position_state.px = -0.01;
        position_state.py = -0.01;
        position_state.pz = -0.01;
        position_state.qz = -0.01;
        position_state.roll = 0.0;
        position_state.pitch = 0.0;
        position_state.yaw = 0.0;
    }

    // 实现外部定位源话题回调函数
    void OdomCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        // 接收到的外部定位数据

        // 四元素转rpy
        tf2::Quaternion quaternion;
        tf2::fromMsg(msg->pose.pose.orientation, quaternion);
        double roll, pitch, yaw;
        tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);

        position_state.px = msg->pose.pose.position.x;
        position_state.py = msg->pose.pose.position.y;
        position_state.pz = msg->pose.pose.position.z;
        position_state.vx = msg->twist.twist.linear.x;
        position_state.vy = msg->twist.twist.linear.y;
        position_state.vz = msg->twist.twist.linear.z;
        position_state.qx = msg->pose.pose.orientation.x;
        position_state.qy = msg->pose.pose.orientation.y;
        position_state.qz = msg->pose.pose.orientation.z;
        position_state.qw = msg->pose.pose.orientation.w;
        position_state.roll = roll;
        position_state.pitch = pitch;
        position_state.yaw = yaw;
    }

    void timerCallback(const ros::TimerEvent &event) override
    {
        position_state.valid = external_mavros.getTimeoutFlag();
        if (position_state.valid)
        {
            position_state.timeout_count = external_mavros.getTimeoutCounter();
        }
    }

    void bindTopic(ros::NodeHandle &nh) override
    {
        nh_ = nh;
        // 【订阅】global pose
        odom_sub = nh.subscribe<nav_msgs::Odometry>(source_topic_name, 10, &GPSExternalPosition::OdomCallback, this);
        // 【定时器】定时任务
        task_timer = nh.createTimer(ros::Duration(0.05), &GPSExternalPosition::timerCallback, this);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber odom_sub;
    ros::Timer task_timer;
    int uav_id;
    std::string uav_name;
    std::string source_topic_name;
};