#include "ExternalPosition.h"

class OdomExternalPosition : public ExternalPosition
{
public:
    OdomExternalPosition()
    {
    }

    void init(int id = 1, std::string name = "uav", std::string souce_topic = "Odometry") override
    {
        // 初始化参数
        uav_id = id;
        uav_name = name;
        source_topic_name = souce_topic;

        std::string topic_prefix = "/" + uav_name + std::to_string(uav_id);
        std::string vision_topic = topic_prefix + "/mavros/vision_pose/pose";
        external_mavros.initParameters(vision_topic, 0.35, 0.1, true, true, 50, 20);
    }

    // 实现外部定位源话题回调函数
    void OdomCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        // 接收到的外部定位数据
        vision_position.external_time = ros::Time::now();
        vision_position.external_px = msg->pose.pose.position.x;
        vision_position.external_py = msg->pose.pose.position.y;
        vision_position.external_pz = msg->pose.pose.position.z;
        vision_position.external_qx = msg->pose.pose.orientation.x;
        vision_position.external_qy = msg->pose.pose.orientation.y;
        vision_position.external_qz = msg->pose.pose.orientation.z;
        vision_position.external_qw = msg->pose.pose.orientation.w;
        external_mavros.updateExternalPosition(vision_position);
        
        // 四元素转rpy
        tf2::Quaternion quaternion;
        tf2::fromMsg(msg->pose.pose.orientation, quaternion);
        double roll, pitch, yaw;
        tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);

        position_state.px = vision_position.external_px;
        position_state.py = vision_position.external_py;
        position_state.pz = vision_position.external_pz;
        position_state.vx = msg->twist.twist.linear.x;
        position_state.vy = msg->twist.twist.linear.y;
        position_state.vz = msg->twist.twist.linear.z;
        position_state.qx = vision_position.external_qx;
        position_state.qy = vision_position.external_qy;
        position_state.qz = vision_position.external_qz;
        position_state.qw = vision_position.external_qw;
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
        // 【订阅】mocap pose
        odom_sub = nh.subscribe<nav_msgs::Odometry>(source_topic_name, 10, &OdomExternalPosition::OdomCallback, this);
        // 【定时器】定时任务
        task_timer = nh.createTimer(ros::Duration(0.05), &OdomExternalPosition::timerCallback, this);
        // 给vision mavros节点也绑定话题
        external_mavros.bindTopic(nh);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber odom_sub;
    ros::Timer task_timer;
    int uav_id;
    std::string uav_name;
    std::string source_topic_name;
};