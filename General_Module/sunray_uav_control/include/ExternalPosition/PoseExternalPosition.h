#include "ExternalPosition.h"

class PoseExternalPosition : public ExternalPosition
{
public:
    PoseExternalPosition()
    {
    }

    void init(int id = 1, std::string name = "uav", std::string souce_topic = "Pose") override
    {
        // 初始化参数
        uav_id = id;
        uav_name = name;
        source_topic_name = souce_topic;

        std::string topic_prefix = "/" + uav_name + std::to_string(uav_id);
        std::string vision_topic = topic_prefix + "/mavros/vision_pose/pose";
        external_mavros.initParameters(vision_topic, 0.35, 0.1, true, true, 50, 20);

        position_state.px = -0.01;
        position_state.py = -0.01;
        position_state.pz = -0.01;
        position_state.qz = -0.01;
        position_state.roll = 0.0;
        position_state.pitch = 0.0;
        position_state.yaw = 0.0;
    }

    // 实现外部定位源话题回调函数
    void PosCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        // 接收到的外部定位数据
        vision_position.external_time = ros::Time::now();
        vision_position.external_px = msg->pose.position.x;
        vision_position.external_py = msg->pose.position.y;
        vision_position.external_pz = msg->pose.position.z;
        vision_position.external_qx = msg->pose.orientation.x;
        vision_position.external_qy = msg->pose.orientation.y;
        vision_position.external_qz = msg->pose.orientation.z;
        vision_position.external_qw = msg->pose.orientation.w;
        external_mavros.updateExternalPosition(vision_position);

        // 四元素转rpy
        tf2::Quaternion quaternion;
        tf2::fromMsg(msg->pose.orientation, quaternion);
        double roll, pitch, yaw;
        tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);

        position_state.px = vision_position.external_px;
        position_state.py = vision_position.external_py;
        position_state.pz = vision_position.external_pz;
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
        // 【订阅】pose
        pos_sub = nh.subscribe<geometry_msgs::PoseStamped>(source_topic_name, 1, &PoseExternalPosition::PosCallback, this);
        // 【定时器】定时任务
        task_timer = nh.createTimer(ros::Duration(0.05), &PoseExternalPosition::timerCallback, this);
        // 给vision mavros节点也绑定话题
        external_mavros.bindTopic(nh);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber pos_sub;
    ros::Subscriber vel_sub;
    ros::Timer task_timer;
    int uav_id;
    std::string uav_name;
    std::string source_topic_name;
};