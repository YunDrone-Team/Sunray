#include "ExternalPosition.h"

class MocapExternalPosition : public ExternalPosition
{
public:
    MocapExternalPosition() {};
    ~MocapExternalPosition() {};
    void init(int id = 1, std::string name = "uav", std::string rigidBody = "uav")
    {
        // 初始化参数
        this->uav_id = id;
        this->uav_name = name;
        this->rigidBody_name = rigidBody;

        std::string topic_prefix = "/" + this->uav_name + std::to_string(this->uav_id);
        std::string vision_topic = topic_prefix + "/mavros/vision_pose/pose";
        external_mavros.initParameters(topic_prefix, 0.35, 0.1, true, true, 50, 20);
    };

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

    void VelCallback(const geometry_msgs::TwistStamped::ConstPtr &msg)
    {
        position_state.vx = msg->twist.linear.x;
        position_state.vy = msg->twist.linear.y;
        position_state.vz = msg->twist.linear.z;
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
        pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node_" + std::to_string(uav_id) + this->rigidBody_name + "/pose", 1, &MocapExternalPosition::PosCallback, this);
        // 【订阅】mocap Twist
        vel_sub = nh.subscribe<geometry_msgs::TwistStamped>("/vrpn_client_node_" + std::to_string(uav_id) + this->rigidBody_name + "/twist", 1, &MocapExternalPosition::VelCallback, this);
        // 【定时器】定时任务
        task_timer = nh.createTimer(ros::Duration(0.05), &MocapExternalPosition::timerCallback, this);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber pos_sub;
    ros::Subscriber vel_sub;
    ros::Timer task_timer;
    int uav_id;
    std::string uav_name;
    std::string rigidBody_name;
};