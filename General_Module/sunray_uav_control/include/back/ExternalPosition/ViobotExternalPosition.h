#include "ExternalPosition.h"

class ViobotExternalPosition : public ExternalPosition
{
public:
    ViobotExternalPosition()
    {
    }

    double normalize(double yaw) {
    // 调整初始值将 -90 映射到 0
    yaw += 90;

    // 保证 yaw 在 -180 到 180 范围内
    if (yaw > 180) {
        yaw -= 360;  // 如果大于 180，减去 360
    } else if (yaw <= -180) {
        yaw += 360;  // 如果小于 -180，加上 360
    }

    return yaw;
    }

    void init(int id = 1, std::string name = "uav", std::string souce_topic = "/baton/loop/odometry") override
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
    void ViobotCallback(const nav_msgs::Odometry::ConstPtr &msg)
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
        //external_mavros.updateExternalPosition(vision_position);
        
        // 四元素转rpy
        tf2::Quaternion quaternion;
        tf2::fromMsg(msg->pose.pose.orientation, quaternion);
        double roll, pitch, yaw;
        double roll_tf,pitch_tf,yaw_tf;
        tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);

        // 将欧拉角从弧度转换为角度
        roll_tf = roll * 180.0 / M_PI;
        pitch_tf = pitch * 180.0 / M_PI;
        yaw_tf = yaw * 180.0 / M_PI;

        roll=pitch_tf;
        pitch=-normalize(roll_tf);
        yaw=normalize(yaw_tf);

        roll= roll * M_PI /180.0;
        pitch= pitch * M_PI /180.0;
        yaw= yaw * M_PI /180.0;

        tf2::Matrix3x3 rot_matrix;
        rot_matrix.setRPY(roll, pitch, yaw);  // 设置欧拉角（roll, pitch, yaw）到旋转矩阵

        tf2::Quaternion quaternion_tf;
        rot_matrix.getRotation(quaternion_tf);  // 获取对应的四元数
        vision_position.external_qx = quaternion_tf.x();
        vision_position.external_qy = quaternion_tf.y();
        vision_position.external_qz = quaternion_tf.z();
        vision_position.external_qw = quaternion_tf.w();
        //std::cout<<"roll  "<< roll<<"  pitch  "<< pitch<<"  yaw  "<<yaw<<std::endl;
        external_mavros.updateExternalPosition(vision_position);


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
        // 【订阅】VIOBOT估计位置（坐标系:VIOBOT系） VIOBOT -> 本节点
        pos_sub =  nh.subscribe<nav_msgs::Odometry>(source_topic_name, 1, &ViobotExternalPosition::ViobotCallback, this);
        // 【定时器】定时任务
        task_timer = nh.createTimer(ros::Duration(0.05), &ViobotExternalPosition::timerCallback, this);
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
