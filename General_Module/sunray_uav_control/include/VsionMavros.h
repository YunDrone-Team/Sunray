#ifndef VSIONMAVROS_H
#define VISIONMAVROS_H

#include <ros/ros.h>

#include "ros_msg_utils.h"

struct VisionPosition
{
    // 时间
    ros::Time external_time;
    // 外部定位数据xyz
    double external_px;
    double external_py;
    double external_pz;
    // 姿态四元素
    double external_qx;
    double external_qy;
    double external_qz;
    double external_qw;
};

// 抽象类，用于处理外部位置信息
class VsionMavros
{
public:
    VsionMavros()
    {
        // 初始化外部定位数据 设置为-0.01
        this->external_position.external_px = -0.01;
        this->external_position.external_py = -0.01;
        this->external_position.external_pz = -0.01;

        // 初始化上一次的定位数据
        this->last_external_position.external_px = -0.01;
        this->last_external_position.external_py = -0.01;
        this->last_external_position.external_pz = -0.01;

        // 初始化其他参数
        this->timeout_flag = false;
        this->timeout_counter = 0.0;
        this->external_position.external_time = ros::Time::now();
    }

    ~VsionMavros() {}

    // 各个参数的初始化
    void
    initParameters(std::string pub_topic = "/mavros/vision_pose/pose",
                   double timeout_threshold = 0.35,
                   double jump_threshold = 0.1,
                   bool publish_external_position = true,
                   bool check_jump = true,
                   double pub_rate = 50,
                   double task_rate = 20.0)
    {
        this->pub_topic = pub_topic;
        this->timeout_threshold = timeout_threshold;
        this->jump_threshold = jump_threshold;
        this->publish_external_position = publish_external_position;
        this->check_jump = check_jump;
        this->pub_rate = pub_rate;
        this->task_rate = task_rate;
    };

    bool getTimeoutFlag()
    {
        return this->timeout_flag;
    }

    double getTimeoutCounter()
    {
        return this->timeout_counter;
    }

    // 更性新外部定位数据
    void updateExternalPosition(VisionPosition ep)
    {
        external_position = ep;
    }

    // 话题绑定
    void bindTopic(ros::NodeHandle &nh)
    {
        // 绑定vision pos
        external_position_pub = nh.advertise<geometry_msgs::PoseStamped>(pub_topic, 10);
        // 定时器
        timer_task = nh.createTimer(ros::Duration(1.0 / task_rate), &VsionMavros::timerCallback, this);
        timer_pub = nh.createTimer(ros::Duration(1.0 / pub_rate), &VsionMavros::timerPubCallback, this);
    }

private:
    VisionPosition external_position;      // 外部定位数据
    VisionPosition last_external_position; // 上一次定位数据
    std::string pub_topic;                 // 发布外部定位信息的话题
    double timeout_threshold;              // 定位超时时间阈值
    double timeout_counter;                // 定位超时时长
    double jump_threshold;                 // 判断数据跳变的阈值
    double pub_rate;                       // 数据发布频率
    double task_rate;                      // 定时任务频率
    bool timeout_flag;                     // 定位超时标志
    bool publish_external_position;        // 是否发布外部定位信息
    bool check_jump;                       // 是否检查数据跳变
    geometry_msgs::PoseStamped pose_msg;   // 发布消息
    ros::Timer timer_task;                 // 定时器 定时处理任务
    ros::Timer timer_pub;                  // 定时器 定时发布外部定位信息
    ros::Publisher external_position_pub;  // 发布外部定位信息

    // 定时回到函数 定时处理各个事项
    void timerCallback(const ros::TimerEvent &event)
    {
        // 超时检测
        timeoutCheck();
        // 检查数据跳变
        checkJump();
    }

    void timerPubCallback(const ros::TimerEvent &event)
    {
        // 发布外部定位到话题
        publishExternalPosition();
        // 更新上一次的定位数据
        updateLastExternalPosition();
    }

    // 超时检测 定时调用改函数
    bool timeoutCheck()
    {
        timeout_counter = (ros::Time::now() - external_position.external_time).toSec();
        if (timeout_counter > timeout_threshold)
        {
            timeout_flag = true;
        }
        else
        {
            timeout_flag = false;
        }
        return timeout_flag;
    }

    // 检查数据是否存在巨大跳变
    bool checkJump()
    {
        // 用上一次的定位数据减去当前的定位数据，如果差值超过一定阈值，则认为存在跳变 默认只检测位置xyz
        if (fabs(external_position.external_px - last_external_position.external_px) > jump_threshold ||
            fabs(external_position.external_py - last_external_position.external_py) > jump_threshold ||
            fabs(external_position.external_pz - last_external_position.external_pz) > jump_threshold)
        {
            return true;
        }
        return false;
    }

    // 发布外部定位信息 如果超时则不发布
    void publishExternalPosition()
    {
        // if (!timeout_flag)
        if (true)
        {
            pose_msg.header.stamp = ros::Time::now();
            pose_msg.header.frame_id = "odom";
            // 如果初始存在偏移量（默认0） 赋值加上偏移量
            pose_msg.pose.position.x = external_position.external_px;
            pose_msg.pose.position.y = external_position.external_py;
            pose_msg.pose.position.z = external_position.external_pz;

            pose_msg.pose.orientation.x = external_position.external_qx;
            pose_msg.pose.orientation.y = external_position.external_qy;
            pose_msg.pose.orientation.z = external_position.external_qz;
            pose_msg.pose.orientation.w = external_position.external_qw;

            external_position_pub.publish(pose_msg);
        }
    }

    // 更新上一次的定位数据
    void updateLastExternalPosition()
    {
        last_external_position = external_position;
    }
};

#endif // VSIONMAVROS_H