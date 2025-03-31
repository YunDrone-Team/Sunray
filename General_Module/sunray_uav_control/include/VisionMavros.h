#ifndef VISIONMAVROS_H
#define VISIONMAVROS_H

#include <ros/ros.h>

#include "ros_msg_utils.h"

// 抽象类，用于处理外部位置信息
class VisionMavros
{
public:
    VisionMavros()
    {
        // 初始化外部定位数据 设置为-0.01表示无效
        this->vision_position.pose.position.x = -0.01;
        this->vision_position.pose.position.y = -0.01;
        this->vision_position.pose.position.z = -0.01;
        this->vision_position.pose.orientation.x = 0;
        this->vision_position.pose.orientation.y = 0;
        this->vision_position.pose.orientation.z = 0;
        this->vision_position.pose.orientation.w = 1;

        // 初始化其他参数
        this->timeout_flag = false;
        this->timeout_counter = 0.0;
        this->vision_position.header.stamp = ros::Time::now();
    }

    ~VisionMavros() {}

    // 各个参数的初始化
    void initParameters(std::string pub_topic = "/mavros/vision_pose/pose",
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

    // 更性新外部定位数据
    void updateExternalPosition(geometry_msgs::PoseStamped ep)
    {
        vision_position = ep;
    }

    // 话题绑定
    void bindTopic(ros::NodeHandle &nh)
    {
        // 【发布】无人机位置和偏航角传输至PX4_EKF2模块用于位置姿态估计 - 本节点 -> mavros -> 飞控
        //  注意：这个话题用于给飞控传输外部定位数据，需要配合PX4中的EKF2参数才能发挥作用
        external_position_pub = nh.advertise<geometry_msgs::PoseStamped>(pub_topic, 10);
        // 定时器
        timer_task = nh.createTimer(ros::Duration(1.0 / task_rate), &VisionMavros::timerCallback, this);
        timer_pub = nh.createTimer(ros::Duration(1.0 / pub_rate), &VisionMavros::timerPubCallback, this);
    }

private:
    geometry_msgs::PoseStamped vision_position;      // 外部定位数据
    geometry_msgs::PoseStamped last_vision_position; // 上一次定位数据
    std::string pub_topic;                             // 发布外部定位信息的话题
    double timeout_threshold;                          // 定位超时时间阈值
    double timeout_counter;                            // 定位超时时长
    double jump_threshold;                             // 判断数据跳变的阈值
    double pub_rate;                                   // 数据发布频率
    double task_rate;                                  // 定时任务频率
    bool timeout_flag;                                 // 定位超时标志
    bool publish_external_position;                    // 是否发布外部定位信息
    bool check_jump;                                   // 是否检查数据跳变
    ros::Timer timer_task;                             // 定时器 定时处理任务
    ros::Timer timer_pub;                              // 定时器 定时发布外部定位信息
    ros::Publisher external_position_pub;              // 发布外部定位信息

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
        last_vision_position = vision_position;
    }

    // 超时检测 定时调用改函数
    bool timeoutCheck()
    {
        timeout_counter = (ros::Time::now() - vision_position.header.stamp).toSec();
        if (timeout_counter > timeout_threshold)
        {
            timeout_flag = false;
        }
        else
        {
            timeout_flag = true;
        }
        return timeout_flag;
    }

    // 检查数据是否存在巨大跳变
    bool checkJump()
    {
        // 用上一次的定位数据减去当前的定位数据，如果差值超过一定阈值，则认为存在跳变 默认只检测位置xyz
        if (fabs(vision_position.pose.position.x - last_vision_position.pose.position.x) > jump_threshold ||
            fabs(vision_position.pose.position.y - last_vision_position.pose.position.y) > jump_threshold ||
            fabs(vision_position.pose.position.z - last_vision_position.pose.position.z) > jump_threshold)
        {
            return true;
        }
        return false;
    }

    // 发布外部定位信息
    void publishExternalPosition()
    {
        // if (!timeout_flag)
        if (true)
        {
            vision_position.header.frame_id = "odom";
            external_position_pub.publish(vision_position);
        }
    }
};

#endif // VISIONMAVROS_H