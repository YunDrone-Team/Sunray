// ROS 头文件
#include <ros/ros.h>
#include <iostream>
#include "ros_msg_utils.h"

using namespace std;
sunray_msgs::UAVState uav_state;

// 无人机状态回调
void uav_state_callback(const sunray_msgs::UAVState::ConstPtr &msg)
{
    uav_state = *msg;    
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ugv_circle");
    ros::NodeHandle nh("~");
    ros::Rate rate(100.0);

    // 创建一个发布器，发布到"/gazebo/set_model_state"主题
    ros::Publisher model_state_pub = nh.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 10);
    // 发布地面小车位置和速度，模拟视觉检测系统
    ros::Publisher target_odom_pub = nh.advertise<nav_msgs::Odometry>("/ugv_odom", 10);


    ros::Subscriber uav_state_sub = nh.subscribe<sunray_msgs::UAVState>("/uav1/sunray/uav_state", 10, uav_state_callback);


    gazebo_msgs::ModelState model_state;
    nav_msgs::Odometry ugv_odom;
    

    float time = 0;
    while (ros::ok())
    {

        float linear_vel = 0.25;
        float circle_radius = 2.0;
        float omega = fabs(linear_vel / circle_radius);

        const float angle = time * omega;
        const float cos_angle = cos(angle);
        const float sin_angle = sin(angle);

        // 无人机在起飞状态，无人车才开始移动
        if(uav_state.landed_state==2)
        {
            model_state.model_name = "target";
            model_state.pose.position.x = circle_radius * cos_angle + 0;
            model_state.pose.position.y = circle_radius * sin_angle + 0;
            model_state.pose.position.z = 0.0;
            model_state.pose.orientation.x = 0.0;
            model_state.pose.orientation.y = 0.0;
            model_state.pose.orientation.z = 0.0;
            model_state.pose.orientation.w = 1.0;
            model_state_pub.publish(model_state);

            ugv_odom.header.stamp = ros::Time::now();
            ugv_odom.header.frame_id = "world";
            ugv_odom.child_frame_id = "base_link";
            ugv_odom.pose.pose = model_state.pose;
            ugv_odom.twist.twist.linear.x = -linear_vel * sin_angle;
            ugv_odom.twist.twist.linear.y = linear_vel * cos_angle;
            ugv_odom.twist.twist.linear.z = 0.0;
            target_odom_pub.publish(ugv_odom);
            time = time + 0.01;
        }

        rate.sleep();
        ros::spinOnce();
    }

    return 0;
}