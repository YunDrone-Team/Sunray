// ros头文件
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <iostream>

// topic 头文件
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Bool.h>
#include <sunray_msgs/Heartbeat.h>
#include <sunray_msgs/Attitude.h>
#include <sunray_msgs/LocalPositionNED.h>
#include <sunray_msgs/ArmCmd.h>
#include <sunray_msgs/AttitudeSetpoint.h>
#include <sunray_msgs/GlobalPositionSetpoint.h>
#include <sunray_msgs/LocalPositionSetpoint.h>


#include "common/mavlink.h"
#include "printf_utils.h"
#include "enum_utils.h"
#include "generic_port.h"
#include "serial_port.h"
#include "udp_port.h"
using namespace std;

ros::Publisher setpoint_raw_attitude_pub, vision_pose_pub;

void timer1_cb(const ros::TimerEvent& e)
{
    sunray_msgs::AttitudeSetpoint setpoint_raw_attitude;
    setpoint_raw_attitude.header.stamp = ros::Time::now();
    setpoint_raw_attitude_pub.publish(setpoint_raw_attitude);
}

void timer2_cb(const ros::TimerEvent& e)
{
    nav_msgs::Odometry test;
    test.header.stamp = ros::Time::now();
    vision_pose_pub.publish(test);

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "fake_topic_pub");
    ros::NodeHandle nh("~");

    string topic_name = "/uav1";

    ros::Timer timer1 = nh.createTimer(ros::Duration(0.1), timer1_cb);
    ros::Timer timer2 = nh.createTimer(ros::Duration(0.01), timer2_cb);  


    setpoint_raw_attitude_pub = nh.advertise<sunray_msgs::AttitudeSetpoint>(topic_name + "/sunray/setpoint_raw/attitude", 1);

    vision_pose_pub = nh.advertise<nav_msgs::Odometry>(topic_name + "/sunray/vision_odom", 1);


    while (ros::ok())
    {
        ros::spinOnce();
        usleep(1000);
    }

    return 0;
}
