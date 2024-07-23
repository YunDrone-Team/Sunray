/*
 * @Author: Yuhua.Qi fatmoonqyp@126.com
 * @Date: 2024-01-24 16:07:28
 * @LastEditors: Yuhua.Qi fatmoonqyp@126.com
 * @LastEditTime: 2024-01-24 16:08:52
 * @FilePath: /Prometheus/home/amov/Sunray-Comm/sunray_comm/src/fake_topic_pub.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
// ros头文件
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <iostream>

// topic 头文件
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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "fake_topic_pub");
    ros::NodeHandle nh("~");

    string topic_prefix = "/uav1";

    ros::Publisher setpoint_raw_attitude_pub = nh.advertise<sunray_msgs::AttitudeSetpoint>(topic_prefix + "/sunray/setpoint_raw/attitude", 10);

    int CMD = 0;
    

    while (ros::ok())
    {
        ros::spinOnce();

        cout << GREEN << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>fake_topic_pub<<<<<<<<<<<<<<<<<<<<<<<<< " << TAIL << endl;
        cout << GREEN << "Please choose the CMD: 1 for AttitudeSetpoint..." << TAIL << endl;
        cin >> CMD;

        switch (CMD)
        {
            case 1:
            {
                sunray_msgs::AttitudeSetpoint setpoint_raw_attitude;
                setpoint_raw_attitude.header.stamp = ros::Time::now();
                setpoint_raw_attitude.type_mask = 0b10000000;
                setpoint_raw_attitude.orientation.x = 0;
                setpoint_raw_attitude.orientation.y = 0;
                setpoint_raw_attitude.orientation.z = 0;
                setpoint_raw_attitude.orientation.w = 1;
                setpoint_raw_attitude.body_rate.x = 0;
                setpoint_raw_attitude.body_rate.y = 0;
                setpoint_raw_attitude.body_rate.z = 0;
                setpoint_raw_attitude.thrust = 0.5;
                setpoint_raw_attitude_pub.publish(setpoint_raw_attitude);

            }

        }
        ros::Duration(0.5).sleep();
    }
    return 0;
}
