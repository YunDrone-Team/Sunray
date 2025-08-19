#include "data_convert.hpp"
#include "mavlink_control.h"
#include <sunray_msgs/ViobotState.h>

sunray_msgs::ViobotState viobot_state;

ros::Publisher viobot_state_pub;
ros::Subscriber sub_stereo3_pose;

void pose_callback(const nav_msgs::OdometryPtr &msg)
{
    mavlink_odometry_t mavlink_odom;
    odometry_msg_convert(msg, mavlink_odom);
    mavlink_save_odometry(mavlink_odom);
    //  mavlink_send_odometry(mavlink_odom);  //直接接收到数据就用串口发送出去

    viobot_state.position[0] = mavlink_odom.x;
    viobot_state.position[1] = mavlink_odom.y;
    viobot_state.position[2] = mavlink_odom.z;

    viobot_state.velocity[0] = mavlink_odom.vx;
    viobot_state.velocity[1] = mavlink_odom.vy;
    viobot_state.velocity[2] = mavlink_odom.vz;

    viobot_state.attitude_q.w = mavlink_odom.q[0];
    viobot_state.attitude_q.x = mavlink_odom.q[1];
    viobot_state.attitude_q.y = mavlink_odom.q[2];
    viobot_state.attitude_q.z = mavlink_odom.q[3];

    viobot_state.header.stamp = ros::Time::now();
    viobot_state_pub.publish(viobot_state);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mavLink");
    ros::NodeHandle nh("~");
    std::string uart_name;
    int baudrate = 57600;
    nh.param<string>("uart_name", uart_name, "/dev/ttyS0");
    nh.param<int>("baudrate", baudrate, 57600);
    mavlink_init(uart_name.c_str(), baudrate);
    // mavlink_control();
    mavlink_send_odometry_thread(); // 先保存再在另外的线程里面发送
    viobot_state_pub = nh.advertise<sunray_msgs::ViobotState>("/sunray_viobot/ViobotState", 10);
    sub_stereo3_pose = nh.subscribe("/baton/stereo3/odometry", 2, pose_callback);
    ros::spin();
    mavlink_deinit();
    return 0;
}
