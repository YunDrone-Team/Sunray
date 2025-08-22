#include "data_convert.hpp"
#include "mavlink_control.h"
#include <sunray_msgs/ViobotState.h>
#include <sunray_viobot_unit/algo_ctrl.h>
#include <sunray_viobot_unit/algo_status.h>

#define ODOM_TIMEOUT 0.3

sunray_msgs::ViobotState viobot_state;

ros::Publisher viobot_state_pub;  // viobot状态发布
ros::Subscriber sub_stereo3_pose; // viobot里程计接收

ros::Subscriber algo_status_sub; // viobot算法状态接收
ros::Publisher pub_stereo3_ctrl; // viobot算法控制指令发布

ros::Timer timer_check_timeout; // 里程计在线检查定时器

sunray_viobot_unit::algo_ctrl algo_set;
bool is_viobot_start;
bool odom_timeout;

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
}

void status_callback(const sunray_viobot_unit::algo_status::ConstPtr &msg)
{
    viobot_state.algo_status = msg->algo_status;
    if (msg->algo_status == "ready")
    {
        is_viobot_start = false;
    }
    else if (msg->algo_status == "stereo3_initializing" || msg->algo_status == "stereo3_running")
    {
        // 算法初始化
        is_viobot_start = true;
    }

    viobot_state.vio_start = is_viobot_start;

    if (is_viobot_start == false) // 如果没有开启算法，自动开启
    {
        algo_set.algo_enable = true;
        pub_stereo3_ctrl.publish(algo_set);
    }
}

void timer_check_timeout_cb(const ros::TimerEvent &event)
{
    odom_timeout = (ros::Time::now() - viobot_state.header.stamp).toSec() > ODOM_TIMEOUT;
    viobot_state.odom_valid = !odom_timeout;

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

    // 初始化相关参数
    is_viobot_start = false;
    algo_set.algo_enable = false;
    algo_set.algo_reboot = false;
    algo_set.algo_reset = false;
    odom_timeout = false;

    mavlink_init(uart_name.c_str(), baudrate);
    // mavlink_control();
    mavlink_send_odometry_thread(); // 先保存再在另外的线程里面发送
    viobot_state_pub = nh.advertise<sunray_msgs::ViobotState>("/sunray_viobot/ViobotState", 10);
    sub_stereo3_pose = nh.subscribe("/baton/stereo3/odometry", 2, pose_callback);

    // 定时检查外部定位数据是否超时
    timer_check_timeout = nh.createTimer(ros::Duration(0.05), timer_check_timeout_cb);

    // 【订阅】viobot/ROS_interfaces -> 本节点
    algo_status_sub = nh.subscribe<sunray_viobot_unit::algo_status>("/baton/algo_status", 2, status_callback);
    // 【发布】发布算法启动话题
    pub_stereo3_ctrl = nh.advertise<sunray_viobot_unit::algo_ctrl>("/baton/stereo3_ctrl", 2);

    ros::spin();
    mavlink_deinit();
    return 0;
}
