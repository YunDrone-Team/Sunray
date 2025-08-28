#pragma once

#include <ros/ros.h>
#include "mavlink_control.h"
#include <sensor_msgs/Imu.h>
#include <sunray_msgs/ViobotState.h>
#include <sunray_viobot_unit/algo_ctrl.h>
#include <sunray_viobot_unit/algo_status.h>
#include <nav_msgs/Odometry.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <Eigen/Eigen>
// #include <vector>
// #include <cmath>

#define ODOM_TIMEOUT 0.3
#define AVERAGE_COUNT 200

class Viobot
{
public:
    Viobot();

    void init(ros::NodeHandle &nh);

    void imuCallback(const sensor_msgs::Imu::ConstPtr &msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
    void algoStatusCallback(const sunray_viobot_unit::algo_status::ConstPtr &msg);
    void timerCheckTimeoutCallback(const ros::TimerEvent &event);

    mavlink_odometry_t getMavlinkOdom() const { return mavlink_odom; }
    std::string getUartName() const { return uart_name; }
    int getBaudrate() const { return baudrate; }

private:
    ros::Subscriber imu_sub_;         // 【订阅】imu
    ros::Subscriber odom_sub_;        // 【订阅】里程计
    ros::Subscriber algo_status_sub_; // 【订阅】算法状态

    ros::Publisher algo_ctrl_pub_;    // 【发布】控制命令
    ros::Publisher viobot_state_pub_; // 【发布】VioBot状态

    ros::Timer check_timeout_timer_; // 【定时器】超时检查

    sunray_msgs::ViobotState viobot_state;  // VioBot状态
    sunray_viobot_unit::algo_ctrl algo_set; // 算法控制
    mavlink_odometry_t mavlink_odom;        // 转换后的mavlink消息

    bool is_viobot_start; // VioBot启动状态
    bool odom_timeout;    // 里程计超时状态

    std::string uart_name; // 串口名称
    int baudrate;          // 波特率

    bool tilted;                                         // 是否倾斜放置
    std::vector<double> ax_values, ay_values, az_values; // 加速度容器
    Eigen::Quaterniond eigen_q_rot;                      // 四元数 eigen
    tf2::Quaternion q_rot;                               // 旋转四元数
    double rot_roll, rot_pitch, rot_yaw;                 // 初始Viobot放置的角度
    bool calculation_done;                               // 偏转角计算是否完成
};
