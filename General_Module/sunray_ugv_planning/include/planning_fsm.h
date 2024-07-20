#ifndef PLANNING_FSM_H
#define PLANNING_FSM_H

#include <ros/ros.h>
#include <boost/format.hpp>

#include <Eigen/Eigen>
#include <iostream>
#include <algorithm>
#include <iostream>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

#include <std_msgs/Bool.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <sunray_msgs/UGVControlCMD.h>


#include "A_star.h"
#include "printf_utils.h"

#include "angles/angles.h"
#include <tf2/utils.h> // getYaw

using namespace std;

#define MIN_DIS 0.1

namespace ugv_planning
{

class Planning_FSM
{
private:
    // ros nh
    ros::NodeHandle global_planner_nh;
    // 【订阅】目标点
    ros::Subscriber goal_sub;
    // 【订阅】无人车odom
    ros::Subscriber ugv_odom_sub;
    // 【订阅】传感器数据 - 全局点云、局部点云、Scan、viobot
    ros::Subscriber Gpointcloud_sub;
    ros::Subscriber Lpointcloud_sub;
    ros::Subscriber laserscan_sub;
    ros::Subscriber viobot_pcl_sub;
    // 【发布】控制指令
    ros::Publisher ugv_control_cmd_pub;
    // 【发布】规划路径(rviz)
    ros::Publisher ros_path_pub;
    // 【定时器】主循环定时器
    ros::Timer mainloop_timer;
    // 【定时器】路径点发布定时器
    ros::Timer path_point_pub_timer;
    // 五种状态机
    enum EXEC_STATE
    {
      WAIT_GOAL,
      PLAN,
      PATH_TRACKING,
      STOP
    };
    EXEC_STATE exec_state;
    // 无人车名字                             
    string ugv_name;    
    // 无人车编号                         
    int ugv_id;                                     
    // 无人车高度
    double ugv_height;
    // 传感器输入flag
    int map_input_source;
    // 路径重规划时间
    double replan_time;
    double path_point_pub_frequency;
    int counter_search;
    // A星规划器
    Astar::Ptr Astar_ptr;
    // A星规划器状态
    int astar_state;
    // 发布的控制指令
    sunray_msgs::UGVControlCMD UGV_CMD;
    // 无人车里程计信息
    nav_msgs::Odometry ugv_odom;
    // 无人车yaw
    float ugv_yaw;
    // 规划得到的路径
    nav_msgs::Path path_cmd;
    // 路经点总数
    int Num_total_wp;
    // 当前执行ID
    int cur_id;
    // 期望偏航角
    float yaw_ref;
    // 规划初始状态及终端状态
    Eigen::Vector3d start_pos, goal_pos;
    // 规划器状态
    bool odom_ready;
    bool map_ready;
    bool get_goal; 
    bool path_ok;
    // 上一条轨迹开始时间
    ros::Time tra_start_time;    
    
    // 回调函数
    void goal_cb(const geometry_msgs::PoseStampedConstPtr& msg);
    void ugv_odom_cb(const nav_msgs::Odometry::ConstPtr &msg);
    void Gpointcloud_cb(const sensor_msgs::PointCloud2ConstPtr &msg);
    void Lpointcloud_cb(const sensor_msgs::PointCloud2ConstPtr &msg);
    void viobot_cb(const sensor_msgs::PointCloud2ConstPtr &msg);
    void laser_cb(const sensor_msgs::LaserScanConstPtr &msg);
    void mainloop_cb(const ros::TimerEvent& e);
    void path_point_pub_cb(const ros::TimerEvent& e);
   
    // 【获取当前时间函数】 单位：秒
    float get_time_in_sec(const ros::Time& begin_time);
    int get_start_point_id(void);
    void printf_exec_state();

    // tool function
    const int get_track_point_id();

    
public:
    Planning_FSM(/* args */)
    {    
    }
    ~Planning_FSM()
    {
    }

    void init(ros::NodeHandle& nh);
};

}

#endif
