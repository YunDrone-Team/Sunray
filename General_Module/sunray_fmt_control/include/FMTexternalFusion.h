#ifndef FMT_EXTERNAL_FUSION_H
#define FMT_EXTERNAL_FUSION_H

#include "ros_msg_utils.h"
#include "sunray_logger.h"
#include <mavros_msgs/StreamRate.h>
#include <math.h>
#include <map>
#include <signal.h>
#include <set>

using namespace std;
using namespace sunray_logger;

#define FMT_TIMEOUT 2.0       // FMT状态超时
#define TRAJECTORY_WINDOW 50  // 轨迹滑窗大小

// 滑动平均滤波器
class MovingAverageFilter
{
public:
    MovingAverageFilter(int size = 5)
    {
        this->size = size;
        this->data = new double[size];
        this->sum = 0;
        this->count = 0;
        this->index = 0;
    }

    ~MovingAverageFilter()
    {
        delete[] data;
    }

    void setSize(int size)
    {
        this->size = size;
        delete[] data;
        this->data = new double[size];
        this->sum = 0;
        this->count = 0;
        this->index = 0;
    }

    void addData(double value)
    {
        if (count < size)
        {
            sum += value;
            data[count++] = value;
        }
        else
        {
            sum -= data[index];
            sum += value;
            data[index] = value;
            index = (index + 1) % size;
        }
    }

    double getAverage()
    {
        return sum / count;
    }

    double filter(double value)
    {
        addData(value);
        return getAverage();
    }

private:
    int size;
    double *data;
    double sum;
    int count;
    int index;
};

#define ODOM_TIMEOUT 0.3

class ExternalPosition
{
public:
    ExternalPosition()
    {
    }
    
    sunray_msgs::ExternalOdom external_odom;                // 外部定位数据
    
    void init(ros::NodeHandle &nh, int external_source = 0, std::string source_topic_name = "Odometry", bool range_sensor = false)
    {
        // 初始化参数
        nh.param<int>("uav_id", uav_id, 1);
        nh.param<std::string>("uav_name", uav_name, "uav");
        uav_name = "/" + uav_name + std::to_string(uav_id);

        // 定时检查外部定位数据是否超时
        timer_check_timeout = nh.createTimer(ros::Duration(0.05), &ExternalPosition::timer_check_timeout_cb, this);

        // 初始化外部定位状态
        external_odom.header.stamp = ros::Time::now();
        external_odom.external_source = external_source;
        external_odom.odom_valid = false;
        external_odom.position[0] = -0.01;
        external_odom.position[1] = -0.01;
        external_odom.position[2] = -0.01;
        external_odom.velocity[0] = 0.0;
        external_odom.velocity[1] = 0.0;
        external_odom.velocity[2] = 0.0;
        external_odom.attitude_q.x = 0;
        external_odom.attitude_q.y = 0;
        external_odom.attitude_q.z = 0;
        external_odom.attitude_q.w = 1;
        external_odom.attitude[0] = 0.0;
        external_odom.attitude[1] = 0.0;
        external_odom.attitude[2] = 0.0;

        // MOCAP和VIOBOT两种外部定位源
        switch (external_source)
        {
        case sunray_msgs::ExternalOdom::ODOM:
            // 【订阅】里程计数据(坐标系:机体系) odom -> 本节点
            odom_sub = nh.subscribe<nav_msgs::Odometry>(source_topic_name, 10, &ExternalPosition::OdomCallback, this);
            break;
        case sunray_msgs::ExternalOdom::POSE:
            // 【订阅】定位数据(坐标系:世界系) pose -> 本节点
            pos_sub = nh.subscribe<geometry_msgs::PoseStamped>(source_topic_name, 10, &ExternalPosition::PosCallback, this);
            break;
        case sunray_msgs::ExternalOdom::GAZEBO:
            source_topic_name = uav_name + "sunray/gazebo/pose";
            // 【订阅】gazebo仿真定位数据(坐标系:世界系) /gazebo/pose -> 本节点
            odom_sub = nh.subscribe<nav_msgs::Odometry >(source_topic_name, 10, &ExternalPosition::OdomCallback, this);
            break;
        case sunray_msgs::ExternalOdom::MOCAP:
            // 【订阅】动捕的定位数据(坐标系:动捕系统惯性系) vrpn -> 本节点
            pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node_" + std::to_string(uav_id) + uav_name + "/pose", 1, &ExternalPosition::PosCallback, this);
            // 【订阅】动捕的定位数据(坐标系:动捕系统惯性系) vrpn -> 本节点
            vel_sub = nh.subscribe<geometry_msgs::TwistStamped>("/vrpn_client_node_" + std::to_string(uav_id) + uav_name + "/twist", 1, &ExternalPosition::VelCallback, this);
            break;
        case sunray_msgs::ExternalOdom::VIOBOT:
            moving_average_filter.setSize(1);
            odom_sub = nh.subscribe<nav_msgs::Odometry>(source_topic_name, 10, &ExternalPosition::viobotCallback, this);
            break;
        default:
            Logger::print_color(int(LogColor::red), LOG_BOLD, "Unknown external position source type - [", external_source, "]");
            break;
        }
    }

    // 实现外部定位源话题回调函数
    void OdomCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        // 四元素转rpy
        tf2::Quaternion quaternion;
        tf2::fromMsg(msg->pose.pose.orientation, quaternion);
        double roll, pitch, yaw;
        tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
        external_odom.header.stamp = ros::Time::now();
        external_odom.position[0] = msg->pose.pose.position.x;
        external_odom.position[1] = msg->pose.pose.position.y;
        external_odom.position[2] = msg->pose.pose.position.z;
        external_odom.velocity[0] = msg->twist.twist.linear.x;
        external_odom.velocity[1] = msg->twist.twist.linear.y;
        external_odom.velocity[2] = msg->twist.twist.linear.z;
        external_odom.attitude_q.x = msg->pose.pose.orientation.x;
        external_odom.attitude_q.y = msg->pose.pose.orientation.y;
        external_odom.attitude_q.z = msg->pose.pose.orientation.z;
        external_odom.attitude_q.w = msg->pose.pose.orientation.w;
        external_odom.attitude[0] = roll;
        external_odom.attitude[1] = pitch;
        external_odom.attitude[2] = yaw;
    }

    // 实现外部定位源话题回调函数
    void PosCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        // 四元素转rpy
        tf2::Quaternion quaternion;
        tf2::fromMsg(msg->pose.orientation, quaternion);
        double roll, pitch, yaw;
        tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
        external_odom.header.stamp = ros::Time::now();
        external_odom.position[0] = msg->pose.position.x;
        external_odom.position[1] = msg->pose.position.y;
        external_odom.position[2] = msg->pose.position.z;
        external_odom.attitude_q.x = msg->pose.orientation.x;
        external_odom.attitude_q.y = msg->pose.orientation.y;
        external_odom.attitude_q.z = msg->pose.orientation.z;
        external_odom.attitude_q.w = msg->pose.orientation.w;
        external_odom.attitude[0] = roll;
        external_odom.attitude[1] = pitch;
        external_odom.attitude[2] = yaw;
    }

    void viobotCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        // 四元素转rpy
        tf2::Quaternion quaternion;
        tf2::fromMsg(msg->pose.pose.orientation, quaternion);
        double roll, pitch, yaw;
        external_odom.header.stamp = ros::Time::now();
        external_odom.position[0] = msg->pose.pose.position.x;
        external_odom.position[1] = msg->pose.pose.position.y;
        external_odom.position[2] = msg->pose.pose.position.z;
        external_odom.velocity[0] = msg->twist.twist.linear.x;
        external_odom.velocity[1] = msg->twist.twist.linear.y;
        external_odom.velocity[2] = msg->twist.twist.linear.z;

        tf2::Quaternion q;
        q.setW(msg->pose.pose.orientation.w);
        q.setX(msg->pose.pose.orientation.x);
        q.setY(msg->pose.pose.orientation.y);
        q.setZ(msg->pose.pose.orientation.z);
        
        // 绕 Z 轴旋转 90°
        tf2::Quaternion q_z;
        q_z.setRPY(0, 0, M_PI / 2);
        
        // 绕 Y 轴旋转 -90°
        tf2::Quaternion q_y;
        q_y.setRPY(0, -M_PI / 2, 0);
        
        // 组合旋转（顺序：先 q_z，再 q_y）
        q = q * q_z * q_y;
        
        // 转欧拉角
        tf2::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);

        external_odom.attitude_q.x = q.getX();
        external_odom.attitude_q.y = q.getY();
        external_odom.attitude_q.z = q.getZ();
        external_odom.attitude_q.w = q.getW();
        external_odom.attitude[0] = roll;
        external_odom.attitude[1] = pitch;
        external_odom.attitude[2] = yaw;
    }

    void VelCallback(const geometry_msgs::TwistStamped::ConstPtr &msg)
    {
        external_odom.velocity[0] = msg->twist.linear.x;
        external_odom.velocity[1] = msg->twist.linear.y;
        external_odom.velocity[2] = msg->twist.linear.z;
    }

    void timer_check_timeout_cb(const ros::TimerEvent &event)
    {
        odom_timeout = (ros::Time::now() - external_odom.header.stamp).toSec() > ODOM_TIMEOUT;
        external_odom.odom_valid = !odom_timeout;
    }

    sunray_msgs::ExternalOdom GetExternalOdom()
    {
        return external_odom;
    }

private:
    ros::Subscriber odom_sub;
    ros::Subscriber pos_sub;
    ros::Subscriber vel_sub;
    ros::Timer timer_check_timeout;
    bool odom_timeout;
    int uav_id;
    std::string uav_name;
    MovingAverageFilter moving_average_filter;
};


class FMTExternalFusion
{
private:
    std::string uav_name;                               // 无人机名称
    int uav_id;                                         // 无人机编号
    int external_source;                                // 外部定位数据来源
    std::set<int> err_msg;                              // 错误消息集合
    std::vector<geometry_msgs::PoseStamped> uav_pos_vector; // 无人机轨迹容器,用于rviz显示
    ros::Time fmt_state_time;                           // FMT状态时间戳
    ros::Timer timer_pub_fmt_state;                     // 定时器 - 发布FMT状态
    ros::Timer time_rviz_pub;                           // 定时器 - RVIZ可视化发布
    ros::Timer timer_pub_vision_pose;                   // 定时器 - 发布vision_pose   
    bool enable_vision_pose{true};                      // 是否发布vision_pose
    geometry_msgs::PoseStamped vision_pose;                 // vision_pose消息

    ExternalPosition ext_pos;                           // 外部定位源的回调和处理

    // 订阅节点句柄
    ros::Subscriber fmt_state_sub;                      // 【订阅】FMT状态
    ros::Subscriber fmt_extended_state_sub;             // 【订阅】FMT扩展状态
    ros::Subscriber fmt_battery_sub;                    // 【订阅】FMT电池状态
    ros::Subscriber fmt_pose_sub;                       // 【订阅】FMT本地位置
    ros::Subscriber fmt_vel_sub;                        // 【订阅】FMT本地速度
    ros::Subscriber fmt_att_sub;                        // 【订阅】FMT姿态
    ros::Subscriber fmt_gps_satellites_sub;             // 【订阅】FMT GPS卫星数量
    ros::Subscriber fmt_gps_state_sub;                  // 【订阅】FMT GPS状态
    ros::Subscriber fmt_gps_raw_sub;                    // 【订阅】FMT GPS原始数据
    ros::Subscriber fmt_pos_target_sub;                 // 【订阅】FMT位置设定值
    ros::Subscriber fmt_att_target_sub;                 // 【订阅】FMT姿态设定值

    // 发布节点
    ros::Publisher uav_odom_pub;                        // 【发布】无人机里程计
    ros::Publisher uav_trajectory_pub;                 // 【发布】无人机轨迹
    ros::Publisher uav_mesh_pub;                       // 【发布】无人机MESH图标
    ros::Publisher vision_pose_pub;                     // 【发布】vision_pose
    ros::Publisher fmt_state_pub;                       // 【发布】FMT综合状态

    // 服务节点
    ros::ServiceClient fmt_stream_rate_client;          // 【服务】FMT数据流设置

public:
    FMTExternalFusion();
    ~FMTExternalFusion();

    sunray_msgs::PX4State fmt_state;                    // FMT状态信息汇总
    std::map<int, std::string> source_map;              // 外部定位数据来源映射

    void init(ros::NodeHandle &nh);                     // 初始化
    void init_fmt_state();                              // 初始化FMT状态
    void show_fmt_state();                              // 显示FMT状态
    
    // FMT特有函数
    bool set_fmt_stream_rate(uint8_t stream_id, uint16_t message_rate, bool on_off); // 设置FMT数据流

    // 回调函数
    void fmt_state_callback(const mavros_msgs::State::ConstPtr &msg);
    void fmt_extended_state_callback(const mavros_msgs::ExtendedState::ConstPtr &msg);
    void fmt_battery_callback(const sensor_msgs::BatteryState::ConstPtr &msg);
    void fmt_pose_callback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void fmt_vel_callback(const geometry_msgs::TwistStamped::ConstPtr &msg);
    void fmt_att_callback(const sensor_msgs::Imu::ConstPtr &msg);
    void fmt_gps_satellites_callback(const std_msgs::UInt32::ConstPtr &msg);
    void fmt_gps_state_callback(const sensor_msgs::NavSatFix::ConstPtr &msg);
    void fmt_gps_raw_callback(const mavros_msgs::GPSRAW::ConstPtr &msg);
    void fmt_pos_target_callback(const mavros_msgs::PositionTarget::ConstPtr &msg);
    void fmt_att_target_callback(const mavros_msgs::AttitudeTarget::ConstPtr &msg);
    
    void timer_pub_vision_pose_cb(const ros::TimerEvent &event);
    // 定时器回调函数
    void timer_pub_fmt_state_cb(const ros::TimerEvent &event);
    void timer_rviz(const ros::TimerEvent &event);
};

FMTExternalFusion::~FMTExternalFusion()
{
}


#endif // FMT_EXTERNAL_FUSION_H
