#include "ros_msg_utils.h"
#include "ExternalPosition.h"

#define PX4_TIMEOUT 2.0       // px4飞控状态超时
#define TRAJECTORY_WINDOW 50  // 轨迹窗口大小

class ExternalFusion
{
private:
    std::string node_name;                                  // 节点名称
    std::string uav_name{""};                               // 无人机名称
    int uav_id;                                             // 无人机编号
    int external_source;                                    // 外部定位数据来源
    std::vector<geometry_msgs::PoseStamped> uav_pos_vector; // 无人机轨迹容器,用于rviz显示
    ros::Time px4_state_time;                               // 无人机状态时间戳
    
    ExternalPosition ext_pos;                               // 外部定位源的回调和处理

    // ROS话题订阅句柄
    ros::Subscriber px4_state_sub;          // 【订阅】无人机状态订阅
    ros::Subscriber px4_extended_state_sub; // 【订阅】无人机状态订阅
    ros::Subscriber px4_battery_sub;        // 【订阅】无人机电池状态订阅
    ros::Subscriber px4_pose_sub;           // 【订阅】无人机位置订阅
    ros::Subscriber px4_vel_sub;            // 【订阅】无人机速度订阅
    ros::Subscriber px4_att_sub;            // 【订阅】无人机姿态订阅
    ros::Subscriber px4_gps_satellites_sub; // 【订阅】无人机gps卫星状态订阅
    ros::Subscriber px4_gps_state_sub;      // 【订阅】无人机gps状态订阅
    ros::Subscriber px4_gps_raw_sub;        // 【订阅】无人机gps原始数据订阅
    ros::Subscriber px4_pos_target_sub;     // 【订阅】px4目标订阅 位置 速度加 速度
    ros::Subscriber px4_att_target_sub;     // 【订阅】无人机姿态订阅

    // ROS话题发布句柄
    ros::Publisher uav_odom_pub;       // 【发布】无人机里程计发布
    ros::Publisher uav_trajectory_pub; // 【发布】无人机轨迹发布
    ros::Publisher uav_mesh_pub;       // 【发布】无人机mesh发布
    ros::Publisher px4_state_pub;      // 【发布】无人机状态

    // 定时器句柄
    ros::Timer timer_rviz_pub;        // 定时发布rviz显示消息
    ros::Timer timer_pub_px4_state;   // 定时发布px4_state

public:
    ExternalFusion() {};
    ~ExternalFusion() {};

    sunray_msgs::PX4State px4_state;                        // 无人机状态信息汇总（用于发布）

    void init(ros::NodeHandle &nh);                                                 // 初始化
    void show_px4_state();                                                          // 显示无人机状态
    void px4_state_callback(const mavros_msgs::State::ConstPtr &msg);               // 无人机状态回调函数
    void px4_extended_state_callback(const mavros_msgs::ExtendedState::ConstPtr &msg);               // 无人机状态回调函数
    void px4_battery_callback(const sensor_msgs::BatteryState::ConstPtr &msg);      // 无人机电池状态回调函数
    void px4_pose_callback(const geometry_msgs::PoseStamped::ConstPtr &msg);        // 无人机位置回调函数
    void px4_vel_callback(const geometry_msgs::TwistStamped::ConstPtr &msg);        // 无人机位置回调函数
    void px4_att_callback(const sensor_msgs::Imu::ConstPtr &msg);                   // 无人机姿态回调函数 从imu获取解析
    void px4_gps_satellites_callback(const std_msgs::UInt32::ConstPtr &msg);        // 无人机gps卫星状态回调函数
    void px4_gps_state_callback(const sensor_msgs::NavSatFix::ConstPtr &msg);       // 无人机gps状态回调函数
    void px4_gps_raw_callback(const mavros_msgs::GPSRAW::ConstPtr &msg);            // 无人机gps原始数据回调函数
    void px4_pos_target_callback(const mavros_msgs::PositionTarget::ConstPtr &msg); // 无人机位置设定值回调函数
    void px4_att_target_callback(const mavros_msgs::AttitudeTarget::ConstPtr &msg); // 无人机姿态设定值回调函数

    void timer_pub_px4_state_cb(const ros::TimerEvent &event);                      // 定时器回调函数
    void timer_rviz(const ros::TimerEvent &e);  
};