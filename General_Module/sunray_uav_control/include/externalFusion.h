#include "ros_msg_utils.h"
#include "ExternalPosition.h"
#include <math.h>
#include <map>
#include "printf_format.h"

using namespace sunray_logger;

#define PX4_TIMEOUT 2.0       // px4状态超时
#define TRAJECTORY_WINDOW 50  // 轨迹窗口大小

std::map<int, std::string> ERR_MSG =
    {
        {1, "Warning: The error between external state and px4 state is too large!"},
        {2, "Warning: The external position is timeout!"},
        {3, "Warning: The distance sensor data is timeout!"}};

class ExternalFusion
{
private:
    std::string node_name;                                  // 节点名称
    std::string uav_name{""};                               // 无人机名称
    std::string source_topic{""};                           // 外部定位数据来源
    int uav_id;                                             // 无人机编号
    int external_source;                                    // 外部定位数据来源
    bool enable_vision_pose{true};
    geometry_msgs::PoseStamped vision_pose;                 // vision_pose消息
    std::vector<geometry_msgs::PoseStamped> uav_pos_vector; // 无人机轨迹容器,用于rviz显示
    std::set<int> err_msg;                                  // 错误信息集合
    ros::Time px4_state_time;                               // 无人机状态时间戳
    bool enable_range_sensor;                               // 是否使用距离传感器数据
    sunray_msgs::PX4State px4_state;                        // 无人机状态信息汇总（用于发布）

    ExternalPosition ext_pos;                     // 外部定位源的回调和处理

    ros::NodeHandle nh_;                    // ros节点句柄
    ros::Subscriber px4_state_sub;          // 【订阅】无人机状态订阅
    ros::Subscriber px4_extended_state_sub; // 【订阅】无人机状态订阅
    ros::Subscriber px4_battery_sub;        // 【订阅】无人机电池状态订阅
    ros::Subscriber px4_pose_sub;           // 【订阅】无人机位置订阅
    ros::Subscriber px4_vel_sub;            // 【订阅】无人机速度订阅
    ros::Subscriber px4_att_sub;            // 【订阅】无人机姿态订阅
    ros::Subscriber px4_gps_satellites_sub; // 【订阅】无人机gps卫星状态订阅
    ros::Subscriber px4_gps_state_sub;      // 【订阅】无人机gps状态订阅
    ros::Subscriber px4_gps_raw_sub;        // 【订阅】无人机gps原始数据订阅
    ros::Subscriber px4_distance_sub;       // 【订阅】无人机距离传感器原始数据订阅
    ros::Subscriber px4_pos_target_sub;     // 【订阅】px4目标订阅 位置 速度加 速度
    ros::Subscriber px4_att_target_sub;     // 【订阅】无人机姿态订阅

    ros::Publisher vision_pose_pub;    // 【发布】发布vision_pose消息
    ros::Publisher uav_odom_pub;       // 【发布】无人机里程计发布
    ros::Publisher uav_trajectory_pub; // 【发布】无人机轨迹发布
    ros::Publisher uav_mesh_pub;       // 【发布】无人机mesh发布
    ros::Publisher px4_state_pub;      // 【发布】无人机状态

    ros::Timer timer_pub_vision_pose; // 定时器发布mavros/vision_pose/pose
    ros::Timer timer_rviz_pub;        // 定时发布rviz显示消息
    ros::Timer timer_pub_px4_state;   // 定时发布px4_state

public:
    ExternalFusion(/* args */);
    ~ExternalFusion();

    std::map<int, std::string> source_map; // 外部定位数据来源映射

    void init(ros::NodeHandle &nh);                                                 // 初始化
    void show_px4_state();                                                          // 显示无人机状态
    void px4_state_callback(const mavros_msgs::State::ConstPtr &msg);               // 无人机状态回调函数
    void px4_extended_state_callback(const mavros_msgs::ExtendedState::ConstPtr &msg);               // 无人机状态回调函数
    void px4_battery_callback(const sensor_msgs::BatteryState::ConstPtr &msg);      // 无人机电池状态回调函数
    void px4_att_callback(const sensor_msgs::Imu::ConstPtr &msg);                   // 无人机姿态回调函数 从imu获取解析
    void timer_pub_px4_state_cb(const ros::TimerEvent &event);                      // 定时器回调函数
    void timer_pub_vision_pose_cb(const ros::TimerEvent &event);                 // 定时器更新和发布
    void timer_rviz(const ros::TimerEvent &e);                                      // 定时发布rviz显示消息
    void px4_pose_callback(const geometry_msgs::PoseStamped::ConstPtr &msg);        // 无人机位置回调函数
    void px4_vel_callback(const geometry_msgs::TwistStamped::ConstPtr &msg);        // 无人机位置回调函数
    void px4_gps_satellites_callback(const std_msgs::UInt32::ConstPtr &msg);        // 无人机gps卫星状态回调函数
    void px4_gps_state_callback(const sensor_msgs::NavSatFix::ConstPtr &msg);       // 无人机gps状态回调函数
    void px4_gps_raw_callback(const mavros_msgs::GPSRAW::ConstPtr &msg);            // 无人机gps原始数据回调函数
    void px4_att_target_callback(const mavros_msgs::AttitudeTarget::ConstPtr &msg); // 无人机姿态设定值回调函数
    void px4_pos_target_callback(const mavros_msgs::PositionTarget::ConstPtr &msg); // 无人机位置设定值回调函数
    void px4_distance_callback(const sensor_msgs::Range::ConstPtr &msg);             //无人机距离传感器原始数据
    // void px4_odom_callback(const nav_msgs::Odometry::ConstPtr &msg);           // 无人机里程计回调函数（同时包含了位置和速度 但是是机体系）
};

ExternalFusion::~ExternalFusion()
{
}
