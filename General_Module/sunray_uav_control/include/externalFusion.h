#include "ExternalPosition.h"
#include <math.h>
#include <map>

#define MOCAP_TIMEOUT 0.35
#define GAZEBO_TIMEOUT 0.1
#define T265_TIMEOUT 0.3
#define UWB_TIMEOUT 0.1
#define GPS_TIMEOUT 1.0
#define VINS_TIMEOUT 0.35
#define VIOBOT_TIMEOUT 0.35
#define ODOM_TIMEOUT 0.35

// 外部定位数据来源枚举
enum external_source
{
    ODOM = 0,
    POSE = 1,
    GAZEBO = 2,
    MOCAP = 3,
    VIOBOT = 4,
    GPS = 5,
    RTK = 6,
    VINS = 7
};

std::map<int, std::string> ERR_MSG = {
    {1, "Warning: The error between external state and px4 state is too large!"},
    {2, "Warning: The external position is timeout!"}};

class ExternalFusion
{
private:
    std::string node_name;                                  // 节点名称
    std::string uav_name{""};                               // 无人机名称
    std::string source_topic{""};                           // 外部定位数据来源
    std::string pub_topic{""};                              // 发布到无人机话题
    std::string uav_prefix{""};                             // 无人机前缀 uav_name+uav_id
    int uav_id;                                             // 无人机编号
    int external_source;                                    // 外部定位数据来源
    double range_hight;                                     // 激光定位数据
    double source_timeout;                                  // 外部定位数据超时时间
    double timeout_counter;                                 // 定位超时时长
    double jump_threshold;                                  // 判断数据跳变的阈值
    double pub_rate;                                        // 数据发布频率
    bool is_timeout;                                        // 定位超时标志
    geometry_msgs::PoseStamped vision_pose;                 // vision_pose消息
    geometry_msgs::PoseStamped last_vision_position;        // 上一刻vision_pose消息
    sunray_msgs::UAVState uav_state;                        // 无人机状态信息
    std::vector<geometry_msgs::PoseStamped> uav_pos_vector; // 无人机轨迹容器,用于rviz显示
    std::set<int> err_msg;                                  // 错误信息集合
    PoseState external_state;                               // 外部定位数据
    PoseState err_state;                                    // 状态误差
    ros::Time px4_state_time;                               // 无人机状态时间戳
    sunray_msgs::ExternalOdom external_odom;                // 外部里程计数据（用于发布）
    sunray_msgs::PX4State px4_state;                        // 无人机状态信息汇总（用于发布）
    ExternalPosition external_position;                     // 外部定位源的回调和处理

    ros::NodeHandle nh_;                    // ros节点句柄
    ros::Subscriber px4_state_sub;          // 【订阅】无人机状态订阅
    ros::Subscriber px4_battery_sub;        // 【订阅】无人机电池状态订阅
    ros::Subscriber px4_odom_sub;           // 【订阅】无人机里程计订阅
    ros::Subscriber px4_pose_sub;           // 【订阅】无人机位置订阅
    ros::Subscriber px4_vel_sub;            // 【订阅】无人机速度订阅
    ros::Subscriber px4_att_sub;            // 【订阅】无人机姿态订阅
    ros::Subscriber px4_gps_satellites_sub; // 【订阅】无人机gps卫星状态订阅
    ros::Subscriber px4_gps_state_sub;      // 【订阅】无人机gps状态订阅
    ros::Subscriber px4_gps_raw_sub;        // 【订阅】无人机gps原始数据订阅

    ros::Subscriber px4_pos_target_sub; // 【订阅】px4目标订阅 位置 速度加 速度
    ros::Subscriber px4_att_target_sub; // 【订阅】无人机姿态订阅

    ros::Publisher vision_pose_pub;    // 【发布】发布vision_pose消息
    ros::Publisher odom_state_pub;     // 【发布】发布定位状态
    ros::Publisher uav_odom_pub;       // 【发布】无人机里程计发布
    ros::Publisher uav_trajectory_pub; // 【发布】无人机轨迹发布
    ros::Publisher uav_mesh_pub;       // 【发布】无人机mesh发布
    ros::Publisher px4_state_pub;      // 【发布】无人机状态

    ros::Timer timer_check;         // 定时器
    ros::Timer timer_pub_mavros;    // 定时器发布mavros/vision_pose/pose
    ros::Timer timer_rviz_pub;      // 定时发布rviz显示消息
    ros::Timer timer_px4_state_pub; // 定时发布px4_state

public:
    ExternalFusion(/* args */);
    ~ExternalFusion();

    std::map<int, std::string> source_map; // 外部定位数据来源映射
    // std::map<int, ExternalPosition> obj_map; // 外部定位数据来源映射

    void init(ros::NodeHandle &nh); // 初始化
    void setup_rviz_color();
    void show_px4_state();                                                          // 显示无人机状态
    void px4_state_callback(const mavros_msgs::State::ConstPtr &msg);               // 无人机状态回调函数
    void px4_battery_callback(const sensor_msgs::BatteryState::ConstPtr &msg);      // 无人机电池状态回调函数
    void px4_odom_callback(const geometry_msgs::PoseStamped::ConstPtr &msg);        // 无人机里程计回调函数（同时包含了位置和速度）
    void px4_att_callback(const sensor_msgs::Imu::ConstPtr &msg);                   // 无人机姿态回调函数 从imu获取解析
    void timer_callback(const ros::TimerEvent &event);                              // 定时器回调函数
    void timer_update_external_state(const ros::TimerEvent &event);                 // 定时器更新和发布
    void timer_rviz(const ros::TimerEvent &e);                                      // 定时发布rviz显示消息
    void px4_pose_callback(const geometry_msgs::PoseStamped::ConstPtr &msg);        // 无人机位置回调函数
    void px4_vel_callback(const geometry_msgs::TwistStamped::ConstPtr &msg);        // 无人机位置回调函数
    void px4_gps_satellites_callback(const std_msgs::UInt32::ConstPtr &msg);        // 无人机gps卫星状态回调函数
    void px4_gps_state_callback(const sensor_msgs::NavSatFix::ConstPtr &msg);       // 无人机gps状态回调函数
    void px4_gps_raw_callback(const mavros_msgs::GPSRAW::ConstPtr &msg);            // 无人机gps原始数据回调函数
    void px4_att_target_callback(const mavros_msgs::AttitudeTarget::ConstPtr &msg); // 无人机姿态设定值回调函数
    void px4_pos_target_callback(const mavros_msgs::PositionTarget::ConstPtr &msg); // 无人机位置设定值回调函数
    bool timeoutCheck();                                                            // 外部定位数据超时检测
    bool checkJump();                                                               // 外部定位数据跳变检测
    // void px4_odom_callback(const nav_msgs::Odometry::ConstPtr &msg);           // 无人机里程计回调函数（同时包含了位置和速度 但是是机体系）
};

ExternalFusion::~ExternalFusion()
{
}
