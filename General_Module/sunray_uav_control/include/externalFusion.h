#include "ExternalPosition.h"
#include "ExternalPositionFactory.h"
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

enum external_source
{
    ODOM = 0,
    POSE,
    GAZEBO,
    MOCAP,
    VIOBOT,
    GPS,
    RTK,
    VINS
};

struct PositionState
{
    double pos_x;
    double pos_y;
    double pos_z;
    double vel_x;
    double vel_y;
    double vel_z;
    double att_x;
    double att_y;
    double att_z;
    double att_w;
    double roll;
    double pitch;
    double yaw;
};

struct PX4State // 无人机状态集合
{
    bool connected;
    bool armed;
    bool landed;
    float battery;
    float battery_percentage;
    std::string mode;
    PositionState position_state;
};

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
    bool listen_uav_state;                                  // 是否订阅无人机状态
    geometry_msgs::PoseStamped vision_pose;                 // vision_pose消息
    sunray_msgs::UAVState uav_state;                        // 无人机状态信息
    std::vector<geometry_msgs::PoseStamped> uav_pos_vector; // 无人机轨迹容器,用于rviz显示
    PX4State px4_state;                                     // 无人机状态集合
    PositionState external_state;                           // 外部定位数据
    PositionState err_state;                                // 状态误差

    ExternalPositioFactory factory;
    std::shared_ptr<ExternalPosition> external_position;

    ros::NodeHandle nh_;             // ros节点句柄
    ros::Subscriber px4_state_sub;   // 无人机状态订阅
    ros::Subscriber px4_battery_sub; // 无人机电池状态订阅
    ros::Subscriber px4_odom_sub;    // 无人机里程计订阅
    ros::Subscriber px4_att_sub;     // 无人机姿态订阅

    ros::Publisher odom_state_pub;           // 发布定位状态
    ros::Publisher uav_state_pub;            // 无人机状态发布
    ros::Publisher uav_pos_vector_pub;       // 无人机轨迹发布
    ros::Publisher uav_pos_vector_clear_pub; // 无人机轨迹清空发布

    ros::Timer timer_task; // 定时器

public:
    ExternalFusion(/* args */);
    ~ExternalFusion();

    std::map<int, std::string> source_map; // 外部定位数据来源映射
    // std::map<int, ExternalPosition> obj_map; // 外部定位数据来源映射

    void init(ros::NodeHandle &nh);                                            // 初始化
    void show_px4_state();                                                     // 显示无人机状态
    void px4_state_callback(const mavros_msgs::State::ConstPtr &msg);          // 无人机状态回调函数
    void px4_battery_callback(const sensor_msgs::BatteryState::ConstPtr &msg); // 无人机电池状态回调函数
    void px4_odom_callback(const geometry_msgs::PoseStamped::ConstPtr &msg);   // 无人机里程计回调函数（同时包含了位置和速度）
    void px4_att_callback(const sensor_msgs::Imu::ConstPtr &msg);              // 无人机姿态回调函数 从imu获取解析
    void timer_callback(const ros::TimerEvent &event);                         // 定时器回调函数
    void px4_odom_callback(const nav_msgs::Odometry::ConstPtr &msg);           // 无人机里程计回调函数（同时包含了位置和速度）
};

ExternalFusion::~ExternalFusion()
{
}
