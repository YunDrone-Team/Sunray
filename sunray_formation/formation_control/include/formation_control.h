#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int8.h>
#include <geometry_msgs/PoseStamped.h>
#include "printf_format.h"
#include <iostream>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include "ros_msg_utils.h"
#include "traj_generator.h"
using namespace sunray_logger;

// 阵型数据
struct FormationData
{
    std::string name;
    double pose_x;
    double pose_y;
    double pose_z;
    double pose_yaw;
};

struct Lissa_params
{
    float lissa_a;
    float lissa_b;
    float lissa_delta;
    float radius_x;
    float radius_y;
    float center_x;
    float center_y;
    float linear_speed;
};

class SunrayFormation
{
public:
    SunrayFormation() {};
    ~SunrayFormation() {};
    void init(ros::NodeHandle &nh_);

private:
    ros::NodeHandle nh;
    bool first_dynamic;                                                      // 重新进入动态编队模式
    bool home_set;                                                           // 是否设置home点
    int agent_id;                                                            // 编号
    int state;                                                               // 状态机
    int formation_type;                                                      // 阵型类型 1:静态阵型 2:动态阵型
    int agent_num;                                                           // 无人机数量
    int agent_type;                                                          // 无人机类型 0:无人机 1:无人车
    int now_idx;                                                             // 当前动态阵型目标点索引
    int dynamic_type;                                                        // 动态阵型类型 0:离线读取 1:在线计算
    float home_pose[3];                                                      // home点
    std::string agent_name;                                                  // 无人机名称
    std::string file_path;                                                   // 配置文件路径
    std::string formation_name;                                              // 阵型名称
    YAML::Node formation_data;                                               // yaml文件数据
    Lissa_params circle_params;                                              // 圆阵参数
    Lissa_params figure_eight_params;                                        // 八字形参数
    traj_generator traj_gen;                                                 // 轨迹生成器
    std::string topic_prefix;                                                // 话题前缀
    std::string node_name;                                                   // 节点名称
    std::map<std::string, FormationData> static_formation_map;               // 静态阵型
    std::map<std::string, std::vector<FormationData>> dynamic_formation_map; // 动态阵型
    std::vector<FormationData> waypoints;                                    // 航点
    std::map<int, nav_msgs::Odometry> agent_state;                           // 无人机状态
    std::map<std::string, int> init_idx;                                     // 动态阵型初始索引
    sunray_msgs::OrcaSetup orca_setup;                                       // 设置orca模式 目标点消息
    sunray_msgs::UAVControlCMD uav_cmd;                                      // 无人机控制指令
    sunray_msgs::UGVControlCMD ugv_cmd;                                       // 无人车控制指令
    sunray_msgs::UAVState uav_state;                                         // 无人机状态
    sunray_msgs::UAVSetup uav_setup;                                         // 无人机模式
    sunray_msgs::OrcaCmd orca_cmd;                                           // orca返回值
    ros::Publisher orca_setup_pub;                                           // 发布orca模式
    ros::Publisher uav_control_cmd;                                          // 发布无人机控制指令
    ros::Publisher uav_setup_pub;                                            // 发布无人机模式
    ros::Publisher ugv_control_cmd;                                          // 发布无人车控制指令
    ros::Subscriber formation_cmd_sub;                                       // 订阅阵型指令
    ros::Subscriber formation_cmd_ground_sub;                                // 订阅地面站阵型指令
    ros::Subscriber orca_cmd_sub;                                            // 订阅orca返回值
    ros::Subscriber goal_sub;                                                // 订阅目标点
    ros::Timer pub_timer;                                                    // 发布无人机状态到orca
    ros::Timer dynamic_timer;                                                // 动态阵型发布
    ros::Timer mode_timer;                                                   // 检查无人机模式
    ros::Time first_dynamic_time;                                            // 第一次进入动态阵型时间
    ros::Time mode_takeoff_time;                                             // 第一次进入模式时间
    ros::Time orca_cmd_time;                                                 // orca返回值时间
    std::map<int, ros::Subscriber> agent_state_sub;                          // 订阅无人机状态
    std::map<int, ros::Publisher> agent_state_pub;                           // 发布无人机状态

    void dynamic_formation_pub(std::string formation_name);                   // 动态编队 离线读取位置 发布位置
    void dynamic_formation_online(std::string name);                          // 动态编队 在线计算
    void leader_formation_pub();                                              // 领航者阵型 发布位置
    void static_formation_pub(std::string formation_name);                    // 预定义阵型 读取文件发布位置
    void formation_cmd_callback(const sunray_msgs::Formation::ConstPtr &msg); // 阵型指令回调
    void orca_cmd_callback(const sunray_msgs::OrcaCmd::ConstPtr &msg);        // orca返回值回调
    void uav_state_cb(const sunray_msgs::UAVState::ConstPtr &msg, int i);     // 无人机状态回调
    void ugv_state_cb(const sunray_msgs::UGVState::ConstPtr &msg, int i);     // 无人车状态回调
    void goal_callback(const geometry_msgs::PoseStamped::ConstPtr &msg);      // 订阅目标点
    void timer_pub_state(const ros::TimerEvent &e);                           // 定时发布无人机状态到orca
    void timer_dynamic_formation(const ros::TimerEvent &e);                   // 定时动态阵型发布
    void agent_mode_check(const ros::TimerEvent &e);                          // 检查无人机模式
    void set_home();                                                          // 设置home点
    void read_formation_yaml();                                               // 读取阵型yaml文件
    void debug();                                                             // 打印信息
    std::tuple<float, float> calculateFollowerPosition(int vehicleId,
                                                       double leaderPos_x,
                                                       double leaderPos_y,
                                                       double yaw,
                                                       double baseSpacing,
                                                       double lateralSpacing);
};