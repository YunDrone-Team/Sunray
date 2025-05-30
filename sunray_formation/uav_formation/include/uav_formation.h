#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int8.h>
#include <geometry_msgs/PoseStamped.h>
#include "printf_format.h"
#include <iostream>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include "ros_msg_utils.h"

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

class SunrayFormation
{
public:
    SunrayFormation() {};
    ~SunrayFormation() {};
    void init(ros::NodeHandle &nh_);

private:
    ros::NodeHandle nh;
    bool first_dynamic;                                                      // 重新进入动态编队模式
    int agent_id;                                                            // 编号
    int state;                                                               // TAKEOFF = 0u,LAND = 1u,HOVER = 2u,FORMATION = 3u,STATIC = 0u,DYNAMIC = 1u,
    int agent_num;                                                           // 无人机数量
    int agent_type;                                                          // 无人机类型 0:无人机 1:无人车
    int now_idx;                                                             // 当前动态阵型目标点索引
    std::string agent_name;                                                  // 无人机名称
    std::string file_path;                                                   // 配置文件路径
    std::string formation_name;                                              // 阵型名称
    YAML::Node formation_data;                                               // yaml文件数据
    std::string topic_prefix;                                                // 话题前缀
    std::map<std::string, FormationData> static_formation_map;               // 静态阵型
    std::map<std::string, std::vector<FormationData>> dynamic_formation_map; // 动态阵型
    std::map<int, geometry_msgs::PoseStamped> agent_state;                   // 无人机状态
    std::map<std::string, int> init_idx;                                     // 动态阵型初始索引
    sunray_msgs::OrcaSetup orca_setup;                                       // 设置orca模式 目标点消息
    sunray_msgs::UAVControlCMD uav_cmd;                                      // 无人机控制指令
    ros::Publisher orca_setup_pub;                                           // 发布orca模式
    ros::Publisher uav_control_cmd;                                          // 发布无人机控制指令
    ros::Publisher ugv_control_cmd;                                          // 发布无人车控制指令
    ros::Subscriber formation_cmd_sub;                                       // 订阅阵型指令
    ros::Subscriber orca_cmd_sub;                                            // 订阅orca返回值
    ros::Timer pub_timer;                                                    // 发布无人机状态到orca
    ros::Timer dynamic_timer;                                                // 动态阵型发布
    ros::Time first_dynamic_time;                                            // 第一次进入动态阵型时间
    std::map<int, ros::Subscriber> agent_state_sub;                          // 订阅无人机状态
    std::map<int, ros::Publisher> agent_state_pub;                           // 发布无人机状态

    void dynamic_formation_pub(std::string formation_name);                   // 动态编队 计算位置 发布位置
    void static_formation_pub(std::string formation_name);                    // 预定义阵型 读取文件发布位置
    void formation_cmd_callback(const sunray_msgs::Formation::ConstPtr &msg); // 阵型指令回调
    void orca_cmd_callback(const sunray_msgs::OrcaCmd::ConstPtr &msg);        // orca返回值回调
    void uav_state_cb(const sunray_msgs::UAVState::ConstPtr &msg, int i);     // 无人机状态回调
    void ugv_state_cb(const sunray_msgs::UGVState::ConstPtr &msg, int i);     // 无人车状态回调
    void timer_pub_state(const ros::TimerEvent &e);                           // 定时发布无人机状态到orca
    void timer_dynamic_formation(const ros::TimerEvent &e);                   // 定时动态阵型发布
};