#include "ros_msg_utils.h"
#include <yaml-cpp/yaml.h>

// 阵型数据
struct FormationData
{
    std::string name;
    double pose_x;
    double pose_y;
    double pose_z;
    double pose_yaw;
};

class LeaderFollower
{
public:
    LeaderFollower() {};
    ~LeaderFollower() {};

    void mainLoop();
    void show_debug_info();                                                             // 打印信息
    void init(ros::NodeHandle &nh);

private:
    std::string node_name;              // 节点名称
    int uav_id;                         // 【参数】无人机ID
    std::string uav_name;               // 【参数】无人机名称
    std::string file_path;                                                   // 配置文件路径
    std::string topic_prefix;                                                   // 配置文件路径

    // 订阅句柄
    ros::Subscriber mission_cmd_sub;    // 【订阅】无人机状态订阅 来自externalFusion节点
    ros::Subscriber follower_cmd_sub;  // 【订阅】控制指令订阅
    ros::Subscriber orca_cmd_sub;  // 【订阅】控制指令订阅

    // 发布句柄
    ros::Publisher control_cmd_pub;           // 【发布】控制指令
    ros::Publisher uav_setup_pub;                // 【发布】无人机指令发布 NED
    ros::Publisher orca_setup_pub;               // 发布orca模式
    ros::Publisher follower_cmd_pub;
    std::map<int, ros::Subscriber> uav_state_sub;                          // 【订阅】无人机状态
    std::map<int, ros::Publisher> agent_state_pub;  

    std::map<int, nav_msgs::Odometry> agent_state;                           // 无人机状态


    ros::Timer agent_state_pub_timer;                                                    // 发布无人机状态到orca


    sunray_msgs::MissionCMD mission_cmd;
    sunray_msgs::FollowerCMD follower_cmd;


    void mission_cmd_cb(const sunray_msgs::MissionCMD::ConstPtr &msg);     
    void follower_cmd_cb(const sunray_msgs::FollowerCMD::ConstPtr &msg);     
    void orca_cmd_cb(const sunray_msgs::OrcaCmd::ConstPtr &msg);     
    void uav_state_cb(const sunray_msgs::UAVState::ConstPtr &msg, int i);     // 无人机状态回调


    void agent_state_pub_timer_cb(const ros::TimerEvent &e);                   // 定时动态阵型发布



    bool first_dynamic;                                                      // 重新进入动态编队模式
    bool home_set;                                                           // 是否设置home点
    bool enable_yaw;                                                           // 是否启用偏航角旋转
    int agent_id;                                                            // 编号
    int state;                                                               // 状态机
    int formation_type;                                                      // 阵型类型 1:静态阵型 2:动态阵型
    int agent_num;                                                           // 无人机数量
    int agent_type;                                                          // 无人机类型 0:无人机 1:无人车
    int now_idx;                                                             // 当前动态阵型目标点索引
    int dynamic_type;                                                        // 动态阵型类型 0:离线读取 1:在线计算
    int leader_id;                                                           // 阵型领导者
    float home_pose[3];                                                      // home点
    float leader_pose[3];                                                    // 领导者位置
    std::string agent_name;                                                  // 无人机名称
    std::string formation_name;                                              // 阵型名称
    YAML::Node formation_data;                                               // yaml文件数据
    // Lissa_params circle_params;                                              // 圆阵参数
    // Lissa_params figure_eight_params;                                        // 八字形参数
    // traj_generator traj_gen;                                                 // 轨迹生成器
    std::map<std::string, FormationData> static_formation_map;               // 静态阵型
    std::map<std::string, std::vector<FormationData>> dynamic_formation_map; // 动态阵型
    std::vector<FormationData> waypoints;                                    // 航点
    std::map<std::string, int> init_idx;                                     // 动态阵型初始索引
    sunray_msgs::OrcaSetup orca_setup;                                       // 设置orca模式 目标点消息
    sunray_msgs::UAVControlCMD uav_cmd;                                      // 无人机控制指令
    sunray_msgs::UGVControlCMD ugv_cmd;                                      // 无人车控制指令
    sunray_msgs::UAVState uav_state;                                         // 无人机状态
    sunray_msgs::UAVSetup uav_setup;                                         // 无人机模式
    sunray_msgs::OrcaCmd orca_cmd;                                           // orca返回值
    ros::Timer dynamic_timer;                                                // 动态阵型发布
    ros::Timer mode_timer;                                                   // 检查无人机模式
    ros::Time first_dynamic_time;                                            // 第一次进入动态阵型时间
    ros::Time mode_takeoff_time;                                             // 第一次进入模式时间
    ros::Time orca_cmd_time;                                                 // orca返回值时间
    ros::Time leader_pose_time;                                              // 领导者位置时间
                       // 发布无人机状态

    void dynamic_formation_pub(std::string formation_name);                   // 动态编队 离线读取位置 发布位置
    void dynamic_formation_online(std::string name);                          // 动态编队 在线计算
    void leader_formation_pub();                                              // 领航者阵型 发布位置
    void static_formation_pub(std::string formation_name);                    // 预定义阵型 读取文件发布位置
    void formation_cmd_callback(const sunray_msgs::Formation::ConstPtr &msg); // 阵型指令回调
    void orca_cmd_callback(const sunray_msgs::OrcaCmd::ConstPtr &msg);        // orca返回值回调
    void ugv_leader_callback(const sunray_msgs::UGVState::ConstPtr &msg);     // 无人车领导者状态回调
    void uav_leader_callback(const sunray_msgs::UAVState::ConstPtr &msg);     // 无人机领导者状态回调
    void goal_callback(const geometry_msgs::PoseStamped::ConstPtr &msg);      // 订阅目标点
    void timer_pub_state(const ros::TimerEvent &e);                           // 定时发布无人机状态到orca
    void agent_mode_check(const ros::TimerEvent &e);                          // 检查无人机模式
    void set_home();                                                          // 设置home点
    void read_formation_yaml();                                               // 读取阵型yaml文件
    double calculateTargetYaw(double target_x, double target_y);
    std::tuple<float, float> calculateFollowerPosition(int vehicleId,
                                                       double leaderPos_x,
                                                       double leaderPos_y,
                                                       double yaw,
                                                       double base_x,
                                                       double base_y);
};