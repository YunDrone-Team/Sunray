#include "ros_msg_utils.h"
#include "formation_config.h"

#include <yaml-cpp/yaml.h>


#define MAX_NUM 10

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

    struct Formation
    {
        bool get_px4_state[MAX_NUM]{false};
        // 阵型中每个无人机的状态(从PX4State中获取)
        sunray_msgs::PX4State px4_state[MAX_NUM];
        // 来自地面端的任务指令
        sunray_msgs::MissionCMD mission_cmd;
        // 来自领机的任务指令
        sunray_msgs::FollowerCMD follower_cmd;
        // 领机的实时位置(从PX4State中获取)
        LLH_Coord leader_point;
        ENU_Coord leader_point_xyz;
        // 领机的期望位置（从MissionCMD中获取）
        LLH_Coord leader_goal_point;
        ENU_Coord leader_goal_point_xyz;
        // 阵型(枚举类型，见MissionCMD中的枚举定义)（从MissionCMD中获取）
        int formation_name;
        // 所有无人机的原点经纬高（从MissionCMD中获取）
        LLH_Coord origin_point;
        bool set_origin_point{false};

        // 从机的实时位置(从PX4State中获取)
        LLH_Coord follower_point[MAX_NUM]; 
        ENU_Coord follower_point_xyz[MAX_NUM];
        // 从机的期望位置（根据MissionCMD解算）
        LLH_Coord follower_goal_point[MAX_NUM];
        ENU_Coord follower_goal_point_xyz[MAX_NUM];
        
        // ORCA状态量(从PX4State中获取)
        nav_msgs::Odometry orca_agent_state[MAX_NUM];
        // ORCA设置指令
        sunray_msgs::OrcaSetup orca_setup;
        // ORCA输出的控制命令（从ORCA中获取）
        sunray_msgs::OrcaCmd orca_cmd;      // 仅有自身的
        sunray_msgs::UAVControlCMD uav_cmd;
    };
    Formation formation;   // [index=1代表1号机]

private:
    std::string node_name;              // 节点名称
    int uav_id;                         // 【参数】无人机ID
    std::string uav_name;               // 【参数】无人机名称
    std::string file_path;                                                   // 配置文件路径
    std::string topic_prefix;                                                   // 配置文件路径
    int agent_num;
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
    std::map<int, ros::Subscriber> px4_state_sub;                          // 【订阅】无人机状态
    sunray_msgs::PX4State px4_state;   

    Control_Utils uav_control_utils;


    void mission_cmd_cb(const sunray_msgs::MissionCMD::ConstPtr &msg);     
    void follower_cmd_cb(const sunray_msgs::FollowerCMD::ConstPtr &msg);     
    void orca_cmd_cb(const sunray_msgs::OrcaCmd::ConstPtr &msg);     
    void uav_state_cb(const sunray_msgs::UAVState::ConstPtr &msg, int i);     // 无人机状态回调

    void px4_state_cb(const sunray_msgs::PX4State::ConstPtr &msg, int i);     // 无人机状态回调

    void agent_state_pub_timer_cb(const ros::TimerEvent &e);                   // 定时动态阵型发布

void pub_orca_agent_state();

    YAML::Node formation_data;                                               // yaml文件数据
    // Lissa_params circle_params;                                              // 圆阵参数
    // Lissa_params figure_eight_params;                                        // 八字形参数
    // traj_generator traj_gen;                                                 // 轨迹生成器
    
double calculateTargetYaw(double pos_x, double pos_y, double current_yaw, double target_x, double target_y);

    void orca_cmd_callback(const sunray_msgs::OrcaCmd::ConstPtr &msg);        // orca返回值回调
};