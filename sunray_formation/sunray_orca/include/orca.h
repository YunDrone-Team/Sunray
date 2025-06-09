#ifndef agent_ORCA_H
#define agent_ORCA_H

#include <ros/ros.h>
#include <iostream>
#include <Eigen/Eigen>
#include <vector>
#include <map>
#include "printf_utils.h"
#include "ros_msg_utils.h"
#include "RVO.h"

using namespace std;

class ORCA
{
public:
    // 构造函数
    ORCA() {};
    // 析构函数
    ~ORCA()
    {
        delete sim;
    };
    // 初始化函数
    void init(ros::NodeHandle &nh);
    // 主循环函数
    bool orca_run();
    bool start_flag{false};
    // 增加达到目标点打印
    bool goal_reached_printed;

private:
    int agent_id;
    int agent_num;
    int orca_state;
    string agent_name;
    string node_name; // 节点名称
    string agent_prefix;

    float geo_fence_min_x;
    float geo_fence_max_x;
    float geo_fence_min_y;
    float geo_fence_max_y;
    float goal_pos[3];
    float goal_yaw;
    float max_yaw_rate;

    bool arrived_goal; // 是否到达目标点
    bool agent_state_ready;

    sunray_msgs::OrcaCmd OrcaCmd;
    sunray_msgs::UAVControlCMD agent_control_cmd;
    std::map<int, nav_msgs::Odometry> agent_state;
    std::map<int, ros::Subscriber> agent_state_sub;
    std::map<int, ros::Subscriber> cmd_sub;
    visualization_msgs::MarkerArray geo_fence_marker_array;
    ros::Publisher cmd_pub;  // 发布话题
    ros::Publisher goal_pub; // 发布话题
    ros::Publisher geo_pub; // 发布话题
    ros::Timer check_timer;

    std::vector<bool> odom_valid;
    RVO::RVOSimulator *sim = new RVO::RVOSimulator();

    // ORCA算法参数
    struct orca_param
    {
        float neighborDist;
        float maxNeighbors;
        float timeHorizon;
        float timeHorizonObst;
        float radius;
        float maxSpeed;
        float time_step;
    };
    orca_param orca_params;

    void agent_state_cb(const nav_msgs::Odometry::ConstPtr &msg, int i);
    void setup_obstacles();
    void setup_agents();
    void setup_cb(const sunray_msgs::OrcaSetup::ConstPtr &msg, int i);
    bool reachedGoal(int i);
    void printf_param();
    double calculateOptimalTurn(double current, double target);
    void checkAgentState(const ros::TimerEvent &e);
};
#endif