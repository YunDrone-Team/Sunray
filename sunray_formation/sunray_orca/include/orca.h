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
    // 初始化函数
    void init(ros::NodeHandle &nh);
    // 主循环函数
    bool orca_run();
    bool start_flag{false};

private:
    int uav_id;
    string uav_name;
    int agent_type;
    string node_name; // 节点名称
    string agent_prefix;
    int uav_num;
    float agent_height;
    RVO::RVOSimulator *sim = new RVO::RVOSimulator();

    // 增加达到目标点打印
    std::vector<bool> goal_reached_printed;

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

    std::map<int, sunray_msgs::UAVState> uav_state;
    std::map<int, geometry_msgs::PoseStamped> goals;
    sunray_msgs::UAVControlCMD uav_control_cmd;
    std::map<int, bool> arrived_goal; // 是否到达目标点

    // 订阅话题
    std::map<int, ros::Subscriber> uav_state_sub;
    std::map<int, ros::Subscriber> goals_sub;

    // 发布话题
    ros::Publisher control_cmd_pub;

    ros::Timer debug_timer;
    void debugCb(const ros::TimerEvent &e);
    void uav_state_cb(const sunray_msgs::UAVState::ConstPtr &msg, int i);
    void goal_cb(const geometry_msgs::PoseStamped::ConstPtr &msg, int i);
    void setup_obstacles();
    void setup_agents();
    void setup_goals(int index);
    bool reachedGoal(int i);
    void printf_param();
};
#endif