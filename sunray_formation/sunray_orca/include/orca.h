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
    // 增加达到目标点打印
    bool goal_reached_printed;

private:
    int agent_id;
    string agent_name;
    string node_name; // 节点名称
    string agent_prefix;
    int agent_num;
    int orca_state;
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

    bool arrived_goal; // 是否到达目标点
    float goal_pos[3];
    float goal_yaw;
    sunray_msgs::OrcaCmd OrcaCmd;
    sunray_msgs::UAVControlCMD agent_control_cmd;
    std::map<int, geometry_msgs::PoseStamped> agent_state;
    std::map<int, ros::Subscriber> agent_state_sub;
    std::map<int, ros::Subscriber> cmd_sub;

    ros::Publisher cmd_pub;  // 发布话题
    ros::Publisher goal_pub;  // 发布话题
    ros::Timer debug_timer;

    void debugCb(const ros::TimerEvent &e);
    void agent_state_cb(const geometry_msgs::PoseStamped::ConstPtr &msg, int i);
    void setup_obstacles();
    void setup_agents();
    void setup_cb(const sunray_msgs::OrcaSetup::ConstPtr &msg, int i);
    bool reachedGoal(int i);
    void printf_param();
};
#endif