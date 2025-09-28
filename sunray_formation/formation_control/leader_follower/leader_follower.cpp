#include "leader_follower.h"

void LeaderFollower::init(ros::NodeHandle &nh)
{
    node_name = ros::this_node::getName();
    // 【参数】无人机编号，默认1号机为leader
    nh.param<int>("uav_id", uav_id, 1);                 
    nh.param<int>("agent_num", agent_num, 1);
    nh.param<std::string>("file_path", file_path, "/home/yundrone/Sunray/sunray_formation/formation_control/config/config.yaml");

    // 无人机名字 = 无人机名字前缀 + 无人机ID
    uav_name = "/uav" + std::to_string(uav_id);

    // 【订阅】任务指令 - 地面端 -> 本节点
    mission_cmd_sub = nh.subscribe<sunray_msgs::MissionCMD>(uav_name + "/sunray/mission_cmd", 10, &LeaderFollower::mission_cmd_cb, this);

    // 【订阅】ORCA指令 - orca_node -> 本节点 (转换为UAVControlCMD，发送给uav_control_node)
    orca_cmd_sub = nh.subscribe<sunray_msgs::OrcaCmd>(uav_name + "/orca_cmd", 10, &LeaderFollower::orca_cmd_cb, this);

    // 【订阅】无人机的状态
    for (int i = 0; i < agent_num; i++)
    {
        topic_prefix = "/uav" + std::to_string(i + 1);

        uav_state_sub[i] = nh.subscribe<sunray_msgs::UAVState>(topic_prefix + "/sunray/uav_state", 10,
                                                                    boost::bind(&LeaderFollower::uav_state_cb, this, _1, i));

        // 【发布】定位状态到 ORCA 节点 - 本节点 -> orca_node
        agent_state_pub[i] = nh.advertise<nav_msgs::Odometry>(topic_prefix + "/orca/agent_state", 10);
    }

    // 领机发布任务指令，从机接收指令
    if(uav_id == 1)
    {
        // 【发布】从机任务指令 - 本节点（领机） -> 本节点（从机）
        follower_cmd_pub = nh.advertise<sunray_msgs::FollowerCMD>(uav_name + "/sunray/follower_cmd", 10);
    }
    else
    {
        // 【订阅】从机任务指令 - 本节点（领机） -> 本节点（从机）
        follower_cmd_sub = nh.subscribe<sunray_msgs::FollowerCMD>(uav_name + "/sunray/follower_cmd", 10, &LeaderFollower::follower_cmd_cb, this);
    }


    // 【发布】无人机控制指令 - 本节点 -> uav_control_node
    control_cmd_pub = nh.advertise<sunray_msgs::UAVControlCMD>(uav_name + "/sunray/uav_control_cmd", 10);
    // 【发布】无人机设置指令 - 本节点 -> uav_control_node
    uav_setup_pub = nh.advertise<sunray_msgs::UAVSetup>(uav_name + "/sunray/setup", 10);
    // 【发布】ORCA设置指令 - 本节点 -> orca_node
    orca_setup_pub = nh.advertise<sunray_msgs::OrcaSetup>(uav_name + "/orca/setup", 10);
    
    // 【定时器】定时发布自身状态至ORCA节点(将所有无人机位置转为orca所需要的格式发布到ORCA节点)
    agent_state_pub_timer = nh.createTimer(ros::Duration(0.05), &LeaderFollower::agent_state_pub_timer_cb, this);

    // this->read_formation_yaml();
}

// 检查当前模式 并进入对应的处理函数中
void LeaderFollower::mainLoop()
{
    
}

void LeaderFollower::show_debug_info()
{
    
}

// ORCA计算得到的速度，直接发布
void LeaderFollower::orca_cmd_cb(const sunray_msgs::OrcaCmd::ConstPtr &msg)
{
    if(mission_cmd.mission == sunray_msgs::MissionCMD::TAKEOFF || mission_cmd.mission == sunray_msgs::MissionCMD::LAND)
    {
        return;
    }

    orca_cmd = *msg;
}

void LeaderFollower::mission_cmd_cb(const sunray_msgs::MissionCMD::ConstPtr &msg)
{
    mission_cmd = *msg;

    // 起飞 - 》 无人机设置

    // 降落 - 》 无人机设置

    // 返航 -》 orca设置

    // 移动 + orca设置 + 发布从机指令
        // 经纬度结算XYZ


}

void LeaderFollower::follower_cmd_cb(const sunray_msgs::FollowerCMD::ConstPtr &msg)
{
    follower_cmd = *msg;



    // 根据领机的经纬高和阵型指令，解算自身期望的XYZ 
}

void LeaderFollower::agent_state_pub_timer_cb(const ros::TimerEvent &e)
{
    for (int i = 0; i < agent_num; i++)
    {
        agent_state_pub[i].publish(agent_state[i]);
    }
}


// 无人机状态回调
void LeaderFollower::uav_state_cb(const sunray_msgs::UAVState::ConstPtr &msg, int i)
{
    // 读取无人机状态数据（仅仅读取自己的）
    if (uav_id - 1 == i)
    {
        uav_state = *msg;
    }

    // 读取里程计数据（for orca）
    agent_state[i].header.stamp = msg->header.stamp;
    agent_state[i].pose.pose.position.x = msg->position[0];
    agent_state[i].pose.pose.position.y = msg->position[1];
    agent_state[i].pose.pose.position.z = msg->position[2];
    agent_state[i].pose.pose.orientation = msg->attitude_q;
}


// // 固定编队
// void LeaderFollower::static_formation_pub(std::string name)
// {
//     /*
//         逻辑：根据name从静态队形map中获取自身目标点，并发布给ORCA
//     */
//     orca_setup.header.stamp = ros::Time::now();
//     orca_setup.cmd = sunray_msgs::OrcaSetup::GOAL_RUN;
//     orca_setup.desired_pos[0] = static_formation_map[name].pose_x;
//     orca_setup.desired_pos[1] = static_formation_map[name].pose_y;
//     orca_setup.desired_pos[2] = static_formation_map[name].pose_z;
//     orca_setup.desired_yaw = 0.0;
//     orca_setup_pub.publish(orca_setup);
//     Logger::info(agent_name, uav_id, " pub goal: ", orca_setup.desired_pos[0], orca_setup.desired_pos[1], orca_setup.desired_pos[2]);
// }



// void LeaderFollower::agent_mode_check(const ros::TimerEvent &e)
// {
//     this->debug();
//     if (state == sunray_msgs::Formation::FORMATION)
//     {
//         if ((ros::Time::now() - orca_cmd_time).toSec() > 1.5)
//         {
//             state = sunray_msgs::Formation::HOVER;
//             Logger::print_color(int(LogColor::red), node_name, "ORCA节点超时，切换至悬停模式");
//         }
//     }
//     if (state == sunray_msgs::Formation::TAKEOFF && agent_type == 0)
//     {
//         float time_diff = ros::Time::now().toSec() - mode_takeoff_time.toSec();
//         if (time_diff < 5 &&
//             uav_state.control_mode != sunray_msgs::UAVSetup::CMD_CONTROL)
//         {
//             if (uav_state.control_mode != sunray_msgs::UAVSetup::CMD_CONTROL)
//             {
//                 uav_setup.header.stamp = ros::Time::now();
//                 uav_setup.cmd = sunray_msgs::UAVSetup::SET_CONTROL_MODE;
//                 uav_setup.control_mode = "CMD_CONTROL";
//                 uav_setup_pub.publish(uav_setup);
//             }
//         }
//         else if (time_diff > 5 && time_diff < 10 && !uav_state.armed)
//         {
//             if (uav_state.control_mode != sunray_msgs::UAVSetup::CMD_CONTROL)
//             {
//                 Logger::print_color(int(LogColor::red), node_name, "无人机模式切换失败，请检查无人机状态");
//                 state = sunray_msgs::Formation::INIT;
//                 return;
//             }
//             uav_setup.header.stamp = ros::Time::now();
//             uav_setup.cmd = sunray_msgs::UAVSetup::ARM;
//             uav_setup_pub.publish(uav_setup);
//         }
//         else if (time_diff > 10 && time_diff < 15)
//         {
//             if (!uav_state.armed)
//             {
//                 Logger::print_color(int(LogColor::red), node_name, "无人机解锁失败，请检查无人机状态");
//                 state = sunray_msgs::Formation::INIT;
//                 return;
//             }
//             uav_cmd.header.stamp = ros::Time::now();
//             uav_cmd.cmd = sunray_msgs::UAVControlCMD::Takeoff;
//             uav_control_cmd.publish(uav_cmd);
//         }
//         else if (time_diff > 15 && time_diff < 20 &&
//                  uav_state.control_mode == sunray_msgs::UAVSetup::CMD_CONTROL && uav_state.armed)
//         {
//             set_home();
//             orca_setup.header.stamp = ros::Time::now();
//             orca_setup.cmd = sunray_msgs::OrcaSetup::GOAL;
//             orca_setup.desired_pos[0] = home_pose[0];
//             orca_setup.desired_pos[1] = home_pose[1];
//             orca_setup.desired_pos[2] = 1.0;
//             orca_setup.desired_yaw = 0.0;
//             orca_setup_pub.publish(orca_setup);
//             state = sunray_msgs::Formation::INIT;
//         }
//     }
//     else if (state == sunray_msgs::Formation::LAND && agent_type == 0)
//     {
//         uav_setup.header.stamp = ros::Time::now();
//         uav_setup.cmd = sunray_msgs::UAVSetup::SET_CONTROL_MODE;
//         uav_setup.control_mode = "LAND_CONTROL";
//         uav_setup_pub.publish(uav_setup);
//         state = sunray_msgs::Formation::INIT;
//     }
//     else if (state == sunray_msgs::Formation::HOVER)
//     {

//         if (agent_type == 0)
//         {
//             uav_cmd.header.stamp = ros::Time::now();
//             uav_cmd.cmd = sunray_msgs::UAVControlCMD::Hover;
//             uav_control_cmd.publish(uav_cmd);
//         }
//         else if (agent_type == 1)
//         {
//             ugv_cmd.header.stamp = ros::Time::now();
//             ugv_cmd.cmd = sunray_msgs::UGVControlCMD::HOLD;
//             ugv_control_cmd.publish(ugv_cmd);
//         }
//         state = sunray_msgs::Formation::INIT;
//     }
// }

// void LeaderFollower::set_home()
// {
//     this->home_pose[0] = agent_state[uav_id - 1].pose.pose.position.x;
//     this->home_pose[1] = agent_state[uav_id - 1].pose.pose.position.y;
//     this->home_pose[2] = agent_state[uav_id - 1].pose.pose.position.z;
//     home_set = true;
// }

// 计算目标角度
double LeaderFollower::calculateTargetYaw(double target_x, double target_y)
{
    // 计算目标点相对于当前位置的角度
    double delta_x = target_x - agent_state[uav_id - 1].pose.pose.position.x;
    double delta_y = target_y - agent_state[uav_id - 1].pose.pose.position.y;
    double target_yaw = atan2(delta_y, delta_x);
    // 限制目标角度在[-pi, pi]范围内
    if (target_yaw < -M_PI)
    {
        target_yaw += 2 * M_PI;
    }
    else if (target_yaw > M_PI)
    {
        target_yaw -= 2 * M_PI;
    }
    // 限制最大转向角度
    double current_yaw = tf::getYaw(agent_state[uav_id - 1].pose.pose.orientation);
    double yaw_diff = target_yaw - current_yaw;
    if (yaw_diff > M_PI)
    {
        yaw_diff -= 2 * M_PI;
    }
    else if (yaw_diff < -M_PI)
    {
        yaw_diff += 2 * M_PI;
    }
    // 如果转向角度超过最大限制，则调整目标角度
    if (fabs(yaw_diff) > M_PI / 6) // 最大转向角度为30度
    {
        if (yaw_diff > 0)
        {
            target_yaw = current_yaw + M_PI / 6;
        }
        else
        {
            target_yaw = current_yaw - M_PI / 6;
        }
    }
    // 返回计算后的目标角度
    if (target_yaw < -M_PI)
    {
        target_yaw += 2 * M_PI;
    }
    else if (target_yaw > M_PI)
    {
        target_yaw -= 2 * M_PI;
    }
    return target_yaw;
}