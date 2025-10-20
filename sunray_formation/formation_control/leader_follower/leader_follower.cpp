#include "leader_follower.h"
// leader_follower
// 领机位置为外部程序指定
// 从机位置根据与领机的相对位置来进行队形切换
// 领机为
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
    mission_cmd_sub = nh.subscribe<sunray_msgs::MissionCMD>("/sunray/mission_cmd", 10, &LeaderFollower::mission_cmd_cb, this);

    // 【订阅】ORCA指令 - orca_node -> 本节点 (转换为UAVControlCMD，发送给uav_control_node)
    orca_cmd_sub = nh.subscribe<sunray_msgs::OrcaCmd>(uav_name + "/orca_cmd", 10, &LeaderFollower::orca_cmd_cb, this);

    // 【订阅】ORCA状态(暂无)



    // 【订阅】无人机的状态（所有无人机）
    for (int i = 1; i <= agent_num; i++)
    {
        topic_prefix = "/uav" + std::to_string(i);

        uav_state_sub[i] = nh.subscribe<sunray_msgs::UAVState>(topic_prefix + "/sunray/uav_state", 10,
                                                                    boost::bind(&LeaderFollower::uav_state_cb, this, _1, i));
        
        // 【订阅】PX4无人机综合状态 - external_fusion_node -> 本节点
        px4_state_sub[i] = nh.subscribe<sunray_msgs::PX4State>(topic_prefix + "/sunray/px4_state", 10, 
                                                                    boost::bind(&LeaderFollower::px4_state_cb, this, _1, i));

        // 【发布】定位状态到 ORCA 节点 - 本节点 -> orca_node
        agent_state_pub[i] = nh.advertise<nav_msgs::Odometry>(topic_prefix + "/orca/agent_state", 10);
    }

    // 如果uav_id=1,则是领机类;其他情况都是从机
    // 领机发布从机任务指令
    // 从机从领机接收任务指令
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
    

    // this->read_formation_yaml();




    uav_control_utils.init(nh, uav_id, node_name);

}

// 无人机状态回调
void LeaderFollower::px4_state_cb(const sunray_msgs::PX4State::ConstPtr &msg, int i)
{
    formation.get_px4_state[i] = true;
    formation.px4_state[i] = *msg;

    // 从机的位置
    formation.follower_point[i].lat = msg->latitude;
    formation.follower_point[i].lon = msg->longitude;
    formation.follower_point[i].alt = msg->altitude;

    if(formation.set_origin_point)
    {
        // 计算从机的xyz位置
        calculate_enu_coordinates(&formation.origin_point, &formation.follower_point[i], &formation.follower_point_xyz[i]);
    }

    // 
    formation.orca_agent_state[i].header.stamp = ros::Time::now();
    formation.orca_agent_state[i].pose.pose.position.x = msg->position[0];
    formation.orca_agent_state[i].pose.pose.position.y = msg->position[1];
    formation.orca_agent_state[i].pose.pose.position.z = msg->position[2];
    formation.orca_agent_state[i].pose.pose.orientation = msg->attitude_q;

    if(i==1)
    {
        formation.leader_point.lat = msg->latitude;
        formation.leader_point.lon = msg->longitude;
        formation.leader_point.alt = msg->altitude;

        if(formation.set_origin_point)
        {
            // 计算从机的xyz位置
            calculate_enu_coordinates(&formation.origin_point, &formation.leader_point, &formation.leader_point_xyz);
        }
    }
}

void LeaderFollower::pub_orca_agent_state()
{
    for (int i = 1; i <= agent_num; i++)
    {
        agent_state_pub[i].publish(formation.orca_agent_state[i]);
    }
}

// 检查当前模式 并进入对应的处理函数中
void LeaderFollower::mainLoop()
{
    // 定时更新ORCA所需要的agent_state
    pub_orca_agent_state();

    
}

void LeaderFollower::show_debug_info()
{
    Logger::print_color(int(LogColor::cyan), ">>>>>>>>>>>>>>>> 领-从集群编队 - [", uav_name, "] <<<<<<<<<<<<<<<<<");

    // 任务指令 - 连接状态、飞控模式、电池状态
    Logger::print_color(int(LogColor::cyan), "-------- 任务指令 - [~/sunray/mission_cmd]");
    if(formation.mission_cmd.mission == sunray_msgs::MissionCMD::TAKEOFF)
    {
        Logger::print_color(int(LogColor::green), "mission_cmd.mission","TAKEOFF");
        Logger::print_color(int(LogColor::green), "Origin Point[lat lon alt]:", formation.mission_cmd.origin_lat, formation.mission_cmd.origin_lon,"[deg*1e7]", formation.mission_cmd.origin_alt, "[m](MSL)");
    }else if(formation.mission_cmd.mission == sunray_msgs::MissionCMD::LAND)
    {
        Logger::print_color(int(LogColor::green), "mission_cmd.mission","LAND");
    }else if(formation.mission_cmd.mission == sunray_msgs::MissionCMD::RETURN)
    {
        Logger::print_color(int(LogColor::green), "mission_cmd.mission","RETURN");
    }else if(formation.mission_cmd.mission == sunray_msgs::MissionCMD::MOVE)
    {
        Logger::print_color(int(LogColor::green), "mission_cmd.mission","MOVE");

        
        Logger::print_color(int(LogColor::green), "Leader Goal Point[lat lon alt]:", formation.leader_goal_point.lat, formation.leader_goal_point.lon,"[deg*1e7]", formation.leader_goal_point.alt, "[m](MSL)");

        Logger::print_color(int(LogColor::green), "Leader Goal Point ENU [x y z]:", formation.leader_goal_point_xyz.x, formation.leader_goal_point_xyz.y, formation.leader_goal_point_xyz.z, "[m]");

        Logger::print_color(int(LogColor::green), "Follower Goal Point[lat lon alt]:", formation.follower_goal_point[uav_id].lat, formation.follower_goal_point[uav_id].lon,"[deg*1e7]", formation.follower_goal_point[uav_id].alt, "[m](MSL)");

        Logger::print_color(int(LogColor::green), "Follower Goal Point ENU [x y z]:", formation.follower_goal_point_xyz[uav_id].x, formation.follower_goal_point_xyz[uav_id].y, formation.follower_goal_point_xyz[uav_id].z, "[m]");


        Logger::print_color(int(LogColor::green), "Follower Point[lat lon alt]:", formation.follower_point[uav_id].lat, formation.follower_point[uav_id].lon,"[deg*1e7]", formation.follower_point[uav_id].alt, "[m](MSL)");

        Logger::print_color(int(LogColor::green), "Follower Point ENU [x y z]:", formation.follower_point_xyz[uav_id].x, formation.follower_point_xyz[uav_id].y, formation.follower_point_xyz[uav_id].z, "[m]");

        Logger::print_color(int(LogColor::green), "ORCA goal_pos  [x y z]:", formation.orca_cmd.goal_pos[0], formation.orca_cmd.goal_pos[1], formation.orca_cmd.goal_pos[2], "[m]");
        Logger::print_color(int(LogColor::green), "ORCA linearcmd [x y z]:", formation.orca_cmd.linear[0], formation.orca_cmd.linear[1], formation.orca_cmd.linear[2], "[m]");
    }




}

// ORCA计算得到的速度，直接发布
void LeaderFollower::orca_cmd_cb(const sunray_msgs::OrcaCmd::ConstPtr &msg)
{
    // 此时不启用ORCA算法
    if(formation.mission_cmd.mission == sunray_msgs::MissionCMD::TAKEOFF || formation.mission_cmd.mission == sunray_msgs::MissionCMD::LAND)
    {
        return;
    }

    formation.orca_cmd = *msg;

    // 如果是停止状态，则发送悬停指令
    if(msg->state == sunray_msgs::OrcaCmd::INIT)
    {

    }else if (msg->state == sunray_msgs::OrcaCmd::STOP)
    {
        formation.uav_cmd.header.stamp = ros::Time::now();
        formation.uav_cmd.cmd = sunray_msgs::UAVControlCMD::Hover;
    }else if (msg->state == sunray_msgs::OrcaCmd::RUN)
    {
        formation.uav_cmd.header.stamp = ros::Time::now();
        formation.uav_cmd.cmd = sunray_msgs::UAVControlCMD::XyVelZPosYaw;
        formation.uav_cmd.desired_vel[0] = msg->linear[0];
        formation.uav_cmd.desired_vel[1] = msg->linear[1];
        formation.uav_cmd.desired_pos[0] = msg->goal_pos[0];
        formation.uav_cmd.desired_pos[1] = msg->goal_pos[1];
        formation.uav_cmd.desired_pos[2] = msg->goal_pos[2];
        formation.uav_cmd.desired_yaw = 0.0;
        formation.uav_cmd.desired_yaw_rate = 0;
    }else if (msg->state == sunray_msgs::OrcaCmd::ARRIVED)
    {
        formation.uav_cmd.header.stamp = ros::Time::now();
        formation.uav_cmd.cmd = sunray_msgs::UAVControlCMD::XyzPosYaw;
        formation.uav_cmd.desired_pos[0] = msg->goal_pos[0];
        formation.uav_cmd.desired_pos[1] = msg->goal_pos[1];
        formation.uav_cmd.desired_pos[2] = msg->goal_pos[2];
        formation.uav_cmd.desired_yaw = 0.0;
    }else
    {
        Logger::info("Wrong orca cmd");
        return;
    }

    control_cmd_pub.publish(formation.uav_cmd);
}

void LeaderFollower::mission_cmd_cb(const sunray_msgs::MissionCMD::ConstPtr &msg)
{
    formation.mission_cmd = *msg;
    
    // 读取阵型
    formation.formation_name = msg->mission_formation;
    // 读取领机的期望经纬高
    formation.leader_goal_point.lat =  msg->leader_lat_ref;
    formation.leader_goal_point.lon =  msg->leader_lon_ref;
    formation.leader_goal_point.alt =  msg->leader_alt_ref;
    // 读取原点
    formation.origin_point.lat =  msg->origin_lat;
    formation.origin_point.lon =  msg->origin_lon;
    formation.origin_point.alt =  msg->origin_alt;
    formation.set_origin_point = true;

    // 计算领机的期望xyz点
    calculate_enu_coordinates(&formation.origin_point, &formation.leader_goal_point, &formation.leader_goal_point_xyz);

    // 计算从机的期望xyz点（以下这部分接口可以写的更加智能，只要保证输入输出即可）
    // 1、生成队形偏移量结构体 
    FormationOffsets dynamic_formation = createCircleFormation(agent_num, 15.0);
    // 2、计算从机的期望xyz位置
    for (int i = 1; i <= agent_num; i++)
    {
        formation.follower_goal_point_xyz[i] = calculateFollowerPosition(&formation.leader_goal_point_xyz, &dynamic_formation, i-1);
    }

    // 计算从机的期望经纬高
    for (int i = 1; i <= agent_num; i++)
    {
        formation.follower_goal_point[i] = enu_to_llh(&formation.origin_point, &formation.follower_goal_point_xyz[i]);
    }
    // Logger::info( "3");

    // 执行MissionCMD指令
    switch (formation.mission_cmd.mission)
    {
    case sunray_msgs::MissionCMD::TAKEOFF:
        // 记录Home点
        // 飞机原地起飞
        uav_control_utils.auto_takeoff();
        break;
    case sunray_msgs::MissionCMD::LAND:
        // 飞机原地降落
        uav_control_utils.auto_land();
        break;
    case sunray_msgs::MissionCMD::RETURN:
        // 飞机返回起飞点并降落
        uav_control_utils.auto_return();
        break;
    case sunray_msgs::MissionCMD::MOVE:
        // 飞机向目标点+目标阵型移动
        // 移动 + orca设置 + 发布从机指令
        // 经纬度结算XYZ    

        // 发布ORCA设置指令
        formation.orca_setup.header.stamp = ros::Time::now();
        formation.orca_setup.cmd = sunray_msgs::OrcaSetup::GOAL_RUN;
        formation.orca_setup.desired_pos[0] = formation.follower_goal_point_xyz[uav_id].x;
        formation.orca_setup.desired_pos[1] = formation.follower_goal_point_xyz[uav_id].y;
        formation.orca_setup.desired_pos[2] = formation.follower_goal_point_xyz[uav_id].z;
        formation.orca_setup.desired_yaw = 0.0;
        orca_setup_pub.publish(formation.orca_setup);
        break;
    }


}

void LeaderFollower::follower_cmd_cb(const sunray_msgs::FollowerCMD::ConstPtr &msg)
{
    // 1号机为领机，不订阅该指令
    if(uav_id == 1)
    {
        return;
    }

    formation.follower_cmd = *msg;

    // 根据领机的经纬高和阵型指令，解算自身期望的XYZ 
}



// 无人机状态回调
void LeaderFollower::uav_state_cb(const sunray_msgs::UAVState::ConstPtr &msg, int i)
{

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
double LeaderFollower::calculateTargetYaw(double pos_x, double pos_y, double current_yaw, double target_x, double target_y)
{
    // 计算目标点相对于当前位置的角度
    double delta_x = target_x - pos_x;
    double delta_y = target_y - pos_y;
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
    // double current_yaw = tf::getYaw(agent_state[uav_id - 1].pose.pose.orientation);
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