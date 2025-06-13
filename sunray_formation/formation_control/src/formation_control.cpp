#include "formation_control.h"

void SunrayFormation::init(ros::NodeHandle &nh_)
{
    nh = nh_;
    nh.param<int>("agent_id", agent_id, 1);
    nh.param<int>("agent_num", agent_num, 1);
    nh.param<int>("agent_type", agent_type, 0);
    nh.param<std::string>("agent_name", agent_name, "uav");
    nh.param<std::string>("file_path", file_path, "/home/yundrone/Sunray/sunray_formation/formation_control/config/config.yaml");

    nh.param<int>("dynamic_type", dynamic_type, 0);
    nh.param<float>("circle/lissa_a", circle_params.lissa_a, 0);
    nh.param<float>("circle/lissa_b", circle_params.lissa_b, 0);
    nh.param<float>("circle/lissa_delta", circle_params.lissa_delta, 0);
    nh.param<float>("circle/radius_x", circle_params.radius_x, 0);
    nh.param<float>("circle/radius_y", circle_params.radius_y, 0);
    nh.param<float>("circle/center_x", circle_params.center_x, 0);
    nh.param<float>("circle/center_y", circle_params.center_y, 0);
    nh.param<float>("circle/linear_speed", circle_params.linear_speed, 0);
    nh.param<float>("figure_eight/lissa_a", figure_eight_params.lissa_a, 0);
    nh.param<float>("figure_eight/lissa_b", figure_eight_params.lissa_b, 0);
    nh.param<float>("figure_eight/lissa_delta", figure_eight_params.lissa_delta, 0);
    nh.param<float>("figure_eight/radius_x", figure_eight_params.radius_x, 0);
    nh.param<float>("figure_eight/radius_y", figure_eight_params.radius_y, 0);
    nh.param<float>("figure_eight/center_x", figure_eight_params.center_x, 0);
    nh.param<float>("figure_eight/center_y", figure_eight_params.center_y, 0);
    nh.param<float>("figure_eight/linear_speed", figure_eight_params.linear_speed, 0);
    // leader_id 1~100为无人机，101~200为无人车
    nh.param<int>("leader_id", leader_id, 1);

    topic_prefix = "/" + agent_name + std::to_string(agent_id);
    node_name = ros::this_node::getName();
    // 【订阅】
    formation_cmd_sub = nh.subscribe<sunray_msgs::Formation>("/sunray/formation_cmd", 10, &SunrayFormation::formation_cmd_callback, this);
    formation_cmd_ground_sub = nh.subscribe<sunray_msgs::Formation>("/sunray/formation_cmd/ground", 10, &SunrayFormation::formation_cmd_callback, this);
    orca_cmd_sub = nh.subscribe<sunray_msgs::OrcaCmd>(topic_prefix + "/orca_cmd", 10, &SunrayFormation::orca_cmd_callback, this);
    goal_sub = nh.subscribe<geometry_msgs::PoseStamped>("/goal_" + std::to_string(agent_id), 10, &SunrayFormation::goal_callback, this);
    // 【发布】
    uav_control_cmd = nh.advertise<sunray_msgs::UAVControlCMD>(topic_prefix + "/sunray/uav_control_cmd", 10);
    ugv_control_cmd = nh.advertise<sunray_msgs::UGVControlCMD>(topic_prefix + "/sunray_ugv/ugv_control_cmd", 10);
    orca_setup_pub = nh.advertise<sunray_msgs::OrcaSetup>(topic_prefix + "/orca/setup", 10);
    uav_setup_pub = nh.advertise<sunray_msgs::UAVSetup>(topic_prefix + "/sunray/setup", 10);
    // 【定时器】
    pub_timer = nh.createTimer(ros::Duration(0.05), &SunrayFormation::timer_pub_state, this);
    dynamic_timer = nh.createTimer(ros::Duration(0.1), &SunrayFormation::timer_dynamic_formation, this);
    mode_timer = nh.createTimer(ros::Duration(1), &SunrayFormation::agent_mode_check, this);
    home_set = false;
    state = sunray_msgs::Formation::INIT;
    formation_type = 0;
    orca_cmd_time = ros::Time::now();

    // 初始化 home_pose 和 leader_pose
    leader_pose_time = ros::Time::now();
    for (int i = 0; i < 3; i++)
    {
        home_pose[i] = 0.0;
        leader_pose[i] = 0.0;
    }

    if (leader_id > 100)
    {
        leader_pose_sub = nh.subscribe<sunray_msgs::UGVState>("/ugv" + std::to_string(leader_id - 100) + "/sunray_ugv/ugv_state", 10, &SunrayFormation::ugv_leader_callback, this);
    }
    else if (leader_id <= 100)
    {
        leader_pose_sub = nh.subscribe<sunray_msgs::UAVState>("/uav" + std::to_string(leader_id) + "/sunray/uav_state", 10, &SunrayFormation::uav_leader_callback, this);
    }

    // 订阅无人机或无人车的状态
    for (int i = 0; i < agent_num; i++)
    {
        topic_prefix = "/" + agent_name + std::to_string(i + 1);
        // 【订阅】uav状态数据
        if (agent_type == 0)
        {
            agent_state_sub[i] = nh.subscribe<sunray_msgs::UAVState>(topic_prefix + "/sunray/uav_state", 10,
                                                                     boost::bind(&SunrayFormation::uav_state_cb, this, _1, i));
        }
        // 【订阅】ugv状态数据
        else if (agent_type == 1)
        {
            agent_state_sub[i] = nh.subscribe<sunray_msgs::UGVState>(topic_prefix + "/sunray_ugv/ugv_state", 10,
                                                                     boost::bind(&SunrayFormation::ugv_state_cb, this, _1, i));
        }
        // 【发布】定位状态到 ORCA 节点
        agent_state_pub[i] = nh.advertise<nav_msgs::Odometry>(topic_prefix + "/orca/agent_state", 10);
    }

    traj_gen.init(agent_num, agent_id);
    this->read_formation_yaml();
}

void SunrayFormation::formation_cmd_callback(const sunray_msgs::Formation::ConstPtr &msg)
{
    // 当指令为FORMATION时，根据formation_type进行队形切换
    if (msg->cmd == sunray_msgs::Formation::FORMATION)
    {
        state = sunray_msgs::Formation::FORMATION;
        formation_name = msg->name;
        // 固定队形
        if (msg->formation_type == sunray_msgs::Formation::STATIC)
        {
            formation_type = sunray_msgs::Formation::STATIC;
            static_formation_pub(msg->name);
        }
        // 动态队形
        else if (msg->formation_type == sunray_msgs::Formation::DYNAMIC)
        {
            formation_type = sunray_msgs::Formation::DYNAMIC;
            if (dynamic_type == 1)
            {
                now_idx = 0;
            }
            else
            {
                now_idx = init_idx[msg->name];
            }
            first_dynamic = true;
            first_dynamic_time = ros::Time::now();
            Logger::print_color(int(LogColor::yellow), node_name, "当前动态队形:", msg->name);
        }
        // 领队模式
        else if (msg->formation_type == 3)
        {
            Logger::print_color(int(LogColor::yellow), node_name, "当前队形: 领队模式");
            formation_type = sunray_msgs::Formation::LEADER;
            now_idx = 0;
        }
    }
    else if (msg->cmd == sunray_msgs::Formation::TAKEOFF)
    {
        state = sunray_msgs::Formation::TAKEOFF;
        mode_takeoff_time = ros::Time::now();
    }
    else if (msg->cmd == sunray_msgs::Formation::LAND)
    {
        state = sunray_msgs::Formation::LAND;
    }
    else if (msg->cmd == sunray_msgs::Formation::HOVER)
    {
        state = sunray_msgs::Formation::HOVER;
    }
    else if (msg->cmd == sunray_msgs::Formation::SET_HOME)
    {
        this->set_home();
    }
    else if (msg->cmd == sunray_msgs::Formation::RETURN_HOME)
    {
        if (!home_set)
        {
            Logger::error(node_name, "Home 点未设置!");
            return;
        }
        state = sunray_msgs::Formation::FORMATION;
        formation_type = sunray_msgs::Formation::GOAL;
        orca_setup.header.stamp = ros::Time::now();
        orca_setup.cmd = sunray_msgs::OrcaSetup::GOAL_RUN;
        orca_setup.desired_pos[0] = this->home_pose[0];
        orca_setup.desired_pos[1] = this->home_pose[1];
        orca_setup.desired_pos[2] = this->home_pose[2];
        orca_setup.desired_yaw = 0.0;
        orca_setup_pub.publish(orca_setup);
    }
}

// orca状态及控制指令回调
void SunrayFormation::orca_cmd_callback(const sunray_msgs::OrcaCmd::ConstPtr &msg)
{
    orca_cmd = *msg;
    orca_cmd_time = ros::Time::now();
    if (state != sunray_msgs::Formation::FORMATION || sunray_msgs::Formation::GOAL)
    {
        return;
    }
    // 如果是停止状态，则发送悬停指令
    if (msg->state == sunray_msgs::OrcaCmd::STOP)
    {
        uav_cmd.header.stamp = ros::Time::now();
        uav_cmd.cmd = sunray_msgs::UAVControlCMD::Hover;

        ugv_cmd.header.stamp = ros::Time::now();
        ugv_cmd.cmd = sunray_msgs::UGVControlCMD::HOLD;
    }
    // 如果是运行状态，则发送速度指令
    else if (msg->state == sunray_msgs::OrcaCmd::RUN)
    {
        double yaw = calculateTargetYaw(msg->goal_pos[0], msg->goal_pos[1]);
        uav_cmd.header.stamp = ros::Time::now();
        uav_cmd.cmd = sunray_msgs::UAVControlCMD::XyVelZPosYaw;
        // uav_cmd.cmd = sunray_msgs::UAVControlCMD::XyzPosVelYaw;
        uav_cmd.desired_vel[0] = msg->linear[0];
        uav_cmd.desired_vel[1] = msg->linear[1];
        uav_cmd.desired_pos[0] = msg->goal_pos[0];
        uav_cmd.desired_pos[1] = msg->goal_pos[1];
        uav_cmd.desired_pos[2] = 1;
        uav_cmd.desired_yaw = yaw;
        // uav_cmd.desired_yaw_rate = 0;

        ugv_cmd.header.stamp = ros::Time::now();
        // ugv_cmd.cmd = sunray_msgs::UGVControlCMD::POS_CONTROL_ENU;
        ugv_cmd.cmd = sunray_msgs::UGVControlCMD::VEL_CONTROL_ENU;
        // ugv_cmd.cmd = sunray_msgs::UGVControlCMD::POS_VEL_CONTROL_ENU;
        ugv_cmd.desired_vel[0] = msg->linear[0];
        ugv_cmd.desired_vel[1] = msg->linear[1];
        ugv_cmd.desired_pos[0] = msg->goal_pos[0];
        ugv_cmd.desired_pos[1] = msg->goal_pos[1];
        ugv_cmd.desired_yaw = yaw;
    }
    // 如果是到达状态，则发送位置指令
    else if (msg->state == sunray_msgs::OrcaCmd::ARRIVED)
    {
        double yaw = calculateTargetYaw(msg->goal_pos[0] + 1, msg->goal_pos[1]);
        uav_cmd.header.stamp = ros::Time::now();
        // uav_cmd.cmd = sunray_msgs::UAVControlCMD::XyzPos;
        uav_cmd.cmd = sunray_msgs::UAVControlCMD::XyzPosYaw;
        uav_cmd.desired_pos[0] = msg->goal_pos[0];
        uav_cmd.desired_pos[1] = msg->goal_pos[1];
        // uav_cmd.desired_vel[0] = msg->linear[0];
        // uav_cmd.desired_vel[1] = msg->linear[1];
        uav_cmd.desired_pos[2] = 1;
        uav_cmd.desired_yaw = yaw;
        ugv_cmd.header.stamp = ros::Time::now();
        ugv_cmd.cmd = sunray_msgs::UGVControlCMD::POS_CONTROL_ENU;
        ugv_cmd.desired_pos[0] = msg->goal_pos[0];
        ugv_cmd.desired_pos[1] = msg->goal_pos[1];
        ugv_cmd.desired_yaw = yaw;
    }
    else
    {
        return;
    }
    // 无人机
    if (agent_type == 0)
    {
        uav_control_cmd.publish(uav_cmd);
    }
    // 无人车
    else if (agent_type == 1)
    {
        ugv_control_cmd.publish(ugv_cmd);
    }
}

// 无人机状态回调
void SunrayFormation::uav_state_cb(const sunray_msgs::UAVState::ConstPtr &msg, int i)
{
    agent_state[i].header.stamp = msg->header.stamp;
    agent_state[i].pose.pose.position.x = msg->position[0];
    agent_state[i].pose.pose.position.y = msg->position[1];
    agent_state[i].pose.pose.position.z = msg->position[2];
    agent_state[i].pose.pose.orientation = msg->attitude_q;
    if (agent_id - 1 == i)
    {
        uav_state = *msg;
    }
}

// 无人车状态回调
void SunrayFormation::ugv_state_cb(const sunray_msgs::UGVState::ConstPtr &msg, int i)
{
    agent_state[i].header.stamp = msg->header.stamp;
    agent_state[i].pose.pose.position.x = msg->position[0];
    agent_state[i].pose.pose.position.y = msg->position[1];
    agent_state[i].pose.pose.position.z = 0;
    agent_state[i].pose.pose.orientation = msg->attitude_q;
}

void SunrayFormation::uav_leader_callback(const sunray_msgs::UAVState::ConstPtr &msg)
{
    leader_pose_time = msg->header.stamp;
    leader_pose[0] = msg->position[0];
    leader_pose[1] = msg->position[1];
    leader_pose[2] = msg->position[2];
}

void SunrayFormation::ugv_leader_callback(const sunray_msgs::UGVState::ConstPtr &msg)
{
    leader_pose_time = msg->header.stamp;
    leader_pose[0] = msg->position[0];
    leader_pose[1] = msg->position[1];
    leader_pose[2] = 0;
}

// 目标点回调
void SunrayFormation::goal_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    state = sunray_msgs::Formation::FORMATION;
    formation_type = sunray_msgs::Formation::GOAL;
    orca_setup.header.stamp = ros::Time::now();
    orca_setup.cmd = sunray_msgs::OrcaSetup::GOAL_RUN;
    orca_setup.desired_pos[0] = msg->pose.position.x;
    orca_setup.desired_pos[1] = msg->pose.position.y;
    orca_setup.desired_pos[2] = msg->pose.position.z;
    orca_setup.desired_yaw = 0.0;
    orca_setup_pub.publish(orca_setup);
}

// 固定编队
void SunrayFormation::static_formation_pub(std::string name)
{
    /*
        逻辑：根据name从静态队形map中获取自身目标点，并发布给ORCA
    */
    orca_setup.header.stamp = ros::Time::now();
    orca_setup.cmd = sunray_msgs::OrcaSetup::GOAL_RUN;
    orca_setup.desired_pos[0] = static_formation_map[name].pose_x;
    orca_setup.desired_pos[1] = static_formation_map[name].pose_y;
    orca_setup.desired_pos[2] = static_formation_map[name].pose_z;
    orca_setup.desired_yaw = 0.0;
    orca_setup_pub.publish(orca_setup);
    Logger::info(agent_name, agent_id, " pub goal: ", orca_setup.desired_pos[0], orca_setup.desired_pos[1], orca_setup.desired_pos[2]);
}

// 动态编队
void SunrayFormation::dynamic_formation_pub(std::string name)
{
    /*
        逻辑：依次从轨迹队列中获取下一个位置作为目标点，并发布给ORCA
    */
    orca_setup.header.stamp = ros::Time::now();
    orca_setup.cmd = sunray_msgs::OrcaSetup::GOAL_RUN;
    now_idx = now_idx % dynamic_formation_map[name].size();
    orca_setup.desired_pos[0] = dynamic_formation_map[name][now_idx].pose_x;
    orca_setup.desired_pos[1] = dynamic_formation_map[name][now_idx].pose_y;
    orca_setup.desired_pos[2] = dynamic_formation_map[name][now_idx].pose_z;
    orca_setup.desired_yaw = 0.0;
    orca_setup_pub.publish(orca_setup);
    // Logger::info(agent_name, agent_id, " pub goal: ", orca_setup.desired_pos[0], orca_setup.desired_pos[1], orca_setup.desired_pos[2]);
}

void SunrayFormation::dynamic_formation_online(std::string name)
{
    std::tuple<float, float> goal;
    if (name == "circle")
    {
        traj_gen.set_param(circle_params.lissa_a,
                           circle_params.lissa_b,
                           M_PI / 2,
                           circle_params.radius_x,
                           circle_params.radius_y,
                           circle_params.center_x,
                           circle_params.center_y,
                           circle_params.linear_speed);
        goal = this->traj_gen.circle_trajectory_online(now_idx);
    }
    else if (name == "figure_eight")
    {
        traj_gen.set_param(figure_eight_params.lissa_a,
                           figure_eight_params.lissa_b,
                           M_PI / 2,
                           figure_eight_params.radius_x,
                           figure_eight_params.radius_y,
                           figure_eight_params.center_x,
                           figure_eight_params.center_y,
                           figure_eight_params.linear_speed);
        goal = this->traj_gen.figure_eight_online(now_idx);
    }
    else
    {
        return;
    }
    orca_setup.header.stamp = ros::Time::now();
    orca_setup.cmd = sunray_msgs::OrcaSetup::GOAL_RUN;
    orca_setup.desired_pos[0] = std::get<0>(goal);
    orca_setup.desired_pos[1] = std::get<1>(goal);
    orca_setup.desired_pos[2] = 1.0;
    orca_setup.desired_yaw = 0.0;
    orca_setup_pub.publish(orca_setup);
}

// 定时发布agent_state
void SunrayFormation::timer_pub_state(const ros::TimerEvent &e)
{
    // 将所有无人机或无人车位置转为orca所需要的格式发布到ORCA节点
    for (int i = 0; i < agent_num; i++)
    {
        agent_state_pub[i].publish(agent_state[i]);
    }
}

// 定时发布动态队形
void SunrayFormation::timer_dynamic_formation(const ros::TimerEvent &e)
{
    // 如果当前模式是动态队形
    if (state == sunray_msgs::Formation::FORMATION && formation_type == sunray_msgs::Formation::DYNAMIC)
    {
        if (dynamic_type == 0)
        {
            // 首次进入会发布第一个位置作为就绪点 等待10s后开始移动
            if (first_dynamic && ros::Time::now() - first_dynamic_time > ros::Duration(10.0))
            {
                first_dynamic = false;
                now_idx = init_idx[formation_name];
            }
            dynamic_formation_pub(formation_name);
        }
        else
        {
            if (first_dynamic && ros::Time::now() - first_dynamic_time > ros::Duration(10.0))
            {
                first_dynamic = false;
                now_idx = 0;
            }
            dynamic_formation_online(formation_name);
        }

        if (!first_dynamic)
        {
            // 更新当前轨迹目标点索引
            now_idx++;
        }
    }
    // 如果是领队模式
    else if (state == sunray_msgs::Formation::FORMATION && formation_type == sunray_msgs::Formation::LEADER)
    {
        leader_formation_pub();
    }
}

void SunrayFormation::leader_formation_pub()
{
    // 如果自己是领队
    if ((agent_type == 0 && agent_id == leader_id) || (agent_type == 1 && agent_id == leader_id - 100))
    {
        // 读取巡航点
        float goal_x = waypoints[now_idx % waypoints.size()].pose_x;
        float goal_y = waypoints[now_idx % waypoints.size()].pose_y;

        orca_setup.header.stamp = ros::Time::now();
        orca_setup.cmd = sunray_msgs::OrcaSetup::GOAL_RUN;
        orca_setup.desired_pos[0] = goal_x;
        orca_setup.desired_pos[1] = goal_y;
        orca_setup.desired_pos[2] = 1.0;
        orca_setup.desired_yaw = 0.0;
        orca_setup_pub.publish(orca_setup);

        if (orca_cmd.goal_pos[0] == goal_x && orca_cmd.goal_pos[1] == goal_y && orca_cmd.state == sunray_msgs::OrcaCmd::ARRIVED)
        {
            now_idx++;
        }
    }
    else
    {
        // 如果自己是跟随者
        // 计算自己相对于领队的位置
        // float leader_yaw = tf::getYaw(agent_state[0].pose.pose.orientation);
        float leader_yaw = 0; // 写死角度 四旋翼旋转太快跟随者跟不上
        int pos_idx = 0;
        if (agent_type == 0 && leader_id > 100) // 无人机leader是ugv时
        {
            pos_idx = agent_id;
        }
        else if ((agent_type == 1 && leader_id > 100) && agent_id < (leader_id % 100))
        {
            pos_idx = agent_id;
        }
        else if ((agent_type == 1 && leader_id > 100) && agent_id > (leader_id % 100))
        {
            pos_idx = agent_id - 1;
        }
        else if (agent_type == 1 && leader_id <= 100)
        {
            pos_idx = agent_id;
        }
        else if (agent_type == 0 && leader_id <= 100 && agent_id > leader_id)
        {
            pos_idx = agent_id - 1;
        }
        else if (agent_type == 0 && leader_id <= 100 && agent_id > leader_id)
        {
            pos_idx = agent_id;
        }
        else
        {
            Logger::error(node_name, "无法计算跟随者位置，请检查无人机编号和领队编号");
        }
        std::tuple<float, float> goal = calculateFollowerPosition(pos_idx,
                                                                  leader_pose[0],
                                                                  leader_pose[1],
                                                                  leader_yaw,
                                                                  1.0,
                                                                  1.0);
        orca_setup.header.stamp = ros::Time::now();
        orca_setup.cmd = sunray_msgs::OrcaSetup::GOAL_RUN;
        orca_setup.desired_pos[0] = std::get<0>(goal);
        orca_setup.desired_pos[1] = std::get<1>(goal);
        orca_setup.desired_pos[2] = 1.0;
        orca_setup.desired_yaw = 0.0;
        orca_setup_pub.publish(orca_setup);
    }
}

void SunrayFormation::agent_mode_check(const ros::TimerEvent &e)
{
    this->debug();
    if (state == sunray_msgs::Formation::FORMATION)
    {
        if ((ros::Time::now() - orca_cmd_time).toSec() > 1.5)
        {
            state = sunray_msgs::Formation::HOVER;
            Logger::print_color(int(LogColor::red), node_name, "ORCA节点超时，切换至悬停模式");
        }
    }
    if (state == sunray_msgs::Formation::TAKEOFF && agent_type == 0)
    {
        float time_diff = ros::Time::now().toSec() - mode_takeoff_time.toSec();
        if (time_diff < 5 &&
            uav_state.control_mode != sunray_msgs::UAVSetup::CMD_CONTROL)
        {
            if (uav_state.control_mode != sunray_msgs::UAVSetup::CMD_CONTROL)
            {
                uav_setup.header.stamp = ros::Time::now();
                uav_setup.cmd = sunray_msgs::UAVSetup::SET_CONTROL_MODE;
                uav_setup.control_mode = "CMD_CONTROL";
                uav_setup_pub.publish(uav_setup);
            }
        }
        else if (time_diff > 5 && time_diff < 10 && !uav_state.armed)
        {
            if (uav_state.control_mode != sunray_msgs::UAVSetup::CMD_CONTROL)
            {
                Logger::print_color(int(LogColor::red), node_name, "无人机模式切换失败，请检查无人机状态");
                state = sunray_msgs::Formation::INIT;
                return;
            }
            uav_setup.header.stamp = ros::Time::now();
            uav_setup.cmd = sunray_msgs::UAVSetup::ARM;
            uav_setup_pub.publish(uav_setup);
        }
        else if (time_diff > 10 && time_diff < 15)
        {
            if (!uav_state.armed)
            {
                Logger::print_color(int(LogColor::red), node_name, "无人机解锁失败，请检查无人机状态");
                state = sunray_msgs::Formation::INIT;
                return;
            }
            uav_cmd.header.stamp = ros::Time::now();
            uav_cmd.cmd = sunray_msgs::UAVControlCMD::Takeoff;
            uav_control_cmd.publish(uav_cmd);
        }
        else if (time_diff > 15 && time_diff < 20 &&
                 uav_state.control_mode == sunray_msgs::UAVSetup::CMD_CONTROL && uav_state.armed)
        {
            set_home();
            orca_setup.header.stamp = ros::Time::now();
            orca_setup.cmd = sunray_msgs::OrcaSetup::GOAL;
            orca_setup.desired_pos[0] = home_pose[0];
            orca_setup.desired_pos[1] = home_pose[1];
            orca_setup.desired_pos[2] = 1.0;
            orca_setup.desired_yaw = 0.0;
            orca_setup_pub.publish(orca_setup);
            state = sunray_msgs::Formation::INIT;
        }
    }
    else if (state == sunray_msgs::Formation::LAND && agent_type == 0)
    {
        uav_setup.header.stamp = ros::Time::now();
        uav_setup.cmd = sunray_msgs::UAVSetup::SET_CONTROL_MODE;
        uav_setup.control_mode = "LAND_CONTROL";
        uav_setup_pub.publish(uav_setup);
        state = sunray_msgs::Formation::INIT;
    }
    else if (state == sunray_msgs::Formation::HOVER)
    {

        if (agent_type == 0)
        {
            uav_cmd.header.stamp = ros::Time::now();
            uav_cmd.cmd = sunray_msgs::UAVControlCMD::Hover;
            uav_control_cmd.publish(uav_cmd);
        }
        else if (agent_type == 1)
        {
            ugv_cmd.header.stamp = ros::Time::now();
            ugv_cmd.cmd = sunray_msgs::UGVControlCMD::HOLD;
            ugv_control_cmd.publish(ugv_cmd);
        }
        state = sunray_msgs::Formation::INIT;
    }
}

void SunrayFormation::set_home()
{
    this->home_pose[0] = agent_state[agent_id - 1].pose.pose.position.x;
    this->home_pose[1] = agent_state[agent_id - 1].pose.pose.position.y;
    this->home_pose[2] = agent_state[agent_id - 1].pose.pose.position.z;
    home_set = true;
}

// 根据ID计算跟随位置
std::tuple<float, float> SunrayFormation::calculateFollowerPosition(int vehicleId, double leaderPos_x, double leaderPos_y, double yaw,
                                                                    double base_x, double base_y)
{
    // 田字型包围 最大8个跟随者
    std::tuple<float, float> followerPos;
    switch (vehicleId)
    {
    case 1:
        followerPos = std::make_tuple(leaderPos_x + base_x, leaderPos_y + base_y);
        break;
    case 2:
        followerPos = std::make_tuple(leaderPos_x + base_x, leaderPos_y - base_y);
        break;
    case 3:
        followerPos = std::make_tuple(leaderPos_x - base_x, leaderPos_y - base_y);
        break;
    case 4:
        followerPos = std::make_tuple(leaderPos_x - base_x, leaderPos_y + base_y);
        break;
    case 5:
        followerPos = std::make_tuple(leaderPos_x, leaderPos_y + base_y);
        break;
    case 6:
        followerPos = std::make_tuple(leaderPos_x + base_x, leaderPos_y);
        break;
    case 7:
        followerPos = std::make_tuple(leaderPos_x, leaderPos_y - base_y);
        break;
    case 8:
        followerPos = std::make_tuple(leaderPos_x - base_x, leaderPos_y);
        break;
    default:
        followerPos = std::make_tuple(leaderPos_x, leaderPos_y);
        break;
    }
    return followerPos;
}

void SunrayFormation::read_formation_yaml()
{
    // 检查文件是否存在
    if (!std::ifstream(file_path).good())
    {
        Logger::warning("配置文件路径错误或文件不存在！");
    }
    else
    {
        formation_data = YAML::LoadFile(file_path);
        Logger::info("获取配置文件成功!");

        // 读取所有固定队形
        try
        {
            Logger::info("固定阵型配置文件路径:");
            YAML::Node static_formation_path = formation_data["static_fromation"]["file_path"];
            for (const auto &path : static_formation_path)
            {
                Logger::print_color(int(LogColor::green), node_name, "  - ", path.as<std::string>());
                YAML::Node static_formation = YAML::LoadFile(path.as<std::string>());
                for (const auto &formation : static_formation["formations"])
                {
                    std::string name = formation["name"].as<std::string>();
                    Logger::info("固定阵型名称: ", name);
                    FormationData pose_data;
                    pose_data.name = name;
                    pose_data.pose_x = formation["uav_pos"]["uav_" + std::to_string(agent_id) + "_x"].as<double>();
                    pose_data.pose_y = formation["uav_pos"]["uav_" + std::to_string(agent_id) + "_y"].as<double>();
                    pose_data.pose_z = formation["uav_pos"]["uav_" + std::to_string(agent_id) + "_z"].as<double>();
                    static_formation_map[name] = pose_data;
                }
            }
        }
        catch (const YAML::Exception &e)
        {
            Logger::error("读取固定阵型配置文件失败！");
        }

        // 读取所有动态队形
        try
        {
            Logger::info("动态阵型配置文件路径:");
            YAML::Node dynamic_formation_path = formation_data["dynamic_fromation"]["file_path"];
            for (const auto &path : dynamic_formation_path)
            {
                Logger::print_color(int(LogColor::green), node_name, "  - ", path.as<std::string>());
                YAML::Node dynamic_formation = YAML::LoadFile(path.as<std::string>());
                // 读取动态队形数据
                std::string name = dynamic_formation["initial_point"]["name"].as<std::string>();
                std::string key = "ReadyPoint_" + std::to_string(agent_id);
                init_idx[name] = dynamic_formation["initial_point"][key].as<int>();
                Logger::info("动态阵型名称:", name);
                std::vector<FormationData> points;
                int point_count = 1;
                while (true)
                {
                    std::string point_key = "point_" + std::to_string(point_count);
                    if (!dynamic_formation[point_key])
                        break;

                    YAML::Node point_node = dynamic_formation[point_key];
                    FormationData pose_data;
                    pose_data.pose_x = point_node[0].as<double>();
                    pose_data.pose_y = point_node[1].as<double>();
                    pose_data.pose_z = point_node[2].as<double>();
                    points.push_back(pose_data);
                    point_count++;
                }
                dynamic_formation_map[name] = points;
                Logger::print_color(int(LogColor::green), node_name, "阵型总点数: ", points.size());
            }
        }
        catch (const YAML::Exception &e)
        {
            Logger::error("读取动态阵型配置文件失败！");
        }
    }

    // 读取航点文件
    try
    {
        Logger::info("航点文件配置文件路径:");
        YAML::Node dynamic_formation_path = formation_data["waypoint"]["file_path"];
        for (const auto &path : dynamic_formation_path)
        {
            Logger::print_color(int(LogColor::green), node_name, "  - ", path.as<std::string>());
            YAML::Node dynamic_formation = YAML::LoadFile(path.as<std::string>());
            // 读取航点数据
            int point_count = 1;
            while (true)
            {
                std::string point_key = "point_" + std::to_string(point_count);
                if (!dynamic_formation[point_key])
                    break;

                YAML::Node point_node = dynamic_formation[point_key];
                FormationData pose_data;
                pose_data.pose_x = point_node[0].as<double>();
                pose_data.pose_y = point_node[1].as<double>();
                pose_data.pose_z = point_node[2].as<double>();
                waypoints.push_back(pose_data);
                point_count++;
            }
            Logger::print_color(int(LogColor::green), node_name, "航点总数: ", waypoints.size());
        }
    }
    catch (const YAML::Exception &e)
    {
        Logger::error("读取航点配置文件失败！");
    }
}

void SunrayFormation::debug()
{
    Logger::print_color(int(LogColor::green), this->node_name, ">>>>>>>>>>>>>>>", this->agent_name, this->agent_id, "<<<<<<<<<<<<<<<");
    if (this->state == sunray_msgs::Formation::INIT)
    {
        Logger::print_color(int(LogColor::green), this->node_name, "当前状态: ", "INIT");
    }
    else if (this->state == sunray_msgs::Formation::TAKEOFF)
    {
        Logger::print_color(int(LogColor::green), this->node_name, "当前状态: ", "TAKEOFF");
    }
    else if (this->state == sunray_msgs::Formation::LAND)
    {
        Logger::print_color(int(LogColor::green), this->node_name, "当前状态: ", "LAND");
    }
    else if (this->state == sunray_msgs::Formation::HOVER)
    {
        Logger::print_color(int(LogColor::green), this->node_name, "当前状态: ", "HOVER");
    }
    else if (this->state == sunray_msgs::Formation::FORMATION)
    {
        Logger::print_color(int(LogColor::green), this->node_name, "当前状态: ", "FORMATION");
    }
    if (this->formation_type == sunray_msgs::Formation::STATIC)
    {
        Logger::print_color(int(LogColor::green), this->node_name, "当前阵型: ", "STATIC");
    }
    else if (this->formation_type == sunray_msgs::Formation::DYNAMIC)
    {
        Logger::print_color(int(LogColor::green), this->node_name, "当前阵型: ", "DYNAMIC");
    }
    else if (this->formation_type == sunray_msgs::Formation::LEADER)
    {
        Logger::print_color(int(LogColor::green), this->node_name, "当前阵型: ", "LEADER");
    }
    Logger::print_color(int(LogColor::green), this->node_name, "当前目标点: ", this->orca_cmd.goal_pos[0], this->orca_cmd.goal_pos[1]);
    Logger::print_color(int(LogColor::green), this->node_name, "当前控制速度: ", this->orca_cmd.linear[0], this->orca_cmd.linear[1]);
}

// 计算目标角度
double SunrayFormation::calculateTargetYaw(double target_x, double target_y)
{
    // 计算目标点相对于当前位置的角度
    double delta_x = target_x - agent_state[agent_id - 1].pose.pose.position.x;
    double delta_y = target_y - agent_state[agent_id - 1].pose.pose.position.y;
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
    double current_yaw = tf::getYaw(agent_state[agent_id - 1].pose.pose.orientation);
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
    if (fabs(yaw_diff) > M_PI / 12) // 最大转向角度为15度
    {
        if (yaw_diff > 0)
        {
            target_yaw = current_yaw + M_PI / 12;
        }
        else
        {
            target_yaw = current_yaw - M_PI / 12;
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