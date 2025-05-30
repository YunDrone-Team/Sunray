#include "uav_formation.h"

void SunrayFormation::init(ros::NodeHandle &nh_)
{
    nh = nh_;
    nh.param<int>("agent_id", agent_id, 1);
    nh.param<int>("agent_num", agent_num, 1);
    nh.param<int>("agent_type", agent_type, 0);
    nh.param<std::string>("agent_name", agent_name, "uav");
    nh.param<std::string>("file_path", file_path, "/home/yundrone/Sunray/sunray_formation/uav_formation/config/config.yaml");

    topic_prefix = "/" + agent_name + std::to_string(agent_id);
    // 【订阅】
    formation_cmd_sub = nh.subscribe<sunray_msgs::Formation>("/sunray/formation_cmd", 10, &SunrayFormation::formation_cmd_callback, this);
    orca_cmd_sub = nh.subscribe<sunray_msgs::OrcaCmd>(topic_prefix + "/orca_cmd", 10, &SunrayFormation::orca_cmd_callback, this);
    // 【发布】
    uav_control_cmd = nh.advertise<sunray_msgs::UAVControlCMD>(topic_prefix + "/sunray/uav_control_cmd", 10);
    orca_setup_pub = nh.advertise<sunray_msgs::OrcaSetup>(topic_prefix + "/orca/setup", 10);
    // 【定时器】
    pub_timer = nh.createTimer(ros::Duration(0.05), &SunrayFormation::timer_pub_state, this);
    dynamic_timer = nh.createTimer(ros::Duration(0.1), &SunrayFormation::timer_dynamic_formation, this);

    // 订阅无人机或者无人车的状态
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
            agent_state_sub[i] = nh.subscribe<sunray_msgs::UGVState>(topic_prefix + "/ugv_state", 10,
                                                                     boost::bind(&SunrayFormation::ugv_state_cb, this, _1, i));
        }
        // 【发布】定位状态到 ORCA 节点
        agent_state_pub[i] = nh.advertise<geometry_msgs::PoseStamped>(topic_prefix + "/orca/agent_state", 10);
    }

    // 检查文件是否存在
    if (!std::ifstream(file_path).good())
    {
        Logger::warning("Failed to load formation yaml file!");
    }
    else
    {
        formation_data = YAML::LoadFile(file_path);
        Logger::info("Load formation yaml file successfully!");

        try
        {
            // 读取所有固定队形
            std::cout << "Static Formation Files:" << std::endl;
            YAML::Node static_formation_path = formation_data["static_fromation"]["file_path"];
            for (const auto &path : static_formation_path)
            {
                std::cout << "  - " << path.as<std::string>() << std::endl;
                YAML::Node static_formation = YAML::LoadFile(path.as<std::string>());
                // 读取所有固定队形
                for (const auto &formation : static_formation["formations"])
                {
                    std::string name = formation["name"].as<std::string>();
                    std::cout << "Static Formation Name: " << name << std::endl;
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
            std::cerr << "YAML parsing error: " << e.what() << std::endl;
        }

        try
        {
            // 读取所有动态队形
            std::cout << "\nDynamic Formation Files:" << std::endl;
            YAML::Node dynamic_formation_path = formation_data["dynamic_fromation"]["file_path"];
            for (const auto &path : dynamic_formation_path)
            {
                std::cout << "  - " << path.as<std::string>() << std::endl;
                YAML::Node dynamic_formation = YAML::LoadFile(path.as<std::string>());
                // 读取动态队形数据
                std::string name = dynamic_formation["initial_point"]["name"].as<std::string>();
                std::string key = "ReadyPoint_" + std::to_string(agent_id);
                init_idx[name] = dynamic_formation["initial_point"][key].as<int>();
                std::cout << agent_name << agent_id << " init_idx: " << init_idx[name] << std::endl;

                std::cout << "Dynamic Formation Name: " << name << std::endl;
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
                std::cout << "points size: " << points.size() << std::endl;
            }
        }
        catch (const YAML::Exception &e)
        {
            std::cerr << "YAML parsing error: " << e.what() << std::endl;
        }
    }
}

void SunrayFormation::formation_cmd_callback(const sunray_msgs::Formation::ConstPtr &msg)
{
    // 当指令为FORMATION时，根据formation_type进行队形切换
    if (msg->cmd == sunray_msgs::Formation::FORMATION)
    {
        formation_name = msg->name;
        // 固定队形
        if (msg->formation_type == sunray_msgs::Formation::STATIC)
        {
            state = sunray_msgs::Formation::STATIC;
            static_formation_pub(msg->name);
        }
        // 动态队形
        else if (msg->formation_type == sunray_msgs::Formation::DYNAMIC)
        {
            state = sunray_msgs::Formation::DYNAMIC;
            now_idx = init_idx[msg->name];
            std::cout << agent_name << agent_id << " init_idx: " << init_idx[msg->name] << std::endl;
            first_dynamic = true;
            first_dynamic_time = ros::Time::now();
            std::cout << "Dynamic formation name: " << msg->name << std::endl;
        }
        // 领队模式
        else if (msg->formation_type == 3)
        {
        }
    }
}

// orca状态及控制指令回调
void SunrayFormation::orca_cmd_callback(const sunray_msgs::OrcaCmd::ConstPtr &msg)
{
    // 如果是停止状态，则发送悬停指令
    if (msg->state == sunray_msgs::OrcaCmd::STOP)
    {
        uav_cmd.header.stamp = ros::Time::now();
        uav_cmd.cmd = sunray_msgs::UAVControlCMD::Hover;
    }
    // 如果是运行状态，则发送速度指令
    else if (msg->state == sunray_msgs::OrcaCmd::RUN)
    {
        uav_cmd.header.stamp = ros::Time::now();
        uav_cmd.cmd = sunray_msgs::UAVControlCMD::XyVelZPosYaw;
        // uav_cmd.cmd = sunray_msgs::UAVControlCMD::XyzPosVelYaw;
        uav_cmd.desired_vel[0] = msg->linear[0];
        uav_cmd.desired_vel[1] = msg->linear[1];
        uav_cmd.desired_pos[0] = msg->goal_pos[0];
        uav_cmd.desired_pos[1] = msg->goal_pos[1];
        uav_cmd.desired_pos[2] = 1;
    }
    // 如果是到达状态，则发送位置指令
    else if (msg->state == sunray_msgs::OrcaCmd::ARRIVED)
    {
        uav_cmd.header.stamp = ros::Time::now();
        uav_cmd.cmd = sunray_msgs::UAVControlCMD::XyzPosYaw;
        uav_cmd.desired_pos[0] = msg->goal_pos[0];
        uav_cmd.desired_pos[1] = msg->goal_pos[1];
        uav_cmd.desired_pos[2] = 1;
    }
    // 无人机
    if (agent_type == 0)
    {
        uav_control_cmd.publish(uav_cmd);
    }
    // 无人车
    else if (agent_type == 1)
    {
    }
}

// 无人机状态回调
void SunrayFormation::uav_state_cb(const sunray_msgs::UAVState::ConstPtr &msg, int i)
{
    agent_state[i].pose.position.x = msg->position[0];
    agent_state[i].pose.position.y = msg->position[1];
    agent_state[i].pose.position.z = msg->position[2];
    agent_state[i].pose.orientation = msg->attitude_q;
}

// 无人车状态回调
void SunrayFormation::ugv_state_cb(const sunray_msgs::UGVState::ConstPtr &msg, int i)
{
    agent_state[i].pose.position.x = msg->position[0];
    agent_state[i].pose.position.y = msg->position[1];
    agent_state[i].pose.position.z = msg->position[2];
    agent_state[i].pose.orientation = msg->attitude_q;
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

// 定时发布agent_state
void SunrayFormation::timer_pub_state(const ros::TimerEvent &e)
{
    // 将所有无人机或无人车位置转为orca所需要的格式发布到ORCA节点
    for (int i = 0; i < agent_num; i++)
    {
        agent_state_pub[i].publish(agent_state[i]);
    }
}

void SunrayFormation::timer_dynamic_formation(const ros::TimerEvent &e)
{
    // 如果当前模式是动态队形
    if (state == sunray_msgs::Formation::DYNAMIC)
    {
        // 定时发布动态队形
        dynamic_formation_pub(formation_name);
        // 首次进入会发布第一个位置作为就绪点 等待10s后开始移动
        if (first_dynamic && ros::Time::now() - first_dynamic_time > ros::Duration(10.0))
        {
            first_dynamic = false;
            now_idx = init_idx[formation_name];
        }
        if (!first_dynamic)
        {
            // 更新当前轨迹目标点索引
            now_idx++;
        }
    }
}