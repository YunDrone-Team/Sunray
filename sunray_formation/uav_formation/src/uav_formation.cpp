#include "uav_formation.h"

void UAVFormation::init(ros::NodeHandle &nh_)
{
    nh = nh_;
    nh.param<int>("uav_id", uav_id, 1);
    nh.param<std::string>("uav_name", uav_name, "uav");
    nh.param<std::string>("file_path", file_path, "/home/yundrone/Sunray/sunray_formation/uav_formation/config/formation.yaml");
    uav_prefix = uav_name + std::to_string(uav_id);
    goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/goal_" + std::to_string(uav_id), 10);
    cmd_sub = nh.subscribe<std_msgs::String>("/sunray/formation_cmd", 10, &UAVFormation::cmd_callback, this);
    if (!std::ifstream(file_path).good())
    {
        Logger::warning("Failed to load formation yaml file!");
        file_exist = false;
    }
    else
    {
        file_exist = true;
        formation_data = YAML::LoadFile(file_path);
        int uav_num = formation_data["uav_num"].as<int>();
        Logger::info("Load formation yaml file successfully!");

        try
        {
            // 加载YAML文件
            // 遍历所有的formation节点
            for (const auto &formation : formation_data["formations"])
            {
                std::string name = formation["name"].as<std::string>();
                std::cout << "Formation Name: " << name << std::endl;
                FormationData formation_data;
                formation_data.name = name;
                formation_data.pose_x = formation["uav_pos"]["uav_" + std::to_string(uav_id) + "_x"].as<double>();
                formation_data.pose_y = formation["uav_pos"]["uav_" + std::to_string(uav_id) + "_y"].as<double>();
                formation_data.pose_z = formation["uav_pos"]["uav_" + std::to_string(uav_id) + "_z"].as<double>();
                formation_map[name] = formation_data;
                std::cout << "Formation data: " << formation_data.pose_x << formation_data.pose_y << formation_data.pose_z << std::endl;
            }
        }
        catch (const YAML::Exception &e)
        {
            std::cerr << "YAML parsing error: " << e.what() << std::endl;
        }
    }
}

void UAVFormation::cmd_callback(const std_msgs::String::ConstPtr &msg)
{
    if (!file_exist)
    {
        Logger::error("Formation yaml file not exist!");
        return;
    }
    std::string formation_name = msg->data;
    Logger::info("Formation name: ", formation_name);
    if (formation_map.find(formation_name) == formation_map.end())
    {
        Logger::error("Formation name not exist!");
        return;
    }
    pre_define_formation_pub(formation_name);
}

void UAVFormation::pre_define_formation_pub(std::string formation_name)
{
    geometry_msgs::PoseStamped goal;
    goal.header.frame_id = "world";
    goal.header.stamp = ros::Time::now();
    goal.pose.position.x = formation_map[formation_name].pose_x;
    goal.pose.position.y = formation_map[formation_name].pose_y;
    goal.pose.position.z = formation_map[formation_name].pose_z;
    goal_pub.publish(goal);
    Logger::info(uav_prefix, " pub goal: ", goal.pose.position.x, goal.pose.position.y, goal.pose.position.z);
}