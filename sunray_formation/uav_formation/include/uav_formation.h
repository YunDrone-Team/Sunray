#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include "printf_format.h"
#include <iostream>
#include <fstream>
#include <yaml-cpp/yaml.h>

using namespace sunray_logger;

struct FormationData
{
    std::string name;
    double pose_x;
    double pose_y;
    double pose_z;
    double pose_yaw;
};

class UAVFormation
{
public:
    UAVFormation(){};
    ~UAVFormation(){};
    void init(ros::NodeHandle &nh_);

private:
    ros::NodeHandle nh;
    ros::Publisher goal_pub;
    ros::Subscriber cmd_sub;
    int uav_id;
    std::string uav_name;
    std::string uav_prefix;
    bool file_exist;
    std::string file_path;
    YAML::Node formation_data; 
    std::map<std::string, FormationData> formation_map;

    void fix_formation_pub();                                         // 固定阵型 计算位置 发布位置
    void pre_define_formation_pub(std::string formation_name); // 预定义阵型 读取文件发布位置
    void cmd_callback(const std_msgs::String::ConstPtr& msg);
};