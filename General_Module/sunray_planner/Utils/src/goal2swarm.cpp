// 订阅move_base_simple/goal话题 转发为多个goal话题 其中以及每个goal的坐标为当前位置的坐标加上一个y轴偏移量 先是加 后是减 以此循环
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Path.h>
#include <tf/transform_listener.h>
#include <sunray_msgs/UAVState.h>
#include <cmath>
#include <yaml-cpp/yaml.h>

class GoalEgoSwarm
{
public:
    GoalEgoSwarm(ros::NodeHandle nh) : nh_(nh)
    {
        nh.param<int>("uav_id", uav_id, 1);
        nh.param<std::string>("uav_name", uav_name, "uav");
        nh.param<int>("uav_num", uav_num, 3);
        nh.param<int>("goal_type", goal_type, 1);
        nh.param<float>("offset", offset, 1.0);
        nh.param<bool>("use_hight", use_hight, true);
        nh.param<float>("z_hight", z_hight, 1.0);
        nh.param<std::string>("goal_topic", goal_topic, "goal");
        nh.param<std::string>("waypoint_file", waypoint_file, "/home/yundrone/Sunray/General_Module/sunray_planner/Utils/config/waypoint.yml");

        if (goal_type == 1)
        {
            std::cout<<"goal_type is 1"<<std::endl;
            // 订阅move_base_simple/goal话题
            goal_sub = nh_.subscribe("/move_base_simple/goal", 1, &GoalEgoSwarm::goalCallback, this);
            // 发布多个goal话题
            for (int i = 0; i < uav_num; i++)
            {
                std::string topic = "/" + goal_topic + "_" + std::to_string(i + 1);
                goal_pub_.push_back(nh_.advertise<geometry_msgs::PoseStamped>(topic, 1));
            }
        }
        else if (goal_type == 2)
        {
            std::cout<<"goal_type is 2"<<std::endl;
            std::string topic = "/" + uav_name + std::to_string(uav_id);
            uav_pos_sub = nh_.subscribe(topic + "/mavros/local_position/pose", 1, &GoalEgoSwarm::uavPosCallback, this);
            uav_state_sub = nh_.subscribe(topic + "/sunray/uav_state", 1, &GoalEgoSwarm::uavStateCallback, this);
            topic = "/" + goal_topic + "_" + std::to_string(uav_id);
            goal_pub_.push_back(nh_.advertise<geometry_msgs::PoseStamped>(topic, 1));
            this->waypointPub();
        }
        else
        {
            std::cout << "Invalid goal_type parameter" << std::endl;
        }
    }

    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        // 获取当前位置
        float x = msg->pose.position.x;
        float y = msg->pose.position.y;
        float z = msg->pose.position.z;
        if (use_hight)
        {
            z = z_hight;
        }
        for (int i = 0; i < uav_num; i++)
        {
            geometry_msgs::PoseStamped goal = *msg;
            // 根据指向位置和偏移量计算目标位置
            // 获取指向方向
            float yaw = tf::getYaw(msg->pose.orientation);
            // std::cout << "yaw: " << yaw / 3.14 * 180 << std::endl;

            // 计算目标位置 与当前指向方向垂直
            // float offset =
            goal.pose.position.x = x - offset * i * cos(M_PI / 2 - yaw);
            goal.pose.position.y = y + offset * i * sin(M_PI / 2 - yaw);
            goal.pose.position.z = z;

            // std::cout << "cos: " << offset * i * cos(M_PI / 2 - yaw) << " sin: " << offset * i * sin(M_PI / 2 - yaw) << std::endl;

            // 发布目标位置
            goal_pub_[i].publish(goal);
        }
    }

    void uavPosCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        pose_cb_flag = true;
        // 获取当前位置
        uav_x = msg->pose.position.x;
        uav_y = msg->pose.position.y;
        uav_z = msg->pose.position.z;
    }

    void uavStateCallback(const sunray_msgs::UAVState::ConstPtr &msg)
    {
        control_state = msg->control_mode;
    }

    void waypointPub()
    {
        int num_points = 0;
        // 读取航点 最高支持10个航点
        std::vector<geometry_msgs::PointStamped> waypoints;
        // 打开航点文件yml
        // 读取yaml文件
        try
        {
            // 读取yaml文件
            YAML::Node config = YAML::LoadFile(waypoint_file);

            // 获取航点数量
            num_points = config["num_points"].as<int>();
            if (num_points > 10)
            {
                std::cout << "航点数量超过10个，请检查配置文件" << std::endl;
                return;
            }

            // 遍历每个航点
            for (int i = 1; i <= num_points; i++)
            {
                std::string waypoint_name = "waypoint" + std::to_string(i);
                YAML::Node waypoint = config[waypoint_name];

                // 获取航点坐标
                double x = waypoint["x"].as<double>();
                double y = waypoint["y"].as<double>();
                double z = waypoint["z"].as<double>();
                geometry_msgs::PointStamped point;
                point.header.frame_id = "map";
                point.point.x = x;
                point.point.y = y;
                point.point.z = z;
                waypoints.push_back(point);

                // 输出航点坐标
                std::cout << "航点" << i << "坐标：(" << x << ", " << y << ", " << z << ")" << std::endl;
            }
        }
        catch (const std::exception &e)
        {
            std::cerr << "读取文件或解析文件内容出错：" << std::endl;
            return;
        }
        if (!pose_cb_flag)
        {
            std::cout << "waiting for pose callback" << std::endl;
        }
        ros::Rate loop_rate(10);
        while (!pose_cb_flag && ros::ok())
        {
            ros::spinOnce();
            loop_rate.sleep();
        }
        std::cout << "start waypoint" << std::endl;

        if (control_state != 2)
        {
            std::cout << "waiting for control state to be CMD_CONTROL" << std::endl;
        }
        while (control_state != 2 && ros::ok())
        {
            ros::spinOnce();
            loop_rate.sleep();
        }
        // 发布航点
        int way = -1;
        for (int i = 0; i < num_points && ros::ok();)
        {

            if (way != i)
            {
                geometry_msgs::PoseStamped goal;
                goal.header.frame_id = "world";
                goal.header.stamp = ros::Time::now();
                goal.pose.position.x = waypoints[i].point.x;
                goal.pose.position.y = waypoints[i].point.y;
                goal.pose.position.z = waypoints[i].point.z;
                goal_pub_[0].publish(goal);
                std::cout << "当前航点：" << waypoints[i].point.x << " " << waypoints[i].point.y << " " << waypoints[i].point.z << std::endl;
                way = i;
            }
            // std::cout<<"uav_x: "<<uav_x<<" uav_y: "<<uav_y<<" uav_z: "<<uav_z<<std::endl;
            // std::cout<<abs(uav_x - waypoints[i].point.x)<<" "<<abs(uav_y - waypoints[i].point.y)<<" "<<abs(uav_z - waypoints[i].point.z)<<std::endl;
            if (abs(uav_x - waypoints[i].point.x) < 0.2 && abs(uav_y - waypoints[i].point.y) < 0.2 && abs(uav_z - waypoints[i].point.z) < 0.2)
            {
                i++;
                
                if (i == num_points)
                {
                    std::cout << "waypoint finished" << std::endl;
                    break;
                }
                std::cout<<"下一个航点"<<std::endl;
            }
            ros::spinOnce();
            loop_rate.sleep();
        }
    }

private:
    int uav_id;
    int control_state;
    std::string uav_name;
    int uav_num;
    int goal_type;
    float offset;
    bool use_hight;
    float z_hight;
    std::string goal_topic;
    std::string waypoint_file;
    ros::NodeHandle nh_;
    ros::Subscriber goal_sub;
    ros::Subscriber uav_pos_sub;
    ros::Subscriber uav_state_sub;

    float uav_x, uav_y, uav_z, uav_yaw;
    bool pose_cb_flag = false;
    // 发布者可能有多个 根据uav_num决定
    std::vector<ros::Publisher> goal_pub_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "goal2swarm");
    ros::NodeHandle nh("~");
    GoalEgoSwarm ges(nh);
    ros::spin();
    return 0;
}