// 订阅move_base_simple/goal话题 转发为多个goal话题 其中以及每个goal的坐标为当前位置的坐标加上一个y轴偏移量 先是加 后是减 以此循环
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Path.h>
#include <tf/transform_listener.h>
#include <cmath>

class GoalEgoSwarm
{
public:
    GoalEgoSwarm(ros::NodeHandle nh) : nh_(nh)
    {
        nh.param<int>("uav_num", uav_num, 3);
        nh.param<float>("offset", offset, 1.0);
        nh.param<bool>("use_hight", use_hight, true);
        nh.param<float>("z_hight", z_hight, 1.0);
        nh.param<std::string>("goal_topic", goal_topic, "goal");
        // 订阅move_base_simple/goal话题
        goal_sub_ = nh_.subscribe("/move_base_simple/goal", 1, &GoalEgoSwarm::goalCallback, this);
        // 发布多个goal话题
        for (int i = 0; i < uav_num; i++)
        {
            std::string topic = "/" + goal_topic + "_" + std::to_string(i+1);
            goal_pub_.push_back(nh_.advertise<geometry_msgs::PoseStamped>(topic, 1));
        }
    }

    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        // 获取当前位置
        float x = msg->pose.position.x;
        float y = msg->pose.position.y;
        float z = msg->pose.position.z;
        if(use_hight)
        {
            z = z_hight;
        }
        
        for (int i = 0; i < uav_num; i++)
        {
            geometry_msgs::PoseStamped goal = *msg;
            // 根据指向位置和偏移量计算目标位置
            // 获取指向方向
            float yaw = tf::getYaw(msg->pose.orientation);
            std::cout << "yaw: " << yaw / 3.14 * 180 << std::endl;

            // 计算目标位置 与当前指向方向垂直
            // float offset =
            goal.pose.position.x = x - offset * i * cos(M_PI / 2 - yaw);
            goal.pose.position.y = y + offset * i * sin(M_PI / 2 - yaw);

            std::cout << "cos: " << offset * i * cos(M_PI / 2 - yaw) << " sin: " << offset * i * sin(M_PI / 2 - yaw) << std::endl;

            // 发布目标位置
            goal_pub_[i].publish(goal);
        }
    }

private:
    int uav_num;
    float offset;
    bool use_hight;
    float z_hight;
    std::string goal_topic;
    ros::NodeHandle nh_;
    ros::Subscriber goal_sub_;
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