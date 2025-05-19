#include <ros/ros.h>
#include <sunray_msgs/UGVControlCMD.h>
#include <sunray_msgs/UGVState.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>
#include <cmath>

class SquareDemo
{
private:
    ros::NodeHandle nh;
    ros::Publisher cmd_pub;
    ros::Subscriber state_sub;
    int ugv_id;
    std::vector<std::pair<double, double>> waypoints;
    size_t current_waypoint;
    bool goal_reached;
    double tolerance;
    sunray_msgs::UGVState current_state;

public:
    SquareDemo() : current_waypoint(0), goal_reached(false), tolerance(0.02)
    {
        // 初始化参数
        nh.param<int>("ugv_id", ugv_id, 1);
        std::string topic_prefix = "/ugv" + std::to_string(ugv_id) + "/sunray_ugv/ugv_control_cmd";

        // 初始化发布器和订阅器
        cmd_pub = nh.advertise<sunray_msgs::UGVControlCMD>(topic_prefix, 10);
        state_sub = nh.subscribe("/ugv" + std::to_string(ugv_id) + "/sunray_ugv/ugv_state", 1, &SquareDemo::stateCallback, this);

        // 定义四边形四个顶点（假设边长为2米）
        waypoints = {
            {2.0, 0.0}, // 点1
            {2.0, 2.0}, // 点2
            {0.0, 2.0}, // 点3
            {0.0, 0.0}  // 点4（回到起点）
        };
    }

    void stateCallback(const sunray_msgs::UGVState::ConstPtr &msg)
    {
        current_state = *msg;
        // 检查是否到达当前目标点
        double dx = current_state.pos_setpoint[0] - current_state.position[0];
        double dy = current_state.pos_setpoint[1] - current_state.position[1];
        if (sqrt(dx * dx + dy * dy) < tolerance)
        {
            goal_reached = true;
        }
    }

    void publishNextWaypoint()
    {
        if (current_waypoint >= waypoints.size())
        {
            ROS_INFO("Square trajectory completed.");
            ros::shutdown();
            return;
        }

        sunray_msgs::UGVControlCMD ugv_cmd;
        ugv_cmd.cmd = sunray_msgs::UGVControlCMD::POS_CONTROL_ENU;
        ugv_cmd.desired_pos[0] = waypoints[current_waypoint].first;
        ugv_cmd.desired_pos[1] = waypoints[current_waypoint].second;
        ugv_cmd.desired_yaw = 0.0; // 保持朝向不变

        cmd_pub.publish(ugv_cmd);
        ROS_INFO("Publishing waypoint %zu",
                 current_waypoint + 1);

        // 等待到达或超时
        goal_reached = false;
        ros::Rate rate(10);
        ros::Time start_time = ros::Time::now();
        while (ros::ok() && !goal_reached && (ros::Time::now() - start_time).toSec() < 20.0)
        {
            ros::spinOnce();
            rate.sleep();
        }

        current_waypoint++;
    }

    void run()
    {
        ros::Rate rate(1);
        while (ros::ok() && current_waypoint < waypoints.size())
        {
            publishNextWaypoint();
            rate.sleep();
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ugv_square_demo");
    SquareDemo demo;
    demo.run();
    return 0;
}