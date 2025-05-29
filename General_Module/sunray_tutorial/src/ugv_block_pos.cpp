#include <ros/ros.h>
#include <sunray_msgs/UGVControlCMD.h>
#include <sunray_msgs/UGVState.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class SquareDemo
{
private:
    ros::NodeHandle nh;
    ros::Publisher cmd_pub;
    ros::Subscriber state_sub;
    int ugv_id;
    std::vector<std::pair<double, double>> waypoints;
    size_t current_waypoint;
    double tolerance;
    sunray_msgs::UGVState current_state;
    bool first_waypoint_sent;
    double max_wait_time;
    ros::Time waypoint_start_time;
    bool state_received;
    double last_distance;

public:
    SquareDemo() : 
        current_waypoint(0), 
        tolerance(0.1), 
        first_waypoint_sent(false),
        max_wait_time(30.0),
        state_received(false),
        last_distance(1000.0)
    {
        // 初始化参数
        nh.param<int>("ugv_id", ugv_id, 1);
        nh.param<double>("tolerance", tolerance, 0.1);
        nh.param<double>("max_wait_time", max_wait_time, 30.0);
        
        std::string topic_prefix = "/ugv" + std::to_string(ugv_id) + "/sunray_ugv/ugv_control_cmd";

        // 初始化发布器和订阅器
        cmd_pub = nh.advertise<sunray_msgs::UGVControlCMD>(topic_prefix, 10);
        state_sub = nh.subscribe("/ugv" + std::to_string(ugv_id) + "/sunray_ugv/ugv_state", 10, &SquareDemo::stateCallback, this);

        // 定义四边形四个顶点（边长2米）
        waypoints = {
            {0.5, 0.0}, // 点1
            {0.5, 0.5}, // 点2
            {0.0, 0.5}, // 点3
            {0.0, 0.0}  // 点4（回到起点）
        };
        
        ROS_INFO("UGV Square Demo initialized for UGV %d", ugv_id);
    }

    void stateCallback(const sunray_msgs::UGVState::ConstPtr &msg)
    {
        current_state = *msg;
        state_received = true;
    }

    // 计算两点之间的距离
    double distanceToWaypoint(double x, double y)
    {
        double dx = x - current_state.position[0];
        double dy = y - current_state.position[1];
        return std::sqrt(dx*dx + dy*dy);
    }

    // 发布目标点
    void publishWaypoint()
    {
        if (current_waypoint >= waypoints.size())
        {
            ROS_INFO("Square trajectory completed.");
            ros::shutdown();
            return;
        }

        double target_x = waypoints[current_waypoint].first;
        double target_y = waypoints[current_waypoint].second;
        
        sunray_msgs::UGVControlCMD ugv_cmd;
        ugv_cmd.cmd = sunray_msgs::UGVControlCMD::POS_CONTROL_ENU;
        ugv_cmd.desired_pos[0] = target_x;
        ugv_cmd.desired_pos[1] = target_y;
        ugv_cmd.desired_yaw = 0.0; // 保持朝向不变

        cmd_pub.publish(ugv_cmd);
        
        if (!first_waypoint_sent) {
            ROS_INFO("Starting square trajectory for UGV %d", ugv_id);
            first_waypoint_sent = true;
        }
        
        ROS_INFO("Publishing waypoint %zu: (%.1f, %.1f)", 
                 current_waypoint+1, target_x, target_y);
        
        waypoint_start_time = ros::Time::now();
        last_distance = distanceToWaypoint(target_x, target_y); // 重置距离
    }

    void run()
    {
        ros::Rate rate(10); // 10Hz控制频率
        
        // 等待首次状态更新
        ROS_INFO("Waiting for first state update...");
        while (ros::ok() && !state_received) {
            ros::spinOnce();
            rate.sleep();
        }
        ROS_INFO("First state received. Starting trajectory.");
        
        // 初始发布第一个目标点
        publishWaypoint();
        
        while (ros::ok() && current_waypoint < waypoints.size())
        {
            ros::spinOnce();
            
            if (!state_received) {
                ROS_WARN_THROTTLE(1.0, "Waiting for state update...");
                rate.sleep();
                continue;
            }
            
            double target_x = waypoints[current_waypoint].first;
            double target_y = waypoints[current_waypoint].second;
            
            // 计算当前距离
            double current_distance = distanceToWaypoint(target_x, target_y);
            last_distance = current_distance;
            
            // 检查是否到达目标点
            if (current_distance < tolerance)
            {
                ROS_INFO("Reached waypoint %zu: (%.1f, %.1f)", 
                         current_waypoint+1, target_x, target_y);
                current_waypoint++;
                
                if (current_waypoint < waypoints.size()) {
                    publishWaypoint();
                } else {
                    ROS_INFO("Trajectory completed!");
                    break;
                }
            }
            // 检查是否正在接近目标点
            else 
            {
                // 显示调试信息（限制频率）
                ROS_INFO_THROTTLE(1.0, "Distance to waypoint %zu: %.2f m", 
                                 current_waypoint+1, current_distance);
                
                // 超时处理
                double elapsed = (ros::Time::now() - waypoint_start_time).toSec();
                if (elapsed > max_wait_time)
                {
                    ROS_WARN("Timeout at waypoint %zu (%.1f s). Moving to next point.", 
                             current_waypoint+1, elapsed);
                    current_waypoint++;
                    
                    if (current_waypoint < waypoints.size()) {
                        publishWaypoint();
                    } else {
                        ROS_INFO("Trajectory completed (with timeout).");
                        break;
                    }
                }
            }
            
            rate.sleep();
        }
        
        // 发送停止指令
        sunray_msgs::UGVControlCMD stop_cmd;
        stop_cmd.cmd = sunray_msgs::UGVControlCMD::HOLD;
        cmd_pub.publish(stop_cmd);
        ROS_INFO("Trajectory finished. Sending HOLD command.");
        
        // 确保停止指令被发送
        ros::Duration(1.0).sleep();
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ugv_square_demo");
    SquareDemo demo;
    demo.run();
    return 0;
}