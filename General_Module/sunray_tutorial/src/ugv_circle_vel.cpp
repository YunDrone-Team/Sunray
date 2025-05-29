#include <ros/ros.h>
#include <sunray_msgs/UGVControlCMD.h>
#include <sunray_msgs/UGVState.h>
#include <cmath>

class CircularStopDemo {
private:
    ros::NodeHandle nh;
    ros::Publisher cmd_pub;
    ros::Subscriber state_sub;
    int ugv_id;
    double linear_speed;
    double circle_radius;
    double circle_center_x;  // 新增圆心x坐标
    double circle_center_y;  // 新增圆心y坐标
    double angular_vel;
    ros::Time start_time;
    bool motion_completed;
    sunray_msgs::UGVState current_state;
    double stop_tolerance;

public:
    CircularStopDemo() : linear_speed(0.5), circle_radius(2.0), circle_center_x(0.0), circle_center_y(0.0), motion_completed(false), stop_tolerance(0.2) {
        nh.param<int>("ugv_id", ugv_id, 1);
        nh.param<double>("linear_speed", linear_speed, 0.5);
        nh.param<double>("circle_radius", circle_radius, 2.0);
        // 新增圆心坐标参数获取
        nh.param<double>("circle_center_x", circle_center_x, 0.0);
        nh.param<double>("circle_center_y", circle_center_y, 0.0);
        
        angular_vel = linear_speed / circle_radius;
        
        std::string topic_prefix = "/ugv" + std::to_string(ugv_id) + "/sunray_ugv/ugv_control_cmd";
        cmd_pub = nh.advertise<sunray_msgs::UGVControlCMD>(topic_prefix, 10);
        state_sub = nh.subscribe("/ugv" + std::to_string(ugv_id) + "/sunray_ugv/ugv_state", 1, &CircularStopDemo::stateCallback, this);
        
        start_time = ros::Time::now();
    }

    void stateCallback(const sunray_msgs::UGVState::ConstPtr& msg) {
        current_state = *msg;
    }

    void generate_commands() {
        if(motion_completed) return;

        sunray_msgs::UGVControlCMD ugv_cmd;
        double t = (ros::Time::now() - start_time).toSec();
        double total_time = 4 * M_PI / angular_vel;

        if(t < total_time) {
            // 圆周运动 - 修改为基于圆心计算速度
            ugv_cmd.cmd = sunray_msgs::UGVControlCMD::VEL_CONTROL_ENU;
            
            // 计算相对于圆心的位置
            double dx = current_state.position[0] - circle_center_x;
            double dy = current_state.position[1] - circle_center_y;
            double dist = std::sqrt(dx*dx + dy*dy);
            
            // 避免除零错误
            if(dist < 1e-3) {
                // 如果正好在圆心，给定初始速度
                ugv_cmd.desired_vel[0] = linear_speed;
                ugv_cmd.desired_vel[1] = 0.0;
            } else {
                // 计算切向速度（垂直于半径方向）
                ugv_cmd.desired_vel[0] = -linear_speed * (dy / dist);
                ugv_cmd.desired_vel[1] = linear_speed * (dx / dist);
            }
            ugv_cmd.desired_yaw = 0.0;
        } else {
            // 切换到位置控制模式并发送圆心作为目标点
            ugv_cmd.cmd = sunray_msgs::UGVControlCMD::POS_CONTROL_ENU;
            ugv_cmd.desired_pos[0] = circle_center_x;  // 目标位置设为圆心x
            ugv_cmd.desired_pos[1] = circle_center_y;  // 目标位置设为圆心y
            ugv_cmd.desired_yaw = 0.0;
            
            // 检查是否到达圆心
            double dx = current_state.position[0] - circle_center_x;
            double dy = current_state.position[1] - circle_center_y;
            if(std::sqrt(dx*dx + dy*dy) < stop_tolerance) {
                ugv_cmd.desired_vel[0] = 0.0;
                ugv_cmd.desired_vel[1] = 0.0;
                motion_completed = true;
                ROS_INFO("Reached circle center. Stopping...");
            }
        }

        cmd_pub.publish(ugv_cmd);
    }

    void run() {
        ros::Rate rate(20);
        while(ros::ok() && !motion_completed) {
            generate_commands();
            ros::spinOnce();
            rate.sleep();
        }
        
        // 持续发送停止指令3秒确保稳定
        ros::Time stop_start = ros::Time::now();
        while((ros::Time::now() - stop_start).toSec() < 3.0) {
            sunray_msgs::UGVControlCMD ugv_cmd;
            ugv_cmd.cmd = sunray_msgs::UGVControlCMD::HOLD;
            cmd_pub.publish(ugv_cmd);
            rate.sleep();
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "ugv_circular_stop_demo");
    CircularStopDemo demo;
    demo.run();
    return 0;
}