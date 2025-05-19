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
    double angular_vel;
    ros::Time start_time;
    bool motion_completed;
    sunray_msgs::UGVState current_state;
    double stop_tolerance;

public:
    CircularStopDemo() : linear_speed(0.5), circle_radius(2.0), motion_completed(false), stop_tolerance(0.2) {
        nh.param<int>("ugv_id", ugv_id, 1);
        nh.param<double>("linear_speed", linear_speed, 0.5);
        nh.param<double>("circle_radius", circle_radius, 2.0);
        
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
            // 圆周运动
            ugv_cmd.cmd = sunray_msgs::UGVControlCMD::VEL_CONTROL_ENU;
            ugv_cmd.desired_vel[0] = -linear_speed * sin(angular_vel * t);
            ugv_cmd.desired_vel[1] = linear_speed * cos(angular_vel * t);
            ugv_cmd.desired_yaw = 0.0;
        } else {
            // 切换到位置控制模式并发送原点作为目标点
            ugv_cmd.cmd = sunray_msgs::UGVControlCMD::POS_CONTROL_ENU;
            ugv_cmd.desired_pos[0] = 0.0;
            ugv_cmd.desired_pos[1] = 0.0;
            ugv_cmd.desired_yaw = 0.0;
            
            // 检查是否到达原点
            double dx = current_state.position[0];
            double dy = current_state.position[1];
            if(sqrt(dx*dx + dy*dy) < stop_tolerance) {
                ugv_cmd.desired_vel[0] = 0.0;
                ugv_cmd.desired_vel[1] = 0.0;
                motion_completed = true;
                ROS_INFO("Reached origin. Stopping...");
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