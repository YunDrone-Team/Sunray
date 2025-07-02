#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sunray_msgs/UGVControlCMD.h>
#include <cmath>

class CircularBodyControl
{
private:
    ros::NodeHandle nh;
    ros::Publisher cmd_pub;
    int ugv_id;
    double linear_speed;  // 车体坐标系X方向线速度 (m/s)
    double circle_radius; // 期望圆周半径 (m)
    double angular_vel;   // 车体坐标系Z轴角速度 (rad/s)
    ros::Time start_time;
    bool motion_completed;
    double target_duration; // 总运行时间（两圈）

public:
    CircularBodyControl() : linear_speed(0.5), circle_radius(1.0), motion_completed(false)
    {
        // 参数初始化
        nh.param<int>("ugv_id", ugv_id, 1);
        nh.param<double>("linear_speed", linear_speed, 0.5);
        nh.param<double>("circle_radius", circle_radius, 0.5);

        // 计算角速度 ω = v/r
        angular_vel = linear_speed / circle_radius;

        // 计算两圈所需时间：周长=2πr，两圈时间= (2*2πr)/v = 4πr/v
        target_duration = (4 * M_PI * circle_radius) / linear_speed;

        // 初始化控制指令发布器
        std::string cmd_topic = "/ugv" + std::to_string(ugv_id) + "/sunray_ugv/ugv_control_cmd";
        cmd_pub = nh.advertise<sunray_msgs::UGVControlCMD>(cmd_topic, 10);

        start_time = ros::Time::now();
    }

    void generate_commands()
    {
        if (motion_completed)
            return;

        sunray_msgs::UGVControlCMD ugv_cmd;
        ugv_cmd.cmd = sunray_msgs::UGVControlCMD::VEL_CONTROL_BODY;

        // 计算已运行时间
        double elapsed_time = (ros::Time::now() - start_time).toSec();

        if (elapsed_time < target_duration + 0.5)
        {
            // 持续发送车体系控制指令
            ugv_cmd.desired_vel[0] = linear_speed; // 车体X方向速度
            ugv_cmd.desired_vel[1] = 0.0;          // 车体Y方向速度
            ugv_cmd.angular_vel = angular_vel;
            cmd_pub.publish(ugv_cmd);              // 车体Z轴角速度
        }
        else
        {
            // 发送停止指令
            ugv_cmd.desired_vel[0] = 0.0;
            ugv_cmd.desired_vel[1] = 0.0;
            ugv_cmd.angular_vel = 0.0;
            cmd_pub.publish(ugv_cmd);
            motion_completed = true;
            ugv_cmd.cmd = sunray_msgs::UGVControlCMD::POS_CONTROL_ENU;
            ugv_cmd.desired_pos[0] = 0.0;
            ugv_cmd.desired_pos[1] = 0.0;
            cmd_pub.publish(ugv_cmd);
            ROS_INFO("Motion completed. Shutting down...");
            ros::shutdown();
        }

        ROS_INFO_THROTTLE(1.0, "Publishing body vel: [%.2f, %.2f] m/s | ang: %.2f rad/s",
                          ugv_cmd.desired_vel[0], ugv_cmd.desired_vel[1], ugv_cmd.angular_vel);
    }

    void run()
    {
        ros::Rate rate(20); // 20Hz控制频率
        while (ros::ok() && !motion_completed)
        {
            generate_commands();
            rate.sleep();
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ugv_circular_body_demo");
    CircularBodyControl controller;
    controller.run();
    return 0;
}