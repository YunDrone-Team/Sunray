// 订阅egp-planner消息 发布控制到sunray
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include "ros_msg_utils.h"

struct CmdValue
{
    double x;
    double y;
    double z;
    double vx;
    double vy;
    double vz;
    double ax;
    double ay;
    double az;
    double yaw;
};

class Ego2Sunray
{
public:
    Ego2Sunray(ros::NodeHandle& nh): nh_(nh)
    {
        nh_.param<std::string>("cmd_sub_topic", cmd_sub_topic_, "/position_cmd");
        nh_.param<std::string>("control_pub_topic", control_pub_topic, "/sunray/uav_control_cmd");

        std::cout << "cmd_sub_topic: " << cmd_sub_topic_ << std::endl;
        std::cout << "control_pub_topic: " << control_pub_topic << std::endl;
        // 订阅ego-planner消息
        ego_sub_ = nh_.subscribe(cmd_sub_topic_, 1, &Ego2Sunray::egoCallback, this);
        // 发布控制到sunray
        cmd_pub_ = nh_.advertise<sunray_msgs::UAVControlCMD>(control_pub_topic, 1);
    }

    void egoCallback(const sunray_msgs::PositionCommand::ConstPtr &msg)
    {
        // double x = msg->position.x;
        // double y = msg->position.y;
        // double z = msg->position.z;
        // double vx = msg->velocity.x;
        // double vy = msg->velocity.y;
        // double vz = msg->velocity.z;
        // double ax = msg->acceleration.x;
        // double ay = msg->acceleration.y;
        // double az = msg->acceleration.z;
        // double yaw = msg->yaw;
        cmd_value.x = msg->position.x;
        cmd_value.y = msg->position.y;
        cmd_value.z = msg->position.z;
        cmd_value.vx = msg->velocity.x;
        cmd_value.vy = msg->velocity.y;
        cmd_value.vz = msg->velocity.z;
        cmd_value.ax = msg->acceleration.x;
        cmd_value.ay = msg->acceleration.y;
        cmd_value.az = msg->acceleration.z;
        cmd_value.yaw = msg->yaw;

        if (cmd_value.x != cmd_value_last.x || cmd_value.y != cmd_value_last.y || cmd_value.z != cmd_value_last.z)
        {
            cmd_.cmd = 4;
            cmd_.desired_pos[0] = cmd_value.x;
            cmd_.desired_pos[1] = cmd_value.y;
            cmd_.desired_pos[2] = cmd_value.z;
            cmd_.desired_vel[0] = cmd_value.vx;
            cmd_.desired_vel[1] = cmd_value.vy;
            cmd_.desired_vel[2] = cmd_value.vz;
            cmd_.desired_acc[0] = cmd_value.ax;
            cmd_.desired_acc[1] = cmd_value.ay;
            cmd_.desired_acc[2] = cmd_value.az;
            cmd_.enable_yawRate = false;
            cmd_.desired_yaw = cmd_value.yaw;
            cmd_pub_.publish(cmd_);
        }
        cmd_value_last = cmd_value;
        
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber ego_sub_;
    ros::Publisher cmd_pub_;
    sunray_msgs::UAVControlCMD cmd_;

    int uav_id_;
    std::string uav_name_{""};
    std::string cmd_sub_topic_{""};
    std::string control_pub_topic{""};

    CmdValue cmd_value;
    CmdValue cmd_value_last;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ego2sunray");
    ros::NodeHandle nh("~");
    Ego2Sunray ego2sunray(nh);
    ros::Rate loop_rate(20);
    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    ros::spin();
    return 0;
}