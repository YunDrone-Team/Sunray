// 订阅egp-planner消息 发布控制到sunray
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include "ros_msg_utils.h"


class Ego2Sunray
{
public:
    Ego2Sunray()
    {
        // 订阅ego-planner消息
        ego_sub_ = nh_.subscribe("/position_cmd", 1, &Ego2Sunray::egoCallback, this);
        // 发布控制到sunray
        cmd_pub_ = nh_.advertise<sunray_msgs::UAVControlCMD>("/uav1/sunray/uav_control_cmd",1);
    }

    void egoCallback(const sunray_msgs::PositionCommand::ConstPtr& msg)
    {
        double x = msg->position.x;
        double y = msg->position.y;
        double z = msg->position.z;
        double vx = msg->velocity.x;
        double vy = msg->velocity.y;
        double vz = msg->velocity.z;
        double ax = msg->acceleration.x;
        double ay = msg->acceleration.y;
        double az = msg->acceleration.z;
        double yaw = msg->yaw;
        cmd_.cmd = 4;
        cmd_.desired_pos[0] = x;
        cmd_.desired_pos[1] = y;
        cmd_.desired_pos[2] = z;
        cmd_.desired_vel[0] = vx;
        cmd_.desired_vel[1] = vy;
        cmd_.desired_vel[2] = vz;
        cmd_.desired_acc[0] = ax;
        cmd_.desired_acc[1] = ay;
        cmd_.desired_acc[2] = az;
        cmd_.enable_yawRate = false;
        cmd_.desired_yaw = yaw;
        cmd_pub_.publish(cmd_);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber ego_sub_;
    ros::Publisher cmd_pub_;
    sunray_msgs::UAVControlCMD cmd_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ego2sunray");
    Ego2Sunray ego2sunray;
    ros::spin();
    return 0;
}