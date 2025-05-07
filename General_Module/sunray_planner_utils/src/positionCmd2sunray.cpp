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

class positionCmd2sunray
{
public:
    positionCmd2sunray(ros::NodeHandle &nh) : nh(nh)
    {
        nh.param<std::string>("cmd_sub_topic", cmd_sub_topic, "/position_cmd");
        nh.param<std::string>("control_pub_topic", control_pub_topic, "/sunray/uav_control_cmd");
        nh.param<int>("vehicle_type", Vehicle_type, 0);
        nh.param<int>("control_type", control_type, 0);
        nh.param<bool>("enable_yaw", enable_yaw, true);

        std::cout << "cmd_sub_topic: " << cmd_sub_topic << std::endl;
        std::cout << "control_pub_topic: " << control_pub_topic << std::endl;
        // 订阅PositionCommand消息
        cmd_sub = nh.subscribe(cmd_sub_topic, 1, &positionCmd2sunray::cmdCallback, this);
        // 发布控制到sunray
        if (Vehicle_type == 0)
        {
            std::cout<<"------------------------uav------------"<<std::endl;
            cmd_pub = nh.advertise<sunray_msgs::UAVControlCMD>(control_pub_topic, 1);
        }
        else if (Vehicle_type == 1)
        {
            std::cout<<"------------------------ugv------------"<<std::endl;
            cmd_pub = nh.advertise<sunray_msgs::UGVControlCMD>(control_pub_topic, 1);
        }
    }

    void cmdCallback(const sunray_msgs::PositionCommand::ConstPtr &msg)
    {
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

        if (Vehicle_type == 0)
        {
            uav_cmd.header.stamp = ros::Time::now();
            if (!enable_yaw)
            {
                cmd_value.yaw = 0;
            }

            if (msg->velocity.x == 0 && msg->velocity.y == 0 && msg->velocity.z == 0)
            {
                uav_cmd.cmd = 4;
            }
            else
            {
                // XyzPosYaw
                if (control_type == 0)
                {
                    uav_cmd.cmd = 4;
                }
                // XyzVelYaw
                else if (control_type == 1)
                {
                    uav_cmd.cmd = 6;
                }
                // XyzPosVelYaw
                else if (control_type == 2)
                {
                    uav_cmd.cmd = 10;
                }
                else
                {
                    std::cout << "control_type error!" << std::endl;
                    return;
                }
                uav_cmd.desired_pos[0] = cmd_value.x;
                uav_cmd.desired_pos[1] = cmd_value.y;
                uav_cmd.desired_pos[2] = cmd_value.z;
                uav_cmd.desired_vel[0] = cmd_value.vx;
                uav_cmd.desired_vel[1] = cmd_value.vy;
                uav_cmd.desired_vel[2] = cmd_value.vz;
                uav_cmd.desired_acc[0] = cmd_value.ax;
                uav_cmd.desired_acc[1] = cmd_value.ay;
                uav_cmd.desired_acc[2] = cmd_value.az;
                uav_cmd.desired_yaw = cmd_value.yaw;
            }
            if (last_uav_cmd.cmd == 4 && uav_cmd.cmd == 4)
            {
                last_uav_cmd = uav_cmd;
                return;
            }
            else
            {
                cmd_pub.publish(uav_cmd);
            }
            // cmd_value_last = cmd_value;
            last_uav_cmd = uav_cmd;
        }

        else if (Vehicle_type == 1)
        {
            // XyPosYaw
            if (control_type == 0)
            {
                ugv_cmd.cmd = 7;
            }
            // XyVelYaw
            else if (control_type == 1)
            {
                ugv_cmd.cmd = 4;
            }
            else
            {
                std::cout << "control_type error!" << std::endl;
                return;
            }
            ugv_cmd.desired_pos[0] = cmd_value.x;
            ugv_cmd.desired_pos[1] = cmd_value.y;
            ugv_cmd.desired_vel[0] = cmd_value.vx;
            ugv_cmd.desired_vel[1] = cmd_value.vy;
            ugv_cmd.desired_yaw = cmd_value.yaw;
            cmd_pub.publish(ugv_cmd);
        }
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber cmd_sub;
    ros::Publisher cmd_pub;
    sunray_msgs::UAVControlCMD uav_cmd;
    sunray_msgs::UAVControlCMD last_uav_cmd;
    sunray_msgs::UGVControlCMD ugv_cmd;
    sunray_msgs::UGVControlCMD last_ugv_cmd;

    int uav_id_;
    int control_type;
    int Vehicle_type;
    bool enable_yaw;
    std::string uav_name;
    std::string cmd_sub_topic;
    std::string control_pub_topic;
    

    CmdValue cmd_value;
    CmdValue cmd_value_last;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cmd2sunray");
    ros::NodeHandle nh("~");
    positionCmd2sunray cmd2sunray(nh);
    ros::Rate loop_rate(20);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    ros::spin();
    return 0;
}