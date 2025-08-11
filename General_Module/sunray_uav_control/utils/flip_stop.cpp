#include "ros_msg_utils.h"
#include "printf_format.h"
#include <numeric>

ros::Publisher setup_pub;
ros::Subscriber imu_sub;

void imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
    // 获取欧拉角
    tf::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    // 判断是否翻转
    if (pitch > M_PI/3 || pitch < -M_PI/3 || roll > M_PI/3 || roll  < -M_PI/3 )
    {
        // 发布sunray setup topic
        sunray_msgs::UAVSetup setup_msg;
        setup_msg.cmd = sunray_msgs::UAVSetup::EMERGENCY_KILL;
        setup_msg.header.stamp = ros::Time::now();
        setup_pub.publish(setup_msg);
        std::cout<< "Flip detected! Emergency kill command sent." << std::endl;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "FlipStopNode");
    ros::NodeHandle nh("~");
    std::cout << "FlipStopNode started." << std::endl;
    int uav_id;
    std::string uav_name, topic_prefix;
    nh.param<int>("uav_id", uav_id, 1);                 // 【参数】无人机编号
    nh.param<std::string>("uav_name", uav_name, "uav"); // 【参数】无人机名称
    topic_prefix = "/" + uav_name + std::to_string(uav_id);
    // 订阅mavros imu topic
    imu_sub = nh.subscribe(topic_prefix + "/mavros/imu/data", 10, imuCallback);
    // 发布sunray setup topic
    setup_pub = nh.advertise<sunray_msgs::UAVSetup>(topic_prefix + "/sunray/setup", 10);
    // 发布sunray control command topic
    ros::spin();
    return 0;
}
