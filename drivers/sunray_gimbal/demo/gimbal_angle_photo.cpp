#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Bool.h>
#include <vector>
#include <utility>  // for std::pair
//@描述：不同角度拍照

int main(int argc, char** argv)
{
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "gimbal_angle_photo_cpp");
    ros::NodeHandle nh;

    ros::Publisher angle_pub = nh.advertise<geometry_msgs::Vector3>("/sunray/gimbal_set_angles", 10);
    ros::Publisher photo_pub = nh.advertise<std_msgs::Bool>("/sunray/gimbal_take_photo", 1);
    ros::Publisher center_pub = nh.advertise<std_msgs::Bool>("/sunray/gimbal_center_cmd", 1);

    ros::Duration startup_delay(1.0); // 稍等启动
    startup_delay.sleep();

    // 自定义你想拍照的 yaw-pitch 点位
    std::vector<std::pair<double, double>> target_points = {
        {-30.0, -10.0},
        {-10.0,   0.0},
        { 30.0,  10.0},
        {-45.0, 5.0},
        {45.0, -5.0}
        //可以继续添加更多点
    };


    for (const auto& [yaw, pitch] : target_points)
    {
        geometry_msgs::Vector3 angle_msg;
        angle_msg.x = yaw;
        angle_msg.y = pitch;
        angle_msg.z = 0.0;

        ROS_INFO_STREAM("转动至 yaw=" << yaw << "°, pitch=" << pitch << "°");
        angle_pub.publish(angle_msg);

        ros::Duration(2.5).sleep();  // 等云台稳定

        std_msgs::Bool photo_msg;
        photo_msg.data = true;
        ROS_INFO("拍照中...");
        photo_pub.publish(photo_msg);

        ros::Duration(1.5).sleep();  // 等相机响应
    }

    //所有角度完成后回中
    std_msgs::Bool center_msg;
    center_msg.data = true;
    center_pub.publish(center_msg);
    ROS_INFO("所有角度拍照完成，云台已回中");
    return 0;
}
