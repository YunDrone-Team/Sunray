#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Bool.h>
#include <vector>
#include <utility>
#include <termios.h>
#include <unistd.h>

// 获取键盘输入（阻塞）
char getKey()
{
    char c;
    struct termios oldt, newt;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    c = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return c;
}

void doPhotoMode(
    ros::Publisher& angle_pub,
    ros::Publisher& photo_pub,
    ros::Publisher& center_pub)
{
    std::vector<std::pair<double, double>> photo_points = {
        {-30.0, -10.0},
        {  0.0,   0.0},
        { 30.0,  10.0}
    };

    for (const auto& [yaw, pitch] : photo_points)
    {
        geometry_msgs::Vector3 angle_msg;
        angle_msg.x = yaw;
        angle_msg.y = pitch;
        angle_msg.z = 0.0;
        ROS_INFO_STREAM("转动至 yaw=" << yaw << "°, pitch=" << pitch << "°");
        angle_pub.publish(angle_msg);
        ros::Duration(2.5).sleep();

        std_msgs::Bool photo_msg;
        photo_msg.data = true;
        ROS_INFO("拍照中...");
        photo_pub.publish(photo_msg);
        ros::Duration(1.5).sleep();
    }

    // 回中
    std_msgs::Bool center;
    center.data = true;
    center_pub.publish(center);
    ROS_WARN("拍照任务完成，云台已回中");
}

void doVideoManualMode(
    ros::Publisher& angle_pub,
    ros::Publisher& video_pub,
    ros::Publisher& center_pub)
{
    double yaw = 0.0;
    double pitch = 0.0;
    const double step = 2.0;

    geometry_msgs::Vector3 angle_msg;
    std_msgs::Bool video_msg;

    bool recording = false;
    ROS_INFO("键盘wasd控制云台方向，空格暂停，r 开始录像，e 结束录像，q 退出");

    while (ros::ok())
    {
        char c = getKey();

        if (c == 'w') pitch += step;
        else if (c == 's') pitch -= step;
        else if (c == 'a') yaw -= step;
        else if (c == 'd') yaw += step;
        else if (c == 'r' && !recording) {
            ROS_INFO("开始录像");
            video_msg.data = true;
            video_pub.publish(video_msg);
            recording = true;
        }
        else if (c == 'e' && recording) {
            ROS_INFO("停止录像");
            video_msg.data = false;
            video_pub.publish(video_msg);
            recording = false;
        }
        else if (c == 'q') {
            if (recording) {
                video_msg.data = false;
                video_pub.publish(video_msg);
                ROS_WARN("停止录像（自动）");
            }
            break;
        }

        // 发送角度
        angle_msg.x = yaw;
        angle_msg.y = pitch;
        angle_msg.z = 0.0;
        angle_pub.publish(angle_msg);
        ros::Duration(0.1).sleep();
    }

    // 回中
    std_msgs::Bool center;
    center.data = true;
    center_pub.publish(center);
    ROS_WARN("录像任务完成，云台已回中");
}

int main(int argc, char** argv)
{
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "gimbal_capture_switch");
    ros::NodeHandle nh;

    ros::Publisher angle_pub  = nh.advertise<geometry_msgs::Vector3>("/sunray/gimbal_set_angles", 10);
    ros::Publisher photo_pub  = nh.advertise<std_msgs::Bool>("/sunray/gimbal_take_photo", 1);
    ros::Publisher video_pub  = nh.advertise<std_msgs::Bool>("/sunray/gimbal_toggle_record", 1);
    ros::Publisher center_pub = nh.advertise<std_msgs::Bool>("/sunray/gimbal_center_cmd", 1);

    ros::Duration(1.0).sleep();

    ROS_INFO("选择模式：p = 拍照模式，v = 手动录像模式");
    char mode = getKey();

    if (mode == 'p') {
        doPhotoMode(angle_pub, photo_pub, center_pub);
    }
    else if (mode == 'v') {
        doVideoManualMode(angle_pub, video_pub, center_pub);
    }
    else {
        ROS_WARN("无效输入，退出");
    }

    return 0;
}
