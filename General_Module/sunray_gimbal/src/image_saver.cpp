#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/Vector3.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <chrono>
#include <filesystem>
#include <thread>
#include <mutex>

cv::Mat latest_image;
std::mutex image_mutex;
bool save_requested = false;


geometry_msgs::Vector3 speed_cmd;
ros::Publisher gimbal_pub;


bool kbhit()
{
    struct termios oldt, newt;
    int ch;
    int oldf;
  
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO); // 非阻塞输入模式
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
  
    ch = getchar();
  
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);
  
    if (ch != EOF) {
        ungetc(ch, stdin);
        return true;
    }
  
    return false;
}

char getch()
{
    struct termios oldt, newt;
    char ch;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO); 
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return ch;
}

// 图像回调：更新当前图像帧
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try {
        cv::Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;
        std::lock_guard<std::mutex> lock(image_mutex);
        latest_image = img.clone();
        save_requested = true;
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge异常: %s", e.what());
    }
}

void publishGimbalSpeed(float yaw, float pitch)
{
    speed_cmd.x = yaw;
    speed_cmd.y = pitch;
    speed_cmd.z = 0;
    gimbal_pub.publish(speed_cmd);
}

int main(int argc, char** argv)
{
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "image_saver_node");
    ros::NodeHandle nh;

    //订阅图像
    ros::Subscriber image_sub = nh.subscribe("/sunray/camera/monocular_down/image_raw", 1, imageCallback);
    //发布速度控制话题
    gimbal_pub = nh.advertise<geometry_msgs::Vector3>("/sunray/gimbal_set_speed", 10);

    std::string save_dir = "/home/robot/saved_images/";  // 替换为你自己的路径
    system(("mkdir -p " + save_dir).c_str()); // 自动创建文件夹

    cv::namedWindow("Live Image", cv::WINDOW_NORMAL);
    ros::Rate rate(30); // 图像显示频率

    ROS_INFO("键盘控制：WASD 控制云台方向，空格停止云台转动，F 保存图片");

    while (ros::ok()) {
        cv::Mat img_copy;

        {
            std::lock_guard<std::mutex> lock(image_mutex);
            if (save_requested && !latest_image.empty()) {
                img_copy = latest_image.clone();
                save_requested = false;
            }
        }

        if (!img_copy.empty()) {
            cv::imshow("Live Image", img_copy);
        }

        int key = cv::waitKey(1);
        if (kbhit()) {
            char c = std::tolower(getch());
            switch (c) {
                case 'w':
                    publishGimbalSpeed(0, -10);
                    ROS_INFO("向上");
                    break;
                case 's':
                    publishGimbalSpeed(0, 10);
                    ROS_INFO("向下");
                    break;
                case 'a':
                    publishGimbalSpeed(-10, 0);
                    ROS_INFO("向左");
                    break;
                case 'd':
                    publishGimbalSpeed(10, 0);
                    ROS_INFO("向右");
                    break;
                case ' ':
                    publishGimbalSpeed(0, 0);
                    ROS_INFO("停下");
                    break;
                case 'f':
                    if (!img_copy.empty()) {
                        std::string filename = save_dir + "frame_" + std::to_string(ros::Time::now().toNSec()) + ".jpg";
                        if (cv::imwrite(filename, img_copy)) {
                            ROS_INFO_STREAM("保存图像成功: " << filename);
                        } else {
                            ROS_WARN("保存图像失败");
                        }
                    } else {
                        ROS_WARN("当前图像为空，无法保存");
                    }
                    break;
                default:
                    break;
            }
        }

        ros::spinOnce();
        rate.sleep();
    }

    cv::destroyWindow("Live Image");
    return 0;
}
