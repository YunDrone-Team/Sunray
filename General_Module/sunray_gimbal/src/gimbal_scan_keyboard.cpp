#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Bool.h>
#include <signal.h>
#include <termios.h>
#include <unistd.h>
#include <thread>
//@描述：云台自动左右上下扫描或者键盘控制云台

// 控制状态
bool manual_mode = false;
double current_yaw = 0.0;
double current_pitch = 0.0;

ros::Publisher speed_pub;
ros::Publisher center_pub;
ros::Subscriber attitude_sub;

// 终端设置
struct termios oldt, newt;

// 非阻塞获取键盘按键
char getKey()
{
    tcgetattr(STDIN_FILENO, &oldt); // 获取终端属性
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO); // 禁用缓冲和回显
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    char c = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt); // 恢复
    return c;
}

// 回调获取实时姿态
void attitudeCallback(const geometry_msgs::Vector3::ConstPtr& msg)
{
    current_yaw = msg->x;
    current_pitch = msg->y;
}

// 退出
void shutdownHandler(int sig)
{
    ROS_WARN("节点退出，发布云台回中...");
    geometry_msgs::Vector3 stop;
    stop.x = 0;
    stop.y = 0;
    stop.z = 0;
    speed_pub.publish(stop);

    std_msgs::Bool center;
    center.data = true;
    center_pub.publish(center);

    ros::Duration(1.0).sleep();
    ros::shutdown();
}

// 键盘监听线程
void keyboardListener()
{
    while (ros::ok())
    {
        char c = getKey();
        geometry_msgs::Vector3 msg;
        msg.x = 0;
        msg.y = 0;

        switch (c)
        {
            case 'w': msg.y = 30; break;   // 上
            case 's': msg.y = -30; break;  // 下
            case 'a': msg.x = 30; break;   // 左
            case 'd': msg.x = -30; break;  // 右
            case ' ': msg.x = 0; msg.y = 0; break;  // 停止
            case 'm':
                manual_mode = !manual_mode;
                ROS_WARN_STREAM("模式切换为: " << (manual_mode ? "手动键盘控制 " : "自动扫描巡逻 "));
                break;
            case 'q':
                shutdownHandler(0);  // 调用退出函数
                return;
            default:
                continue;
        }

        if (manual_mode)
        {
            speed_pub.publish(msg);
            ROS_INFO_STREAM("键盘控制: yaw = " << msg.x << ", pitch = " << msg.y);
        }
    }
}

// 扫描模式
void scanLoop()
{
    static int yaw_dir = 1;
    static int pitch_dir = 1;
    static int pitch_count = 0;

    const int yaw_speed = 30;
    const int pitch_speed = 30;

    const double yaw_min = -45.0;
    const double yaw_max = 45.0;

    // 实时角度判断
    if (current_yaw >= yaw_max) yaw_dir = -1;
    if (current_yaw <= yaw_min) yaw_dir = 1;

    if (pitch_count > 40) {
        pitch_dir *=-1;
        pitch_count = 0;
    }

    geometry_msgs::Vector3 msg;
    msg.x = yaw_speed * yaw_dir;
    msg.y = pitch_speed * pitch_dir;
    msg.z = 0;
    speed_pub.publish(msg);

    pitch_count++;
}

int main(int argc, char** argv)
{
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "gimbal_scan_keyboard_cpp");
    ros::NodeHandle nh;

    signal(SIGINT, shutdownHandler);

    speed_pub = nh.advertise<geometry_msgs::Vector3>("/sunray/gimbal_set_speed", 10);
    center_pub = nh.advertise<std_msgs::Bool>("/sunray/gimbal_center_cmd", 1);
    attitude_sub = nh.subscribe("/sunray/gimbal_attitude",10,attitudeCallback);
    std::thread keyboard_thread(keyboardListener);


    ros::Rate rate(10);  // 10 Hz

    ROS_INFO("云台扫描 + 键盘控制启动：按 m 切换模式，q 退出");

    while (ros::ok())
    {
        if (!manual_mode)
        {
            scanLoop(); 
        }
        
        ros::spinOnce();
        rate.sleep();
    }

    keyboard_thread.join();
    return 0;
}
