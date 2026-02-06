#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Bool.h>
#include <signal.h>

ros::Publisher center_pub;
ros::Publisher speed_pub;

void shutdownHandler(int sig)
{
    ROS_WARN("检测到退出信号，速度设置为0并发布回中指令...");
    
    // 停止云台移动
    geometry_msgs::Vector3 stop_msg;
    stop_msg.x = 0;
    stop_msg.y = 0;
    stop_msg.z = 0;
    speed_pub.publish(stop_msg);

    // 发布回中指令
    std_msgs::Bool center_msg;
    center_msg.data = true;
    center_pub.publish(center_msg);


    ros::Duration(1.0).sleep();  // 确保消息有时间发出
    ros::shutdown();
}


int main(int argc, char** argv)
{
    setlocale(LC_ALL,"");

    ros::init(argc, argv, "gimbal_target_lock_cpp");
    ros::NodeHandle nh;

    speed_pub = nh.advertise<geometry_msgs::Vector3>("/sunray/gimbal_set_speed", 10);
    center_pub = nh.advertise<std_msgs::Bool>("/sunray/gimbal_center_cmd", 1);

    // 捕捉 Ctrl+C 退出信号
    signal(SIGINT, shutdownHandler);

    ros::Rate loop_rate(10);  // 10 Hz

    int yaw_speed = 50;
    int pitch_speed = 0;
    int direction = 1;

    while (ros::ok())
    {
        geometry_msgs::Vector3 msg;
        msg.x = yaw_speed * direction;  // yaw
        msg.y = pitch_speed;            // pitch
        msg.z = 0;

        speed_pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();

        // 每1秒切换方向
        static int count = 0;
        count++;
        if (count > 10) 
        {
            direction *= -1;
            count = 0;
        }
    }

    return 0;
}
