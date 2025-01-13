#include <ros/ros.h>
#include "ros_msg_utils.h"
#include "printf_utils.h"
#include <csignal>
using namespace std;

/*
这些变量存储无人机的状态、目标位置、控制命令等信息：
uav_cmd：无人机的控制指令。
setup：无人机的设置指令。
stop_flag：控制任务停止的标志
*/

sunray_msgs::UAVState uav_state;
sunray_msgs::UAVControlCMD uav_cmd;
sunray_msgs::UAVSetup setup;

bool stop_flag{false};

void mySigintHandler(int sig)
{
    //ROS_INFO("[takeoff_hover_land] exit...");
    std::cout<<"[takeoff_hover_land] exit..."<<std::endl;

    ros::shutdown();
    exit(EXIT_SUCCESS); // 或者使用 exit(0)

}

void stop_tutorial_cb(const std_msgs::Empty::ConstPtr &msg)
{
    stop_flag = true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "takeoff_node");
    ros::NodeHandle nh("~");

    ros::Rate rate(20.0);
    // 获取 ROS 参数，设置默认值。如果没有在参数服务器上找到这些参数，会使用默认值。
    int uav_id;

    signal(SIGINT, mySigintHandler);

    string uav_name, target_tpoic_name;
    // 【参数】无人机编号
    nh.param<int>("uav_id", uav_id, 1);
    // 【参数】无人机名称
    nh.param<string>("uav_name", uav_name, "uav");

    uav_name = uav_name + std::to_string(uav_id);
    string topic_prefix = "/" + uav_name;
    // 【订阅】任务结束
    ros::Subscriber stop_tutorial_sub = nh.subscribe<std_msgs::Empty>(topic_prefix + "/sunray/stop_tutorial", 1, stop_tutorial_cb);

    // 【发布】无人机控制指令 （本节点 -> sunray_control_node）
    ros::Publisher control_cmd_pub = nh.advertise<sunray_msgs::UAVControlCMD>(topic_prefix + "/sunray/uav_control_cmd", 1);
    // 【发布】无人机设置指令（本节点 -> sunray_control_node）
    ros::Publisher uav_setup_pub = nh.advertise<sunray_msgs::UAVSetup>(topic_prefix + "/sunray/setup", 1);

    // 变量初始化
    uav_cmd.header.stamp = ros::Time::now();
    uav_cmd.cmd = 102;
    uav_cmd.desired_pos[0] = 0.0;
    uav_cmd.desired_pos[1] = 0.0;
    uav_cmd.desired_pos[2] = 0.0;
    uav_cmd.desired_vel[0] = 0.0;
    uav_cmd.desired_vel[1] = 0.0;
    uav_cmd.desired_vel[2] = 0.0;
    uav_cmd.desired_acc[0] = 0.0;
    uav_cmd.desired_acc[1] = 0.0;
    uav_cmd.desired_acc[2] = 0.0;
    uav_cmd.desired_att[0] = 0.0;
    uav_cmd.desired_att[1] = 0.0;
    uav_cmd.desired_att[2] = 0.0;
    uav_cmd.desired_yaw = 0.0;
    uav_cmd.desired_yaw_rate = 0.0;

    // 固定的浮点显示
    cout.setf(ios::fixed);
    // setprecision(n) 设显示小数精度为2位
    cout << setprecision(2);
    // 左对齐
    cout.setf(ios::left);
    // 强制显示小数点
    cout.setf(ios::showpoint);
    // 强制显示符号
    cout.setf(ios::showpos);

    // 打印相关信息
    cout << GREEN << ">>>>>>>>>>>>>>>> " << ros::this_node::getName() << " <<<<<<<<<<<<<<<<" << TAIL << endl;
    cout << GREEN << "uav_id                    : " << uav_id << " " << TAIL << endl;
    cout << GREEN << "uav_name                  : " << uav_name << " " << TAIL << endl;

    ros::Duration(0.5).sleep();
    // 解锁
    cout << "arm" << endl;
    setup.cmd = 1;
    uav_setup_pub.publish(setup);
    ros::Duration(1.5).sleep();

    // 切换到指令控制模式
    cout << "switch CMD_CONTROL" << endl;
    setup.cmd = 4;
    setup.control_state = "CMD_CONTROL";
    uav_setup_pub.publish(setup);
    ros::Duration(1.0).sleep();

    // 起飞
    cout << "takeoff" << endl;
    uav_cmd.cmd = 100;
    control_cmd_pub.publish(uav_cmd);
    ros::Duration(10).sleep();

    // 悬停
    cout << "hover" << endl;
    uav_cmd.cmd = 105;
    control_cmd_pub.publish(uav_cmd);
    ros::Duration(5).sleep();

    // 降落
    cout << "land" << endl;
    uav_cmd.cmd = 101;
    control_cmd_pub.publish(uav_cmd);
    // 关键
    ros::Duration(0.5).sleep();
}
