#include <ros/ros.h>
#include "ros_msg_utils.h"
#include "printf_utils.h"

using namespace std;

/*
这些变量存储无人机的状态、目标位置、控制命令等信息：
current_pose：当前无人机的位置。
uav_cmd：无人机的控制指令。
uav_control_state：无人机的控制状态。
setup：无人机的设置指令。
stop_flag：控制任务停止的标志
*/
geometry_msgs::PoseStamped current_pose;
sunray_msgs::UAVControlCMD uav_cmd;
sunray_msgs::UAVSetup setup;
float target_yaw;
bool stop_flag{false};

/*回调函数*/
// stop_tutorial_cb: 处理停止任务的空消息，如果收到消息，设置 stop_flag 为 true。
void stop_tutorial_cb(const std_msgs::Empty::ConstPtr &msg)
{
    stop_flag = true;
}
// pose_cb: 处理当前无人机位置的回调函数，更新 current_pose。
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    current_pose = *msg;
}

/*
参数读取：通过 ros::NodeHandle::param 读取参数，如无人机编号 uav_id、无人机名称 uav_name、是否为仿真模式 sim_mode 等。
话题订阅：
根据仿真模式决定订阅仿真位置 /sunray/gazebo_pose 或实际位置 /vrpn_client_node/pose。
订阅无人机状态、控制状态、当前位置信息、任务停止信号。
话题发布：
发布控制命令 (/sunray/uav_control_cmd)。
发布无人机设置指令 (/sunray/setup)。
*/
int main(int argc, char **argv)
{
    ros::init(argc, argv, "takeoff_node");
    ros::NodeHandle nh("~");

    ros::Rate rate(20.0);

    int uav_id;
    string uav_name;
    // 【参数】无人机编号
    nh.param<int>("uav_id", uav_id, 1);
    // 【参数】无人机名称
    nh.param<string>("uav_name", uav_name, "uav");
    uav_name = uav_name + std::to_string(uav_id);
    string topic_prefix = "/" + uav_name;
    // 【订阅】任务结束
    ros::Subscriber stop_tutorial_sub = nh.subscribe<std_msgs::Empty>(topic_prefix + "/sunray/stop_tutorial", 1, stop_tutorial_cb);
    // 【订阅】无人机位置
    ros::Subscriber pose_sub = nh.subscribe(topic_prefix + "/mavros/local_position/pose", 10, pose_cb);

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
    ros::Duration(1.0).sleep();

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
    ros::Duration(2).sleep();

    // 定义正方形的顶点,并导入容器储存
    std::vector<std::tuple<double, double, double>> vertices = {
        std::make_tuple(0.9, -0.9, 0.8),  // Start point
        std::make_tuple(0.9, 0.9, 0.8),   // Right point
        std::make_tuple(-0.9, 0.9, 0.8),  // Top-right point
        std::make_tuple(-0.9, -0.9, 0.8), // Top-left point
        std::make_tuple(0.9, -0.9, 0.8),  // Start point
        std::make_tuple(0, 0, 0.8)        // Back to start point
    };

    for (const auto &vertex : vertices)
    {
        // 输出目标点坐标
        // 在每个顶点发送控制指令直到无人机到达该点
        cout << "go to point: (" << std::get<0>(vertex) << " " << std::get<1>(vertex) << " " << std::get<2>(vertex) << ")" << endl;
        while (ros::ok())
        {
            // 如果停止将直接降落
            if (stop_flag)
            {
                cout << "land" << endl;
                uav_cmd.cmd = 101;
                control_cmd_pub.publish(uav_cmd);
                ros::Duration(0.5).sleep();
                break;
            }
            uav_cmd.header.stamp = ros::Time::now();
            uav_cmd.cmd = 1;
            uav_cmd.desired_pos[0] = std::get<0>(vertex);
            uav_cmd.desired_pos[1] = std::get<1>(vertex);
            uav_cmd.desired_pos[2] = std::get<2>(vertex);
            control_cmd_pub.publish(uav_cmd);

            if (fabs(current_pose.pose.position.x - std::get<0>(vertex)) < 0.15 &&
                fabs(current_pose.pose.position.y - std::get<1>(vertex)) < 0.15 &&
                fabs(current_pose.pose.position.z - std::get<2>(vertex)) < 0.15)
            {
                // 停下1秒等待无人机速度降下来
                ros::Duration(1.0).sleep();
                break;
            }
            ros::spinOnce();
            rate.sleep();
        }
    }

    // 降落
    cout << "land" << endl;
    uav_cmd.cmd = 101;
    control_cmd_pub.publish(uav_cmd);
    ros::Duration(0.5).sleep();
}
