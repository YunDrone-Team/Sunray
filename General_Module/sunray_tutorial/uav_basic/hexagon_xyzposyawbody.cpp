/*
    起飞降落例程：hexagon_xyzposyawbody.cpp
    程序功能：使用XyzPosYawBody接口进行六边形轨迹飞行
*/

#include "ros_msg_utils.h"

using namespace sunray_logger;
using namespace std;

int uav_id;
string node_name;
string uav_name;
sunray_msgs::UAVControlCMD uav_cmd;
sunray_msgs::UAVState uav_state;

void mySigintHandler(int sig)
{
    std::cout << "[hexagon_xyzposyawbody] exit..." << std::endl;

    ros::shutdown();
    exit(EXIT_SUCCESS); // 或者使用 exit(0)
}

void uav_state_callback(const sunray_msgs::UAVState::ConstPtr &msg)
{
    uav_state = *msg;
}

int main(int argc, char **argv)
{
    // 设置日志
    Logger::init_default();

    ros::init(argc, argv, "hexagon_xyzposyawbody");
    ros::NodeHandle nh("~");
    ros::Rate rate(20.0);

    signal(SIGINT, mySigintHandler);

    node_name = ros::this_node::getName();
    node_name = "["+node_name+"]:";

    // 【参数】无人机编号
    nh.param<int>("uav_id", uav_id, 1);
    // 【参数】无人机名称
    nh.param<string>("uav_name", uav_name, "uav");
    uav_name = "/" + uav_name + std::to_string(uav_id);

    // 【订阅】无人机状态
    ros::Subscriber uav_state_sub = nh.subscribe<sunray_msgs::UAVState>(uav_name + "/sunray/uav_state", 10, uav_state_callback);
    // 【发布】无人机控制指令 （本节点 -> sunray_control_node）
    ros::Publisher control_cmd_pub = nh.advertise<sunray_msgs::UAVControlCMD>(uav_name + "/sunray/uav_control_cmd", 1);

    ros::Duration(0.5).sleep();
    // 控制辅助类 - 初始化
    Control_Utils uav_control_utils;
    uav_control_utils.init(nh, uav_id, node_name);

    // 初始化检查：等待PX4连接
    int times = 0;
    while (ros::ok() && !uav_state.connected)
    {
        ros::spinOnce();
        ros::Duration(1.0).sleep();
        if (times++ > 5)
            Logger::print_color(int(LogColor::red), node_name, "Wait for UAV connect...");
    }

    // 控制辅助类 - 自动起飞
    uav_control_utils.auto_takeoff();

    // 以上: 无人机已成功起飞，进入自由任务模式
    Logger::print_color(int(LogColor::green), node_name, "Wait 5 sec and then send Move cmd...");

    ros::Duration(5.0).sleep();

    // 以上: 无人机已成功起飞，进入任务模式

    // 定义六边型的顶点,并导入容器储存
    std::tuple<double, double, double> vertex = std::make_tuple(1, 0, 0);
    float yaw;

    // 依次执行
    for (int i = 0; i < 8; ++i)
    {
        ros::spinOnce();
        uav_cmd.header.stamp = ros::Time::now();
        uav_cmd.cmd = sunray_msgs::UAVControlCMD::XyzPosYawBody;
        uav_cmd.desired_pos[0] = std::get<0>(vertex);
        uav_cmd.desired_pos[1] = std::get<1>(vertex);
        uav_cmd.desired_pos[2] = 1 - uav_state.position[2];
        uav_cmd.desired_yaw = 0;
        control_cmd_pub.publish(uav_cmd);
        ros::Duration(5).sleep();

        if (i == 0 || i == 6)
        {
            yaw = 120.0;
        }
        else
        {
            yaw = 60.0;
        }
        if (i == 7)
        {
            break;
        }

        uav_cmd.header.stamp = ros::Time::now();
        uav_cmd.cmd = sunray_msgs::UAVControlCMD::XyzPosYawBody;
        uav_cmd.desired_pos[0] = 0;
        uav_cmd.desired_pos[1] = 0;
        uav_cmd.desired_pos[2] = 0;
        uav_cmd.desired_yaw = yaw / 180.0 * M_PI;
        control_cmd_pub.publish(uav_cmd);
        ros::Duration(2).sleep();
    }

    Logger::print_color(int(LogColor::green), node_name, "Wait 5 sec and then send Land cmd...");
    ros::Duration(5.0).sleep();

    // 控制辅助类 - 自动降落
    uav_control_utils.auto_land();

    // Demo 结束
    Logger::print_color(int(LogColor::green), node_name, "Demo finished, quit!");

    return 0;
}
