/*
    起飞降落例程：block_xyzpos.cpp
    程序功能：使用XyzPos接口进行四边形轨迹飞行
*/

#include "ros_msg_utils.h"

int uav_id;
string node_name;
string uav_name;
sunray_msgs::UAVControlCMD uav_cmd;
sunray_msgs::UAVState uav_state;

void mySigintHandler(int sig)
{
    std::cout << "[block_xyzpos] exit..." << std::endl;

    ros::shutdown();
    exit(EXIT_SUCCESS); // 或者使用 exit(0)
}

// 无人机状态回调
void uav_state_callback(const sunray_msgs::UAVState::ConstPtr &msg)
{
    uav_state = *msg;
}

int main(int argc, char **argv)
{
    // 设置日志
    Logger::init_default();

    ros::init(argc, argv, "block_xyzpos");
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

    // 定义正方形的顶点,并导入容器储存
    std::vector<std::tuple<double, double, double>> vertices = 
    {
        std::make_tuple(0.9, -0.9, 1.0),  // Start point
        std::make_tuple(0.9, 0.9, 1.0),   // Right point
        std::make_tuple(-0.9, 0.9, 1.0),  // Top-right point
        std::make_tuple(-0.9, -0.9, 1.0), // Top-left point
        std::make_tuple(0.9, -0.9, 1.0),  // Start point
        std::make_tuple(0, 0, 1.0)        // Back to start point
    };

    // 依次执行五个点
    for (const auto &vertex : vertices)
    {
        // 输出目标点坐标
        // 在每个顶点发送控制指令直到无人机到达该点
        Logger::print_color(int(LogColor::green), node_name, "Go to point", std::get<0>(vertex), std::get<1>(vertex), std::get<2>(vertex));

        while (ros::ok())
        {
            uav_cmd.header.stamp = ros::Time::now();
            uav_cmd.cmd = sunray_msgs::UAVControlCMD::XyzPos;
            uav_cmd.desired_pos[0] = std::get<0>(vertex);
            uav_cmd.desired_pos[1] = std::get<1>(vertex);
            uav_cmd.desired_pos[2] = std::get<2>(vertex);
            control_cmd_pub.publish(uav_cmd);

            if (fabs(uav_state.position[0] - std::get<0>(vertex)) < 0.15 &&
                fabs(uav_state.position[1] - std::get<1>(vertex)) < 0.15 &&
                fabs(uav_state.position[2] - std::get<2>(vertex)) < 0.15)
            {
                // 停下1秒等待无人机速度降下来
                ros::Duration(1.0).sleep();
                break;
            }
            ros::spinOnce();
            rate.sleep();
        }
    }

    Logger::print_color(int(LogColor::green), node_name, "Wait 5 sec and then send Land cmd...");
    ros::Duration(5.0).sleep();

    // 控制辅助类 - 自动降落
    uav_control_utils.auto_land();

    // Demo 结束
    Logger::print_color(int(LogColor::green), node_name, "Demo finished, quit!");

    return 0;
}
