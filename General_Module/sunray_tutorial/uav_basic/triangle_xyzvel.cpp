/*
    起飞降落例程：triangle_xyzvel.cpp
    程序功能：使用XyzVel接口进行三角形轨迹飞行，最后返航降落
*/

#include "ros_msg_utils.h"

int uav_id;
string node_name;
string uav_name;
sunray_msgs::UAVState uav_state;
sunray_msgs::UAVSetup uav_setup;
sunray_msgs::UAVControlCMD uav_cmd;

void mySigintHandler(int sig)
{
    std::cout << "[triangle_xyzvel] exit..." << std::endl;

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

    ros::init(argc, argv, "triangle_xyzvel");
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
    // 【发布】无人机设置指令（本节点 -> sunray_control_node）
    ros::Publisher uav_setup_pub = nh.advertise<sunray_msgs::UAVSetup>(uav_name + "/sunray/setup", 1);
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
    Logger::print_color(int(LogColor::green), node_name, "Wait 5 sec and then send XyzVel cmd...");

    ros::Duration(5.0).sleep();

    // 定义三角形中心位置及高度
    double center_x = 0;
    double center_y = 0;
    double triangle_radius = 1.0; // 三角形外接圆半径
    double height = 1.0;          
    int num_points = 20;// 定义每条边的点数
    
    // Define the proportional gain and maximum velocity
    double k_p = 1.0;
    double z_k_p = 0.5;
    double max_vel = 1.0;

    // 定义三角形顶点
    std::vector<std::pair<double, double>> vertices = {
        {center_x + triangle_radius, center_y},                    // 顶点1：正前方
        {center_x - triangle_radius * cos(M_PI/3), center_y + triangle_radius * sin(M_PI/3)},  // 顶点2：左后方
        {center_x - triangle_radius * cos(M_PI/3), center_y - triangle_radius * sin(M_PI/3)}   // 顶点3：右后方
    };

    geometry_msgs::PoseStamped pose;
    Logger::print_color(int(LogColor::green), node_name, "Start triangle.");
    // 遍历三角形的三条边
    for (int vertex = 0; vertex < 3; vertex++) 
    {
        // 当前边的起点和终点
        int start_vertex = vertex;
        int end_vertex = (vertex + 1) % 3;
        
        double start_x = vertices[start_vertex].first;
        double start_y = vertices[start_vertex].second;
        double end_x = vertices[end_vertex].first;
        double end_y = vertices[end_vertex].second;

        for (int i = 0; i < num_points; i++)
        {
            double pose_next = static_cast<double>(i) / num_points;
            pose.pose.position.x = start_x + pose_next * (end_x - start_x);
            pose.pose.position.y = start_y + pose_next * (end_y - start_y);
            pose.pose.position.z = height;

            // Send setpoints until the drone reaches the target point
            while (ros::ok())
            {
                // Calculate the distance to the target position
                double dx = pose.pose.position.x - uav_state.position[0];
                double dy = pose.pose.position.y - uav_state.position[1];
                double dz = pose.pose.position.z - uav_state.position[2];

                // Calculate the desired velocity using a proportional controller
                double vx = k_p * dx;
                double vy = k_p * dy;
                double vz = z_k_p * dz;
                // Limit the velocities to a maximum value
                vx = min(max(vx, -max_vel), max_vel);
                vy = min(max(vy, -max_vel), max_vel);
                vz = min(max(vz, -max_vel), max_vel);

                uav_cmd.header.stamp = ros::Time::now();
                uav_cmd.cmd = sunray_msgs::UAVControlCMD::XyzVel;
                uav_cmd.desired_vel[0] = vx;
                uav_cmd.desired_vel[1] = vy;
                uav_cmd.desired_vel[2] = vz;
                control_cmd_pub.publish(uav_cmd);

                // Check if the drone has reached the target point
                if (fabs(uav_state.position[0] - pose.pose.position.x) < 0.15 &&
                    fabs(uav_state.position[1] - pose.pose.position.y) < 0.15)
                {
                    break;
                }

                ros::spinOnce();
                ros::Duration(0.1).sleep();
            }
        }
    }

    // 发布悬停指令
    Logger::print_color(int(LogColor::green), node_name, "Send UAV Hover cmd.");
    uav_cmd.header.stamp = ros::Time::now();
    uav_cmd.cmd = sunray_msgs::UAVControlCMD::Hover;
    control_cmd_pub.publish(uav_cmd);

    Logger::print_color(int(LogColor::green), node_name, "Wait 5 sec and then send Land cmd...");
    ros::Duration(5.0).sleep();

    // 控制辅助类 - 自动返航降落
    uav_control_utils.auto_return();

    // Demo 结束
    Logger::print_color(int(LogColor::green), node_name, "Demo finished, quit!");
    
    return 0;
}
