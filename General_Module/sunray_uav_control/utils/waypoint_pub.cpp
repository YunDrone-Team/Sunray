#include <ros/ros.h>
#include <printf_format.h>
#include <sunray_msgs/UAVWayPoint.h>
#include <sunray_msgs/UAVControlCMD.h>
#include <sunray_msgs/UAVState.h>

using namespace sunray_logger;

sunray_msgs::UAVControlCMD uav_cmd;
sunray_msgs::UAVState uav_state;
std::string node_name;

// 无人机状态回调
void uav_state_callback(const sunray_msgs::UAVState::ConstPtr &msg)
{
    uav_state = *msg;
}

int main(int argc, char **argv)
{
    // 设置日志
    Logger::init_default();
    Logger::setPrintLevel(false);
    Logger::setPrintTime(false);
    Logger::setPrintToFile(false);
    Logger::setFilename("~/Documents/Sunray_log.txt");

    ros::init(argc, argv, "uav_waypoint_publisher");
    ros::NodeHandle nh("~");
    int uav_id = 1;
    int wp_num = 5;
    int wp_end_type = 3;
    int wp_type = 1;
    int wp_yaw_type = 0;
    bool wp_takeoff = true;
    bool dowaypoint = false;
    float move_vel = 1.0;
    float vel_p = 1.0;
    float z_height = 10.0;
    std::string uav_name;
    node_name = ros::this_node::getName();

    nh.param<int>("uav_id", uav_id, 1);
    nh.param<int>("wp_num", wp_num, 5);
    nh.param<int>("wp_type", wp_type, 0);
    nh.param<int>("wp_end_type", wp_end_type, 0);
    nh.param<int>("wp_yaw_type", wp_yaw_type, 0);
    nh.param<bool>("wp_takeoff", wp_takeoff, true);
    nh.param<bool>("dowaypoint",  dowaypoint, false);
    nh.param<float>("wp_move_vel", move_vel, 1.0);
    nh.param<float>("wp_vel_p", vel_p, 1.0);
    nh.param<float>("wp_z_height", z_height, 1.0);
    nh.param<std::string>("uav_name", uav_name, "uav");

    double wp_point_1[3] = {0.0, 0.0, 0.0};
    double wp_point_2[3] = {0.0, 0.0, 0.0};
    double wp_point_3[3] = {0.0, 0.0, 0.0};
    double wp_point_4[3] = {0.0, 0.0, 0.0};
    double wp_point_5[3] = {0.0, 0.0, 0.0};
    double wp_point_6[3] = {0.0, 0.0, 0.0};
    double wp_point_7[3] = {0.0, 0.0, 0.0};
    double wp_point_8[3] = {0.0, 0.0, 0.0};
    double wp_point_9[3] = {0.0, 0.0, 0.0};
    double wp_point_10[3] = {0.0, 0.0, 0.0};
    double wp_circle_point[2] = {0.0, 0.0};

    nh.getParam("wp_point_1_x", wp_point_1[0]);
    nh.getParam("wp_point_1_y", wp_point_1[1]);
    nh.getParam("wp_point_1_z", wp_point_1[2]);
    nh.getParam("wp_point_1_yaw", wp_point_1[3]);
    nh.getParam("wp_point_2_x", wp_point_2[0]);
    nh.getParam("wp_point_2_y", wp_point_2[1]);
    nh.getParam("wp_point_2_z", wp_point_2[2]);
    nh.getParam("wp_point_2_yaw", wp_point_2[3]);
    nh.getParam("wp_point_3_x", wp_point_3[0]);
    nh.getParam("wp_point_3_y", wp_point_3[1]);
    nh.getParam("wp_point_3_z", wp_point_3[2]);
    nh.getParam("wp_point_3_yaw", wp_point_3[3]);
    nh.getParam("wp_point_4_x", wp_point_4[0]);
    nh.getParam("wp_point_4_y", wp_point_4[1]);
    nh.getParam("wp_point_4_z", wp_point_4[2]);
    nh.getParam("wp_point_4_yaw", wp_point_4[3]);
    nh.getParam("wp_point_5_x", wp_point_5[0]);
    nh.getParam("wp_point_5_y", wp_point_5[1]);
    nh.getParam("wp_point_5_z", wp_point_5[2]);
    nh.getParam("wp_point_5_yaw", wp_point_5[3]);
    nh.getParam("wp_point_6_x", wp_point_6[0]);
    nh.getParam("wp_point_6_y", wp_point_6[1]);
    nh.getParam("wp_point_6_z", wp_point_6[2]);
    nh.getParam("wp_point_6_yaw", wp_point_6[3]);
    nh.getParam("wp_point_7_x", wp_point_7[0]);
    nh.getParam("wp_point_7_y", wp_point_7[1]);
    nh.getParam("wp_point_7_z", wp_point_7[2]);
    nh.getParam("wp_point_7_yaw", wp_point_7[3]);
    nh.getParam("wp_point_8_x", wp_point_8[0]);
    nh.getParam("wp_point_8_y", wp_point_8[1]);
    nh.getParam("wp_point_8_z", wp_point_8[2]);
    nh.getParam("wp_point_8_yaw", wp_point_8[3]);
    nh.getParam("wp_point_9_x", wp_point_9[0]);
    nh.getParam("wp_point_9_y", wp_point_9[1]);
    nh.getParam("wp_point_9_z", wp_point_9[2]);
    nh.getParam("wp_point_9_yaw", wp_point_9[3]);
    nh.getParam("wp_point_10_x", wp_point_10[0]);
    nh.getParam("wp_point_10_y", wp_point_10[1]);
    nh.getParam("wp_point_10_z", wp_point_10[2]);
    nh.getParam("wp_point_10_yaw", wp_point_10[3]);
    nh.getParam("wp_circle_point_x", wp_circle_point[0]);
    nh.getParam("wp_circle_point_y", wp_circle_point[1]);

    // 打印参数
    std::cout << "uav_id: " << uav_id << std::endl;
    std::cout << "wp_num: " << wp_num << std::endl;
    std::cout << "wp_type: " << wp_type << std::endl;
    std::cout << "wp_end_type: " << wp_end_type << std::endl;
    std::cout << "wp_yaw_type: " << wp_yaw_type << std::endl;
    std::cout << "wp_takeoff: " << wp_takeoff << std::endl;
    std::cout << "move_vel: " << move_vel << std::endl;
    std::cout << "vel_p: " << vel_p << std::endl;
    std::cout << "z_height: " << z_height << std::endl;
    std::cout << "uav_name: " << uav_name << std::endl;

    std::cout << "wp_point_1: " << wp_point_1[0] << " " << wp_point_1[1] << " " << wp_point_1[2] << " " << wp_point_1[3] << std::endl;
    std::cout << "wp_point_2: " << wp_point_2[0] << " " << wp_point_2[1] << " " << wp_point_2[2] << " " << wp_point_2[3] << std::endl;
    std::cout << "wp_point_3: " << wp_point_3[0] << " " << wp_point_3[1] << " " << wp_point_3[2] << " " << wp_point_3[3] << std::endl;
    std::cout << "wp_point_4: " << wp_point_4[0] << " " << wp_point_4[1] << " " << wp_point_4[2] << " " << wp_point_4[3] << std::endl;
    std::cout << "wp_point_5: " << wp_point_5[0] << " " << wp_point_5[1] << " " << wp_point_5[2] << " " << wp_point_5[3] << std::endl;
    std::cout << "wp_point_6: " << wp_point_6[0] << " " << wp_point_6[1] << " " << wp_point_6[2] << " " << wp_point_6[3] << std::endl;
    std::cout << "wp_point_7: " << wp_point_7[0] << " " << wp_point_7[1] << " " << wp_point_7[2] << " " << wp_point_7[3] << std::endl;
    std::cout << "wp_point_8: " << wp_point_8[0] << " " << wp_point_8[1] << " " << wp_point_8[2] << " " << wp_point_8[3] << std::endl;
    std::cout << "wp_point_9: " << wp_point_9[0] << " " << wp_point_9[1] << " " << wp_point_9[2] << " " << wp_point_9[3] << std::endl;
    std::cout << "wp_point_10: " << wp_point_10[0] << " " << wp_point_10[1] << " " << wp_point_10[2] << " " << wp_point_10[3] << std::endl;
    std::cout << "wp_circle_point: " << wp_circle_point[0] << " " << wp_circle_point[1] << std::endl;

    std::string topic_prefix = "/" + uav_name + std::to_string(uav_id);
    ros::Subscriber uav_state_sub = nh.subscribe<sunray_msgs::UAVState>(topic_prefix + "/sunray/uav_state", 10, uav_state_callback);
    ros::Publisher waypoint_pub = nh.advertise<sunray_msgs::UAVWayPoint>(topic_prefix + "/sunray/uav_waypoint", 10);
    ros::Publisher control_cmd_pub = nh.advertise<sunray_msgs::UAVControlCMD>(topic_prefix + "/sunray/uav_control_cmd", 1);

    sunray_msgs::UAVWayPoint waypoint_msg;
    waypoint_msg.header.stamp = ros::Time::now();
    waypoint_msg.wp_num = wp_num;
    waypoint_msg.wp_type = wp_type;
    waypoint_msg.wp_end_type = wp_end_type;
    waypoint_msg.wp_yaw_type = wp_yaw_type;
    waypoint_msg.wp_takeoff = wp_takeoff;
    waypoint_msg.wp_move_vel = move_vel;
    waypoint_msg.wp_vel_p = vel_p;
    waypoint_msg.z_height = z_height;
    waypoint_msg.wp_point_1 = {wp_point_1[0], wp_point_1[1], wp_point_1[2], wp_point_1[3]};
    waypoint_msg.wp_point_2 = {wp_point_2[0], wp_point_2[1], wp_point_2[2], wp_point_2[3]};
    waypoint_msg.wp_point_3 = {wp_point_3[0], wp_point_3[1], wp_point_3[2], wp_point_3[3]};
    waypoint_msg.wp_point_4 = {wp_point_4[0], wp_point_4[1], wp_point_4[2], wp_point_4[3]};
    waypoint_msg.wp_point_5 = {wp_point_5[0], wp_point_5[1], wp_point_5[2], wp_point_5[3]};
    waypoint_msg.wp_point_6 = {wp_point_6[0], wp_point_6[1], wp_point_6[2], wp_point_6[3]};
    waypoint_msg.wp_point_7 = {wp_point_7[0], wp_point_7[1], wp_point_7[2], wp_point_7[3]};
    waypoint_msg.wp_point_8 = {wp_point_8[0], wp_point_8[1], wp_point_8[2], wp_point_8[3]};
    waypoint_msg.wp_point_9 = {wp_point_9[0], wp_point_9[1], wp_point_9[2], wp_point_9[3]};
    waypoint_msg.wp_point_10 = {wp_point_10[0], wp_point_10[1], wp_point_10[2], wp_point_10[3]};
    waypoint_msg.wp_circle_point = {wp_circle_point[0], wp_circle_point[1]};
    ros::Duration(0.5).sleep();
    

    // 初始化检查：等待PX4连接
    int times = 0;
    while (ros::ok() && !uav_state.connected)
    {
        ros::spinOnce();
        ros::Duration(1.0).sleep();
        if (times++ > 5)
            Logger::print_color(int(LogColor::red), node_name, ": Wait for UAV connect...");
    }
    Logger::print_color(int(LogColor::green), node_name, ": UAV connected!");
    ros::Duration(0.5).sleep();
    Logger::print_color(int(LogColor::green), node_name, ": Send waypoint to UAV");
    // 发布waypoint
    waypoint_pub.publish(waypoint_msg);
    ros::Duration(2).sleep();
    // 执行航点
    if(dowaypoint){
        Logger::print_color(int(LogColor::green), node_name, ": running dowaypoint");
        uav_cmd.cmd = sunray_msgs::UAVControlCMD::Waypoint;
        control_cmd_pub.publish(uav_cmd);
    }
    
    ros::Duration(0.5).sleep();
    return 0;
}