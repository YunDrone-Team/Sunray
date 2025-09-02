#include "ros_msg_utils.h"

int main(int argc, char **argv)
{
    // 设置日志
    Logger::init_default();
    Logger::setPrintLevel(false);
    Logger::setPrintTime(false);
    Logger::setPrintToFile(false);
    Logger::setFilename("~/Documents/Sunray_log.txt");

    ros::init(argc, argv, "waypoint_pub");
    ros::NodeHandle nh("~");

    int uav_id;                         // 无人机ID
    string uav_name;                    // 无人机名字
    nh.param<int>("uav_id", uav_id, 1);                 // 【参数】无人机编号
    nh.param<std::string>("uav_name", uav_name, "uav"); // 【参数】无人机名字前缀
    string topic_prefix = "/" + uav_name + to_string(uav_id);               

    // 【发布】无人机航点数据
    ros::Publisher uav_waypoint_pub = nh.advertise<sunray_msgs::WayPoint>(topic_prefix + "/sunray/uav_waypoint", 1);

    int input;
    cout << YELLOW << "Please input to start:" << TAIL << endl;
    cin >> input;

    sunray_msgs::WayPoint uav_wp;
    uav_wp.start = true;
    uav_wp.wp_num = 2;
    // 1: 悬停 2: 降落 3: 返航
    uav_wp.wp_end_type = 3;
    // 1: 固定值 2: 朝向下一个航点
    uav_wp.wp_yaw_type = 2;
    uav_wp.wp_move_vel = 1.0;

    sunray_msgs::Point point;
    point.x = 1.0;
    point.y = 1.0;
    point.z = 1.0;
    point.yaw = 0.0;
    uav_wp.wp_points.push_back(point);

    point.x = 2.0;
    point.y = 2.0;
    point.z = 1.0;
    point.yaw = 0.0;
    uav_wp.wp_points.push_back(point);

    uav_waypoint_pub.publish(uav_wp);

    ros::Duration(0.5).sleep();
    return 0;
}