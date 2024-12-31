#include <ros/ros.h>
#include <sunray_msgs/UAVWayPoint.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "uav_waypoint_publisher");
    ros::NodeHandle nh("~");
    int uav_id = 1;
    int wp_num = 5;
    int wp_type = 0;
    bool wp_land = true;
    bool wp_return = false;
    bool wp_takeoff = true;
    bool wp_fix_yaw = false;
    float move_vel = 1.0;
    float z_height = 10.0;
    float yaw_angle = 0.0;
    std::string uav_name;

    nh.param<int>("uav_id", uav_id, 1);
    nh.param<int>("wp_num", wp_num, 5);
    nh.param<int>("wp_type", wp_type, 0);
    nh.param<bool>("wp_land", wp_land, true);
    nh.param<bool>("wp_return", wp_return, false);
    nh.param<bool>("wp_takeoff", wp_takeoff, true);
    nh.param<bool>("wp_fix_yaw", wp_fix_yaw, false);
    nh.param<float>("move_vel", move_vel, 1.0);
    nh.param<float>("z_height", z_height, 1.0);
    nh.param<float>("yaw_angle", yaw_angle, 0.0);
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

    nh.getParam("wp_point_1_x", wp_point_1[0]);
    nh.getParam("wp_point_1_y", wp_point_1[1]);
    nh.getParam("wp_point_1_z", wp_point_1[2]);
    nh.getParam("wp_point_2_x", wp_point_2[0]);
    nh.getParam("wp_point_2_y", wp_point_2[1]);
    nh.getParam("wp_point_2_z", wp_point_2[2]);
    nh.getParam("wp_point_3_x", wp_point_3[0]);
    nh.getParam("wp_point_3_y", wp_point_3[1]);
    nh.getParam("wp_point_3_z", wp_point_3[2]);
    nh.getParam("wp_point_4_x", wp_point_4[0]);
    nh.getParam("wp_point_4_y", wp_point_4[1]);
    nh.getParam("wp_point_4_z", wp_point_4[2]);
    nh.getParam("wp_point_5_x", wp_point_5[0]);
    nh.getParam("wp_point_5_y", wp_point_5[1]);
    nh.getParam("wp_point_5_z", wp_point_5[2]);
    nh.getParam("wp_point_6_x", wp_point_6[0]);
    nh.getParam("wp_point_6_y", wp_point_6[1]);
    nh.getParam("wp_point_6_z", wp_point_6[2]);
    nh.getParam("wp_point_7_x", wp_point_7[0]);
    nh.getParam("wp_point_7_y", wp_point_7[1]);
    nh.getParam("wp_point_7_z", wp_point_7[2]);
    nh.getParam("wp_point_8_x", wp_point_8[0]);
    nh.getParam("wp_point_8_y", wp_point_8[1]);
    nh.getParam("wp_point_8_z", wp_point_8[2]);
    nh.getParam("wp_point_9_x", wp_point_9[0]);
    nh.getParam("wp_point_9_y", wp_point_9[1]);
    nh.getParam("wp_point_9_z", wp_point_9[2]);
    nh.getParam("wp_point_10_x", wp_point_10[0]);
    nh.getParam("wp_point_10_y", wp_point_10[1]);
    nh.getParam("wp_point_10_z", wp_point_10[2]);

    // 打印参数
    std::cout << "uav_id: " << uav_id << std::endl;
    std::cout << "wp_num: " << wp_num << std::endl;
    std::cout << "wp_type: " << wp_type << std::endl;
    std::cout << "wp_land: " << wp_land << std::endl;
    std::cout << "wp_return: " << wp_return << std::endl;
    std::cout << "wp_takeoff: " << wp_takeoff << std::endl;
    std::cout << "wp_fix_yaw: " << wp_fix_yaw << std::endl;
    std::cout << "move_vel: " << move_vel << std::endl;
    std::cout << "z_height: " << z_height << std::endl;
    std::cout << "yaw_angle: " << yaw_angle << std::endl;
    std::cout << "uav_name: " << uav_name << std::endl;

    std::cout << "wp_point_1: " << wp_point_1[0] << " " << wp_point_1[1] << " " << wp_point_1[2] << std::endl;
    std::cout << "wp_point_2: " << wp_point_2[0] << " " << wp_point_2[1] << " " << wp_point_2[2] << std::endl;
    std::cout << "wp_point_3: " << wp_point_3[0] << " " << wp_point_3[1] << " " << wp_point_3[2] << std::endl;
    std::cout << "wp_point_4: " << wp_point_4[0] << " " << wp_point_4[1] << " " << wp_point_4[2] << std::endl;
    std::cout << "wp_point_5: " << wp_point_5[0] << " " << wp_point_5[1] << " " << wp_point_5[2] << std::endl;
    std::cout << "wp_point_6: " << wp_point_6[0] << " " << wp_point_6[1] << " " << wp_point_6[2] << std::endl;
    std::cout << "wp_point_7: " << wp_point_7[0] << " " << wp_point_7[1] << " " << wp_point_7[2] << std::endl;
    std::cout << "wp_point_8: " << wp_point_8[0] << " " << wp_point_8[1] << " " << wp_point_8[2] << std::endl;
    std::cout << "wp_point_9: " << wp_point_9[0] << " " << wp_point_9[1] << " " << wp_point_9[2] << std::endl;
    std::cout << "wp_point_10: " << wp_point_10[0] << " " << wp_point_10[1] << wp_point_10[2] << std::endl;

    std::string topic_prefix = "/" + uav_name + std::to_string(uav_id);
    ros::Publisher waypoint_pub = nh.advertise<sunray_msgs::UAVWayPoint>(topic_prefix + "/sunray/uav_waypoint", 10);
    std::cout << topic_prefix + "/sunray/uav_waypoint" << std::endl;

    sunray_msgs::UAVWayPoint waypoint_msg;
    waypoint_msg.header.stamp = ros::Time::now();
    waypoint_msg.wp_num = wp_num;
    waypoint_msg.wp_type = wp_type;
    waypoint_msg.wp_land = wp_land;
    waypoint_msg.wp_return = wp_return;
    waypoint_msg.wp_takeoff = wp_takeoff;
    waypoint_msg.wp_fix_yaw = wp_fix_yaw;
    waypoint_msg.move_vel = move_vel;
    waypoint_msg.z_height = z_height;
    waypoint_msg.yaw_angle = yaw_angle;
    waypoint_msg.wp_point_1 = {wp_point_1[0], wp_point_1[1], wp_point_1[2]};
    waypoint_msg.wp_point_2 = {wp_point_2[0], wp_point_2[1], wp_point_2[2]};
    waypoint_msg.wp_point_3 = {wp_point_3[0], wp_point_3[1], wp_point_3[2]};
    waypoint_msg.wp_point_4 = {wp_point_4[0], wp_point_4[1], wp_point_4[2]};
    waypoint_msg.wp_point_5 = {wp_point_5[0], wp_point_5[1], wp_point_5[2]};
    waypoint_msg.wp_point_6 = {wp_point_6[0], wp_point_6[1], wp_point_6[2]};
    waypoint_msg.wp_point_7 = {wp_point_7[0], wp_point_7[1], wp_point_7[2]};
    waypoint_msg.wp_point_8 = {wp_point_8[0], wp_point_8[1], wp_point_8[2]};
    waypoint_msg.wp_point_9 = {wp_point_9[0], wp_point_9[1], wp_point_9[2]};
    waypoint_msg.wp_point_10 = {wp_point_10[0], wp_point_10[1], wp_point_10[2]};
    ros::Duration(0.5).sleep();
    waypoint_pub.publish(waypoint_msg);
    // ros::Rate loop_rate(1);
    // while (ros::ok())
    // {
    //     waypoint_pub.publish(waypoint_msg);
    //     ros::spinOnce();
    //     loop_rate.sleep();
    //     break;
    // }
    ros::Duration(0.5).sleep();
    return 0;
}
