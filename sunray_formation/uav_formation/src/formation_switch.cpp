// ros头文件
#include <ros/ros.h>
#include <iostream>
#include "printf_utils.h"
#include <signal.h>
#include "ros_msg_utils.h"

using namespace std;
sunray_msgs::UAVState uav_state;
sunray_msgs::UAVControlCMD uav_cmd;
sunray_msgs::UAVSetup setup;
sunray_msgs::UAVState uav_control_state;
sunray_msgs::Formation formation_cmd;

void mySigintHandler(int sig)
{
    ROS_INFO("[uav_command_pub] exit...");
    ros::shutdown();
    exit(0);
}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "uav_command_pub");
    ros::NodeHandle nh("~");
    signal(SIGINT, mySigintHandler);
    int uav_id;
    int uav_num;
    string uav_name{""};
    nh.param("uav_id", uav_id, 1);
    nh.param("uav_num", uav_num, 1);
    nh.param<string>("uav_name", uav_name, "uav");

    uav_name = uav_name + std::to_string(uav_id);
    ros::Publisher formation_pub = nh.advertise<sunray_msgs::Formation>("/sunray/formation_cmd", 10);
    int CMD = 0;

    while (ros::ok())
    {
        ros::spinOnce();
        cout << GREEN << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>formation Terminal Control<<<<<<<<<<<<<<<<<<<<<<<<< " << TAIL << endl;
        cout << GREEN << "setup: "
             << YELLOW << "101 " << GREEN << "takeoff"
             << YELLOW << " 102 " << GREEN << "land,"
             << YELLOW << " 103 " << GREEN << "hover,"
             << YELLOW << " 104 " << GREEN << "set home,"
             << YELLOW << " 105 " << GREEN << "return home" << TAIL << endl;
        cout << GREEN << "CMD: "
             << YELLOW << "1 " << GREEN << "正三角形,"
             << YELLOW << " 2 " << GREEN << "倒三角形,"
             << YELLOW << " 3 " << GREEN << "横向一字阵型,"
             << YELLOW << " 4 " << GREEN << "纵向一字阵型,"
             << YELLOW << " 5 " << GREEN << "动态圆形阵型,"
             << YELLOW << " 6 " << GREEN << "动态八字阵型,"
             << YELLOW << " 7 " << GREEN << "领队模式" << TAIL << endl;
        cin >> CMD;

        switch (CMD)
        {
        case 1:
            formation_cmd.cmd = sunray_msgs::Formation::FORMATION;
            formation_cmd.formation_type = sunray_msgs::Formation::STATIC;
            formation_cmd.name = "triangle";
            formation_pub.publish(formation_cmd);
            break;
        case 2:
            formation_cmd.cmd = sunray_msgs::Formation::FORMATION;
            formation_cmd.formation_type = sunray_msgs::Formation::STATIC;
            formation_cmd.name = "triangle_reverse";
            formation_pub.publish(formation_cmd);
            break;
        case 3:
            formation_cmd.cmd = sunray_msgs::Formation::FORMATION;
            formation_cmd.formation_type = sunray_msgs::Formation::STATIC;
            formation_cmd.name = "line_horizontal";
            formation_pub.publish(formation_cmd);
            break;
        case 4:
            formation_cmd.cmd = sunray_msgs::Formation::FORMATION;
            formation_cmd.formation_type = sunray_msgs::Formation::STATIC;
            formation_cmd.name = "line_vertical";
            formation_pub.publish(formation_cmd);
            break;
        case 5:
            formation_cmd.cmd = sunray_msgs::Formation::FORMATION;
            formation_cmd.formation_type = sunray_msgs::Formation::DYNAMIC;
            formation_cmd.name = "circle";
            formation_pub.publish(formation_cmd);
            break;
        case 6:
            formation_cmd.cmd = sunray_msgs::Formation::FORMATION;
            formation_cmd.formation_type = sunray_msgs::Formation::DYNAMIC;
            formation_cmd.name = "figure_eight";
            formation_pub.publish(formation_cmd);
            break;
        case 7:
            formation_cmd.cmd = sunray_msgs::Formation::FORMATION;
            formation_cmd.formation_type = sunray_msgs::Formation::LEADER;
            formation_pub.publish(formation_cmd);
            break;

        case 101:
        {
            formation_cmd.cmd = sunray_msgs::Formation::TAKEOFF;
            formation_cmd.name = "";
            formation_pub.publish(formation_cmd);
            break;
        }
        case 102:
        {
            formation_cmd.cmd = sunray_msgs::Formation::LAND;
            formation_cmd.name = "";
            formation_pub.publish(formation_cmd);
            break;
        }
        case 103:
            formation_cmd.cmd = sunray_msgs::Formation::HOVER;
            formation_cmd.name = "";
            formation_pub.publish(formation_cmd);
            break;
        case 104:
            formation_cmd.cmd = sunray_msgs::Formation::SET_HOME;
            formation_cmd.name = "";
            formation_pub.publish(formation_cmd);
            break;
        case 105:
            formation_cmd.cmd = sunray_msgs::Formation::RETURN_HOME;
            formation_cmd.name = "";
            formation_pub.publish(formation_cmd);
            break;
        }

        ros::Duration(0.5).sleep();
    }
    return 0;
}
