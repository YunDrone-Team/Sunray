// ros头文件
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <iostream>
#include <boost/format.hpp>
#include <sunray_msgs/UGVControlCMD.h>
// topic 头文件
#include <geometry_msgs/PoseStamped.h>
#include "ugv_control.h"

using namespace std;

#define NODE_NAME "ugv_terminal_control"
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>全 局 变 量<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
string ugv_name;
int ugv_id = 1;
sunray_msgs::UGVControlCMD ugv_control_cmd; // 无人车外部控制指令
ros::Publisher command_pub;
float state_desired[3];
string ugv_prefix = "ugv" + std::to_string(ugv_id);
string topic_prefix = "/" + ugv_prefix;

std::string format_float_two_decimal(float value)
{
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(2) << value;
    return oss.str();
}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{  
    // 设置日志
    Logger::init_default();
    Logger::setPrintLevel(false);
    Logger::setPrintTime(false);
    Logger::setPrintToFile(false);
    Logger::setFilename("~/Documents/Sunray_log.txt");

    ros::init(argc, argv, NODE_NAME);
    ros::NodeHandle nh("~");

    // 无人机编号 1号无人机则为1
    nh.param("ugv_id", ugv_id, 1);

    ugv_name = "/ugv" + std::to_string(ugv_id);

    // 【发布】控制指令
    command_pub = nh.advertise<sunray_msgs::UGVControlCMD>(topic_prefix + "/sunray_ugv/ugv_control_cmd", 10);

    // Waiting for input
    int start_flag = 0;

    while (ros::ok())
    {
        Logger::print_color(int(LogColor::green), ">>>>>>>>>>>>>>>>>>>>>>>>>>>>UGV Terminal Control<<<<<<<<<<<<<<<<<<<<<<<<< ");
        Logger::print_color(int(LogColor::green), "Please choose the action: 0 for INIT, 1 for HOLD, 2 for POS_CONTROL_ENU ,3 for VEL_CONTROL_ENU 4 for VEL_CONTROL_BODY, 5 for Point_Control_with_Astar, 6 for POS_VEL_CONTROL_ENU");
        cin >> start_flag;
        if (start_flag == 0)
        {
            ugv_control_cmd.cmd = sunray_msgs::UGVControlCMD::INIT;
            command_pub.publish(ugv_control_cmd);
        }
        else if (start_flag == 1)
        {
            Logger::print_color(int(LogColor::green), "In HOLD");
            Logger::print_color(int(LogColor::green), "desired state: --- linear_vel_x(body) [m/s]");
            cin >> state_desired[0];
            Logger::print_color(int(LogColor::green), "desired state: --- linear_vel_y(body) [m/s]");
            cin >> state_desired[1];
            Logger::print_color(int(LogColor::green), "desired state: --- yawRate(body) [deg/s]");
            cin >> state_desired[2];

            ugv_control_cmd.cmd = sunray_msgs::UGVControlCMD::HOLD;
            ugv_control_cmd.desired_vel[0] = state_desired[0];
            ugv_control_cmd.desired_vel[1] = state_desired[1];
            ugv_control_cmd.angular_vel = state_desired[2] / 180.0 * M_PI;
            command_pub.publish(ugv_control_cmd);
            Logger::print_color(int(LogColor::blue), "state_desired [linear angular] : " + format_float_two_decimal(state_desired[0]) + " [m/s] " + format_float_two_decimal(state_desired[1]) + " [m/s] " + format_float_two_decimal(state_desired[2]) + " [deg/s] ");
        }
        else if (start_flag == 2)
        {
            Logger::print_color(int(LogColor::green), "Move in POS_CONTROL_ENU, Pls input the desired position and yaw");
            Logger::print_color(int(LogColor::green), "desired state: --- linear_pos_x(enu) [m]");
            cin >> state_desired[0];
            Logger::print_color(int(LogColor::green), "desired state: --- linear_pos_y(enu) [m]");
            cin >> state_desired[1];
            Logger::print_color(int(LogColor::green), "desired state: --- yaw [deg]");
            cin >> state_desired[2];

            ugv_control_cmd.cmd = sunray_msgs::UGVControlCMD::POS_CONTROL_ENU;
            ugv_control_cmd.desired_pos[0] = state_desired[0];
            ugv_control_cmd.desired_pos[1] = state_desired[1];
            ugv_control_cmd.desired_yaw = state_desired[2] / 180.0 * M_PI;
            command_pub.publish(ugv_control_cmd);
            Logger::print_color(int(LogColor::blue), "state_desired[position] : " + format_float_two_decimal(state_desired[0]) + " [m] " + format_float_two_decimal(state_desired[1]) + " [m] " + format_float_two_decimal(state_desired[2]) + " [deg] ");
        }
        else if (start_flag == 3)
        {
            Logger::print_color(int(LogColor::green), "Move in VEL_CONTROL_ENU, Pls input the desired position and yaw");
            Logger::print_color(int(LogColor::green), "desired state: --- linear_pos_x(enu) [m/s]");
            cin >> state_desired[0];
            Logger::print_color(int(LogColor::green), "desired state: --- linear_pos_y(enu) [m/s]");
            cin >> state_desired[1];
            // cout << "desired state: --- yaw [deg]"<<endl;
            // cin >> state_desired[2];

            ugv_control_cmd.cmd = sunray_msgs::UGVControlCMD::VEL_CONTROL_ENU;
            ugv_control_cmd.desired_pos[0] = state_desired[0];
            ugv_control_cmd.desired_pos[1] = state_desired[1];
            // ugv_control_cmd.desired_yaw = state_desired[2]/180.0*M_PI;
            command_pub.publish(ugv_control_cmd);
            Logger::print_color(int(LogColor::blue), "state_desired[velolity] : " + format_float_two_decimal(state_desired[0]) + " [m/s] " + format_float_two_decimal(state_desired[1]) + " [m/s] ");
        }
        // else if (start_flag == 5)
        // {
        //     cout << "Move in Point control, Pls input the desired velocity and angular "<<endl;
        //     cout << "desired state: --- x [m/s] "<<endl;
        //     cin >> state_desired[0];
        //     // cout << "desired state: --- y [m/s]"<<endl;
        //     // cin >> state_desired[1];
        //     cout << "desired state: --- yaw [deg/s]"<<endl;
        //     cin >> state_desired[2];

        //     ugv_control_cmd.cmd = 5;
        //     ugv_control_cmd.desired_vel[0] = state_desired[0];
        //     // ugv_control_cmd.desired_vel[1] = state_desired[1];
        //     ugv_control_cmd.angular_vel  = state_desired[2]/180.0*M_PI;
        //     command_pub.publish(ugv_control_cmd);
        //     cout << "state_desired [X Y] : " << state_desired[0] << " [ m/s ] "<< state_desired[1] <<" [ m/s ] "<< endl;
        //     cout << "state_desired [YAW] : " << state_desired[2] << " [ deg ] "<< endl;
        // }
        else if (start_flag == 4)
        {
            Logger::print_color(int(LogColor::green), "Move in VEL_CONTROL_BODY, Pls input the desired position and yaw");
            Logger::print_color(int(LogColor::green), "desired state: --- linear_pos_x(body) [m/s]");
            cin >> state_desired[0];
            Logger::print_color(int(LogColor::green), "desired state: --- linear_pos_y(body) [m/s]");
            cin >> state_desired[1];
            Logger::print_color(int(LogColor::green), "desired state: --- yaw(body) [deg/s]");
            cin >> state_desired[2];

            ugv_control_cmd.cmd = sunray_msgs::UGVControlCMD::VEL_CONTROL_BODY;
            ugv_control_cmd.desired_vel[0] = state_desired[0];
            ugv_control_cmd.desired_vel[1] = state_desired[1];
            ugv_control_cmd.angular_vel = state_desired[2] / 180.0 * M_PI;
            command_pub.publish(ugv_control_cmd);
            cout << "state_desired [X Y] : " << state_desired[0] << " [ m/s ] " << state_desired[1] << " [ m/s ] " << endl;
            Logger::print_color(int(LogColor::blue), "state_desired[X Y] : " + format_float_two_decimal(state_desired[0]) + " [m/s] " + format_float_two_decimal(state_desired[1]) + " [m/s] ");
            cout << "state_desired [YAW] : " << state_desired[2] << " [ deg/s ] " << endl;
            Logger::print_color(int(LogColor::blue), "state_desired[YAW] : " + format_float_two_decimal(state_desired[2]) + " [deg/s] ");
        }
        else if (start_flag == 5)
        {
            Logger::print_color(int(LogColor::green), "Move in Point_Control_with_Astar, Pls input the desired position");
            Logger::print_color(int(LogColor::green), "desired state: --- linear_pos_x(body) [m]");
            cin >> state_desired[0];
            Logger::print_color(int(LogColor::green), "desired state: --- linear_pos_y(body) [m]");
            cin >> state_desired[1];
            ugv_control_cmd.cmd = sunray_msgs::UGVControlCMD::Point_Control_with_Astar;
            ugv_control_cmd.desired_pos[0] = state_desired[0];
            ugv_control_cmd.desired_pos[1] = state_desired[1];
            command_pub.publish(ugv_control_cmd);
            Logger::print_color(int(LogColor::blue), "state_desired[X Y] : " + format_float_two_decimal(state_desired[0]) + " [m] " + format_float_two_decimal(state_desired[1]) + " [m] ");
        }
        else
        {
            Logger::print_color(int(LogColor::red), "Wrong input!!!");
        }
        ros::Duration(0.5).sleep();
    }
    ros::Duration(0.5).sleep();
}