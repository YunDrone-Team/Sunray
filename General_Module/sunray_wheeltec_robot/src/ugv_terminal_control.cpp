//ros头文件
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <iostream>
#include <boost/format.hpp>

//topic 头文件
#include <sunray_msgs/UGVControlCMD.h>
#include <geometry_msgs/PoseStamped.h>

using namespace std;

#define NODE_NAME "ugv_terminal_control"
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>全 局 变 量<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
string ugv_name;
int ugv_id;
sunray_msgs::UGVControlCMD ugv_control_cmd;         // 无人车外部控制指令
ros::Publisher command_pub;
float state_desired[3];
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, NODE_NAME);
    ros::NodeHandle nh("~");
    
    // 无人机编号 1号无人机则为1
    nh.param("ugv_id", ugv_id, 1);

    ugv_name = "/ugv" + std::to_string(ugv_id);

    // 【发布】控制指令
    command_pub = nh.advertise<sunray_msgs::UGVControlCMD>(ugv_name + "/sunray/ugv_control_cmd", 10);

    // Waiting for input
    int start_flag = 0;
    
    while (ros::ok()) 
    {
        cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>UGV Terminal Control<<<<<<<<<<<<<<<<<<<<<<<<< "<< endl;
        cout << "Please choose the action: 0 for Hold, 1 for Direct Control BODY, 2 for Direct Control ENU , 3 for Point Control, 4 for Astar Goal..."<<endl;
        cin >> start_flag;
        if (start_flag == 0)
        {
            ugv_control_cmd.cmd = sunray_msgs::UGVControlCMD::Hold;
            command_pub.publish(ugv_control_cmd);
        }
        else if (start_flag == 1)
        {
            cout << "Move in Direct Control, Pls input the desired velocity and angular  velocity "<<endl;
            cout << "desired state: --- linear_vel_x(body) [m/s] "<<endl;
            cin >> state_desired[0];
            cout << "desired state: --- linear_vel_y(body) [m/s] "<<endl;
            cin >> state_desired[1];
            cout << "desired state: --- angular_vel(body) [deg/s]"<<endl;
            cin >> state_desired[2];

            ugv_control_cmd.cmd = sunray_msgs::UGVControlCMD::Direct_Control_BODY;
            ugv_control_cmd.linear_vel[0] = state_desired[0];
            ugv_control_cmd.linear_vel[1] = state_desired[1];
            ugv_control_cmd.angular_vel = state_desired[2]/180.0*M_PI;
            command_pub.publish(ugv_control_cmd);
            cout << "state_desired [linear angular] : " << state_desired[0] << " [m/s] "<< state_desired[1] << " [m/s] "<< state_desired[2] <<" [deg/s] "<< endl;
        } 
        else if (start_flag == 2)
        {
            cout << "Move in Direct Control, Pls input the desired velocity and angular  velocity "<<endl;
            cout << "desired state: --- linear_vel_x(enu) [m/s] "<<endl;
            cin >> state_desired[0];
            cout << "desired state: --- linear_vel_y(enu) [m/s] "<<endl;
            cin >> state_desired[1];
            cout << "desired state: --- yaw [deg]"<<endl;
            cin >> state_desired[2];

            ugv_control_cmd.cmd = sunray_msgs::UGVControlCMD::Direct_Control_ENU;
            ugv_control_cmd.linear_vel[0] = state_desired[0];
            ugv_control_cmd.linear_vel[1] = state_desired[1];
            ugv_control_cmd.desired_yaw = state_desired[2]/180.0*M_PI;
            command_pub.publish(ugv_control_cmd);
            cout << "state_desired [linear angular] : " << state_desired[0] << " [m/s] "<< state_desired[1] << " [m/s] "<< state_desired[2] <<" [deg] "<< endl;
        } 
        else if (start_flag == 3)
        {
            cout << "Move in Point control, Pls input the desired position"<<endl;
            cout << "desired state: --- x [m] "<<endl;
            cin >> state_desired[0];
            cout << "desired state: --- y [m]"<<endl;
            cin >> state_desired[1];
            cout << "desired state: --- yaw [deg]"<<endl;
            cin >> state_desired[2];

            ugv_control_cmd.cmd = sunray_msgs::UGVControlCMD::Point_Control;
            ugv_control_cmd.desired_pos[0] = state_desired[0];
            ugv_control_cmd.desired_pos[1] = state_desired[1];
            ugv_control_cmd.desired_yaw = state_desired[2]/180.0*M_PI;
            command_pub.publish(ugv_control_cmd);
            cout << "state_desired [X Y] : " << state_desired[0] << " [ m ] "<< state_desired[1] <<" [ m ] "<< endl;
            cout << "state_desired [YAW] : " << state_desired[2] << " [ deg ] "<< endl;
        }
        else if (start_flag == 4)
        {
            cout << "Move in Point_Control_with_Astar, Pls input the goal"<<endl;
            cout << "desired state: --- x [m] "<<endl;
            cin >> state_desired[0];
            cout << "desired state: --- y [m]"<<endl;
            cin >> state_desired[1];
            cout << "desired state: --- yaw [deg]"<<endl;
            cin >> state_desired[2];

            ugv_control_cmd.cmd = sunray_msgs::UGVControlCMD::Point_Control_with_Astar;
            ugv_control_cmd.desired_pos[0] = state_desired[0];
            ugv_control_cmd.desired_pos[1] = state_desired[1];
            ugv_control_cmd.desired_yaw = state_desired[2]/180.0*M_PI;
            command_pub.publish(ugv_control_cmd);
            cout << "state_desired [X Y] : " << state_desired[0] << " [ m ] "<< state_desired[1] <<" [ m ] "<< endl;
            cout << "state_desired [YAW] : " << state_desired[2] << " [ deg ] "<< endl;
        }
        else
        {
            cout << "Wrong input."<<endl;
        }
        ros::Duration(0.5).sleep();
    }
    return 0;
}