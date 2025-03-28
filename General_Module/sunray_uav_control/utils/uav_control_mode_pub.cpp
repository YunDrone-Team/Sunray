// ros头文件
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <iostream>

// topic 头文件
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/RCIn.h>
#include <nav_msgs/Path.h>

// #include "controller_test.h"
#include "printf_utils.h"

#include "ros_msg_utils.h"

using namespace std;
#define TRA_WINDOW 2000
sunray_msgs::UAVState uav_state;
sunray_msgs::UAVControlCMD uav_cmd;
std_msgs::Int32 uav_control_state;
std::vector<geometry_msgs::PoseStamped> posehistory_vector_;

bool flag = false;

void uav_state_cb(const sunray_msgs::UAVState::ConstPtr &msg)
{
    uav_state = *msg;
}

void uav_control_state_cb(const std_msgs::Int32::ConstPtr &msg)
{
    uav_control_state = *msg;
}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "uav_control_mode_pub");
    ros::NodeHandle nh("~");

    int uav_id;
    bool sim_mode;
    nh.param("uav_id", uav_id, 1);
    nh.param("sim_mode", sim_mode, true);

    string uav_name = "/uav" + std::to_string(uav_id);

    //【订阅】状态信息
    ros::Subscriber uav_state_sub = nh.subscribe<sunray_msgs::UAVState>(uav_name + "/sunray/state", 1, uav_state_cb);

    //【发布】UAVCommand
    ros::Publisher uav_command_pub = nh.advertise<sunray_msgs::UAVSetup>(uav_name + "/sunray/setup", 1);


    sunray_msgs::UAVSetup uav_setup;
    int CMD = 0;
    bool arming;
    string px4_mode;
    string control_mode;
    int tmp = 0;

    while (ros::ok())
    {
        ros::spinOnce();

        cout << GREEN << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>UAV Terminal Control<<<<<<<<<<<<<<<<<<<<<<<<< " << TAIL << endl;
        cout << GREEN << "Please choose the Mode: 0 for ARMING, 1 for SET_PX4_MODE, 2 for REBOOT_PX4, 3 for SET_CONTROL_MODE..." << TAIL << endl;
        cin >> CMD;

        switch (CMD)
        {
        case 0:
            cout << "set arming mode: 0 for disarm, 1 for arm" << endl;
            cin >> tmp;
            
            if(tmp){
                uav_setup.cmd  = 1;
            }else{
                uav_setup.cmd  = 0;
            }
            cout << "arming "<< std::boolalpha << tmp << endl;
            uav_command_pub.publish(uav_setup);
            break;
        case 1:
            cout << "set px4 mode: 0 for OFFBOARD, 1 for AUTO.LAND, 2 for POSCTL, 3 for AUTO.RTL" << endl;
            cin >> tmp;
            uav_setup.cmd  = 2;
            if(tmp == 0){
                uav_setup.px4_mode = "OFFBOARD";
            }else if(tmp == 1){
                uav_setup.px4_mode = "AUTO.LAND";
            }
            else if(tmp == 2){
                uav_setup.px4_mode = "POSCTL";
            }
            else if(tmp == 3){
                uav_setup.px4_mode = "AUTO.RTL";
            }
            else{
                cout << "Mode error " << endl;
            }
            cout << "px4 mode: " << px4_mode << endl;
            uav_command_pub.publish(uav_setup);
            break;
        case 2:
            cout << "confirm whether to restart PX4: 0 for false, 1 for true" << endl;
            cin >> tmp;
            uav_setup.cmd  =3;
            if(tmp){
                uav_setup.cmd  = 3;
            }else{
                break;
            }
            cout << "reboot px4" << endl;
            uav_command_pub.publish(uav_setup);
            break;
        case 3:
            cout << "set control mode: 0 for INIT, 1 for RC_CONTROL, 2 for HOVER_CONTROL, 3 for CMD_CONTROL, 4 for LAND_CONTROL" << endl;
            cin >> tmp;
            uav_setup.cmd  = 4;
            if(tmp == 0){
                uav_setup.control_mode = "INIT";
            }else if(tmp == 1){
                uav_setup.control_mode = "RC_CONTROL";
            }
            else if(tmp == 2){
                uav_setup.control_mode = "HOVER_CONTROL";
            }
            else if(tmp == 3){
                uav_setup.control_mode = "CMD_CONTROL";
            }
            else if(tmp == 4){
                uav_setup.control_mode = "LAND_CONTROL";
            }
            else{
                cout << "Mode error " << endl;
            }
            cout << "control mode: " << control_mode << endl;
            uav_command_pub.publish(uav_setup);
            break;
        case 5:
            cout << "CMD ERROR" << endl;
            break;
        }

        ros::Duration(0.5).sleep();
    }
    return 0;
}
