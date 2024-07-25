#include <ros/ros.h>
#include <sunray_msgs/UAVControlCMD.h>
#include <sunray_msgs/UAVState.h>
#include <sunray_msgs/UAVSetup.h>

using namespace  std;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "takeoff_node");
    ros::NodeHandle nh("~");

    ros::Publisher setup_pub = nh.advertise<sunray_msgs::UAVSetup>("/uav1/sunray/setup",1);
    ros::Publisher cmd_pub = nh.advertise<sunray_msgs::UAVControlCMD>("/uav1/sunray/uav_control_cmd",1);

    sunray_msgs::UAVSetup setup;
    sunray_msgs::UAVControlCMD uav_cmd;
    ros::Duration(0.5).sleep();
    // 解锁
    cout<<"arm"<<endl;
    setup.cmd = 0;
    setup.arming = 1;
    setup_pub.publish(setup);
    ros::Duration(1.0).sleep();

    // 切换到指令控制模式
    cout<<"switch CMD_CONTROL"<<endl;
    setup.cmd = 3;
    setup.control_state = "CMD_CONTROL";
    setup_pub.publish(setup);
    ros::Duration(1.0).sleep();

    // 起飞
    cout<<"takeoff"<<endl;
    uav_cmd.cmd = 1;
    uav_cmd.cmd_id = 0;
    cmd_pub.publish(uav_cmd);
    ros::Duration(5).sleep();
    
    // 悬停
    cout<<"hover"<<endl;
    uav_cmd.cmd = 2;
    cmd_pub.publish(uav_cmd);
    ros::Duration(5).sleep();

    // 降落
    cout<<"land"<<endl;
    uav_cmd.cmd = 3;
    cmd_pub.publish(uav_cmd);

}
