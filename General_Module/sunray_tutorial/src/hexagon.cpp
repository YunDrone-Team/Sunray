#include <ros/ros.h>
#include <sunray_msgs/UAVControlCMD.h>
#include <sunray_msgs/UAVState.h>
#include <sunray_msgs/UAVSetup.h>
#include <geometry_msgs/PoseStamped.h>

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
    ros::Duration(2).sleep();
    std::tuple<double, double, double> vertex = std::make_tuple(2,  0,  0);
    int yaw;
    int cmd_id = 1;
    for(int i = 0; i < 7; ++i) {
        
        uav_cmd.header.stamp = ros::Time::now();
        uav_cmd.cmd = sunray_msgs::UAVControlCMD::XYZ_POS_BODY;
        uav_cmd.desired_pos[0] = std::get<0>(vertex);
        uav_cmd.desired_pos[1] = std::get<1>(vertex);
        uav_cmd.desired_pos[2] = std::get<2>(vertex);
        uav_cmd.desired_yaw = 0;
        uav_cmd.enable_yawRate = 0;
        uav_cmd.cmd_id = cmd_id;
        cmd_pub.publish(uav_cmd);
        cmd_id += 1;
        ros::Duration(5).sleep();
        
        if(i == 0){
            yaw = 120;   
        }
        else{
            yaw = 60;
        }
        uav_cmd.header.stamp = ros::Time::now();
        uav_cmd.cmd = sunray_msgs::UAVControlCMD::XYZ_POS_BODY;
        uav_cmd.desired_pos[0] = 0;
        uav_cmd.desired_pos[1] = 0;
        uav_cmd.desired_pos[2] = 0;
        uav_cmd.desired_yaw = yaw / 180.0 * M_PI;
        uav_cmd.enable_yawRate = 0;
        uav_cmd.cmd_id = cmd_id;
        cmd_pub.publish(uav_cmd);
        cmd_id += 1;
        ros::Duration(2).sleep();
    }

    // 降落
    cout<<"land"<<endl;
    uav_cmd.cmd = 3;
    uav_cmd.cmd_id = cmd_id;
    cmd_pub.publish(uav_cmd);

}
