#include <ros/ros.h>
#include <sunray_msgs/UAVControlCMD.h>
#include <sunray_msgs/UAVState.h>
#include <sunray_msgs/UAVSetup.h>
#include <geometry_msgs/PoseStamped.h>

using namespace  std;

geometry_msgs::PoseStamped current_pose;

void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    current_pose = *msg;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "takeoff_node");
    ros::NodeHandle nh("~");

    ros::Publisher setup_pub = nh.advertise<sunray_msgs::UAVSetup>("/uav1/sunray/setup",1);
    ros::Publisher cmd_pub = nh.advertise<sunray_msgs::UAVControlCMD>("/uav1/sunray/uav_control_cmd",1);
    ros::Subscriber pose_sub = nh.subscribe("/uav1/mavros/local_position/pose", 10, pose_cb);

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

    // Define the square's vertices
    std::vector<std::tuple<double, double, double>> vertices = {
        std::make_tuple( 2, -2, 1),  // Start point
        std::make_tuple( 2,  2, 1),  // Right point
        std::make_tuple(-2,  2, 1),  // Top-right point
        std::make_tuple(-2, -2, 1),  // Top-left point
        std::make_tuple( 2, -2, 1),  // Start point
        std::make_tuple( 0,  0, 1)   // Back to start point
    };

    for (const auto& vertex : vertices) {

        // Send setpoints until the drone reaches the target point
        cout<<"go to point: ("<< std::get<0>(vertex) <<" " <<std::get<1>(vertex) <<" "<<std::get<2>(vertex)<<")"<<endl;
        while(ros::ok()) {

            uav_cmd.header.stamp = ros::Time::now();
            uav_cmd.cmd = sunray_msgs::UAVControlCMD::XYZ_POS;
            uav_cmd.desired_pos[0] = std::get<0>(vertex);
            uav_cmd.desired_pos[1] = std::get<1>(vertex);
            uav_cmd.desired_pos[2] = std::get<2>(vertex);
            uav_cmd.desired_yaw = 0;
            uav_cmd.enable_yawRate = 0;
            uav_cmd.cmd_id = uav_cmd.cmd_id + 1;
            cmd_pub.publish(uav_cmd);

            // Check if the drone has reached the target point
            if (fabs(current_pose.pose.position.x - std::get<0>(vertex)) < 0.5 &&
                fabs(current_pose.pose.position.y - std::get<1>(vertex)) < 0.5 &&
                fabs(current_pose.pose.position.z - std::get<2>(vertex)) < 0.5) {
                break;
            }

            ros::spinOnce();
            ros::Duration(0.5).sleep();
        }
    }

    // 降落
    cout<<"land"<<endl;
    uav_cmd.cmd = 3;
    cmd_pub.publish(uav_cmd);

}
