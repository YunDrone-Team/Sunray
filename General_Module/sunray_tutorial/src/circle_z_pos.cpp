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
    ros::Duration(5).sleep();

    // Define the circle's center and radius
    double center_x = 0;
    double center_y = 0;
    double radius = 1;

    // Define the number of points on the circle
    int num_points = 50;
    // Define the proportional gain and maximum velocity
    double k_p = 1.5; // proportional gain
    double max_vel = 1.0; // maximum velocity (m/s)

    geometry_msgs::PoseStamped pose;

    for (int i = 0; i < num_points; i++) {
        double theta = i * 2 * M_PI / num_points;
        pose.pose.position.x = center_x + radius * cos(theta);
        pose.pose.position.y = center_y + radius * sin(theta);
        pose.pose.position.z = 1; // fixed altitude

        // Send setpoints until the drone reaches the target point
        while(ros::ok()) {
            // Calculate the distance to the target position
            double dx = pose.pose.position.x - current_pose.pose.position.x;
            double dy = pose.pose.position.y - current_pose.pose.position.y;

            // Calculate the desired velocity using a proportional controller
            double vx = k_p * dx;
            double vy = k_p * dy;

            // Limit the velocities to a maximum value
            vx = min(max(vx, -max_vel), max_vel);
            vy = min(max(vy, -max_vel), max_vel);

            uav_cmd.header.stamp = ros::Time::now();
            uav_cmd.cmd = sunray_msgs::UAVControlCMD::XY_VEL_Z_POS;
            uav_cmd.desired_vel[0] = vx;
            uav_cmd.desired_vel[1] = vy;
            uav_cmd.desired_vel[2] = 0.0;
            uav_cmd.desired_pos[0] = 0.0;
            uav_cmd.desired_pos[1] = 0.0;
            uav_cmd.desired_pos[2] = 1;
            uav_cmd.desired_yaw = 0;
            uav_cmd.enable_yawRate = 0;
            // uav_cmd.cmd_id = uav_cmd.cmd_id + 1;
            cmd_pub.publish(uav_cmd);

            // Check if the drone has reached the target point
            if (fabs(current_pose.pose.position.x - pose.pose.position.x) < 0.2 &&
                fabs(current_pose.pose.position.y - pose.pose.position.y) < 0.2 ) {
                break;
            }

            ros::spinOnce();
            ros::Duration(0.1).sleep();
        }
        
    }

    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 1;
    while(ros::ok()) {
        // Calculate the distance to the target position
        double dx = pose.pose.position.x - current_pose.pose.position.x;
        double dy = pose.pose.position.y - current_pose.pose.position.y;

        // Calculate the desired velocity using a proportional controller
        double vx = k_p * dx;
        double vy = k_p * dy;

        // Limit the velocities to a maximum value
        vx = min(max(vx, -max_vel), max_vel);
        vy = min(max(vy, -max_vel), max_vel);

        uav_cmd.header.stamp = ros::Time::now();
        uav_cmd.cmd = sunray_msgs::UAVControlCMD::XY_VEL_Z_POS;
        uav_cmd.desired_vel[0] = vx;
        uav_cmd.desired_vel[1] = vy;
        uav_cmd.desired_vel[2] = 0.0;
        uav_cmd.desired_pos[0] = 0.0;
        uav_cmd.desired_pos[1] = 0.0;
        uav_cmd.desired_pos[2] = 1;
        uav_cmd.desired_yaw = 0;
        uav_cmd.enable_yawRate = 0;
        // uav_cmd.cmd_id = uav_cmd.cmd_id + 1;
        cmd_pub.publish(uav_cmd);

        // Check if the drone has reached the target point
        if (fabs(current_pose.pose.position.x - pose.pose.position.x) < 0.2 &&
            fabs(current_pose.pose.position.y - pose.pose.position.y) < 0.2 ) {
            break;
        }

        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }

    // 降落
    cout<<"land"<<endl;
    uav_cmd.cmd = 3;
    cmd_pub.publish(uav_cmd);

}
