#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>


ros::Publisher pub;
geometry_msgs::PoseStamped latest_pose;
bool has_pose = false;


void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    latest_pose = *msg;
    latest_pose.header.frame_id = "map";
    has_pose = true;

    ROS_INFO_THROTTLE(1.0, "VRPN原始x: %.3f米, 转换后视觉x: %.3f米", 
                     msg->pose.position.x, 
                     latest_pose.pose.position.x);
    
    ROS_INFO_THROTTLE(1.0, "VRPN原始y: %.3f米, 转换后视觉y: %.3f米", 
                     msg->pose.position.y, 
                     latest_pose.pose.position.y);                 

    ROS_INFO_THROTTLE(1.0, "VRPN原始z: %.3f米, 转换后视觉z: %.3f米", 
                     msg->pose.position.z, 
                     latest_pose.pose.position.z);
}

void stateCallback(const mavros_msgs::State::ConstPtr& state) {
    ROS_INFO("FCU connected: %s, armed: %s, mode: %s",
             state->connected ? "YES" : "NO",
             state->armed ? "YES" : "NO",
             state->mode.c_str());
}

int main(int argc, char** argv) {
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "vrpn_to_vision_pose");
    ros::NodeHandle nh;
    pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 1);
    ros::Subscriber sub = nh.subscribe("/vrpn_client_node_1/fmt1/pose", 1, poseCallback);
    ros::Subscriber state_sub = nh.subscribe("/mavros/state", 10, stateCallback);

    ros::Rate rate(50); // 50Hz
    while (ros::ok()) {
        ros::spinOnce();
        if (has_pose) {
            geometry_msgs::PoseStamped vision_pose = latest_pose;

            // FLU → RFL 坐标系转换
            // FLU: Forward, Left, Up
            // RFL: Right, Forward, Up (x = -L, y = F, z = U)
            double flu_x = latest_pose.pose.position.x;  // Forward
            double flu_y = latest_pose.pose.position.y;  // Left
            double flu_z = latest_pose.pose.position.z;  // Up
            
            // 转换为 RFL 坐标系
            vision_pose.pose.position.x = -flu_y;  // x = -L
            vision_pose.pose.position.y = flu_x;   // y = F
            vision_pose.pose.position.z = flu_z;   // z = U

            latest_pose.header.stamp = ros::Time::now();
            pub.publish(vision_pose);
        }
        rate.sleep();
    }
    return 0;
}
