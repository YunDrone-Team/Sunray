#include <ros/ros.h>
#include <csignal>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>

bool ros_exit_flag = false;
ros::Publisher pub_fastlio_Path;

void sigHandle(int sig) {

    ros_exit_flag = true;
    ROS_WARN("Catch sig %d", sig);
}

void fastlioOdomCallback(const nav_msgs::Odometry::ConstPtr &odom) {

    std::cout << odom->pose.pose.position.z << std::endl;
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "fastlio_traj_eva");
    ros::NodeHandle nh;

    signal(SIGINT, sigHandle);

    pub_fastlio_Path = nh.advertise<nav_msgs::Path>("/fastlio_path", 10);

    // 订阅
    ros::Subscriber sub_fastlio_odom = nh.subscribe("/Odometry", 100, fastlioOdomCallback);

    while(true) {

        if(ros_exit_flag) {

            return 0;
        }

        ros::spinOnce();
    }

    return 0;
}

