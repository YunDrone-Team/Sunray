#include <ros/ros.h>
#include <csignal>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include "common_lib.h"
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>

QD init_q = QD::Identity();
V3D init_p = V3D::Zero();

ros::Publisher pub_fastlio_Path, pub_reloPath;

bool ros_exit_flag = false, init_flag = false;

void sigHandle(int sig) {

    ros_exit_flag = true;
    ROS_WARN("Catch sig %d", sig);
}

void reloOdomCallback(const nav_msgs::Odometry::ConstPtr &odom) {

    if(init_flag) {

        geometry_msgs::PoseStamped relo_odom;
        relo_odom.header = odom->header;
        relo_odom.pose.position.x = odom->pose.pose.position.x;
        relo_odom.pose.position.y = odom->pose.pose.position.y;
        relo_odom.pose.position.z = odom->pose.pose.position.z;
        relo_odom.pose.orientation.x = odom->pose.pose.orientation.x;
        relo_odom.pose.orientation.y = odom->pose.pose.orientation.y;
        relo_odom.pose.orientation.z = odom->pose.pose.orientation.z;
        relo_odom.pose.orientation.w = odom->pose.pose.orientation.w;

        static nav_msgs::Path relo_path;
        relo_path.header = relo_odom.header;
        relo_path.poses.push_back(relo_odom);
        pub_reloPath.publish(relo_path);
    }
}

void fastlioOdomCallback(const nav_msgs::Odometry::ConstPtr &odom) {

    if(init_flag) {

        double x = odom->pose.pose.position.x, y = odom->pose.pose.position.y, z = odom->pose.pose.position.z;
        double qw = odom->pose.pose.orientation.w, qx = odom->pose.pose.orientation.x,
               qy = odom->pose.pose.orientation.y, qz = odom->pose.pose.orientation.z;

        V3D p = init_q * V3D(x, y, z) + init_p;
        QD  q = init_q * QD(qw, qx, qy, qz);

        geometry_msgs::PoseStamped fastlio_odom;
        fastlio_odom.header = odom->header;
        fastlio_odom.pose.position.x = p.x();
        fastlio_odom.pose.position.y = p.y();
        fastlio_odom.pose.position.z = p.z();
        fastlio_odom.pose.orientation.x = q.x();
        fastlio_odom.pose.orientation.y = q.y();
        fastlio_odom.pose.orientation.z = q.z();
        fastlio_odom.pose.orientation.w = q.w();

        static nav_msgs::Path fastlio_path;
        fastlio_path.header = fastlio_odom.header;
        fastlio_path.poses.push_back(fastlio_odom);
        pub_fastlio_Path.publish(fastlio_path);
    }
}

void initPoseCallback(const nav_msgs::Odometry::ConstPtr &init_pose) {

    init_flag = true;
    init_p = V3D(init_pose->pose.pose.position.x, init_pose->pose.pose.position.y, init_pose->pose.pose.position.z);
    init_q = QD(init_pose->pose.pose.orientation.w, init_pose->pose.pose.orientation.x, init_pose->pose.pose.orientation.y, init_pose->pose.pose.orientation.z);

}

void syncOdomCallback(const nav_msgs::Odometry::ConstPtr &relo_odom, const nav_msgs::Odometry::ConstPtr &fastlio_odom) {

    if(init_flag) {

        double x = fastlio_odom->pose.pose.position.x, y = fastlio_odom->pose.pose.position.y, z = fastlio_odom->pose.pose.position.z;
        double qw = fastlio_odom->pose.pose.orientation.w, qx = fastlio_odom->pose.pose.orientation.x,
               qy = fastlio_odom->pose.pose.orientation.y, qz = fastlio_odom->pose.pose.orientation.z;

        double relo_x = relo_odom->pose.pose.position.x, relo_y = relo_odom->pose.pose.position.y, relo_z = relo_odom->pose.pose.position.z;
        double relo_qw = relo_odom->pose.pose.orientation.w, relo_qx = relo_odom->pose.pose.orientation.x,
               relo_qy = relo_odom->pose.pose.orientation.y, relo_qz = relo_odom->pose.pose.orientation.z;

        V3D p = init_q * V3D(x, y, z) + init_p;
        QD  q = init_q * QD(qw, qx, qy, qz);

        ROS_INFO("Timestamp: %f, fastlio_z: %f, relo_z: %f", fastlio_odom->header.stamp.toSec(), p.z(), relo_z);
    }
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "traj_eva");
    ros::NodeHandle nh;

    signal(SIGINT, sigHandle);

    // 发布
    pub_fastlio_Path = nh.advertise<nav_msgs::Path>("/fastlio_path", 10);
    pub_reloPath = nh.advertise<nav_msgs::Path>("/relo_path", 10);

    // 订阅
    ros::Subscriber sub_initPose = nh.subscribe("/init_pose", 100, initPoseCallback);
    ros::Subscriber sub_relo_odom = nh.subscribe("/relo_odom", 100, reloOdomCallback);
    ros::Subscriber sub_fastlio_odom = nh.subscribe("/Odometry", 100, fastlioOdomCallback);

    message_filters::Subscriber<nav_msgs::Odometry> relo_odom_sub(nh, "/relo_odom", 100);
    message_filters::Subscriber<nav_msgs::Odometry> fastlio_odom_sub(nh, "/Odometry", 100);
    typedef message_filters::sync_policies::ExactTime<nav_msgs::Odometry, nav_msgs::Odometry> syncPolicy;
    message_filters::Synchronizer<syncPolicy> sync_odom(syncPolicy(100), relo_odom_sub, fastlio_odom_sub);
    sync_odom.registerCallback(boost::bind(&syncOdomCallback, _1, _2));

    while(true) {

        if(ros_exit_flag) {

            return 0;
        }

        ros::spinOnce();
    }

    return 0;
}

