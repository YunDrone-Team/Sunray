#include <ros/ros.h>
#include <iostream>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>

ros::Publisher cloud_pub;
ros::Publisher odom_pub;

pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;

void subCallback(const nav_msgs::OdometryConstPtr &odom_ptr, const sensor_msgs::PointCloud2ConstPtr &pointCloud_ptr) {

    Eigen::Quaterniond q(odom_ptr->pose.pose.orientation.w, odom_ptr->pose.pose.orientation.x, odom_ptr->pose.pose.orientation.y, odom_ptr->pose.pose.orientation.z);

    // 绕 Z 轴旋转 90°
    Eigen::Quaterniond q_z(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()));

    // 绕 Y 轴旋转 -90°
    Eigen::Quaterniond q_y(Eigen::AngleAxisd(-M_PI / 2, Eigen::Vector3d::UnitY()));

    // 组合旋转（顺序：先 q_z，再 q_y）
    q = q * q_z * q_y;

    Eigen::Vector3d t(odom_ptr->pose.pose.position.x,
                      odom_ptr->pose.pose.position.y,
                      odom_ptr->pose.pose.position.z);

    // 转换相机坐标系（z 轴朝前 -> x 轴朝前）
    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    transform << 0, 0, 1, 0,
            -1, 0, 0, 0,
            0, -1, 0, 0,
            0, 0, 0, 1;

    Eigen::Matrix4d odom = Eigen::Matrix4d::Identity();
    odom.block<3, 3>(0, 0) = q.toRotationMatrix();
    odom.block<3, 1>(0, 3) = t;

    transform = odom * transform;

    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	
    pcl::fromROSMsg(*pointCloud_ptr, *input_cloud);
    voxel_filter.setLeafSize(0.1f, 0.1f, 0.1f);
    voxel_filter.setInputCloud(input_cloud);
    voxel_filter.filter(*transformed_cloud);

    for(const auto &pt : *transformed_cloud) {

        if(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z < 0.3 * 0.3) {

            continue;
        }

        output_cloud->emplace_back(pt);
    }

    pcl::transformPointCloud(*output_cloud, *output_cloud, transform);

    sensor_msgs::PointCloud2 output_cloud_ros;
    pcl::toROSMsg(*output_cloud, output_cloud_ros);
    output_cloud_ros.header = odom_ptr->header;

   nav_msgs::Odometry odom_ros;
   odom_ros.pose.pose.position.x = t.x();
   odom_ros.pose.pose.position.y = t.y();
   odom_ros.pose.pose.position.z = t.z();
   odom_ros.pose.pose.orientation.x = q.x();
   odom_ros.pose.pose.orientation.y = q.y();
   odom_ros.pose.pose.orientation.z = q.z();
   odom_ros.pose.pose.orientation.w = q.w();
   odom_ros.header = odom_ptr->header;

   odom_pub.publish(odom_ros);
    cloud_pub.publish(output_cloud_ros);
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "viobot");
    ros::NodeHandle nh;

    message_filters::Subscriber<nav_msgs::Odometry>  odom_sub(nh, "/baton/stereo3/odometry", 10);
    message_filters::Subscriber<sensor_msgs::PointCloud2>  pointCloud_sub(nh, "/berxel_camera/depth/berxel_cloudpoint", 10);
    typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::PointCloud2> syncPolicy;
    message_filters::Synchronizer<syncPolicy> sync_odom(syncPolicy(10), odom_sub, pointCloud_sub);
    sync_odom.registerCallback(boost::bind(&subCallback, _1, _2));

    cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/berxel_pointCloud", 1);
    odom_pub = nh.advertise<nav_msgs::Odometry>("/berxel_odom", 1);
    ros::spin();

    return 0;
}
