#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <vector>
#include <cmath>
#include <Eigen/Eigen>
#include <nav_msgs/Odometry.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

ros::Publisher odom_pub, pointCloud_pub;
bool calculation_done = false;
Eigen::Vector3d mc_pos = Eigen::Vector3d::Zero();
Eigen::Quaterniond rotation = Eigen::Quaterniond::Identity(), mc_attitude = Eigen::Quaterniond::Identity();

Eigen::Matrix4d poseToTransformMatrix(const geometry_msgs::PoseStamped::ConstPtr &msg) {

    // 提取位置
    Eigen::Vector3d position(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);

    // 提取姿态（四元数）
    Eigen::Quaterniond orientation(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);

    orientation.normalize();

    // 构建4×4变换矩阵
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = orientation.toRotationMatrix();  // 旋转部分
    T.block<3, 1>(0, 3) = position;                        // 平移部分

    return T;
}

Eigen::Matrix4d poseToTransformMatrix(const nav_msgs::Odometry::ConstPtr &msg) {

    // 提取位置
    auto pose = msg->pose.pose;

    Eigen::Vector3d position(pose.position.x, pose.position.y, pose.position.z);
    
    // 提取姿态（四元数）
    Eigen::Quaterniond orientation(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);

    orientation.normalize();
    
    // 构建4×4变换矩阵
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = orientation.toRotationMatrix();  // 旋转部分
    T.block<3, 1>(0, 3) = position;                        // 平移部分
    
    return T;
}

// IMU消息回调函数
void imuCallback(const sensor_msgs::Imu::ConstPtr &msg) {

    static std::vector<double> ax_values, ay_values, az_values;

    // 如果已经完成计算，则忽略后续消息
    if (calculation_done) return;

    // 存储加速度计数据
    ax_values.push_back(msg->linear_acceleration.x);
    ay_values.push_back(msg->linear_acceleration.y);
    az_values.push_back(msg->linear_acceleration.z);

    // 当收集到足够的样本时计算平均值和角度
    if (ax_values.size() >= 200) {

        double sum_ax = 0, sum_ay = 0, sum_az = 0;
        for (size_t i = 0; i < ax_values.size(); ++i) {

            sum_ax += ax_values[i];
            sum_ay += ay_values[i];
            sum_az += az_values[i];
        }
        
        double avg_ax = sum_ax / ax_values.size();
        double avg_ay = sum_ay / ay_values.size();
        double avg_az = sum_az / az_values.size();
        
        // 计算x轴的倾斜角度
        float X_R = atan2(avg_ay, avg_az);
        // 计算y轴的倾斜角度
        float Y_R = -atan2(avg_ax, avg_az);

        // Z轴的旋转为0
        float Z_R = 0;

        rotation = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ()) *
                   Eigen::AngleAxisd(Y_R, Eigen::Vector3d::UnitY()) *
                   Eigen::AngleAxisd(X_R, Eigen::Vector3d::UnitX());

        rotation.normalize();

        // 打印结果
        ROS_INFO("=== IMU Tilt Calculation Results ===");
        ROS_INFO("Collected %zu samples", ax_values.size());
        ROS_INFO("Roll (around X-axis):  %.2f degrees", X_R * 180.0 / M_PI);
        ROS_INFO("Pitch (around Y-axis): %.2f degrees", Y_R * 180.0 / M_PI);
        
        calculation_done = true;
    }
}

void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {

    if (!calculation_done) return;
    
    static Eigen::Matrix4d odom_first = Eigen::Matrix4d::Identity();
    static bool odom_first_flag = true;

    if (odom_first_flag) {

        odom_first = poseToTransformMatrix(msg); 
        odom_first_flag = false;
    } 

    Eigen::Matrix4d odom_current = poseToTransformMatrix(msg);
    Eigen::Matrix4d odom_relative = odom_first.inverse() * odom_current;

    Eigen::Vector3d pos = odom_relative.block<3, 1>(0, 3);
    Eigen::Quaterniond attitude = Eigen::Quaterniond(odom_relative.block<3, 3>(0, 0));

    attitude = rotation * attitude * rotation.inverse();
    tf2::Matrix3x3 m(tf2::Quaternion(attitude.x(), attitude.y(), attitude.z(), attitude.w()));
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    pos = rotation * pos;

    // 发布矫正后的odom
    nav_msgs::Odometry odom_msg;
    odom_msg.header = msg->header;
    odom_msg.header.frame_id = "world";
    odom_msg.pose.pose.orientation.w = attitude.w();
    odom_msg.pose.pose.orientation.x = attitude.x();
    odom_msg.pose.pose.orientation.y = attitude.y();
    odom_msg.pose.pose.orientation.z = attitude.z();
    odom_msg.pose.pose.position.x = pos[0];
    odom_msg.pose.pose.position.y = pos[1];
    odom_msg.pose.pose.position.z = pos[2];
    odom_pub.publish(odom_msg);

    static int index = 0;

    if (index++ % 10 != 0) {

        return;
    } 

    std::cout << "yaw : " << yaw / M_PI * 180   << "  pitch: " << pitch / M_PI * 180   << "  roll: " << roll / M_PI * 180   << "  x: " << pos[0]    << "  y: " << pos[1]    << "  z: " << pos[2] << std::endl;
}
    
void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg) { 

   if (!calculation_done) return;
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *input_cloud);
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    output_cloud->reserve(input_cloud->size());
    
    for (const auto &point : input_cloud->points) {
     
        Eigen::Vector3d rotated_point = rotation * Eigen::Vector3d(point.x, point.y, point.z);
        output_cloud->points.push_back(pcl::PointXYZ(rotated_point.x(), rotated_point.y(), rotated_point.z()));
    }
    
    output_cloud->width = output_cloud->points.size();
    output_cloud->height = 1;
    output_cloud->is_dense = true; 

    sensor_msgs::PointCloud2 pointCloud_msg;
    pcl::toROSMsg(*output_cloud, pointCloud_msg);
    pointCloud_msg.header = msg->header;
    pointCloud_msg.header.frame_id = "world";
    pointCloud_pub.publish(pointCloud_msg);
}

void vrpnCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {

    static Eigen::Matrix4d mc_first = Eigen::Matrix4d::Identity();
    static bool mc_first_flag = true;

    if (mc_first_flag) {

        mc_first = poseToTransformMatrix(msg);
        mc_first_flag = false;
    }

    Eigen::Matrix4d mc_current = poseToTransformMatrix(msg);
    Eigen::Matrix4d mc_relative = mc_first.inverse() * mc_current;
    mc_pos = mc_relative.block<3, 1>(0, 3);
    mc_attitude = Eigen::Quaterniond(mc_relative.block<3, 3>(0, 0));
}

int main(int argc, char** argv) {
    
    ros::init(argc, argv, "transform_odom");
    ros::NodeHandle nh;
    
    ROS_INFO("Starting transform odom and pointCloud");

    ros::Subscriber imu_sub  = nh.subscribe("/livox/imu", 10, imuCallback);
    ros::Subscriber odom_sub = nh.subscribe("/Odometry", 1, odomCallback);
    ros::Subscriber pointCloud_sub = nh.subscribe("/PointCloud", 1, pointCloudCallback);

    odom_pub = nh.advertise<nav_msgs::Odometry>("/sunray/odometry", 1);
    pointCloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/sunray/pointCloud", 1);

    std::cout.setf(std::ios::fixed);
    std::cout << std::setprecision(2);
    std::cout.setf(std::ios::left);
    std::cout.setf(std::ios::showpoint);
    std::cout.setf(std::ios::showpos);

    ros::Rate rate(500); 
    
    while (ros::ok()) {

        ros::spinOnce();
        rate.sleep();
    }
    
    return 0;
}
