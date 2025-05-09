#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <livox_ros_driver2/CustomMsg.h>
#include <livox_ros_driver2/CustomPoint.h>
#include <sensor_msgs/Imu.h>
#include <Eigen/Eigen>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <iostream>
#include <bitset>
#include <visualization_msgs/Marker.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_datatypes.h>

ros::Publisher pointcloud_pub, odom_pub;
float filter_distance;

sensor_msgs::PointCloud2 livox2pointcloud(const livox_ros_driver2::CustomMsg::ConstPtr &livox_msg)
{
    sensor_msgs::PointCloud2 cloud_msg;
    cloud_msg.header.frame_id = livox_msg->header.frame_id;
    cloud_msg.header.stamp = livox_msg->header.stamp;
    cloud_msg.header.seq = livox_msg->header.seq;
    cloud_msg.height = 1;
    cloud_msg.width = livox_msg->points.size();
    cloud_msg.fields.resize(7);
    cloud_msg.fields[0].offset = 0;
    cloud_msg.fields[0].name = "offset_time";
    cloud_msg.fields[0].count = 1;
    cloud_msg.fields[0].datatype = sensor_msgs::PointField::UINT32;
    cloud_msg.fields[1].offset = 4;
    cloud_msg.fields[1].name = "x";
    cloud_msg.fields[1].count = 1;
    cloud_msg.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
    cloud_msg.fields[2].offset = 8;
    cloud_msg.fields[2].name = "y";
    cloud_msg.fields[2].count = 1;
    cloud_msg.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
    cloud_msg.fields[3].offset = 12;
    cloud_msg.fields[3].name = "z";
    cloud_msg.fields[3].count = 1;
    cloud_msg.fields[3].datatype = sensor_msgs::PointField::FLOAT32;
    cloud_msg.fields[4].offset = 16;
    cloud_msg.fields[4].name = "intensity";
    cloud_msg.fields[4].count = 1;
    cloud_msg.fields[4].datatype = sensor_msgs::PointField::FLOAT32;
    cloud_msg.fields[5].offset = 20;
    cloud_msg.fields[5].name = "tag";
    cloud_msg.fields[5].count = 1;
    cloud_msg.fields[5].datatype = sensor_msgs::PointField::UINT8;
    cloud_msg.fields[6].offset = 21;
    cloud_msg.fields[6].name = "line";
    cloud_msg.fields[6].count = 1;
    cloud_msg.fields[6].datatype = sensor_msgs::PointField::UINT8;
    cloud_msg.point_step = sizeof(livox_ros_driver2::CustomPoint);
    cloud_msg.row_step = cloud_msg.width * cloud_msg.point_step;
    cloud_msg.data.resize(cloud_msg.row_step);

    sensor_msgs::PointCloud2Iterator<uint32_t> iter_offset_time(cloud_msg, "offset_time");
    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");
    sensor_msgs::PointCloud2Iterator<float> iter_intensity(cloud_msg, "intensity");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_tag(cloud_msg, "tag");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_line(cloud_msg, "line");
    size_t filtered_points = 0;
    for (const auto &livox_p : livox_msg->points)
    {
        // 把x y z小于filter_distance的点过滤掉
        if (abs(livox_p.x) < filter_distance && abs(livox_p.y) < filter_distance && abs(livox_p.z) < filter_distance)
            continue;
        *iter_offset_time = livox_p.offset_time;
        *iter_x = livox_p.x;
        *iter_y = livox_p.y;
        *iter_z = livox_p.z;
        *iter_intensity = livox_p.reflectivity;
        *iter_tag = livox_p.tag;
        *iter_line = livox_p.line;

        ++iter_offset_time;
        ++iter_x;
        ++iter_y;
        ++iter_z;
        ++iter_intensity;
        ++iter_tag;
        ++iter_line;
        ++filtered_points;
    }
    // 重新计算长度
    cloud_msg.data.resize(filtered_points * cloud_msg.point_step);
    cloud_msg.width = filtered_points;
    return cloud_msg;
}

// Callback function for custom message subscriber
void customMsgCallback(const livox_ros_driver2::CustomMsg::ConstPtr &livox_msg)
{
    pointcloud_pub.publish(livox2pointcloud(livox_msg));
    // ROS_INFO_STREAM("Converted livox_ros_driver2::CustomMsg to sensor_msgs::PointClouds2");
}

std::vector<sensor_msgs::Imu> imu_buffer;
Eigen::Matrix3d rotation_matrix;
Eigen::Vector4d q_rotation;
float x_R, y_R;
int imu_buffer_size = 0;
bool init = false;

// 计算重力方向和xy的倾斜角度
void calculateGravityAndTilt()
{
    if (imu_buffer_size >= 10 && !init)
    {
        // 计算平均值
        float sum_x = 0, sum_y = 0, sum_z = 0;
        for (const auto &imu : imu_buffer)
        {
            sum_x += imu.linear_acceleration.x;
            sum_y += imu.linear_acceleration.y;
            sum_z += imu.linear_acceleration.z;
        }
        float avg_x = sum_x / imu_buffer_size;
        float avg_y = sum_y / imu_buffer_size;
        float avg_z = sum_z / imu_buffer_size;
        // 计算重力方向
        float gravity = sqrt(avg_x * avg_x + avg_y * avg_y + avg_z * avg_z);
        // 计算x轴的倾斜角度
        float X_R = atan2(avg_y, avg_z);
        // 计算y轴的倾斜角度
        float Y_R = -atan2(avg_x, avg_z);
        x_R = X_R;
        y_R = Y_R;
        // Z轴的旋转为0
        float Z_R = 0;
        // 生成四元素
        q_rotation = Eigen::Vector4d(cos(X_R / 2) * cos(-Y_R / 2) * cos(Z_R / 2) + sin(X_R / 2) * sin(-Y_R / 2) * sin(Z_R / 2),
                                        sin(X_R / 2) * cos(-Y_R / 2) * cos(Z_R / 2) - cos(X_R / 2) * sin(-Y_R / 2) * sin(Z_R / 2),
                                        cos(X_R / 2) * sin(-Y_R / 2) * cos(Z_R / 2) + sin(X_R / 2) * cos(-Y_R / 2) * sin(Z_R / 2),
                                        cos(X_R / 2) * cos(-Y_R / 2) * sin(Z_R / 2) - sin(X_R / 2) * sin(-Y_R / 2) * cos(Z_R / 2));
        // std::cout<<"q_rotation:"<<q_rotation[0]<<q_rotation[1]<<q_rotation[2]<<q_rotation[3]<<std::endl;
        // 生成旋转矩阵
        rotation_matrix = Eigen::AngleAxisd(Z_R, Eigen::Vector3d::UnitZ()) *
                          Eigen::AngleAxisd(Y_R, Eigen::Vector3d::UnitY()) *
                          Eigen::AngleAxisd(X_R, Eigen::Vector3d::UnitX());
        init = true;
    }
}

Eigen::Vector4d quaternion_multiply(const Eigen::Vector4d& q1, const Eigen::Vector4d& q2) {
    Eigen::Vector4d q3;
    q3[0] = q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3];
    q3[1] = q1[0] * q2[1] + q1[1] * q2[0] + q1[2] * q2[3] - q1[3] * q2[2];
    q3[2] = q1[0] * q2[2] - q1[1] * q2[3] + q1[2] * q2[0] + q1[3] * q2[1];
    q3[3] = q1[0] * q2[3] + q1[1] * q2[2] - q1[2] * q2[1] + q1[3] * q2[0];
    return q3;
}

Eigen::Vector4d quaternion_inverse(const Eigen::Vector4d& q) {
    return Eigen::Vector4d(q[0], -q[1], -q[2], -q[3]);
}

Eigen::Vector4d transform_quaternion(const Eigen::Vector4d& q1, const Eigen::Vector4d& q2) {
    Eigen::Vector4d q1_inv = quaternion_inverse(q1);
    return quaternion_multiply(q1_inv, q2);
}

void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    if (!init)
    {
        calculateGravityAndTilt();
    }
    else
    {
        Eigen::Vector4d q_odom(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
        Eigen::Vector4d q3 = transform_quaternion(q_rotation, q_odom);
        double roll_1 = atan2(2 * (q3[0] * q3[1] + q3[2] * q3[3]), 1 - 2 * (q3[1] * q3[1] + q3[2] * q3[2]));
        double pitch_1 = asin(2 * (q3[0] * q3[2] - q3[3] * q3[1]));
        double yaw_1 = atan2(2 * (q3[0] * q3[3] + q3[1] * q3[2]), 1 - 2 * (q3[2] * q3[2] + q3[3] * q3[3]));

        double roll_2 = roll_1 - x_R;
        double pitch_2 = pitch_1 - y_R;
        double yaw_2 = yaw_1;
        Eigen::Vector4d q4(cos(roll_2 / 2) * cos(pitch_2 / 2) * cos(yaw_2 / 2) + sin(roll_2 / 2) * sin(pitch_2 / 2) * sin(yaw_2 / 2),
                            sin(roll_2 / 2) * cos(pitch_2 / 2) * cos(yaw_2 / 2) - cos(roll_2 / 2) * sin(pitch_2 / 2) * sin(yaw_2 / 2),
                            cos(roll_2 / 2) * sin(pitch_2 / 2) * cos(yaw_2 / 2) + sin(roll_2 / 2) * cos(pitch_2 / 2) * sin(yaw_2 / 2),
                            cos(roll_2 / 2) * cos(pitch_2 / 2) * sin(yaw_2 / 2) - sin(roll_2 / 2) * sin(pitch_2 / 2) * cos(yaw_2 / 2));
        
        Eigen::Quaterniond q(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
        // 四元素转欧拉角
        tf::Quaternion tf_q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        // tf::Matrix3x3 m(tf_q);
        // double roll, pitch, yaw;
        // m.getRPY(roll, pitch, yaw);
        // std::cout << "yaw_O: " << yaw / M_PI * 180 << " pitch_O: " << pitch / M_PI * 180 << " roll_O: " << roll / M_PI * 180 << std::endl;

        // tf::Quaternion tf_q_c(q4[1], q4[2], q4[3], q4[0]);
        // tf::Matrix3x3 m_(tf_q_c);
        // m_.getRPY(roll, pitch, yaw);
        // std::cout << "yaw_C: " << yaw / M_PI * 180 << " pitch_C: " << pitch / M_PI * 180 << " roll_C: " << roll / M_PI * 180 << std::endl;
        
        // 计算平移
        // Eigen::Matrix3d R = q_rotation.toRotationMatrix();
        // // 更新旋转矩阵
        // Eigen::Matrix3d rotation = R * rotation_matrix;
        Eigen::Vector3d t(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
        // 平移矫正
        t = rotation_matrix * t;
        // std::cout<<"**********"<<std::endl;
        // std::cout<<msg->pose.pose.position.x<<" "<< msg->pose.pose.position.y<<" "<< msg->pose.pose.position.z<<std::endl;
        // std::cout<<"x:"<<t[0]<<std::endl;
        // std::cout<<"y:"<<t[1]<<std::endl;
        // std::cout<<"z:"<<t[2]<<std::endl;
        // 发布矫正后的odom
        nav_msgs::Odometry odom_msg_convert;
        odom_msg_convert.header = msg->header;
        odom_msg_convert.pose.pose.orientation.w = q4[0];
        odom_msg_convert.pose.pose.orientation.x = q4[1];
        odom_msg_convert.pose.pose.orientation.y = q4[2];
        odom_msg_convert.pose.pose.orientation.z = q4[3];
        odom_msg_convert.pose.pose.position.x = t[0];
        odom_msg_convert.pose.pose.position.y = t[1];
        odom_msg_convert.pose.pose.position.z = t[2];
        odom_pub.publish(odom_msg_convert);

    }
}

void imuCallback(const sensor_msgs::Imu::ConstPtr &imu_msg)
{
    if (imu_buffer_size < 10)
    {
        imu_buffer.push_back(*imu_msg);
        imu_buffer_size++;
    }
    // else
    // {
    //      calculateGravityAndTilt();
    // }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "livox2pointcloud_node");
    ros::NodeHandle nh("~"); // Private node handle to access private parameters

    std::string pointcloud_topic;
    std::string livox_topic;
    std::string imu_topic;
    std::string odom_topic_sub, odom_topic_pub;
    bool odom_convert;
    nh.param<std::string>("pointcloud_topic", pointcloud_topic, "/livox/pointcloud2");
    nh.param<std::string>("livox_topic", livox_topic, "/livox/lidar");
    nh.param<std::string>("imu_topic", imu_topic, "/livox/imu");
    nh.param<std::string>("odom_topic_sub", odom_topic_sub, "/Odometry");
    nh.param<std::string>("odom_topic_pub", odom_topic_pub, "/Odometry_convert");
    nh.param<float>("filter_distance", filter_distance, 0.5);
    nh.param<bool>("odom_convert", odom_convert, false);
    // filter_distance = filter_distance*filter_distance;
    pointcloud_pub = nh.advertise<sensor_msgs::PointCloud2>(pointcloud_topic, 10);
    odom_pub = nh.advertise<nav_msgs::Odometry>(odom_topic_pub, 10);
    ros::Subscriber custom_msg_sub, imu_sub, odom_sub;
    custom_msg_sub = nh.subscribe<livox_ros_driver2::CustomMsg>(livox_topic, 10, customMsgCallback);
    if (odom_convert)
    {
        imu_sub = nh.subscribe<sensor_msgs::Imu>(imu_topic, 10, imuCallback);
        odom_sub = nh.subscribe<nav_msgs::Odometry>(odom_topic_sub, 10, odomCallback);
    }

    std::cout.setf(std::ios::fixed);
    std::cout << std::setprecision(2);
    std::cout.setf(std::ios::left);
    std::cout.setf(std::ios::showpoint);
    std::cout.setf(std::ios::showpos);

    ros::spin();

    return 0;
}
