#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Dense>

class PointCloudAligner {
private:
    ros::NodeHandle nh;
    ros::Subscriber sub_mocap;
    ros::Subscriber sub_cloud;
    ros::Publisher pub_cloud;

    Eigen::Matrix4d T_world_odom;
    bool is_initialized = false;

public:
    PointCloudAligner() {
        // 订阅动捕真值，获取初始位姿
        sub_mocap = nh.subscribe("/vrpn_client_node_1/ugv1/pose", 1, &PointCloudAligner::mocapCallback, this);
        // 订阅 FAST-LIO 输出的点云
        sub_cloud = nh.subscribe("/cloud_registered", 1, &PointCloudAligner::cloudCallback, this);
        // 发布给 EGO-Planner 的转换后点云
        pub_cloud = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered_aligned", 1);
        
        T_world_odom.setIdentity();
    }

    // 只在第一帧捕获初始位姿作为坐标系偏移
    void mocapCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        if (!is_initialized) {
            Eigen::Quaterniond q(msg->pose.orientation.w, 
                                 msg->pose.orientation.x, 
                                 msg->pose.orientation.y, 
                                 msg->pose.orientation.z);
            Eigen::Vector3d t(msg->pose.position.x, 
                              msg->pose.position.y, 
                              msg->pose.position.z);

            T_world_odom.block<3, 3>(0, 0) = q.toRotationMatrix();
            T_world_odom.block<3, 1>(0, 3) = t;
            
            is_initialized = true;
            ROS_INFO("Alignment Initialized: Mocap Origin set at [%.2f, %.2f, %.2f]", t.x(), t.y(), t.z());
        }
    }

    void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        if (!is_initialized) return;

        // 1. 将 ROS 消息转换为 PCL 点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

        // 2. 执行坐标变换: P_world = T_world_odom * P_fastlio
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_aligned(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud(*cloud, *cloud_aligned, T_world_odom.cast<float>());

        // 3. 转换回 ROS 消息并发布
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(*cloud_aligned, output);
        output.header = msg->header;
        output.header.frame_id = "world"; // 强制设置为动捕坐标系名称
        pub_cloud.publish(output);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "pointcloud_aligner");
    PointCloudAligner aligner;
    ros::spin();
    return 0;
}