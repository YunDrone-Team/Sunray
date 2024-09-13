#include <ros/ros.h>
#include <ros/console.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include "ros_msg_utils.h"

class PointCloudTransformer {
public:
    PointCloudTransformer(ros::NodeHandle& nh) : nh_(nh){
        odom_sub_ = nh_.subscribe("/uav1/sunray/gazebo_pose", 10, &PointCloudTransformer::odomCallback, this);
        // Create a ROS subscriber for the input point cloud
        sub_ = nh_.subscribe("/velodyne_points", 1, &PointCloudTransformer::cloudCallback, this);

        // Create a ROS publisher for the output point cloud
        pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/pointcloud_world", 1);
        pos_cmd_sub_ =  nh_.subscribe("/position_cmd", 1, &PointCloudTransformer::position_command_cb, this);
        cmd_pub_ = nh_.advertise<sunray_msgs::UAVControlCMD>("/uav1/sunray/uav_control_cmd",1);
    }

    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& input) {
        // Create a container for the data.
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);

        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(*input, pcl_pc2);
        pcl::fromPCLPointCloud2(pcl_pc2, *temp_cloud);

        pcl_ros::transformPointCloud(*temp_cloud, *cloud_transformed, transform_);

        sensor_msgs::PointCloud2 cloud_publish;
        pcl::toROSMsg(*cloud_transformed, cloud_publish);
        cloud_publish.header = input->header;
        cloud_publish.header.frame_id = "world";

        pub_.publish(cloud_publish);
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg) {
        tf::StampedTransform transform;
        transform.stamp_ = odom_msg->header.stamp;
        transform.frame_id_ = "world";
        transform.child_frame_id_ = "base_link";
        transform.setOrigin(tf::Vector3(odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y, odom_msg->pose.pose.position.z));
        transform.setRotation(tf::Quaternion(odom_msg->pose.pose.orientation.x, odom_msg->pose.pose.orientation.y, odom_msg->pose.pose.orientation.z, odom_msg->pose.pose.orientation.w));
        br_.sendTransform(transform);
    }

    void run() {
        tf::TransformListener listener(nh_);
        ros::Rate rate(10.0);
        while (nh_.ok()) {
            try {
                listener.lookupTransform("world", "base_link", ros::Time(), transform_);
            } catch (tf::TransformException ex) {
                ROS_WARN("%s", ex.what());
            }

            ros::spinOnce();
            rate.sleep();
        }

        ros::spin();
    }

    void position_command_cb(const sunray_msgs::PositionCommand::ConstPtr& msg) {
        double x = msg->position.x;
        double y = msg->position.y;
        double z = msg->position.z;
        double yaw = msg->yaw;
        cmd_.cmd = 4;
        cmd_.desired_pos[0] = x;
        cmd_.desired_pos[1] = y;
        cmd_.desired_pos[2] = z;
        cmd_.enable_yawRate = false;
        cmd_.desired_yaw = yaw;
        cmd_pub_.publish(cmd_);
        }

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber pos_cmd_sub_;
    tf::TransformBroadcaster br_;
    tf::StampedTransform transform_;
    ros::Publisher cmd_pub_;
    sunray_msgs::UAVControlCMD cmd_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "point_cloud_transform");
    ros::NodeHandle nh;
    PointCloudTransformer transformer(nh);
    transformer.run();
    return 0;
}