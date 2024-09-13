#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/PointCloud2.h"
#include <nav_msgs/Odometry.h>
#include "tf/message_filter.h"
#include <tf/transform_broadcaster.h>
#include "message_filters/subscriber.h"
#include "laser_geometry/laser_geometry.h"

class LaserScanToPointCloud
{
public:
    ros::NodeHandle n_;
    laser_geometry::LaserProjection projector_;
    tf::TransformListener listener_;
    message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_;
    tf::MessageFilter<sensor_msgs::LaserScan> laser_notifier_;
    ros::Publisher scan_pub_;
    ros::Subscriber odom_sub_;
    tf::TransformBroadcaster br_;
    tf::StampedTransform transform_;

    LaserScanToPointCloud(ros::NodeHandle n) : n_(n),
                                               laser_sub_(n_, "/scan", 1),
                                               laser_notifier_(laser_sub_, listener_, "world", 1)
    {
        laser_notifier_.registerCallback(
            boost::bind(&LaserScanToPointCloud::scanCallback, this, _1));
        laser_notifier_.setTolerance(ros::Duration(0.01)); // 0.01
        scan_pub_ = n_.advertise<sensor_msgs::PointCloud2>("/pointcloud_world", 10);
        odom_sub_ = n_.subscribe("/uav1/sunray/gazebo_pose", 10, &LaserScanToPointCloud::odomCallback, this);
    }

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_in)
    {
        sensor_msgs::PointCloud2 cloud;
        try
        {
            projector_.transformLaserScanToPointCloud(
                "world", *scan_in, cloud, listener_);
        }
        catch (tf::TransformException &e)
        {
            std::cout << e.what();
            return;
        }
        scan_pub_.publish(cloud);
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
};

int main(int argc, char **argv)
{

    ros::init(argc, argv, "my_scan_to_cloud");
    ros::NodeHandle n;
    ros::NodeHandle n_priv("~");
    double freq;
    n_priv.param<double>("frequency", freq, 50.0);
    ros::Rate loop_rate(freq);

    LaserScanToPointCloud lstopc(n);

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}