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
    ros::NodeHandle n;
    laser_geometry::LaserProjection projector_;
    tf::TransformListener listener_;
    message_filters::Subscriber<sensor_msgs::LaserScan>* laser_sub_;
    tf::MessageFilter<sensor_msgs::LaserScan>* laser_notifier_;
    ros::Publisher cloud_pub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber pos_sub_;
    tf::TransformBroadcaster br_;
    tf::StampedTransform transform_;

    int uav_id;
    int pos_odom;      // 0: position, 1: odom
    std::string uav_name{""};
    std::string scan_topic_{""};
    std::string odom_topic_{""};
    std::string pos_topic_{""};
    std::string cloud_topic_{""};
    std::string world_frame_{""};
    std::string base_frame_{""};

    void init(ros::NodeHandle n)
    {
        n.param<int>("uav_id", uav_id, 1);
        n.param<int>("pos_odom", pos_odom, 0);
        n.param<std::string>("uav_name", uav_name, "uav");
        n.param<std::string>("scan_topic", scan_topic_, "/scan");
        n.param<std::string>("cloud_topic", cloud_topic_, "/cloud");
        n.param<std::string>("odom_topic", odom_topic_, "/odometry");
        n.param<std::string>("pos_topic", pos_topic_, "/position");
        n.param<std::string>("world_frame", world_frame_, "world");
        n.param<std::string>("base_frame", base_frame_, "base_link");
        std::cout << "pos_odom: " << pos_odom << std::endl;
        std::cout << "pos_topic: " << pos_topic_ << std::endl;
        std::cout << "odom_topic: " << odom_topic_ << std::endl;
        laser_sub_ = new message_filters::Subscriber<sensor_msgs::LaserScan>(n, scan_topic_, 1);
        laser_notifier_ = new tf::MessageFilter<sensor_msgs::LaserScan>(*laser_sub_, listener_, world_frame_, 1);
        laser_notifier_->registerCallback(
            boost::bind(&LaserScanToPointCloud::scanCallback, this, _1));
        laser_notifier_->setTolerance(ros::Duration(0.1)); // 0.01
        cloud_pub_ = n.advertise<sensor_msgs::PointCloud2>(cloud_topic_, 10);
        if(pos_odom == 1)
        {
            odom_sub_ = n.subscribe(odom_topic_, 10, &LaserScanToPointCloud::odomCallback, this);
        }
        else if(pos_odom == 0)
        {
            pos_sub_ = n.subscribe(pos_topic_, 10, &LaserScanToPointCloud::posCallback, this);
        }
        
    }

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_in)
    {
        sensor_msgs::PointCloud2 cloud;
        try
        {
            projector_.transformLaserScanToPointCloud(
                world_frame_, *scan_in, cloud, listener_);
        }
        catch (tf::TransformException &e)
        {
            std::cout << e.what();
            return;
        }
        cloud_pub_.publish(cloud);
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg) {
        // std::cout<<"odomCallback"<<std::endl;
        transform_.stamp_ = odom_msg->header.stamp;
        transform_.frame_id_ = world_frame_;
        transform_.child_frame_id_ = base_frame_;
        transform_.setOrigin(tf::Vector3(odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y, odom_msg->pose.pose.position.z));
        transform_.setRotation(tf::Quaternion(odom_msg->pose.pose.orientation.x, odom_msg->pose.pose.orientation.y, odom_msg->pose.pose.orientation.z, odom_msg->pose.pose.orientation.w));
        br_.sendTransform(transform_);
    }

    void posCallback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg) {
        // std::cout<<"posCallback"<<std::endl;
        transform_.stamp_ = pose_msg->header.stamp;
        transform_.frame_id_ = world_frame_;
        transform_.child_frame_id_ = base_frame_;
        transform_.setOrigin(tf::Vector3(pose_msg->pose.position.x, pose_msg->pose.position.y, pose_msg->pose.position.z));
        transform_.setRotation(tf::Quaternion(pose_msg->pose.orientation.x, pose_msg->pose.orientation.y, pose_msg->pose.orientation.z, pose_msg->pose.orientation.w));
        br_.sendTransform(transform_);
    }

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "scan_to_cloud");
    // ros::NodeHandle n;
    ros::NodeHandle n_priv("~");
    double freq;
    n_priv.param<double>("frequency", freq, 50.0);
    ros::Rate loop_rate(freq);
    
    LaserScanToPointCloud lstopc;
    lstopc.init(n_priv);

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}