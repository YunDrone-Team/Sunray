#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include <boost/bind.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

class FuelPointCloudFovFilter {
public:
  explicit FuelPointCloudFovFilter(ros::NodeHandle& private_nh) : private_nh_(private_nh) {
    private_nh_.param<std::string>("input_cloud_topic", input_cloud_topic_, "/cloud_registered");
    private_nh_.param<std::string>("odom_topic", odom_topic_, "/Odometry");
    private_nh_.param<std::string>(
        "output_cloud_topic", output_cloud_topic_, "/fuel/cloud_registered_fov");
    private_nh_.param<double>("left_angle", left_angle_, 0.69222);
    private_nh_.param<double>("right_angle", right_angle_, 0.68901);
    private_nh_.param<int>("sync_queue_size", sync_queue_size_, 10);

    constexpr double kPi = 3.14159265358979323846;
    if (left_angle_ <= 0.0 || left_angle_ > kPi || right_angle_ <= 0.0 ||
        right_angle_ > kPi) {
      throw std::invalid_argument("left_angle and right_angle must be in (0, pi]");
    }
    if (sync_queue_size_ <= 0) {
      throw std::invalid_argument("sync_queue_size must be positive");
    }

    cloud_pub_ = private_nh_.advertise<sensor_msgs::PointCloud2>(output_cloud_topic_, 1);
    cloud_sub_.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(
        private_nh_, input_cloud_topic_, sync_queue_size_));
    odom_sub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(
        private_nh_, odom_topic_, sync_queue_size_));
    synchronizer_.reset(new message_filters::TimeSynchronizer<sensor_msgs::PointCloud2,
                                                               nav_msgs::Odometry>(
        *cloud_sub_, *odom_sub_, sync_queue_size_));
    synchronizer_->registerCallback(
        boost::bind(&FuelPointCloudFovFilter::filterCallback, this, _1, _2));

    ROS_INFO_STREAM("FUEL point cloud FOV filter: cloud=" << input_cloud_topic_
                                                           << ", odom=" << odom_topic_
                                                           << ", output=" << output_cloud_topic_
                                                           << ", left=" << left_angle_
                                                           << " rad, right=" << right_angle_
                                                           << " rad");
  }

private:
  void filterCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg,
                      const nav_msgs::OdometryConstPtr& odom_msg) {
    pcl::PointCloud<pcl::PointXYZI> input_cloud;
    pcl::fromROSMsg(*cloud_msg, input_cloud);

    pcl::PointCloud<pcl::PointXYZI> output_cloud;
    output_cloud.header = input_cloud.header;
    output_cloud.points.reserve(input_cloud.points.size());

    const auto& pose = odom_msg->pose.pose;
    const double q_norm_sq = pose.orientation.x * pose.orientation.x +
                             pose.orientation.y * pose.orientation.y +
                             pose.orientation.z * pose.orientation.z +
                             pose.orientation.w * pose.orientation.w;
    if (q_norm_sq < 1e-12) {
      ROS_WARN_THROTTLE(1.0, "Drop point cloud because odometry quaternion is invalid.");
      return;
    }

    const double sin_yaw =
        2.0 * (pose.orientation.w * pose.orientation.z +
               pose.orientation.x * pose.orientation.y) /
        q_norm_sq;
    const double cos_yaw =
        1.0 - 2.0 * (pose.orientation.y * pose.orientation.y +
                     pose.orientation.z * pose.orientation.z) /
                  q_norm_sq;
    const double yaw = std::atan2(sin_yaw, cos_yaw);
    const double heading_cos = std::cos(yaw);
    const double heading_sin = std::sin(yaw);

    for (const auto& point : input_cloud.points) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
        continue;
      }

      const double dx = point.x - pose.position.x;
      const double dy = point.y - pose.position.y;
      const double forward = heading_cos * dx + heading_sin * dy;
      const double left = -heading_sin * dx + heading_cos * dy;
      const double horizontal_angle = std::atan2(left, forward);

      if (horizontal_angle >= -right_angle_ && horizontal_angle <= left_angle_) {
        output_cloud.points.push_back(point);
      }
    }

    output_cloud.width = static_cast<uint32_t>(output_cloud.points.size());
    output_cloud.height = 1;
    output_cloud.is_dense = true;

    sensor_msgs::PointCloud2 output_msg;
    pcl::toROSMsg(output_cloud, output_msg);
    output_msg.header = cloud_msg->header;
    cloud_pub_.publish(output_msg);

    ROS_DEBUG_THROTTLE(1.0, "FUEL FOV filter kept %zu of %zu points.",
                       output_cloud.points.size(), input_cloud.points.size());
  }

  ros::NodeHandle private_nh_;
  ros::Publisher cloud_pub_;
  std::unique_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> cloud_sub_;
  std::unique_ptr<message_filters::Subscriber<nav_msgs::Odometry>> odom_sub_;
  std::unique_ptr<message_filters::TimeSynchronizer<sensor_msgs::PointCloud2,
                                                    nav_msgs::Odometry>>
      synchronizer_;

  std::string input_cloud_topic_;
  std::string odom_topic_;
  std::string output_cloud_topic_;
  double left_angle_;
  double right_angle_;
  int sync_queue_size_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "fuel_pointcloud_fov_filter");
  ros::NodeHandle private_nh("~");

  try {
    FuelPointCloudFovFilter filter(private_nh);
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL_STREAM("Failed to start FUEL point cloud FOV filter: " << error.what());
    return 1;
  }

  return 0;
}
