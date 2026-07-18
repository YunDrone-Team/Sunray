#include <ros/ros.h>

#include <pcl_ros/transforms.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cmath>
#include <cstddef>
#include <string>

namespace {

constexpr double kRadiansToDegrees = 180.0 / 3.14159265358979323846;

bool isFinite(const geometry_msgs::Vector3& value) {
  return std::isfinite(value.x) && std::isfinite(value.y) &&
         std::isfinite(value.z);
}

}  // namespace

class NavRLPointCloudLeveler {
 public:
  NavRLPointCloudLeveler()
      : private_nh_("~"), correction_(Eigen::Matrix4f::Identity()) {
    private_nh_.param<std::string>("imu_topic", imu_topic_, "/livox/imu");
    private_nh_.param<std::string>("input_topic", input_topic_,
                                   "/cloud_registered_body");
    private_nh_.param<std::string>("output_topic", output_topic_,
                                   "/cloud_registered_body_aligned");
    private_nh_.param<std::string>("output_frame_id", output_frame_id_,
                                   "navrl_lidar_aligned");
    private_nh_.param("calibration_samples", calibration_samples_, 200);
    private_nh_.param("minimum_gravity_norm", minimum_gravity_norm_, 1.0);

    if (calibration_samples_ <= 0) {
      ROS_WARN("NavRL point cloud leveler: calibration_samples must be "
               "positive; using 200");
      calibration_samples_ = 200;
    }
    if (minimum_gravity_norm_ <= 0.0) {
      ROS_WARN("NavRL point cloud leveler: minimum_gravity_norm must be "
               "positive; using 1.0 m/s^2");
      minimum_gravity_norm_ = 1.0;
    }

    cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(output_topic_, 2);
    imu_sub_ = nh_.subscribe(imu_topic_, calibration_samples_,
                             &NavRLPointCloudLeveler::imuCallback, this);
    cloud_sub_ = nh_.subscribe(input_topic_, 2,
                               &NavRLPointCloudLeveler::cloudCallback, this);

    ROS_INFO_STREAM("NavRL point cloud leveler waiting for "
                    << calibration_samples_ << " stationary IMU samples on "
                    << imu_topic_);
    ROS_INFO_STREAM("NavRL point cloud leveler: " << input_topic_ << " -> "
                                                   << output_topic_);
  }

 private:
  void imuCallback(const sensor_msgs::ImuConstPtr& msg) {
    if (calibrated_) {
      return;
    }
    if (!isFinite(msg->linear_acceleration)) {
      ROS_WARN_THROTTLE(2.0,
                        "NavRL point cloud leveler: ignoring non-finite IMU "
                        "acceleration");
      return;
    }

    acceleration_sum_.x() += msg->linear_acceleration.x;
    acceleration_sum_.y() += msg->linear_acceleration.y;
    acceleration_sum_.z() += msg->linear_acceleration.z;
    ++sample_count_;

    if (sample_count_ < static_cast<std::size_t>(calibration_samples_)) {
      return;
    }

    const Eigen::Vector3d average =
        acceleration_sum_ / static_cast<double>(sample_count_);
    if (average.norm() < minimum_gravity_norm_) {
      ROS_ERROR_STREAM("NavRL point cloud leveler: average gravity norm "
                       << average.norm() << " m/s^2 is too small; restarting "
                       << "IMU calibration");
      acceleration_sum_.setZero();
      sample_count_ = 0;
      return;
    }

    // Match the leveling convention used by transform_odom_pointCloud:
    // estimate fixed roll/pitch from startup gravity and leave yaw unchanged.
    const double roll = std::atan2(average.y(), average.z());
    const double pitch = -std::atan2(average.x(), average.z());
    const Eigen::Matrix3d rotation =
        (Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()))
            .toRotationMatrix();
    correction_.block<3, 3>(0, 0) = rotation.cast<float>();
    calibrated_ = true;
    imu_sub_.shutdown();

    ROS_INFO_STREAM("NavRL point cloud leveler calibrated: roll="
                    << roll * kRadiansToDegrees
                    << " deg, pitch=" << pitch * kRadiansToDegrees
                    << " deg, gravity_norm=" << average.norm() << " m/s^2");
  }

  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
    if (!calibrated_) {
      ROS_WARN_THROTTLE(
          2.0,
          "NavRL point cloud leveler: dropping point cloud until stationary "
          "IMU calibration completes");
      return;
    }

    sensor_msgs::PointCloud2 output;
    pcl_ros::transformPointCloud(correction_, *msg, output);
    output.header = msg->header;
    if (!output_frame_id_.empty()) {
      output.header.frame_id = output_frame_id_;
    }
    cloud_pub_.publish(output);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber imu_sub_;
  ros::Subscriber cloud_sub_;
  ros::Publisher cloud_pub_;

  std::string imu_topic_;
  std::string input_topic_;
  std::string output_topic_;
  std::string output_frame_id_;
  int calibration_samples_{200};
  double minimum_gravity_norm_{1.0};

  bool calibrated_{false};
  std::size_t sample_count_{0};
  Eigen::Vector3d acceleration_sum_{Eigen::Vector3d::Zero()};
  Eigen::Matrix4f correction_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "navrl_point_cloud_leveler");
  NavRLPointCloudLeveler leveler;
  ros::spin();
  return 0;
}
