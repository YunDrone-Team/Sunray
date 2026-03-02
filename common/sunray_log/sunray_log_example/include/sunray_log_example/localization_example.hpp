#ifndef LOCALIZATION_EXAMPLE_HPP
#define LOCALIZATION_EXAMPLE_HPP

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <string>

class LocalizationExample {
  public:
    explicit LocalizationExample(ros::NodeHandle& nh);

    // 从参数服务器读取配置，注册订阅者
    void Init();

  private:
    // 里程计回调：打印位姿并对异常高度告警
    void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg);

    ros::NodeHandle& nh_;
    ros::Subscriber odom_sub_;
    int uav_id_;
    double height_warn_threshold_;  // 高度异常告警阈值（米）
};

#endif  // LOCALIZATION_EXAMPLE_HPP