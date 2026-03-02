#include "sunray_log_example/localization_example.hpp"
#include "sunray_log.hpp"

LocalizationExample::LocalizationExample(ros::NodeHandle& nh) : nh_(nh), uav_id_(1), height_warn_threshold_(3.0) {}

void LocalizationExample::Init() {
    // 从参数服务器读取配置
    nh_.param<int>("uav_id", uav_id_, 1);
    nh_.param<double>("height_warn_threshold", height_warn_threshold_, 3.0);

    std::string odom_topic = "/uav" + std::to_string(uav_id_) + "/sunray/odometry";

    SUNRAY_INFO("LocalizationExample 初始化: uav_id={}, 订阅话题={}", uav_id_, odom_topic);
    SUNRAY_WARN("高度告警阈值: {:.2f} m", height_warn_threshold_);

    odom_sub_ = nh_.subscribe(odom_topic, 10, &LocalizationExample::OdomCallback, this);

    SUNRAY_DEBUG("订阅者注册完成，等待里程计数据...");
}

void LocalizationExample::OdomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    double z = msg->pose.pose.position.z;

    double vx = msg->twist.twist.linear.x;
    double vy = msg->twist.twist.linear.y;
    double vz = msg->twist.twist.linear.z;

    // DEBUG 级别打印详细位姿（仅写入文件，不显示在控制台）
    SUNRAY_DEBUG("位姿: x={:.3f}  y={:.3f}  z={:.3f} | 速度: vx={:.3f}  vy={:.3f}  vz={:.3f}", x, y, z, vx, vy, vz);

    // INFO 级别打印简略位置（控制台和文件都显示）
    SUNRAY_INFO("uav{} 位置: ({:.3f}, {:.3f}, {:.3f})", uav_id_, x, y, z);

    // 高度异常告警
    if (z > height_warn_threshold_) {
        SUNRAY_WARN("uav{} 高度异常: z={:.3f} m 超过阈值 {:.2f} m", uav_id_, z, height_warn_threshold_);
    }

    // 位置数据无效检查
    if (std::isnan(x) || std::isnan(y) || std::isnan(z)) {
        SUNRAY_ERROR("uav{} 收到无效里程计数据(NaN)", uav_id_);
    }
}
