#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "ros/ros.h"

ros::Subscriber sub_;
ros::Publisher pub_;

void PointCallback(geometry_msgs::PointStamped data_);

int main(int argc, char** argv) {
    // 初始化ros节点
    ros::init(argc, argv, "uav_ugv_bridge");
    ros::NodeHandle nh;
    // 注册订阅者与发布者
    sub_ = nh.subscribe<geometry_msgs::PointStamped>("/target_global_position",
                                                     1, &PointCallback);
    pub_ = nh.advertise<geometry_msgs::PoseStamped>("/goal_1", 1);
    ros::spin();
    return 0;
}

void PointCallback(geometry_msgs::PointStamped data_) {
    geometry_msgs::PoseStamped data;
    data.header.frame_id = "world";
    data.pose.position.x = data_.point.x;
    data.pose.position.y = data_.point.y;
    data.pose.position.z = data_.point.z;
    pub_.publish(data);
}
