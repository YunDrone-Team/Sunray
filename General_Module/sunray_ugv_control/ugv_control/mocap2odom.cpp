#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include "nav_msgs/Odometry.h"

class MocapToOdomBridge {
public:
    MocapToOdomBridge() {}
		// 由于tf变换需要等动捕数据，因此需要手动初始化
		void init(ros::NodeHandle &nh){
        // 订阅动捕系统发布的点
        sub_ = nh.subscribe("/vrpn_client_node_1/ugv1/pose", 10, &MocapToOdomBridge::publish_static_transform, this);
				// 转换后的里程计
				odom_pub_ = nh.advertise<nav_msgs::Odometry>("/mocap2odom",10);

        ROS_INFO("ROS 1 Mocap-Odom Bridge Node Started.");
    }

		// 这个作为动捕里程计的回调函数，同步fastlio与动捕之间的tf
    void publish_static_transform(geometry_msgs::PoseStamped mocap_data_) {
        nav_msgs::Odometry data_;
        data_.pose.pose.position.x = mocap_data_.pose.position.x;
				data_.pose.pose.position.y = mocap_data_.pose.position.y;
				data_.pose.pose.position.z = mocap_data_.pose.position.z;
				data_.pose.pose.orientation.w = mocap_data_.pose.orientation.w;
				data_.pose.pose.orientation.x = mocap_data_.pose.orientation.x;
				data_.pose.pose.orientation.y = mocap_data_.pose.orientation.y;
				data_.pose.pose.orientation.z = mocap_data_.pose.orientation.z;
				odom_pub_.publish(data_);
    }

private:
    
    ros::Subscriber sub_;
		ros::Subscriber target_sub_;
    ros::Publisher odom_pub_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "mocap_odom_bridge");
		ros::NodeHandle nh;
		// 手动初始化订阅与发布者
    MocapToOdomBridge bridge;
		bridge.init(nh);
    ros::spin();
    return 0;
}
