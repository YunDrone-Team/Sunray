#include <ros/ros.h>
#include "ros_msg_utils.h"
#include "printf_format.h"

using namespace sunray_logger;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test");
    ros::NodeHandle nh("~");
    ros::Rate rate(20); // 20Hz

    while (ros::ok())
    {

        // 发布TF用于RVIZ显示（用于sensor显示）
        static tf2_ros::TransformBroadcaster broadcaster;
        geometry_msgs::TransformStamped tfs;
        //  世界坐标系
        tfs.header.frame_id = "world";      
        tfs.header.stamp = ros::Time::now();
        //  局部坐标系（如传感器坐标系）
        tfs.child_frame_id = "/base_link"; 
        //  坐标系相对信息设置，局部坐标系相对于世界坐标系的坐标（偏移量）
        tfs.transform.translation.x = 0.0;
        tfs.transform.translation.y = 0.0;
        tfs.transform.translation.z = 0.0;
        tfs.transform.rotation.x = 0.0;
        tfs.transform.rotation.y = 0.0;
        tfs.transform.rotation.z = 0.0;
        tfs.transform.rotation.w = 1.0;
        broadcaster.sendTransform(tfs);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
