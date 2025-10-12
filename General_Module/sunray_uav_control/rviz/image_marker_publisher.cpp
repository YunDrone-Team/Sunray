#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_marker_publisher");
    ros::NodeHandle nh;
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    ros::Rate rate(1); // 1Hz

    while (ros::ok()) 
    {
        visualization_msgs::Marker marker;
        
        // 设置标记的基本属性
        marker.header.frame_id = "world"; // 坐标系，请根据实际情况修改
        marker.header.stamp = ros::Time::now();
        marker.ns = "image_marker";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::MESH_RESOURCE; // 关键：类型改为 MESH_RESOURCE
        marker.action = visualization_msgs::Marker::ADD;
        
        // 设置位置和姿态
        marker.pose.position.x = 0.0;
        marker.pose.position.y = 0.0;
        marker.pose.position.z = 0.0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        
        // 设置尺寸
        marker.scale.x = 1.0;  // 长度缩放因子
        marker.scale.y = 1.0;  // 宽度缩放因子
        marker.scale.z = 0.01; // 厚度，设为很薄的值
        
        // 设置颜色 (通常设置为白色不透明，以便纹理正常显示)
        marker.color.r = 1.0f;
        marker.color.g = 1.0f;
        marker.color.b = 1.0f;
        marker.color.a = 1.0f; // 必须非零
        
        // 关键：指定网格资源文件路径
        // 将路径替换为你转换后得到的 .dae 或 .stl 文件路径
        // 注意使用 file:// 协议和绝对路径
        marker.mesh_resource = std::string("package://sunray_uav_control/meshes/uav.mesh");
        
        // 可选：使用嵌入的材料（如果模型文件包含）
        marker.mesh_use_embedded_materials = false;
        
        // marker.lifetime = ros::Duration();
        
        // 发布标记
        marker_pub.publish(marker);
        
        ROS_INFO("Published image marker (as MESH_RESOURCE)");
        rate.sleep();
    }
    return 0;
}