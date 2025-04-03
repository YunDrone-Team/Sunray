// #ifndef MAP_GENERATOR_H
// #define MAP_GENERATOR_H

#include "Astar.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>

#include <octomap/octomap.h>
#include <octomap/Pointcloud.h>
#include <octomap/ColorOcTree.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf/transform_listener.h>

#include <sunray_msgs/UGVState.h>

class MapGenerator
{
public:
    MapGenerator() {};
    ~MapGenerator() {};

    void init(ros::NodeHandle &nh, float map_min_x, float map_min_y, float map_max_x, float map_max_y, float map_resolution = 0.1, float inflate_size = 0);
    void PointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg);
    void updateMapFromPointCloud(const sensor_msgs::PointCloud2::ConstPtr &msg);
    void updateMapFromLaserScan(const sensor_msgs::LaserScan::ConstPtr &msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
    void ugvStateCallback(const sunray_msgs::UGVState::ConstPtr &msg);
    void PublishOctomap();
    void PublishOccupancyMap();
    void inflateOccupancyMap();

    octomap_msgs::Octomap octomap_msg;
    nav_msgs::OccupancyGrid occupancy_grid_msg;
    nav_msgs::Path path_msg;
    GridWithWeights *grid;

    octomap::OcTree *tree;
    octomap::OcTree *global_map;    // 没用上
    octomap::OcTree *local_map;     // 没用上
    nav_msgs::Odometry odom;
    tf::StampedTransform transform;
    laser_geometry::LaserProjection projector;

    int map_min_x;
    int map_max_x;
    int map_min_y;
    int map_max_y;
    float map_resolution;
    float inflate_size;
    std::map<std::pair<int, int>, int> occupancy_map;
    bool is_odom_received = false;

    ros::Subscriber odom_sub;
    ros::Subscriber ugv_state;
    ros::Subscriber laser_sub;
    ros::Publisher octomap_pub;
    ros::Publisher occupancy_pub;
};

void MapGenerator::init(ros::NodeHandle &nh, float map_min_x, float map_min_y, float map_max_x, float map_max_y, float map_resolution, float inflate_size)
{
    int odom_type, ugv_id;
    std::string odom_topic, laser_topic;
    std::string octomap_topic, occupancy_topic;
    nh.param<int>("ugv_id", ugv_id, 1);
    nh.param<int>("odom_type", odom_type, 2);
    nh.param<std::string>("odom_topic", odom_topic, "/car_odom");
    nh.param<std::string>("laser_topic", laser_topic, "/scan");
    nh.param<std::string>("octomap_topic", octomap_topic, "/octomap_tree");
    nh.param<std::string>("occupancy_topic", occupancy_topic, "/occupancy_grid");

    std::string ugv_prefix = "/ugv" + std::to_string(ugv_id);

    laser_sub = nh.subscribe<sensor_msgs::LaserScan>(laser_topic, 1, &MapGenerator::updateMapFromLaserScan, this);
    octomap_pub = nh.advertise<octomap_msgs::Octomap>(octomap_topic, 1);
    occupancy_pub = nh.advertise<nav_msgs::OccupancyGrid>(occupancy_topic, 10);

    if (odom_type == 1)
    {
        odom_sub = nh.subscribe<nav_msgs::Odometry>(odom_topic, 1, &MapGenerator::odomCallback, this);
    }
    else
    {
        ugv_state = nh.subscribe<sunray_msgs::UGVState>(ugv_prefix + "/sunray_ugv/ugv_state", 1, &MapGenerator::ugvStateCallback, this);
    }

    this->map_min_x = map_min_x;
    this->map_max_x = map_max_x;
    this->map_min_y = map_min_y;
    this->map_max_y = map_max_y;
    this->map_resolution = map_resolution;
    this->inflate_size = inflate_size;
    tree = new octomap::OcTree(map_resolution);
    octomap::point3d minPt(map_min_x, map_min_y, -5.0); // 最小点
    octomap::point3d maxPt(map_max_x, map_max_y, 5.0);  // 最大点
    tree->setBBXMin(minPt);
    tree->setBBXMax(maxPt);

    grid = new GridWithWeights(static_cast<int>((map_max_x - map_min_x) / map_resolution),
                               static_cast<int>((map_max_y - map_min_y) / map_resolution));

    // 设置 OccupancyGrid 参数
    occupancy_grid_msg.info.width = static_cast<unsigned int>((this->map_max_x - this->map_min_x) / this->map_resolution);
    occupancy_grid_msg.info.height = static_cast<unsigned int>((this->map_max_y - this->map_min_y) / this->map_resolution);
    occupancy_grid_msg.info.resolution = this->map_resolution;
    occupancy_grid_msg.info.origin.position.x = this->map_min_x;
    occupancy_grid_msg.info.origin.position.y = this->map_min_y;
    occupancy_grid_msg.info.origin.position.z = 0.0;
    occupancy_grid_msg.info.origin.orientation.w = 1.0; // 无旋转

    // 初始化栅格数据（默认值 -1 表示未知）
    occupancy_grid_msg.data.resize(occupancy_grid_msg.info.width * occupancy_grid_msg.info.height, -1);
};

void MapGenerator::PublishOctomap()
{
    octomap_msg.header.frame_id = "odom";
    octomap_msgs::fullMapToMsg(*tree, octomap_msg);
    octomap_pub.publish(octomap_msg);
}

void MapGenerator::PublishOccupancyMap()
{
    occupancy_grid_msg.header.frame_id = "odom";
    occupancy_grid_msg.header.stamp = ros::Time::now();

    // // 6. 遍历 OctoMap，投影到 2D 平面（例如 z=0 附近）
    // double z_min = 0.0; // 地面高度下限
    // double z_max = 1;   // 地面高度上限（可根据机器人高度调整）

    // for (octomap::OcTree::leaf_iterator it = tree->begin_leafs(); it != tree->end_leafs(); ++it)
    // {
    //     // std::cout << "it.getZ()" << it.getZ() << std::endl;
    //     if (it.getZ() >= z_min && it.getZ() <= z_max)
    //     {
    //         // 获取体素的 2D 坐标（忽略 z 轴）
    //         int grid_x = static_cast<int>((it.getX() - min_x) / occupancy_grid_msg.info.resolution);
    //         int grid_y = static_cast<int>((it.getY() - min_y) / occupancy_grid_msg.info.resolution);

    //         // 检查坐标是否在合法范围内
    //         if (grid_x >= 0 && grid_x < occupancy_grid_msg.info.width && grid_y >= 0 && grid_y < occupancy_grid_msg.info.height)
    //         {
    //             // 将占据概率转换为 OccupancyGrid 的值（0-100）
    //             double occupancy = it->getOccupancy();
    //             int8_t value = (occupancy > tree->getOccupancyThres()) ? 100 : 0;
    //             occupancy_grid_msg.data[grid_y * occupancy_grid_msg.info.width + grid_x] = value;
    //         }
    //     }
    // }

    occupancy_pub.publish(occupancy_grid_msg);
}

void MapGenerator::updateMapFromPointCloud(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
}

void MapGenerator::updateMapFromLaserScan(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    if (!is_odom_received)
    {
        std::cout << "No odometry data received yet." << std::endl;
        return;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // 使用 laser_geometry 将 LaserScan 转换为 PointCloud2
    sensor_msgs::PointCloud2 cloud_msg;
    projector.projectLaser(*msg, cloud_msg);

    // 将 sensor_msgs::PointCloud2 转换为 pcl::PCLPointCloud2
    pcl::PCLPointCloud2 pcl_cloud;
    pcl_conversions::toPCL(cloud_msg, pcl_cloud);

    // 转换到世界坐标系 根据odom数据进行转换
    pcl::fromPCLPointCloud2(pcl_cloud, *temp_cloud);
    pcl_ros::transformPointCloud(*temp_cloud, *cloud_transformed, transform);

    occupancy_grid_msg.data.resize(occupancy_grid_msg.info.width * occupancy_grid_msg.info.height, -1);
    grid->clear_walls();
    for (const auto &point : *cloud_transformed)
    {
        // Extract XYZ coordinates and RGB color from the point
        // float x = point.x;
        // float y = point.y;
        // float z = point.z;
        // 过滤距离自己很近的点
        if (abs(point.x - odom.pose.pose.position.x) < 0.4 || abs(point.y - odom.pose.pose.position.y) < 0.4)
        {
            continue;
        }

        // Convert color to octomap::ColorOcTreeNode
        octomap::point3d octomap_point(point.x, point.y, point.z);

        // Insert the node into the octomap
        tree->updateNode(octomap_point, true);

        for (float x = -this->inflate_size; x <= this->inflate_size; x += 0.05)
        {
            for (float y = -this->inflate_size; y <= this->inflate_size; y += 0.05)
            {
                pcl::PointXYZ new_point;
                new_point.x = point.x + x;
                new_point.y = point.y + y;

                int grid_x = static_cast<int>((new_point.x - this->map_min_x) / this->map_resolution);
                int grid_y = static_cast<int>((new_point.y - this->map_min_y) / this->map_resolution);

                // 检查坐标是否在合法范围内
                if (grid_x >= 0 && grid_x < occupancy_grid_msg.info.width && grid_y >= 0 && grid_y < occupancy_grid_msg.info.height)
                {
                    occupancy_grid_msg.data[grid_y * occupancy_grid_msg.info.width + grid_x] = 100;
                    add_wall(*grid, grid_x, grid_y);
                }
            }
        }
    }

    octomap_msg.header.frame_id = "odom";
    octomap_msgs::fullMapToMsg(*tree, octomap_msg);
    PublishOccupancyMap();
    PublishOctomap();
    // std::cout << "Map updated from point cloud." << std::endl;
}

void MapGenerator::odomCallback(const nav_msgs::Odometry::ConstPtr &odom_msg)
{
    is_odom_received = true;
    odom = *odom_msg;
    transform.stamp_ = odom_msg->header.stamp;
    transform.frame_id_ = "odom";
    transform.child_frame_id_ = "base_link";
    transform.setOrigin(tf::Vector3(odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y, odom_msg->pose.pose.position.z));
    transform.setRotation(tf::Quaternion(odom_msg->pose.pose.orientation.x, odom_msg->pose.pose.orientation.y, odom_msg->pose.pose.orientation.z, odom_msg->pose.pose.orientation.w));
}

void MapGenerator::ugvStateCallback(const sunray_msgs::UGVState::ConstPtr &msg)
{
    is_odom_received = true;
    odom.pose.pose.position.x = msg->position[0];
    odom.pose.pose.position.y = msg->position[1];
    double yaw = msg->yaw;
    odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);

    transform.stamp_ = odom.header.stamp;
    transform.frame_id_ = "odom";
    transform.child_frame_id_ = "base_link";
    transform.setOrigin(tf::Vector3(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z));
    transform.setRotation(tf::Quaternion(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w));
}

// #endif // MAP_GENERATOR_H