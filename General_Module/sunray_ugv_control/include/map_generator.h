// #ifndef MAP_GENERATOR_H
// #define MAP_GENERATOR_H

#include "Astar.h"
#include "ros/ros.h"

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>

#include <octomap/octomap.h>
#include <octomap/Pointcloud.h>
#include <octomap/ColorOcTree.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

#include "sensor_msgs/PointCloud2.h"
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
    void updateMapFromLaserScan(const sensor_msgs::LaserScan::ConstPtr &msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
    void ugvStateCallback(const sunray_msgs::UGVState::ConstPtr &msg);
    void PublishOctomap();
    nav_msgs::OccupancyGrid projectOctomapSlice(const octomap::OcTree &octree, double height);
    void inflateOccupancyGrid(nav_msgs::OccupancyGrid &grid, double inflation_radius, double resolution);

    octomap_msgs::Octomap octomap_msg;
    nav_msgs::OccupancyGrid octomap_grid_msg;
    nav_msgs::Path path_msg;
    GridWithWeights *astar_grid;

    octomap::OcTree *tree;
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

    // 定时器
    ros::Timer timer;
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

    astar_grid = new GridWithWeights(static_cast<int>((map_max_x - map_min_x) / map_resolution),
                               static_cast<int>((map_max_y - map_min_y) / map_resolution));
};

void MapGenerator::PublishOctomap()
{
    octomap_msg.header.frame_id = "odom";
    octomap_msgs::fullMapToMsg(*tree, octomap_msg);
    octomap_pub.publish(octomap_msg);

    octomap_grid_msg = projectOctomapSlice(*tree, 0);
    occupancy_pub.publish(octomap_grid_msg);
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

    // astar_grid->clear_walls();
    octomap::Pointcloud octoCloud;
    for (const auto &point : *cloud_transformed)
    {
        // 过滤距离自己很近的点
        if (abs(point.x - odom.pose.pose.position.x) < 0.2 || abs(point.y - odom.pose.pose.position.y) < 0.2)
        {
            continue;
        }
        octoCloud.push_back(octomap::point3d(point.x, point.y, point.z));
    }
    tree->insertPointCloud(octoCloud, octomap::point3d(0.0, 0.0, 0.0));
    octomap_msg.header.frame_id = "odom";
    octomap_msgs::fullMapToMsg(*tree, octomap_msg);
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


nav_msgs::OccupancyGrid MapGenerator::projectOctomapSlice(const octomap::OcTree &octree, double height)
{
    // 设置OccupancyGrid参数
    nav_msgs::OccupancyGrid grid;
    grid.header.frame_id = "odom"; // 根据实际情况设置
    grid.info.resolution = this->map_resolution;
    grid.info.width = static_cast<unsigned int>((this->map_max_x - this->map_min_x) / this->map_resolution);
    grid.info.height = static_cast<unsigned int>((this->map_max_y - this->map_min_y) / this->map_resolution);
    grid.info.origin.position.x = this->map_min_x;
    grid.info.origin.position.y = this->map_min_y;
    grid.info.origin.position.z = 0;
    grid.info.origin.orientation.w = 1.0;

    // 初始化网格数据(-1表示未知)
    grid.data.resize(grid.info.width * grid.info.height, -1);

    // 定义高度范围(考虑一定厚度)
    double z_min = height - this->map_resolution;
    double z_max = height + this->map_resolution;

    // 遍历所有叶子节点
    for (octomap::OcTree::leaf_iterator it = octree.begin_leafs();
         it != octree.end_leafs(); ++it)
    {
        // 检查节点是否在指定高度范围内和地图范围内
        if (it.getZ() >= z_min && it.getZ() <= z_max && it.getX() >= this->map_min_x && it.getX() <= this->map_max_x && it.getY() >= this->map_min_y && it.getY() <= this->map_max_y)
        {
            // 计算网格坐标
            int grid_x = (it.getX() - this->map_min_x) / this->map_resolution;
            int grid_y = (it.getY() - this->map_min_y) / this->map_resolution;

            // 检查坐标是否在范围内
            if (grid_x >= 0 && grid_x < grid.info.width &&
                grid_y >= 0 && grid_y < grid.info.height)
            {
                // 设置占用值(0-100)
                int index = grid_y * grid.info.width + grid_x;
                if (octree.isNodeOccupied(*it))
                {
                    grid.data[index] = 100; // 占用
                    // 占用表示障碍物 添加到astar_grid中
                    add_wall(*astar_grid, grid_x, grid_y);
                }
                else
                {
                    grid.data[index] = 0; // 空闲
                    remove_wall(*astar_grid, grid_x, grid_y);
                }
                
            }
        }
    }

    // 膨胀处理
    if (this->inflate_size > 0)
    {
        inflateOccupancyGrid(grid, this->inflate_size, this->map_resolution);
    }

    return grid;
}

void MapGenerator::inflateOccupancyGrid(nav_msgs::OccupancyGrid &grid, double inflation_radius, double resolution)
{
    // 计算膨胀半径对应的像素数
    int inflation_cells = std::ceil(inflation_radius / resolution);

    // 创建临时网格用于膨胀
    std::vector<int8_t> inflated_data = grid.data;

    // 定义8邻域方向
    const int dx8[] = {-1, 0, 1, -1, 1, -1, 0, 1};
    const int dy8[] = {-1, -1, -1, 0, 0, 1, 1, 1};

    // 遍历所有占用单元格
    for (int y = 0; y < grid.info.height; ++y)
    {
        for (int x = 0; x < grid.info.width; ++x)
        {
            int index = y * grid.info.width + x;

            // 如果是占用单元格
            if (grid.data[index] == 100)
            {
                add_wall(*astar_grid, x, y);
                // 膨胀到周围单元格
                for (int r = 1; r <= inflation_cells; ++r)
                {
                    for (int dir = 0; dir < 8; ++dir)
                    {
                        int nx = x + dx8[dir] * r;
                        int ny = y + dy8[dir] * r;

                        if (nx >= 0 && nx < grid.info.width &&
                            ny >= 0 && ny < grid.info.height)
                        {
                            int nindex = ny * grid.info.width + nx;

                            // 只膨胀到空闲或未知区域
                            if (inflated_data[nindex] != 100)
                            {
                                // 计算距离并设置衰减值
                                double distance = r * resolution;
                                if (distance <= inflation_radius)
                                {
                                    inflated_data[nindex] = 100; // 或者使用衰减值
                                    add_wall(*astar_grid, nx, ny);
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    grid.data = inflated_data;
}