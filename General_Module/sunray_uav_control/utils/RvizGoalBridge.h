/*
本程序功能：
    1. 订阅rviz 2D Nav Goal话题，将其转换为UAVControlCMD指令
    2. 发布控制指令到UAV控制系统
    3. 提供安全检查机制：地理围栏、UAV状态验证等
    4. 支持参数配置和可选启用/禁用功能
    5. 作为独立节点运行，不影响现有UAVControl系统
*/

#ifndef RVIZ_GOAL_BRIDGE_H
#define RVIZ_GOAL_BRIDGE_H

#include "ros_msg_utils.h"

class RvizGoalBridge
{
public:
    RvizGoalBridge();
    ~RvizGoalBridge();
    
    void init(ros::NodeHandle &nh);
    void run();

private:
    // 基本参数
    std::string node_name;
    int uav_id;
    std::string uav_name;
    
    // 功能参数
    struct BridgeParams
    {
        bool enable_bridge;           // 是否启用桥接功能
        bool check_uav_state;         // 是否检查UAV状态
        bool check_geo_fence;         // 是否检查地理围栏
        bool auto_adjust_height;      // 是否自动调整高度
        double min_height;            // 最小允许高度
        double default_height;        // 默认飞行高度
        double goal_timeout;          // 目标点超时时间
    };
    BridgeParams bridge_params;
    
    // 地理围栏参数
    struct GeofenceParams
    {
        double x_min, x_max;
        double y_min, y_max;  
        double z_min, z_max;
    };
    GeofenceParams geofence;
    
    // UAV状态信息
    sunray_msgs::UAVState uav_state;
    bool uav_state_received;
    ros::Time last_uav_state_time;
    
    // 最后一个目标点信息
    geometry_msgs::PoseStamped last_goal;
    ros::Time last_goal_time;
    bool goal_active;
    
    // ROS句柄
    ros::NodeHandle* nh_ptr;
    
    // 订阅器
    ros::Subscriber rviz_goal_sub;
    ros::Subscriber uav_state_sub;
    
    // 发布器
    ros::Publisher uav_cmd_pub;
    ros::Publisher goal_viz_pub;  // 用于可视化当前目标点
    
    // 定时器
    ros::Timer status_timer;
    
    // 回调函数
    void rvizGoalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void uavStateCallback(const sunray_msgs::UAVState::ConstPtr &msg);
    void statusTimerCallback(const ros::TimerEvent &event);
    
    // 工具函数
    bool isUAVReadyForNavigation();
    bool isGoalInGeofence(const geometry_msgs::PoseStamped &goal);
    double adjustGoalHeight(double requested_height);
    sunray_msgs::UAVControlCMD createControlCommand(const geometry_msgs::PoseStamped &goal);
    void publishGoalVisualization(const geometry_msgs::PoseStamped &goal);
    void printStatus();
    void loadParameters();
    
    // 坐标转换工具
    double quaternionToYaw(const geometry_msgs::Quaternion &q);
    geometry_msgs::Quaternion yawToQuaternion(double yaw);
};

#endif // RVIZ_GOAL_BRIDGE_H
