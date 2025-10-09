/*
Rviz Goal Bridge Node Implementation
独立的rviz 2D Nav Goal桥接节点实现
*/

#include "RvizGoalBridge.h"

RvizGoalBridge::RvizGoalBridge()
    : uav_state_received(false)
    , goal_active(false)
    , nh_ptr(nullptr)
{
    // 初始化UAV状态
    uav_state.connected = false;
    uav_state.armed = false;
    uav_state.control_mode = 0; // INIT
    uav_state.odom_valid = false;
}

RvizGoalBridge::~RvizGoalBridge()
{
}

void RvizGoalBridge::init(ros::NodeHandle &nh)
{
    nh_ptr = &nh;
    node_name = ros::this_node::getName();
    
    // 加载参数
    loadParameters();
    
    // 检查是否启用桥接功能
    if (!bridge_params.enable_bridge)
    {
        ROS_WARN("[%s] Rviz Goal Bridge is DISABLED by parameter", node_name.c_str());
        return;
    }
    
    // 初始化订阅器
    rviz_goal_sub = nh.subscribe<geometry_msgs::PoseStamped>(
        "/move_base_simple/goal", 1, &RvizGoalBridge::rvizGoalCallback, this);
    
    uav_state_sub = nh.subscribe<sunray_msgs::UAVState>(
        uav_name + "/sunray/uav_state", 10, &RvizGoalBridge::uavStateCallback, this);
    
    // 初始化发布器
    uav_cmd_pub = nh.advertise<sunray_msgs::UAVControlCMD>(
        uav_name + "/sunray/uav_control_cmd", 1);
    
    goal_viz_pub = nh.advertise<geometry_msgs::PoseStamped>(
        uav_name + "/sunray/rviz_goal_current", 1);
    
    // 初始化定时器（用于状态监控和打印）
    status_timer = nh.createTimer(ros::Duration(2.0), &RvizGoalBridge::statusTimerCallback, this);
    
    ROS_INFO("[%s] Rviz Goal Bridge initialized successfully", node_name.c_str());
    ROS_INFO("[%s] Subscribed to: /move_base_simple/goal", node_name.c_str());
    ROS_INFO("[%s] Publishing to: %s", node_name.c_str(), (uav_name + "/sunray/uav_control_cmd").c_str());
}

void RvizGoalBridge::run()
{
    if (!bridge_params.enable_bridge)
    {
        ROS_WARN_THROTTLE(10.0, "[%s] Bridge disabled, spinning without functionality", node_name.c_str());
    }
    
    ros::spin();
}

void RvizGoalBridge::loadParameters()
{
    ros::NodeHandle& nh = *nh_ptr;
    
    // 基本参数
    nh.param<int>("uav_id", uav_id, 1);
    nh.param<std::string>("uav_name", uav_name, "uav");
    uav_name = "/" + uav_name + std::to_string(uav_id);
    
    // 桥接功能参数
    nh.param<bool>("bridge_params/enable_bridge", bridge_params.enable_bridge, true);
    nh.param<bool>("bridge_params/check_uav_state", bridge_params.check_uav_state, true);
    nh.param<bool>("bridge_params/check_geo_fence", bridge_params.check_geo_fence, true);
    nh.param<bool>("bridge_params/auto_adjust_height", bridge_params.auto_adjust_height, true);
    nh.param<double>("bridge_params/min_height", bridge_params.min_height, 0.5);
    nh.param<double>("bridge_params/default_height", bridge_params.default_height, 1.5);
    nh.param<double>("bridge_params/goal_timeout", bridge_params.goal_timeout, 10.0);
    
    // 地理围栏参数
    nh.param<double>("geo_fence/x_min", geofence.x_min, -10.0);
    nh.param<double>("geo_fence/x_max", geofence.x_max, 10.0);
    nh.param<double>("geo_fence/y_min", geofence.y_min, -10.0);
    nh.param<double>("geo_fence/y_max", geofence.y_max, 10.0);
    nh.param<double>("geo_fence/z_min", geofence.z_min, 0.1);
    nh.param<double>("geo_fence/z_max", geofence.z_max, 3.0);
    
    // 参数验证
    if (bridge_params.min_height < 0.1)
    {
        bridge_params.min_height = 0.1;
        ROS_WARN("[%s] min_height too low, adjusted to 0.1m", node_name.c_str());
    }
    
    if (bridge_params.default_height < bridge_params.min_height)
    {
        bridge_params.default_height = bridge_params.min_height + 0.5;
        ROS_WARN("[%s] default_height adjusted to %.2fm", node_name.c_str(), bridge_params.default_height);
    }
    
    ROS_INFO("[%s] Parameters loaded successfully", node_name.c_str());
}

void RvizGoalBridge::rvizGoalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    if (!bridge_params.enable_bridge)
    {
        return;
    }
    
    ROS_INFO("[%s] === Rviz 2D Nav Goal Received ===", node_name.c_str());
    
    // 检查UAV状态
    if (bridge_params.check_uav_state && !isUAVReadyForNavigation())
    {
        ROS_WARN("[%s] UAV not ready for navigation, goal ignored", node_name.c_str());
        return;
    }
    
    // 检查地理围栏
    if (bridge_params.check_geo_fence && !isGoalInGeofence(*msg))
    {
        ROS_ERROR("[%s] Goal outside geofence, rejected!", node_name.c_str());
        return;
    }
    
    // 保存目标点信息
    last_goal = *msg;
    last_goal_time = ros::Time::now();
    goal_active = true;
    
    // 调整高度（如果需要）
    if (bridge_params.auto_adjust_height)
    {
        last_goal.pose.position.z = adjustGoalHeight(msg->pose.position.z);
    }
    
    // 创建控制指令
    sunray_msgs::UAVControlCMD control_cmd = createControlCommand(last_goal);
    
    // 发布控制指令
    uav_cmd_pub.publish(control_cmd);
    
    // 发布可视化目标点
    publishGoalVisualization(last_goal);
    
    // 打印目标信息
    double yaw = quaternionToYaw(last_goal.pose.orientation);
    ROS_INFO("[%s] Goal Position [X Y Z]: [%.2f, %.2f, %.2f] m", 
             node_name.c_str(), 
             last_goal.pose.position.x, 
             last_goal.pose.position.y, 
             last_goal.pose.position.z);
    ROS_INFO("[%s] Goal Yaw: %.1f deg", node_name.c_str(), yaw * 180.0 / M_PI);
    ROS_INFO("[%s] Control command published", node_name.c_str());
}

void RvizGoalBridge::uavStateCallback(const sunray_msgs::UAVState::ConstPtr &msg)
{
    uav_state = *msg;
    uav_state_received = true;
    last_uav_state_time = ros::Time::now();
}

void RvizGoalBridge::statusTimerCallback(const ros::TimerEvent &event)
{
    if (!bridge_params.enable_bridge)
    {
        return;
    }
    
    // 检查UAV状态超时
    if (bridge_params.check_uav_state && uav_state_received)
    {
        double state_age = (ros::Time::now() - last_uav_state_time).toSec();
        if (state_age > 1.0)
        {
            ROS_WARN_THROTTLE(5.0, "[%s] UAV state timeout (%.1fs)", node_name.c_str(), state_age);
        }
    }
    
    // 检查目标点超时
    if (goal_active)
    {
        double goal_age = (ros::Time::now() - last_goal_time).toSec();
        if (goal_age > bridge_params.goal_timeout)
        {
            goal_active = false;
            ROS_INFO("[%s] Goal timeout, marking as inactive", node_name.c_str());
        }
    }
    
    // 定期打印状态
    static int counter = 0;
    if (++counter % 15 == 0)  // 每30秒打印一次
    {
        printStatus();
    }
}

bool RvizGoalBridge::isUAVReadyForNavigation()
{
    if (!uav_state_received)
    {
        ROS_WARN("[%s] No UAV state received yet", node_name.c_str());
        return false;
    }
    
    // 检查连接状态
    if (!uav_state.connected)
    {
        ROS_WARN("[%s] UAV not connected", node_name.c_str());
        return false;
    }
    
    // 检查解锁状态
    if (!uav_state.armed)
    {
        ROS_WARN("[%s] UAV not armed", node_name.c_str());
        return false;
    }
    
    // 检查控制模式（2 = CMD_CONTROL）
    if (uav_state.control_mode != 2)
    {
        ROS_WARN("[%s] UAV not in CMD_CONTROL mode (current: %d)", node_name.c_str(), uav_state.control_mode);
        return false;
    }
    
    // 检查起飞状态
    if (uav_state.landed_state == 1)  // LANDED_STATE_ON_GROUND
    {
        ROS_WARN("[%s] UAV still on ground", node_name.c_str());
        return false;
    }
    
    // 检查定位有效性
    if (!uav_state.odom_valid)
    {
        ROS_WARN("[%s] UAV odometry invalid", node_name.c_str());
        return false;
    }
    
    return true;
}

bool RvizGoalBridge::isGoalInGeofence(const geometry_msgs::PoseStamped &goal)
{
    double x = goal.pose.position.x;
    double y = goal.pose.position.y;
    double z = goal.pose.position.z;
    
    if (bridge_params.auto_adjust_height)
    {
        z = adjustGoalHeight(z);
    }
    
    bool in_fence = (x >= geofence.x_min && x <= geofence.x_max &&
                     y >= geofence.y_min && y <= geofence.y_max &&
                     z >= geofence.z_min && z <= geofence.z_max);
    
    if (!in_fence)
    {
        ROS_ERROR("[%s] Goal [%.2f, %.2f, %.2f] outside geofence", node_name.c_str(), x, y, z);
        ROS_ERROR("[%s] Geofence: X[%.1f, %.1f] Y[%.1f, %.1f] Z[%.1f, %.1f]", 
                  node_name.c_str(),
                  geofence.x_min, geofence.x_max,
                  geofence.y_min, geofence.y_max,
                  geofence.z_min, geofence.z_max);
    }
    
    return in_fence;
}

double RvizGoalBridge::adjustGoalHeight(double requested_height)
{
    double adjusted_height = requested_height;
    
    // 如果高度过低或为0，使用默认高度
    if (requested_height < bridge_params.min_height)
    {
        adjusted_height = bridge_params.default_height;
        ROS_INFO("[%s] Height adjusted from %.2f to %.2f m", 
                 node_name.c_str(), requested_height, adjusted_height);
    }
    
    // 限制在地理围栏范围内
    if (adjusted_height > geofence.z_max)
    {
        adjusted_height = geofence.z_max;
        ROS_WARN("[%s] Height limited to geofence max: %.2f m", node_name.c_str(), adjusted_height);
    }
    
    return adjusted_height;
}

sunray_msgs::UAVControlCMD RvizGoalBridge::createControlCommand(const geometry_msgs::PoseStamped &goal)
{
    sunray_msgs::UAVControlCMD cmd;
    
    cmd.header.stamp = ros::Time::now();
    cmd.header.frame_id = "rviz_goal_bridge";
    
    // 使用XyzPosYaw控制模式
    cmd.cmd = sunray_msgs::UAVControlCMD::XyzPosYaw;
    
    // 设置目标位置
    cmd.desired_pos[0] = goal.pose.position.x;
    cmd.desired_pos[1] = goal.pose.position.y;
    cmd.desired_pos[2] = goal.pose.position.z;
    
    // 设置目标偏航角
    cmd.desired_yaw = quaternionToYaw(goal.pose.orientation);
    
    // 清零其他字段
    cmd.desired_vel[0] = 0.0;
    cmd.desired_vel[1] = 0.0;
    cmd.desired_vel[2] = 0.0;
    cmd.desired_acc[0] = 0.0;
    cmd.desired_acc[1] = 0.0;
    cmd.desired_acc[2] = 0.0;
    cmd.desired_yaw_rate = 0.0;
    
    return cmd;
}

void RvizGoalBridge::publishGoalVisualization(const geometry_msgs::PoseStamped &goal)
{
    geometry_msgs::PoseStamped viz_goal = goal;
    viz_goal.header.stamp = ros::Time::now();
    viz_goal.header.frame_id = "world";
    
    goal_viz_pub.publish(viz_goal);
}

double RvizGoalBridge::quaternionToYaw(const geometry_msgs::Quaternion &q)
{
    tf2::Quaternion tf_q;
    tf2::fromMsg(q, tf_q);
    
    double roll, pitch, yaw;
    tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
    
    return yaw;
}

geometry_msgs::Quaternion RvizGoalBridge::yawToQuaternion(double yaw)
{
    tf2::Quaternion tf_q;
    tf_q.setRPY(0, 0, yaw);
    
    return tf2::toMsg(tf_q);
}

void RvizGoalBridge::printStatus()
{
    ROS_INFO("[%s] === Rviz Goal Bridge Status ===", node_name.c_str());
    ROS_INFO("[%s] Bridge enabled: %s", node_name.c_str(), bridge_params.enable_bridge ? "YES" : "NO");
    
    if (!bridge_params.enable_bridge)
    {
        return;
    }
    
    ROS_INFO("[%s] UAV state received: %s", node_name.c_str(), uav_state_received ? "YES" : "NO");
    
    if (uav_state_received)
    {
        ROS_INFO("[%s] UAV: %s | %s | Mode:%d | Odom:%s", 
                 node_name.c_str(),
                 uav_state.connected ? "Connected" : "Disconnected",
                 uav_state.armed ? "Armed" : "Disarmed", 
                 uav_state.control_mode,
                 uav_state.odom_valid ? "Valid" : "Invalid");
        
        ROS_INFO("[%s] UAV Position: [%.2f, %.2f, %.2f]", 
                 node_name.c_str(),
                 uav_state.position[0], uav_state.position[1], uav_state.position[2]);
    }
    
    ROS_INFO("[%s] Current goal active: %s", node_name.c_str(), goal_active ? "YES" : "NO");
    
    if (goal_active)
    {
        double yaw = quaternionToYaw(last_goal.pose.orientation);
        ROS_INFO("[%s] Goal: [%.2f, %.2f, %.2f] @ %.1f deg", 
                 node_name.c_str(),
                 last_goal.pose.position.x, 
                 last_goal.pose.position.y, 
                 last_goal.pose.position.z,
                 yaw * 180.0 / M_PI);
    }
    
    ROS_INFO("[%s] Ready for navigation: %s", 
             node_name.c_str(), 
             isUAVReadyForNavigation() ? "YES" : "NO");
}
