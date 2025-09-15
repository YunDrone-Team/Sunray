#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/Point.h>
#include <sunray_logger.h>
#include "ros_msg_utils.h"
#include "printf_utils.h"

using namespace sunray_logger;
using namespace std;

// 全局变量定义
sunray_msgs::UAVState uav_state;    // 无人机状态
sunray_msgs::UAVControlCMD uav_cmd; // 无人机控制命令
sunray_msgs::UAVSetup uav_setup;    // 无人机设置
nav_msgs::Odometry stereo_odom;     // 立体视觉里程计
sensor_msgs::Image depth_image;     // 深度图像
string node_name;                   // 节点名称

// 目标点设置（支持动态更新）
geometry_msgs::Point target_pos;   // 目标位置
bool target_set = false;           // 目标是否设置
bool depth_image_received = false; // 是否收到深度图像
bool odom_received = false;        // 是否收到里程计数据
bool target_updated = false;       // 目标是否被更新

// 飞行控制参数
double flight_height = 0.6;   // 飞行高度，单位：米
double forward_vel = 0.5;     // 前进速度，单位：m/s
double rotate_speed = 0.3;    // 旋转速度，单位：rad/s
int obstacle_threshold = 1000; // 小于1000mm的点视为障碍物
double ratio_threshold = 0.3; // 30%的点视为有障碍物

// 避障状态枚举
enum AvoidanceState {
    NAVIGATING_TO_TARGET,    // 正常导航到目标
    AVOIDING_OBSTACLE,       // 正在避障
    CLEARING_OBSTACLE,       // 清除障碍物后的缓冲状态
    TURNING_TO_TARGET        // 转向目标方向
};

// 是否启用避障
bool obstacle_avoidance_enabled = true;

// 避障状态管理
AvoidanceState current_state = NAVIGATING_TO_TARGET;
ros::Time state_change_time;           // 状态改变时间
double clear_obstacle_duration = 2.0;  // 清除障碍后的缓冲时间（秒）
int avoid_direction = 0;               // 避障方向：1=左转，-1=右转，0=无

// 简化的障碍物信息结构体
struct SimpleObstacleInfo
{
    bool front_blocked;    // 前方是否有障碍物
    bool left_clear;       // 左边是否畅通
    bool right_clear;      // 右边是否畅通
    double front_distance; // 前方距离
};

// 状态名称转换函数
string get_state_name(AvoidanceState state) {
    switch(state) {
        case NAVIGATING_TO_TARGET: return "NAVIGATING_TO_TARGET";
        case AVOIDING_OBSTACLE: return "AVOIDING_OBSTACLE";
        case CLEARING_OBSTACLE: return "CLEARING_OBSTACLE";
        case TURNING_TO_TARGET: return "TURNING_TO_TARGET";
        default: return "UNKNOWN";
    }
}

// 回调函数：无人机状态
void uav_state_cb(const sunray_msgs::UAVState::ConstPtr &msg)
{
    uav_state = *msg;
}

// 回调函数：立体视觉里程计
void stereo_odom_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
    stereo_odom = *msg;
    odom_received = true;
}

// 回调函数：深度图像
void depth_image_cb(const sensor_msgs::Image::ConstPtr &msg)
{
    depth_image = *msg;
    depth_image_received = true;
}

// 回调函数：动态目标点更新
void target_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    // 更新目标点
    target_pos.x = msg->pose.position.x;
    target_pos.y = msg->pose.position.y;
    target_pos.z = flight_height; // 保持飞行高度不变
    target_set = true;
    target_updated = true;
    
    // 重置避障状态机
    current_state = NAVIGATING_TO_TARGET;
    avoid_direction = 0;

    Logger::print_color(int(LogColor::magenta), node_name, ": 收到新目标点: [" + std::to_string(target_pos.x) + ", " + std::to_string(target_pos.y) + ", " + std::to_string(target_pos.z) + "]，重置避障状态机");
}

// 简单的深度图像处理函数
SimpleObstacleInfo process_depth_image_simple()
{
    SimpleObstacleInfo info;
    // 默认初始化：前方畅通，左右都畅通
    info.front_blocked = false;
    info.left_clear = true;
    info.right_clear = true;
    info.front_distance = 10.0;

    // 如果没有收到深度图像，返回默认值
    if (!depth_image_received)
    {
        return info;
    }

    try
    {
        // 将ROS图像消息转换为OpenCV格式
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(depth_image, sensor_msgs::image_encodings::MONO16);
        cv::Mat depth_mat = cv_ptr->image;

        int rows = depth_mat.rows; // 图像高度
        int cols = depth_mat.cols; // 图像宽度

        // 将图像分成三个区域：左、中、右
        int region_width = cols / 3;    // 每个区域的宽度
        int check_height = rows / 5;    // 检查区域的高度（图像中间部分）
        int start_row = 2 * rows / 5;   // 开始检查的行（避开天空和地面）

        // 定义三个检查区域
        cv::Rect left_roi(0, start_row, region_width, check_height);                 // 左侧区域
        cv::Rect center_roi(region_width, start_row, region_width, check_height);    // 中间区域
        cv::Rect right_roi(2 * region_width, start_row, region_width, check_height); // 右侧区域

        // 计算每个区域的平均深度
        cv::Mat left_region = depth_mat(left_roi);
        cv::Mat center_region = depth_mat(center_roi);
        cv::Mat right_region = depth_mat(right_roi);

        // 把区域内的0值（无效值）排除在外
        left_region.setTo(0, left_region == 0);
        center_region.setTo(0, center_region == 0);
        right_region.setTo(0, right_region == 0);
        
        // 计算每个区域内小于阈值的点的比例
        double left_obstacle_ratio = (double)cv::countNonZero(left_region < obstacle_threshold) / (region_width * check_height);
        double center_obstacle_ratio = (double)cv::countNonZero(center_region < obstacle_threshold) / (region_width * check_height);
        double right_obstacle_ratio = (double)cv::countNonZero(right_region < obstacle_threshold) / (region_width * check_height);
        // 计算前方距离（中间区域的平均深度，排除0值）
        if (center_obstacle_ratio > ratio_threshold)
        {
            info.front_blocked = true;
        }
        if (left_obstacle_ratio > ratio_threshold)
        {
            info.left_clear = false;
        }
        if (right_obstacle_ratio > ratio_threshold)
        {
            info.right_clear = false;
        }
        info.front_distance = cv::mean(center_region, center_region > 0)[0] / 1000.0; // 转换为米

        // 将原本的深度图像转换成彩色，标记三个区域，并把三个区域内的障碍物比例显示出来，同时显示该区域是否有障碍物，把转换后的图像发布成新话题
        cv::Mat depth_color;
        double min_val, max_val;
        cv::minMaxLoc(depth_mat, &min_val, &max_val);
        depth_mat.convertTo(depth_color, CV_8U, 255.0 / max_val);
        cv::applyColorMap(depth_color, depth_color, cv::COLORMAP_JET);
        // 标记区域
        cv::rectangle(depth_color, left_roi, cv::Scalar(255, 255, 255), 1);
        cv::rectangle(depth_color, center_roi, cv::Scalar(255, 255, 255), 1);
        cv::rectangle(depth_color, right_roi, cv::Scalar(255, 255, 255), 1);
        // 显示障碍物比例和状态
        std::string left_text = cv::format("Left: %.1f%%", left_obstacle_ratio * 100);
        std::string center_text = cv::format("Center: %.1f%%", center_obstacle_ratio * 100);
        std::string right_text = cv::format("Right: %.1f%%", right_obstacle_ratio * 100);
        cv::putText(depth_color, left_text, cv::Point(10, start_row - 10), cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(255, 255, 255), 1);
        cv::putText(depth_color, center_text, cv::Point(region_width + 10, start_row - 10), cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(255, 255, 255), 1);
        cv::putText(depth_color, right_text, cv::Point(2 * region_width + 10, start_row - 10), cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(255, 255, 255), 1);
        // 发布彩色深度图像
        static ros::Publisher depth_color_pub = ros::NodeHandle().advertise<sensor_msgs::Image>("/baton/depth_color", 1);
        sensor_msgs::ImagePtr depth_color_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", depth_color).toImageMsg();
        depth_color_pub.publish(depth_color_msg);   
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("深度图像转换错误: %s", e.what());
    }

    return info;
}

// 计算朝向目标的偏航角
double calculate_target_yaw()
{
    if (!target_set)
    {
        return 0.0; // 如果没有设置目标，保持当前朝向
    }

    // 计算目标相对于当前位置的角度
    double dx = target_pos.x - uav_state.position[0];
    double dy = target_pos.y - uav_state.position[1];

    // 使用atan2计算目标方向角度
    double target_yaw = atan2(dy, dx);

    return target_yaw;
}

// 检查是否面向目标
bool is_facing_target()
{
    if (!target_set)
    {
        return true;
    }

    double target_yaw = calculate_target_yaw();
    double current_yaw = uav_state.attitude[2]; // 当前偏航角

    // 计算角度差
    double yaw_diff = target_yaw - current_yaw;

    // 将角度差限制在[-π, π]范围内
    while (yaw_diff > M_PI)
        yaw_diff -= 2 * M_PI;
    while (yaw_diff < -M_PI)
        yaw_diff += 2 * M_PI;

    // 如果角度差小于15度，认为已经面向目标
    return fabs(yaw_diff) < (15.0 * M_PI / 180.0);
}

// 检查是否到达目标
bool is_target_reached()
{
    if (!target_set)
    {
        return false;
    }

    double dx = target_pos.x - uav_state.position[0];
    double dy = target_pos.y - uav_state.position[1];
    double distance = sqrt(dx * dx + dy * dy);

    return distance < 0.5; // 距离小于0.5米认为到达目标
}

int main(int argc, char **argv)
{
    // 设置日志系统
    Logger::init_default();
    Logger::setPrintLevel(false);
    Logger::setPrintTime(false);
    Logger::setPrintToFile(false);
    Logger::setFilename("~/Documents/Sunray_log.txt");

    // 初始化ROS节点
    ros::init(argc, argv, "simple_obstacle_avoidance");
    ros::NodeHandle nh("~");
    ros::Rate rate(20.0); // 20Hz控制频率
    node_name = ros::this_node::getName();

    // 读取参数
    int uav_id;
    string uav_name;
    string target_topic_name;
    nh.param<int>("uav_id", uav_id, 1);                                      // 无人机ID，默认为1
    nh.param<string>("uav_name", uav_name, "uav");                           // 无人机名称，默认为"uav"
    nh.param<string>("target_topic_name", target_topic_name, "/target_pos"); // 目标点话题名称
    nh.param<double>("flight_height", flight_height, 0.6);                   // 飞行高度，默认1米
    nh.param<double>("forward_vel", forward_vel, 0.5);                       // 前进速度，默认0.5m/s
    nh.param<double>("rotate_speed", rotate_speed, 0.3);                     // 旋转速度，默认0.3rad/s
    nh.param<int>("obstacle_threshold", obstacle_threshold, 1000);         // 障碍物阈值，默认1000mm
    nh.param<double>("ratio_threshold", ratio_threshold, 0.3);               // 障碍物比例阈值，默认30%
    nh.param<double>("target_x", target_pos.x, 3.0);                         // 初始目标X坐标，默认3米
    nh.param<double>("target_y", target_pos.y, 0.0);                         // 初始目标Y坐标，默认0米
    nh.param<double>("clear_obstacle_duration", clear_obstacle_duration, 2.0);// 清除障碍后的缓冲时间（秒）

    target_pos.z = flight_height; // 目标高度设置为飞行高度
    target_set = true;            // 标记目标已设置

    // 构建无人机话题名称
    uav_name = "/" + uav_name + std::to_string(uav_id);

    // 创建订阅者
    ros::Subscriber uav_state_sub = nh.subscribe<sunray_msgs::UAVState>(uav_name + "/sunray/uav_state", 1, uav_state_cb);
    ros::Subscriber stereo_odom_sub = nh.subscribe<nav_msgs::Odometry>("/baton/stereo3/odometry", 1, stereo_odom_cb);
    ros::Subscriber depth_image_sub = nh.subscribe<sensor_msgs::Image>("/baton/depth_image", 1, depth_image_cb);
    // 动态目标点订阅者 - 支持运行时更新目标点
    ros::Subscriber target_sub = nh.subscribe<geometry_msgs::PoseStamped>(target_topic_name, 1, target_cb);

    // 创建发布者
    ros::Publisher control_cmd_pub = nh.advertise<sunray_msgs::UAVControlCMD>(uav_name + "/sunray/uav_control_cmd", 1);
    ros::Publisher uav_setup_pub = nh.advertise<sunray_msgs::UAVSetup>(uav_name + "/sunray/setup", 1);

    // 初始化控制命令
    uav_cmd.header.stamp = ros::Time::now();
    uav_cmd.cmd = sunray_msgs::UAVControlCMD::Hover;
    uav_cmd.desired_pos[0] = 0.0;
    uav_cmd.desired_pos[1] = 0.0;
    uav_cmd.desired_pos[2] = 0.0;
    uav_cmd.desired_vel[0] = 0.0;
    uav_cmd.desired_vel[1] = 0.0;
    uav_cmd.desired_vel[2] = 0.0;
    uav_cmd.desired_acc[0] = 0.0;
    uav_cmd.desired_acc[1] = 0.0;
    uav_cmd.desired_acc[2] = 0.0;
    uav_cmd.desired_att[0] = 0.0;
    uav_cmd.desired_att[1] = 0.0;
    uav_cmd.desired_att[2] = 0.0;
    uav_cmd.desired_yaw = 0.0;
    uav_cmd.desired_yaw_rate = 0.0;

    ros::Duration(0.5).sleep();

    // 等待无人机连接
    int times = 0;
    while (ros::ok() && !uav_state.connected)
    {
        ros::spinOnce();
        ros::Duration(1.0).sleep();
        if (times++ > 5)
            Logger::print_color(int(LogColor::red), node_name, ": 等待无人机连接...");
    }
    Logger::print_color(int(LogColor::green), node_name, ": 无人机已连接!");

    // 设置控制模式为命令控制
    while (ros::ok() && uav_state.control_mode != sunray_msgs::UAVSetup::CMD_CONTROL)
    {
        uav_setup.cmd = sunray_msgs::UAVSetup::SET_CONTROL_MODE;
        uav_setup.control_mode = "CMD_CONTROL";
        uav_setup_pub.publish(uav_setup);
        Logger::print_color(int(LogColor::green), node_name, ": 设置控制模式为命令控制");
        ros::Duration(1.0).sleep();
        ros::spinOnce();
    }
    Logger::print_color(int(LogColor::green), node_name, ": 控制模式设置成功!");

    // 解锁无人机
    Logger::print_color(int(LogColor::green), node_name, ": 5秒后解锁无人机...");
    for (int i = 5; i >= 1; i--) {
        Logger::print_color(int(LogColor::green), node_name, ": " + std::to_string(i) + "秒后解锁...");
        ros::Duration(1.0).sleep();
    }

    while (ros::ok() && !uav_state.armed)
    {
        uav_setup.cmd = sunray_msgs::UAVSetup::ARM;
        uav_setup_pub.publish(uav_setup);
        Logger::print_color(int(LogColor::green), node_name, ": 正在解锁无人机");
        ros::Duration(1.0).sleep();
        ros::spinOnce();
    }
    Logger::print_color(int(LogColor::green), node_name, ": 无人机解锁成功!");

    // 起飞
    while (ros::ok() && abs(uav_state.position[2] - uav_state.home_pos[2] - uav_state.takeoff_height) > 0.2)
    {
        uav_cmd.cmd = sunray_msgs::UAVControlCMD::Takeoff;
        control_cmd_pub.publish(uav_cmd);
        Logger::print_color(int(LogColor::green), node_name, ": 正在起飞");
        ros::Duration(4.0).sleep();
        ros::spinOnce();
    }
    Logger::print_color(int(LogColor::green), node_name, ": 起飞成功!");

    ros::Duration(1.0).sleep();

    // 开始避障飞行主循环
    // Logger::print_color(int(LogColor::green), node_name, ": 开始简易避障飞行...");
    // Logger::print_color(int(LogColor::green), node_name, ": 初始目标位置: [" + std::to_string(target_pos.x) + ", " + std::to_string(target_pos.y) + ", " + std::to_string(target_pos.z) + "]");
    // Logger::print_color(int(LogColor::cyan), node_name, ": 飞行规则：只能前进和转向，不能后退和左右平移!");
    // Logger::print_color(int(LogColor::cyan), node_name, ": 支持动态目标更新，话题名: " + target_topic_name);

    while (ros::ok())
    {
        // 检查目标是否被更新
        if (target_updated) {
            target_updated = false;
            obstacle_avoidance_enabled = true; // 每次目标更新后启用避障
            Logger::print_color(int(LogColor::magenta), node_name, ": 目标已更新，重新导航至新目标点");
        }

        // 检查是否到达目标
        if (is_target_reached()) {
            obstacle_avoidance_enabled = false; // 到达目标后禁用避障
            Logger::print_color(int(LogColor::green), node_name, ": 已到达目标点，悬停中... (等待新目标或手动控制)");
            // 悬停控制
            uav_cmd.header.stamp = ros::Time::now();
            uav_cmd.cmd = sunray_msgs::UAVControlCMD::Hover;

            control_cmd_pub.publish(uav_cmd);
            ros::spinOnce();
            rate.sleep();
            continue;
        }

        // 处理深度图像，检测障碍物
        SimpleObstacleInfo obstacle_info = process_depth_image_simple();

        double vx = 0.0;       // X方向速度（前进/后退）
        double vy = 0.0;       // Y方向速度（左右）
        double yaw_rate = 0.0; // 偏航角速度（转向）

        if (obstacle_avoidance_enabled)
        {
            // 状态机避障逻辑
            AvoidanceState prev_state = current_state;
            
            switch(current_state) {
                case NAVIGATING_TO_TARGET:
                {
                    if (obstacle_info.front_blocked) {
                        // 检测到障碍物，切换到避障状态
                        current_state = AVOIDING_OBSTACLE;
                        state_change_time = ros::Time::now();
                        
                        // 选择避障方向
                        if (obstacle_info.left_clear && !obstacle_info.right_clear) {
                            avoid_direction = 1;  // 向左转
                        } else if (!obstacle_info.left_clear && obstacle_info.right_clear) {
                            avoid_direction = -1; // 向右转
                        } else if (obstacle_info.left_clear && obstacle_info.right_clear) {
                            avoid_direction = -1; // 两边都畅通，选择向右转
                        } else {
                            avoid_direction = 1;  // 三面都有障碍，向左转寻找出路
                        }
                        
                        Logger::print_color(int(LogColor::yellow), node_name, ": 检测到障碍物! 距离: " +
                                          std::to_string(obstacle_info.front_distance) + "米，切换到避障状态");
                    } else if (!is_facing_target()) {
                        // 前方畅通但未面向目标，切换到转向目标状态
                        current_state = TURNING_TO_TARGET;
                        state_change_time = ros::Time::now();
                    } else {
                        // 面向目标且前方畅通，正常前进
                        vx = forward_vel;
                        yaw_rate = 0.0;
                        
                        double dx = target_pos.x - uav_state.position[0];
                        double dy = target_pos.y - uav_state.position[1];
                        double distance_to_target = sqrt(dx * dx + dy * dy);
                        
                        Logger::print_color(int(LogColor::green), node_name, ": 正常导航，朝目标前进，距离: " +
                                          std::to_string(distance_to_target) + "米");
                    }
                    break;
                }
                
                case AVOIDING_OBSTACLE:
                {
                    // 执行避障转向
                    vx = 0.0; // 停止前进
                    yaw_rate = avoid_direction * rotate_speed;
                    
                    string direction_str = (avoid_direction == 1) ? "左转" : "右转";
                    Logger::print_color(int(LogColor::yellow), node_name, ": 正在避障 - " + direction_str);
                    
                    // 如果前方畅通且已避障一定时间，进入清除障碍状态
                    if (!obstacle_info.front_blocked && 
                        ros::Time::now() - state_change_time > ros::Duration(1.0)) {
                        current_state = CLEARING_OBSTACLE;
                        state_change_time = ros::Time::now();
                        Logger::print_color(int(LogColor::cyan), node_name, ": 前方畅通，进入障碍物清除状态");
                    }
                    break;
                }
                
                case CLEARING_OBSTACLE:
                {
                    // 清除障碍物状态：慢速前进，确保远离障碍物
                    vx = forward_vel * 0.6;  // 60%速度前进
                    yaw_rate = 0.0;
                    
                    Logger::print_color(int(LogColor::cyan), node_name, ": 清除障碍物中，慢速前进");
                    
                    // 检查是否完成清除
                    if (ros::Time::now() - state_change_time > ros::Duration(clear_obstacle_duration)) {
                        current_state = NAVIGATING_TO_TARGET;
                        avoid_direction = 0;  // 重置避障方向
                        Logger::print_color(int(LogColor::green), node_name, ": 障碍物清除完成，恢复正常导航");
                    } else if (obstacle_info.front_blocked) {
                        // 如果清除过程中又遇到障碍物，重新开始避障
                        current_state = AVOIDING_OBSTACLE;
                        state_change_time = ros::Time::now();
                        Logger::print_color(int(LogColor::yellow), node_name, ": 清除过程中再次遇到障碍物，重新避障");
                    }
                    break;
                }
                
                case TURNING_TO_TARGET:
                {
                    // 转向目标方向
                    if (obstacle_info.front_blocked) {
                        // 转向过程中遇到障碍物，立即切换到避障状态
                        current_state = AVOIDING_OBSTACLE;
                        state_change_time = ros::Time::now();
                        
                        // 重新选择避障方向
                        if (obstacle_info.left_clear && !obstacle_info.right_clear) {
                            avoid_direction = 1;
                        } else if (!obstacle_info.left_clear && obstacle_info.right_clear) {
                            avoid_direction = -1;
                        } else {
                            avoid_direction = -1; // 默认向右
                        }
                        
                        Logger::print_color(int(LogColor::yellow), node_name, ": 转向过程中遇到障碍物，切换到避障状态");
                    } else if (is_facing_target()) {
                        // 已面向目标，切换到正常导航
                        current_state = NAVIGATING_TO_TARGET;
                        Logger::print_color(int(LogColor::green), node_name, ": 已面向目标，恢复正常导航");
                    } else {
                        // 继续转向目标
                        vx = 0.0;
                        
                        double target_yaw = calculate_target_yaw();
                        double current_yaw = uav_state.attitude[2];
                        double yaw_diff = target_yaw - current_yaw;
                        
                        // 将角度差限制在[-π, π]范围内
                        while (yaw_diff > M_PI) yaw_diff -= 2 * M_PI;
                        while (yaw_diff < -M_PI) yaw_diff += 2 * M_PI;
                        
                        yaw_rate = (yaw_diff > 0) ? rotate_speed : -rotate_speed;
                        
                        Logger::print_color(int(LogColor::cyan), node_name, ": 转向目标方向，角度差: " +
                                          std::to_string(yaw_diff * 180.0 / M_PI) + "度");
                    }
                    break;
                }
            }
            
            // 状态变化日志
            if (prev_state != current_state) {
                Logger::print_color(int(LogColor::magenta), node_name, ": 状态切换: " + 
                                  get_state_name(prev_state) + " -> " + get_state_name(current_state));
            }
        }
        
        // 发送控制命令（使用机体坐标系XyVelZPosYawBody模式）
        uav_cmd.header.stamp = ros::Time::now();
        uav_cmd.cmd = sunray_msgs::UAVControlCMD::XyVelZPosYawrateBody;  // 机体坐标系XY速度，Z位置，偏航角控制
        uav_cmd.desired_vel[0] = vx;                                // X方向速度（机体前进方向）
        uav_cmd.desired_vel[1] = vy;                                // Y方向速度（机体左右方向，这里始终为0）
        uav_cmd.desired_pos[2] = flight_height - uav_state.position[2];                     // Z方向位置（保持飞行高度）
        uav_cmd.desired_yaw_rate = yaw_rate;  // 期望偏航角速率

        control_cmd_pub.publish(uav_cmd);

        ros::spinOnce();
        rate.sleep();
    }

    // // 降落程序
    // while (ros::ok() && uav_state.control_mode != sunray_msgs::UAVSetup::LAND_CONTROL && uav_state.landed_state != 1)
    // {
    //     uav_cmd.cmd = sunray_msgs::UAVControlCMD::Land;
    //     control_cmd_pub.publish(uav_cmd);
    //     Logger::print_color(int(LogColor::green), node_name, ": 正在降落");
    //     ros::Duration(4.0).sleep();
    //     ros::spinOnce();
    // }

    // // 等待降落完成
    // while (ros::ok() && uav_state.landed_state != 1)
    // {
    //     Logger::print_color(int(LogColor::green), node_name, ": 降落中...");
    //     ros::Duration(1.0).sleep();
    //     ros::spinOnce();
    // }

    // Logger::print_color(int(LogColor::green), node_name, ": 降落成功!");
    // Logger::print_color(int(LogColor::green), node_name, ": 带动态目标更新的简易避障演示完成!");
    return 0;
}