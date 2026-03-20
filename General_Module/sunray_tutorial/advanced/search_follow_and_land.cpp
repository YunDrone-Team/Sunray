/*
程序功能：搜索二维码目标，跟随移动的二维码，当目标静止后降落
功能说明：
1. 无人机自动起飞到指定高度
2. 支持多种搜索模式：圆形、螺旋、栅格搜索
3. 检测到二维码后切换到跟随模式
4. 跟随移动的二维码目标
5. 检测目标静止状态
6. 目标静止一段时间后自动降落到二维码上
*/

#include "ros_msg_utils.h"
#include "utils.hpp"
#include <Eigen/Dense>
#include <iomanip>

using namespace std;
using namespace Eigen;
using namespace sunray_logger;

// 搜索模式枚举
enum SearchMode
{
    CIRCLE_CURRENT = 0,  // 以当前位置为圆心画圆
    CIRCLE_FIXED = 1,    // 以指定坐标为圆心画圆
    SPIRAL = 2,          // 螺旋搜索
    GRID = 3             // 栅格搜索
};

// 任务状态枚举
enum MissionState
{
    SEARCHING,   // 搜索目标
    FOLLOWING,   // 跟随目标
    LANDING      // 降落
};

// 全局变量
string node_name;
int uav_id;
string uav_name;

sunray_msgs::UAVState uav_state;
sunray_msgs::UAVSetup uav_setup;
sunray_msgs::UAVControlCMD uav_cmd;

// 任务状态
MissionState mission_state = SEARCHING;
int search_attempt_count = 0;  // 搜索尝试次数

// 二维码检测相关
bool target_detected = false;
bool uav_state_received = false;
double target_rel_x = 0.0;
double target_rel_y = 0.0;
double target_rel_z = 0.0;
double target_rel_yaw = 0.0;
ros::Time last_detection_time;

// 滤波器
MovingAverageFilter x_filter(5);
MovingAverageFilter y_filter(5);
MovingAverageFilter z_filter(5);
MovingAverageFilter yaw_filter(5);

// 静止检测相关
vector<double> position_history_x;
vector<double> position_history_y;
const int HISTORY_SIZE = 50;  // 2.5秒的历史数据 (50 * 0.05s)
const double STATIONARY_THRESHOLD = 0.1;  // 静止判断阈值(米)
ros::Time stationary_start_time;
bool is_stationary = false;

// 目标世界系位置估计
double target_global_x = 0.0;
double target_global_y = 0.0;

void update_target_global_position()
{
    const double yaw = uav_state.attitude[2];
    target_global_x = uav_state.position[0] + cos(yaw) * target_rel_x - sin(yaw) * target_rel_y;
    target_global_y = uav_state.position[1] + sin(yaw) * target_rel_x + cos(yaw) * target_rel_y;
}

// 信号处理
void mySigintHandler(int sig)
{
    Logger::print_color(int(LogColor::yellow), node_name, "Exit...");
    ros::shutdown();
    exit(EXIT_SUCCESS);
}

// 无人机状态回调
void uav_state_callback(const sunray_msgs::UAVState::ConstPtr &msg)
{
    uav_state = *msg;
    uav_state_received = true;
}

// 二维码检测回调
void qrcode_callback(const sunray_msgs::TargetsInFrameMsg::ConstPtr &msg)
{
    if (msg->targets.size() > 0)
    {
        // 异常值过滤
        if (abs(msg->targets[0].px) > 10 || abs(msg->targets[0].py) > 10 || abs(msg->targets[0].pz) > 10)
        {
            return;
        }

        target_detected = true;
        last_detection_time = ros::Time::now();

        target_rel_x = x_filter.filter(msg->targets[0].px);
        target_rel_y = y_filter.filter(-msg->targets[0].py);
        target_rel_z = z_filter.filter(-msg->targets[0].pz);
        target_rel_yaw = yaw_filter.filter(-msg->targets[0].yaw);
    }
}

// 检查目标是否静止
bool check_target_stationary(double stationary_time_threshold)
{
    update_target_global_position();

    // 添加目标世界系位置到历史记录
    position_history_x.push_back(target_global_x);
    position_history_y.push_back(target_global_y);

    // 保持历史记录大小
    if (position_history_x.size() > HISTORY_SIZE)
    {
        position_history_x.erase(position_history_x.begin());
        position_history_y.erase(position_history_y.begin());
    }

    // 需要足够的历史数据
    if (position_history_x.size() < HISTORY_SIZE)
    {
        return false;
    }

    // 计算目标世界系位置变化范围
    double max_x = *max_element(position_history_x.begin(), position_history_x.end());
    double min_x = *min_element(position_history_x.begin(), position_history_x.end());
    double max_y = *max_element(position_history_y.begin(), position_history_y.end());
    double min_y = *min_element(position_history_y.begin(), position_history_y.end());

    double range_x = max_x - min_x;
    double range_y = max_y - min_y;

    // 判断是否静止
    if (range_x < STATIONARY_THRESHOLD && range_y < STATIONARY_THRESHOLD)
    {
        if (!is_stationary)
        {
            // 刚开始静止
            is_stationary = true;
            stationary_start_time = ros::Time::now();
            Logger::print_color(int(LogColor::cyan), node_name, "Target started to be stationary");
        }

        double stationary_duration = (ros::Time::now() - stationary_start_time).toSec();

        std::ostringstream oss;
        oss << std::fixed << std::setprecision(1)
            << "Target stationary for " << stationary_duration << "s / " << stationary_time_threshold
            << "s, global range: [" << range_x << ", " << range_y << "]";
        Logger::print_color(int(LogColor::cyan), node_name, oss.str());

        return stationary_duration >= stationary_time_threshold;
    }
    else
    {
        // 目标在移动
        if (is_stationary)
        {
            Logger::print_color(int(LogColor::yellow), node_name, "Target started moving again");
        }
        is_stationary = false;
        return false;
    }
}

// 飞向目标点并检测
bool fly_to_point_and_detect(double target_x, double target_y, double search_height,
                              double k_p, double max_vel, ros::Rate &rate,
                              ros::Publisher &control_cmd_pub)
{
    while (ros::ok())
    {
        ros::spinOnce();

        // 检查是否检测到目标
        if (target_detected && (ros::Time::now() - last_detection_time).toSec() < 0.5)
        {
            Logger::print_color(int(LogColor::green), node_name, "Target detected during search!");
            return true;
        }

        // 计算速度指令
        double dx = target_x - uav_state.position[0];
        double dy = target_y - uav_state.position[1];

        double vx = k_p * dx;
        double vy = k_p * dy;

        // 限制速度
        vx = min(max(vx, -max_vel), max_vel);
        vy = min(max(vy, -max_vel), max_vel);

        // 发送控制指令
        uav_cmd.header.stamp = ros::Time::now();
        uav_cmd.cmd = sunray_msgs::UAVControlCMD::XyVelZPos;
        uav_cmd.desired_vel[0] = vx;
        uav_cmd.desired_vel[1] = vy;
        uav_cmd.desired_pos[2] = search_height;
        control_cmd_pub.publish(uav_cmd);

        // 检查是否到达目标点
        if (fabs(uav_state.position[0] - target_x) < 0.2 &&
            fabs(uav_state.position[1] - target_y) < 0.2)
        {
            break;
        }

        rate.sleep();
    }
    return false;
}

// 圆形搜索
bool search_circle(double center_x, double center_y, double search_height,
                   double search_radius, double search_speed, int search_circles,
                   ros::Rate &rate, ros::Publisher &control_cmd_pub)
{
    int num_points = 50;
    double k_p = 1.0;

    for (int circle = 0; circle < search_circles && ros::ok(); circle++)
    {
        for (int i = 0; i < num_points && ros::ok(); i++)
        {
            double theta = i * 2 * M_PI / num_points;
            double target_x = center_x + search_radius * cos(theta);
            double target_y = center_y + search_radius * sin(theta);

            if (fly_to_point_and_detect(target_x, target_y, search_height, k_p, search_speed,
                                        rate, control_cmd_pub))
            {
                return true;
            }
        }
    }
    return false;
}

// 螺旋搜索
bool search_spiral(double center_x, double center_y, double search_height,
                   double search_radius, double search_speed, int search_circles,
                   ros::Rate &rate, ros::Publisher &control_cmd_pub)
{
    int total_points = 100 * search_circles;
    double k_p = 1.0;

    for (int i = 0; i < total_points && ros::ok(); i++)
    {
        double progress = (double)i / total_points;
        double current_radius = progress * search_radius;
        double theta = progress * search_circles * 2 * M_PI;

        double target_x = center_x + current_radius * cos(theta);
        double target_y = center_y + current_radius * sin(theta);

        if (fly_to_point_and_detect(target_x, target_y, search_height, k_p, search_speed,
                                    rate, control_cmd_pub))
        {
            return true;
        }
    }
    return false;
}

// 栅格搜索
bool search_grid(double start_x, double start_y, double search_height,
                 double grid_width, double grid_height, double search_speed, int grid_lines,
                 bool start_from_right, ros::Rate &rate, ros::Publisher &control_cmd_pub)
{
    double k_p = 1.0;
    double x_spacing = grid_width / (grid_lines - 1);   // x方向间距
    double y_spacing = grid_height / (grid_lines - 1);  // y方向间距

    // row 控制 y 方向（左右，换行方向），col 控制 x 方向（前后，每行内移动）
    for (int row = 0; row < grid_lines && ros::ok(); row++)
    {
        // 计算当前行的 y 坐标
        // start_from_right=false: 从 start_y 开始，向右移动（y减小）
        // start_from_right=true: 从 start_y 开始，向左移动（y增大）
        double current_y = start_from_right ? (start_y + row * y_spacing) : (start_y - row * y_spacing);

        // 蛇形路径：偶数行从前往后，奇数行从后往前
        bool forward = (row % 2 == 0);

        if (forward)
        {
            // 从前往后：x 从小到大
            for (int col = 0; col < grid_lines && ros::ok(); col++)
            {
                double target_x = start_x + col * x_spacing;
                double target_y = current_y;

                if (fly_to_point_and_detect(target_x, target_y, search_height, k_p, search_speed,
                                            rate, control_cmd_pub))
                {
                    return true;
                }
            }
        }
        else
        {
            // 从后往前：x 从大到小
            for (int col = grid_lines - 1; col >= 0 && ros::ok(); col--)
            {
                double target_x = start_x + col * x_spacing;
                double target_y = current_y;

                if (fly_to_point_and_detect(target_x, target_y, search_height, k_p, search_speed,
                                            rate, control_cmd_pub))
                {
                    return true;
                }
            }
        }
    }
    return false;
}

// 跟随模式
double compute_smooth_velocity(double error, double k_p, double max_vel)
{
    const double deadband = 0.04;   // 小于该误差时不再修正，减少围绕目标的小幅摆动
    const double slow_zone = 0.30;  // 接近目标时主动减速，降低过冲

    if (abs(error) < deadband)
    {
        return 0.0;
    }

    double scaled_error = error;
    if (abs(error) < slow_zone)
    {
        scaled_error = error * abs(error) / slow_zone;
    }

    return min(max(k_p * scaled_error, -max_vel), max_vel);
}

void follow_target(double k_p_xy, double k_p_z, double k_p_yaw,
                   double max_vel, double max_vel_z, double max_yaw,
                   double follow_height, ros::Publisher &control_cmd_pub)
{
    // 计算速度指令
    double x_vel = compute_smooth_velocity(target_rel_x, k_p_xy, max_vel);
    double y_vel = compute_smooth_velocity(target_rel_y, k_p_xy, max_vel);

    // 修正：target_rel_z是负值（下视相机），所以实际高度 = -target_rel_z
    // 期望高度差 = follow_height - (-target_rel_z) = follow_height + target_rel_z
    double z_vel = min(max((follow_height + target_rel_z) * k_p_z, -max_vel_z), max_vel_z);

    // 小角度不调整，避免抽搐
    double yaw = 0.0;
    if (abs(target_rel_yaw) > 1.0)
    {
        yaw = min(max((target_rel_yaw / 180.0 * M_PI) * k_p_yaw, -max_yaw), max_yaw);
    }

    // 发送控制指令
    uav_cmd.header.stamp = ros::Time::now();
    uav_cmd.cmd = sunray_msgs::UAVControlCMD::XyzVelYawBody;
    uav_cmd.desired_vel[0] = x_vel;
    uav_cmd.desired_vel[1] = y_vel;
    uav_cmd.desired_vel[2] = z_vel;
    uav_cmd.desired_yaw = yaw;
    control_cmd_pub.publish(uav_cmd);
}

// 降落模式
bool landing_procedure(double error_xy, double error_z, double drop_height,
                       double land_vel, double land_time,
                       ros::Publisher &control_cmd_pub, ros::Publisher &uav_setup_pub)
{
    // 最终降落阶段
    if (abs(target_rel_x) < error_xy && abs(target_rel_y) < error_xy && abs(target_rel_z) < error_z)
    {
        Logger::print_color(int(LogColor::green), node_name, "Final landing phase!");

        ros::Time stop_time = ros::Time::now();
        while ((ros::Time::now() - stop_time).toSec() < land_time)
        {
            uav_cmd.header.stamp = ros::Time::now();
            uav_cmd.cmd = sunray_msgs::UAVControlCMD::XyzVelYawBody;
            uav_cmd.desired_vel[0] = 0;
            uav_cmd.desired_vel[1] = 0;
            uav_cmd.desired_vel[2] = -land_vel;
            uav_cmd.desired_yaw = 0;
            control_cmd_pub.publish(uav_cmd);
            ros::Duration(0.1).sleep();
        }

        Logger::print_color(int(LogColor::green), node_name, "Landing successful!");
        uav_setup.header.stamp = ros::Time::now();
        uav_setup.cmd = sunray_msgs::UAVSetup::EMERGENCY_KILL;
        uav_setup_pub.publish(uav_setup);
        return true;
    }

    // 调整位置并下降
    double yaw_cmd = 0.0;
    if (abs(target_rel_yaw) > 1.0)
    {
        yaw_cmd = target_rel_yaw / 180.0 * M_PI / 2;
    }

    uav_cmd.header.stamp = ros::Time::now();
    uav_cmd.cmd = sunray_msgs::UAVControlCMD::XyzPosYawBody;
    uav_cmd.desired_pos[0] = target_rel_x / 2;
    uav_cmd.desired_pos[1] = target_rel_y / 2;
    uav_cmd.desired_pos[2] = -drop_height;
    uav_cmd.desired_yaw = yaw_cmd;

    // 优先调整水平距离
    if (abs(target_rel_z) < 1.0 && abs(target_rel_z) > 0.2 &&
        (abs(target_rel_x) > 2 * error_xy || abs(target_rel_y) > 2 * error_xy))
    {
        uav_cmd.desired_pos[2] = -0.05;
    }

    control_cmd_pub.publish(uav_cmd);
    return false;
}

int main(int argc, char **argv)
{
    // 设置日志
    Logger::init_default();
    Logger::setPrintLevel(false);
    Logger::setPrintTime(false);

    ros::init(argc, argv, "search_follow_and_land");
    ros::NodeHandle nh("~");
    ros::Rate rate(20.0);

    signal(SIGINT, mySigintHandler);

    node_name = ros::this_node::getName();
    node_name = "[" + node_name + "]:";

    // 参数读取
    nh.param<int>("uav_id", uav_id, 1);
    nh.param<string>("uav_name", uav_name, "uav");

    // 搜索参数
    int search_mode;
    double search_height, search_radius, search_speed, center_x, center_y;
    int search_circles;
    int max_search_attempts;  // 最大搜索尝试次数
    bool grid_start_from_right;  // 栅格搜索起始方向
    double grid_width, grid_height;  // 栅格搜索区域的长宽
    nh.param<int>("search_mode", search_mode, 2);  // 默认螺旋搜索
    nh.param<double>("search_height", search_height, 3.0);
    nh.param<double>("search_radius", search_radius, 5.0);
    nh.param<double>("search_speed", search_speed, 0.5);
    nh.param<int>("search_circles", search_circles, 3);
    nh.param<int>("max_search_attempts", max_search_attempts, 3);  // 默认最多搜索3次
    nh.param<double>("center_x", center_x, 0.0);
    nh.param<double>("center_y", center_y, 0.0);
    nh.param<bool>("grid_start_from_right", grid_start_from_right, true);  // 默认从右往左
    nh.param<double>("grid_width", grid_width, 3.0);  // 栅格搜索x方向宽度
    nh.param<double>("grid_height", grid_height, 3.0);  // 栅格搜索y方向高度

    // 跟随参数
    double k_p_xy, k_p_z, k_p_yaw, max_vel, max_vel_z, max_yaw, follow_height;
    nh.param<double>("k_p_xy", k_p_xy, 1.2);
    nh.param<double>("k_p_z", k_p_z, 0.5);
    nh.param<double>("k_p_yaw", k_p_yaw, 0.04);
    nh.param<double>("max_vel", max_vel, 0.5);
    nh.param<double>("max_vel_z", max_vel_z, 0.2);
    nh.param<double>("max_yaw", max_yaw, 0.4);
    nh.param<double>("follow_height", follow_height, 1.0);

    // 降落参数
    double error_xy, error_z, drop_height, land_vel, land_time, stationary_time;
    nh.param<double>("error_xy", error_xy, 0.05);
    nh.param<double>("error_z", error_z, 0.25);
    nh.param<double>("drop_height", drop_height, 0.15);
    nh.param<double>("land_vel", land_vel, 0.3);
    nh.param<double>("land_time", land_time, 1.5);
    nh.param<double>("stationary_time", stationary_time, 3.0);

    uav_name = "/" + uav_name + to_string(uav_id);

    // ROS通信
    ros::Subscriber uav_state_sub = nh.subscribe<sunray_msgs::UAVState>(
        uav_name + "/sunray/uav_state", 10, uav_state_callback);
    ros::Subscriber qrcode_sub = nh.subscribe<sunray_msgs::TargetsInFrameMsg>(
        uav_name + "/sunray_detect/qrcode_detection_ros", 1, qrcode_callback);
    ros::Publisher control_cmd_pub = nh.advertise<sunray_msgs::UAVControlCMD>(
        uav_name + "/sunray/uav_control_cmd", 1);
    ros::Publisher uav_setup_pub = nh.advertise<sunray_msgs::UAVSetup>(
        uav_name + "/sunray/setup", 1);

    // 控制辅助类初始化
    Control_Utils uav_control_utils;
    uav_control_utils.init(nh, uav_id, node_name);

    // 等待无人机连接
    Logger::print_color(int(LogColor::blue), node_name, "Waiting for UAV connection...");
    int times = 0;
    while (ros::ok() && !uav_state.connected)
    {
        ros::spinOnce();
        ros::Duration(1.0).sleep();
        if (times++ > 5)
            Logger::print_color(int(LogColor::red), node_name, "Wait for UAV connect...");
    }
    Logger::print_color(int(LogColor::green), node_name, "UAV connected!");

    // 自动起飞
    Logger::print_color(int(LogColor::blue), node_name, "Starting auto takeoff...");
    uav_control_utils.auto_takeoff();
    Logger::print_color(int(LogColor::green), node_name, "Takeoff completed!");

    ros::Duration(3.0).sleep();
    ros::spinOnce();

    // 上升到搜索高度
    Logger::print_color(int(LogColor::blue), node_name, "Ascending to search height...");
    while (ros::ok() && fabs(uav_state.position[2] - search_height) > 0.3)
    {
        uav_cmd.header.stamp = ros::Time::now();
        uav_cmd.cmd = sunray_msgs::UAVControlCMD::XyVelZPos;
        uav_cmd.desired_vel[0] = 0.0;
        uav_cmd.desired_vel[1] = 0.0;
        uav_cmd.desired_pos[2] = search_height;
        control_cmd_pub.publish(uav_cmd);
        ros::spinOnce();
        rate.sleep();
    }
    Logger::print_color(int(LogColor::green), node_name, "Reached search height!");

    // 打印搜索模式
    string mode_names[] = {"CIRCLE_CURRENT", "CIRCLE_FIXED", "SPIRAL", "GRID"};
    std::ostringstream mode_str;
    mode_str << "Search mode: " << mode_names[search_mode];
    Logger::print_color(int(LogColor::blue), node_name, mode_str.str());

    // 打印栅格搜索方向
    if (search_mode == GRID)
    {
        std::ostringstream grid_dir_str;
        grid_dir_str << "Grid start direction: " << (grid_start_from_right ? "RIGHT to LEFT" : "LEFT to RIGHT");
        Logger::print_color(int(LogColor::blue), node_name, grid_dir_str.str());
    }

    // 确定搜索中心
    double actual_center_x, actual_center_y;
    if (search_mode == CIRCLE_CURRENT || search_mode == SPIRAL)
    {
        actual_center_x = uav_state.position[0];
        actual_center_y = uav_state.position[1];
    }
    else
    {
        actual_center_x = center_x;
        actual_center_y = center_y;

        // 先飞到指定中心位置
        fly_to_point_and_detect(actual_center_x, actual_center_y, search_height,
                                1.0, search_speed, rate, control_cmd_pub);
    }

    // 主循环
    int stable_detection_count = 0;
    ros::Time last_state_print_time = ros::Time::now();

    while (ros::ok())
    {
        ros::spinOnce();

        // 每秒打印一次状态
        if ((ros::Time::now() - last_state_print_time).toSec() > 1.0)
        {
            std::ostringstream status_oss;
            status_oss << "State: ";
            if (mission_state == SEARCHING)
                status_oss << "SEARCHING (attempt " << search_attempt_count + 1 << "/" << max_search_attempts
                          << ", circles: " << search_circles << ")";
            else if (mission_state == FOLLOWING)
                status_oss << "FOLLOWING (height: " << std::fixed << std::setprecision(2) << -target_rel_z << "m)";
            else if (mission_state == LANDING)
                status_oss << "LANDING";
            Logger::print_color(int(LogColor::cyan), node_name, status_oss.str());
            last_state_print_time = ros::Time::now();
        }

        switch (mission_state)
        {
            case SEARCHING:
            {
                Logger::print_color(int(LogColor::blue), node_name, "Starting search...");

                bool found = false;
                switch (search_mode)
                {
                    case CIRCLE_CURRENT:
                    case CIRCLE_FIXED:
                        found = search_circle(actual_center_x, actual_center_y, search_height,
                                            search_radius, search_speed, search_circles,
                                            rate, control_cmd_pub);
                        break;

                    case SPIRAL:
                        found = search_spiral(actual_center_x, actual_center_y, search_height,
                                            search_radius, search_speed, search_circles,
                                            rate, control_cmd_pub);
                        break;

                    case GRID:
                        found = search_grid(center_x, center_y, search_height,
                                        grid_width, grid_height, search_speed, search_circles,
                                        grid_start_from_right, rate, control_cmd_pub);
                        break;

                    default:
                        Logger::print_color(int(LogColor::red), node_name, "Unknown search mode!");
                        return 0;
                }

                search_attempt_count++;

                if (found)
                {
                    mission_state = FOLLOWING;
                    stable_detection_count = 0;
                    Logger::print_color(int(LogColor::green), node_name, "Target found! Switching to FOLLOWING");
                }
                else
                {
                    if (search_attempt_count >= max_search_attempts)
                    {
                        Logger::print_color(int(LogColor::yellow), node_name, "Max search attempts reached, landing...");
                        uav_cmd.cmd = sunray_msgs::UAVControlCMD::Land;
                        control_cmd_pub.publish(uav_cmd);
                        ros::Duration(0.5).sleep();
                        return 0;
                    }
                    else
                    {
                        Logger::print_color(int(LogColor::yellow), node_name, "Target not found, continuing search...");
                    }
                }
                break;
            }

            case FOLLOWING:
            {
                // 检查是否丢失目标
                if ((ros::Time::now() - last_detection_time).toSec() > 2.0)
                {
                    Logger::print_color(int(LogColor::yellow), node_name, "Target lost! Switching back to SEARCHING");
                    mission_state = SEARCHING;
                    stable_detection_count = 0;
                    // 重置静止检测
                    position_history_x.clear();
                    position_history_y.clear();
                    is_stationary = false;
                }
                else
                {
                    stable_detection_count++;

                    // 只要目标有效就持续跟随，避免从搜索切换后出现控制空窗
                    follow_target(k_p_xy, k_p_z, k_p_yaw, max_vel, max_vel_z, max_yaw,
                                follow_height, control_cmd_pub);

                    if (stable_detection_count > 10)
                    {
                        // 检查目标是否静止
                        if (check_target_stationary(stationary_time))
                        {
                            mission_state = LANDING;
                            Logger::print_color(int(LogColor::green), node_name, "Target stationary! Switching to LANDING");
                        }
                    }
                }
                break;
            }

            case LANDING:
            {
                // 检查是否丢失目标
                if ((ros::Time::now() - last_detection_time).toSec() > 1.0)
                {
                    const double lost_time = (ros::Time::now() - last_detection_time).toSec();
                    const bool close_to_target_before_loss =
                        abs(target_rel_x) < 2.0 * error_xy &&
                        abs(target_rel_y) < 2.0 * error_xy;
                    const bool low_altitude_before_loss = abs(target_rel_z) < 0.6;

                    if (close_to_target_before_loss && low_altitude_before_loss)
                    {
                        Logger::print_color(int(LogColor::yellow), node_name,
                                            "Lost target at low altitude, continuing final descent...");
                        uav_cmd.header.stamp = ros::Time::now();
                        uav_cmd.cmd = sunray_msgs::UAVControlCMD::XyzVelYawBody;
                        uav_cmd.desired_vel[0] = 0.0;
                        uav_cmd.desired_vel[1] = 0.0;
                        uav_cmd.desired_vel[2] = -land_vel;
                        uav_cmd.desired_yaw = 0.0;
                        control_cmd_pub.publish(uav_cmd);

                        if (lost_time > 3.0)
                        {
                            Logger::print_color(int(LogColor::yellow), node_name,
                                                "Blind descent timeout reached, switching to Land command");
                            uav_cmd.header.stamp = ros::Time::now();
                            uav_cmd.cmd = sunray_msgs::UAVControlCMD::Land;
                            control_cmd_pub.publish(uav_cmd);
                            ros::Duration(0.5).sleep();
                            return 0;
                        }
                    }
                    else
                    {
                        Logger::print_color(int(LogColor::yellow), node_name, "Lost target during landing! Rising...");
                        uav_cmd.header.stamp = ros::Time::now();
                        uav_cmd.cmd = sunray_msgs::UAVControlCMD::XyzPosYawBody;
                        uav_cmd.desired_pos[0] = 0;
                        uav_cmd.desired_pos[1] = 0;
                        uav_cmd.desired_pos[2] = 0.1;
                        uav_cmd.desired_yaw = 0;
                        control_cmd_pub.publish(uav_cmd);

                        if (lost_time > 5.0)
                        {
                            Logger::print_color(int(LogColor::red), node_name, "Landing timeout, emergency land!");
                            uav_cmd.cmd = sunray_msgs::UAVControlCMD::Land;
                            control_cmd_pub.publish(uav_cmd);
                            ros::Duration(0.5).sleep();
                            return 0;
                        }
                    }
                }
                else
                {
                    if (landing_procedure(error_xy, error_z, drop_height, land_vel, land_time,
                                        control_cmd_pub, uav_setup_pub))
                    {
                        Logger::print_color(int(LogColor::green), node_name, "Mission completed!");
                        ros::Duration(0.5).sleep();
                        return 0;
                    }
                }
                break;
            }
        }

        ros::Duration(0.05).sleep();
    }

    return 0;
}
