/*
    程序功能：无人机自主起飞，多种方式搜索二维码，发现二维码后将其全局坐标发布到话题
    功能说明：
    1. 无人机自动起飞到指定高度
    2. 支持多种搜索模式：
       - 模式0 (circle_current): 以当前位置为圆心画圆搜索
       - 模式1 (circle_fixed): 以指定坐标为圆心画圆搜索
       - 模式2 (spiral): 螺旋搜索（从中心向外扩展）
       - 模式3 (grid): 栅格搜索（来回扫描）
    3. 通过下视相机检测二维码
    4. 检测到二维码后，将相对坐标转换为全局坐标
    5. 将全局坐标发布到新话题供无人车使用
*/

#include "ros_msg_utils.h"
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

// 全局变量
string node_name;
int uav_id;
string uav_name;

sunray_msgs::UAVState uav_state;
sunray_msgs::UAVSetup uav_setup;
sunray_msgs::UAVControlCMD uav_cmd;

// 二维码检测相关
bool target_detected = false;
bool uav_state_received = false;
double target_rel_x = 0.0;  // 相对坐标 (相机坐标系)
double target_rel_y = 0.0;
double target_rel_z = 0.0;
ros::Time last_detection_time;

// 滤波相关
const int FILTER_SIZE = 10;           // 滤波窗口大小
const double OUTLIER_THRESHOLD = 2.0; // 异常值阈值(米)
vector<double> filter_x, filter_y, filter_z;
int detection_count = 0;              // 有效检测计数

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

// 计算中值
double get_median(vector<double> &data)
{
    if (data.empty()) return 0.0;
    vector<double> sorted_data = data;
    sort(sorted_data.begin(), sorted_data.end());
    int n = sorted_data.size();
    if (n % 2 == 0)
        return (sorted_data[n/2 - 1] + sorted_data[n/2]) / 2.0;
    else
        return sorted_data[n/2];
}

// 检查是否为异常值
bool is_outlier(double value, vector<double> &data, double threshold)
{
    if (data.size() < 3) return false;  // 数据太少，不判断异常
    double median = get_median(data);
    return fabs(value - median) > threshold;
}

// 二维码检测回调 (下视相机)
void qrcode_callback(const sunray_msgs::TargetsInFrameMsg::ConstPtr &msg)
{
    if (msg->targets.size() > 0)
    {
        last_detection_time = ros::Time::now();

        // 获取目标相对位置 (相机坐标系下)
        double new_x = msg->targets[0].px;
        double new_y = msg->targets[0].py;
        double new_z = msg->targets[0].pz;

        // 异常值检测：如果距离太远（超过10米）直接丢弃
        if (fabs(new_x) > 10.0 || fabs(new_y) > 10.0 || fabs(new_z) > 10.0)
        {
            std::ostringstream oss;
            oss << std::fixed << std::setprecision(2)
                << "QRCode outlier rejected (too far): [" << new_x << ", " << new_y << ", " << new_z << "]";
            Logger::print_color(int(LogColor::yellow), node_name, oss.str());
            return;
        }

        // 如果已有数据，检查是否为异常值
        if (filter_x.size() >= 3)
        {
            if (is_outlier(new_x, filter_x, OUTLIER_THRESHOLD) ||
                is_outlier(new_y, filter_y, OUTLIER_THRESHOLD) ||
                is_outlier(new_z, filter_z, OUTLIER_THRESHOLD))
            {
                std::ostringstream oss;
                oss << std::fixed << std::setprecision(2)
                    << "QRCode outlier rejected: [" << new_x << ", " << new_y << ", " << new_z << "]";
                Logger::print_color(int(LogColor::yellow), node_name, oss.str());
                return;
            }
        }

        // 添加到滤波器
        filter_x.push_back(new_x);
        filter_y.push_back(new_y);
        filter_z.push_back(new_z);

        // 保持滤波器窗口大小
        if (filter_x.size() > FILTER_SIZE)
        {
            filter_x.erase(filter_x.begin());
            filter_y.erase(filter_y.begin());
            filter_z.erase(filter_z.begin());
        }

        // 使用中值滤波计算稳定值
        target_rel_x = get_median(filter_x);
        target_rel_y = get_median(filter_y);
        target_rel_z = get_median(filter_z);

        detection_count++;
        target_detected = true;

        std::ostringstream oss;
        oss << std::fixed << std::setprecision(2)
            << "QRCode detected! Raw: [" << new_x << ", " << new_y << ", " << new_z
            << "], Filtered: [" << target_rel_x << ", " << target_rel_y << ", " << target_rel_z
            << "], Count: " << detection_count;
        Logger::print_color(int(LogColor::green), node_name, oss.str());
    }
}

// 将相对坐标转换为全局坐标
geometry_msgs::Point convert_to_global(double rel_x, double rel_y, double rel_z)
{
    geometry_msgs::Point global_pos;

    // 获取无人机当前位置和姿态
    double uav_x = uav_state.position[0];
    double uav_y = uav_state.position[1];
    double uav_z = uav_state.position[2];
    double uav_yaw = uav_state.attitude[2];  // 偏航角

    // 下视相机坐标系到机体坐标系的转换
    double body_x = rel_x;
    double body_y = -rel_y;
    double body_z = -rel_z;

    // 机体坐标系到全局坐标系的转换 (只考虑yaw角)
    double cos_yaw = cos(uav_yaw);
    double sin_yaw = sin(uav_yaw);

    double global_offset_x = cos_yaw * body_x - sin_yaw * body_y;
    double global_offset_y = sin_yaw * body_x + cos_yaw * body_y;
    double global_offset_z = body_z;

    // 计算全局坐标
    global_pos.x = uav_x + global_offset_x;
    global_pos.y = uav_y + global_offset_y;
    global_pos.z = uav_z + global_offset_z;

    return global_pos;
}

// 检查是否检测到目标 (需要足够多的稳定检测)
bool check_target_detected(geometry_msgs::Point &target_global_pos)
{
    // 需要至少5次有效检测才确认目标
    const int MIN_DETECTION_COUNT = 5;

    if (target_detected &&
        detection_count >= MIN_DETECTION_COUNT &&
        (ros::Time::now() - last_detection_time).toSec() < 0.5)
    {
        std::ostringstream oss;
        oss << "Target confirmed with " << detection_count << " detections!";
        Logger::print_color(int(LogColor::green), node_name, oss.str());
        return true;
    }
    return false;
}

// 飞到目标正上方并精确定位
bool fly_above_target_and_localize(double search_height, double search_speed,
                                    ros::Rate &rate, ros::Publisher &control_cmd_pub,
                                    geometry_msgs::Point &target_global_pos)
{
    Logger::print_color(int(LogColor::blue), node_name, "Flying to target's overhead position...");

    double k_p = 0.8;
    int stable_count = 0;
    const int STABLE_THRESHOLD = 20;  // 需要稳定20次才认为到达正上方
    const double POSITION_THRESHOLD = 0.15;  // 位置误差阈值(米)

    // 清空滤波器，重新收集数据
    filter_x.clear();
    filter_y.clear();
    filter_z.clear();
    detection_count = 0;

    while (ros::ok())
    {
        ros::spinOnce();

        // 检查是否还能检测到目标
        if ((ros::Time::now() - last_detection_time).toSec() > 2.0)
        {
            Logger::print_color(int(LogColor::yellow), node_name, "Lost target while approaching!");
            return false;
        }

        // 计算需要移动的距离（让目标在正下方，即相对xy为0）
        // 下视相机坐标系: px为前方(机体x), py为右方(机体-y), pz为下方(机体-z)
        // 如果目标在前方(px>0)，无人机需要向前飞
        // 如果目标在右方(py>0)，无人机需要向右飞(机体y负方向)
        double body_vx = target_rel_x;   // 机体x方向速度
        double body_vy = -target_rel_y;  // 机体y方向速度（相机y与机体y相反）

        // 转换到全局坐标系
        double uav_yaw = uav_state.attitude[2];
        double cos_yaw = cos(uav_yaw);
        double sin_yaw = sin(uav_yaw);
        double global_vx = cos_yaw * body_vx - sin_yaw * body_vy;
        double global_vy = sin_yaw * body_vx + cos_yaw * body_vy;

        // 计算速度指令
        double vx = k_p * global_vx;
        double vy = k_p * global_vy;

        // 调试日志
        std::ostringstream debug_oss;
        debug_oss << std::fixed << std::setprecision(2)
            << "rel:[" << target_rel_x << "," << target_rel_y
            << "] body_v:[" << body_vx << "," << body_vy
            << "] yaw:" << std::setprecision(1) << (uav_yaw * 180.0 / M_PI)
            << " global_v:[" << std::setprecision(2) << vx << "," << vy << "]";
        Logger::print_color(int(LogColor::cyan), node_name, debug_oss.str());

        // 限制速度
        vx = min(max(vx, -search_speed), search_speed);
        vy = min(max(vy, -search_speed), search_speed);

        // 发送控制指令
        uav_cmd.header.stamp = ros::Time::now();
        uav_cmd.cmd = sunray_msgs::UAVControlCMD::XyVelZPos;
        uav_cmd.desired_vel[0] = vx;
        uav_cmd.desired_vel[1] = vy;
        uav_cmd.desired_pos[2] = search_height;
        control_cmd_pub.publish(uav_cmd);

        // 检查是否已经在目标正上方
        if (fabs(target_rel_x) < POSITION_THRESHOLD && fabs(target_rel_y) < POSITION_THRESHOLD)
        {
            stable_count++;
            std::ostringstream stabilizing_str;
            stabilizing_str << "Above target, stabilizing... " << stable_count << "/" << STABLE_THRESHOLD;
            Logger::print_color(int(LogColor::blue), node_name, stabilizing_str.str());

            if (stable_count >= STABLE_THRESHOLD)
            {
                // 已稳定在目标正上方，计算全局坐标
                std::ostringstream rel_pos_str;
                rel_pos_str << "Stable above target! Final relative pos: ["
                    << target_rel_x << ", " << target_rel_y << ", " << target_rel_z << "]";
                Logger::print_color(int(LogColor::green), node_name, rel_pos_str.str());

                target_global_pos = convert_to_global(target_rel_x, target_rel_y, target_rel_z);
                std::ostringstream global_pos_str;
                global_pos_str << "Target global position: ["
                    << target_global_pos.x << ", " << target_global_pos.y << ", " << target_global_pos.z << "]";
                Logger::print_color(int(LogColor::green), node_name, global_pos_str.str());
                return true;
            }
        }
        else
        {
            stable_count = 0;  // 重置稳定计数
        }

        rate.sleep();
    }
    return false;
}

// 飞向目标点并检测
bool fly_to_point_and_detect(double target_x, double target_y, double search_height,
                              double k_p, double max_vel, ros::Rate &rate,
                              ros::Publisher &control_cmd_pub,
                              geometry_msgs::Point &target_global_pos)
{
    while (ros::ok())
    {
        ros::spinOnce();

        // 检查是否检测到二维码
        if (check_target_detected(target_global_pos))
        {
            // 检测到目标，先飞到正上方再精确定位
            if (fly_above_target_and_localize(search_height, max_vel, rate,
                                               control_cmd_pub, target_global_pos))
            {
                return true;
            }
            else
            {
                // 飞到正上方失败（丢失目标），继续搜索
                Logger::print_color(int(LogColor::yellow), node_name,
                    "Failed to localize, continuing search...");
                // 重置检测状态
                target_detected = false;
                detection_count = 0;
                filter_x.clear();
                filter_y.clear();
                filter_z.clear();
            }
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
                   ros::Rate &rate, ros::Publisher &control_cmd_pub,
                   geometry_msgs::Point &target_global_pos)
{
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(2)
        << "Circle search. Center: [" << center_x << ", " << center_y
        << "], Radius: " << std::setprecision(1) << search_radius << " m";
    Logger::print_color(int(LogColor::blue), node_name, oss.str());

    int num_points = 50;
    double k_p = 1.0;

    for (int circle = 0; circle < search_circles && ros::ok(); circle++)
    {
        std::ostringstream circle_str;
        circle_str << "Circle " << circle + 1 << "/" << search_circles;
        Logger::print_color(int(LogColor::blue), node_name, circle_str.str());

        for (int i = 0; i < num_points && ros::ok(); i++)
        {
            double theta = i * 2 * M_PI / num_points;
            double target_x = center_x + search_radius * cos(theta);
            double target_y = center_y + search_radius * sin(theta);

            if (fly_to_point_and_detect(target_x, target_y, search_height, k_p, search_speed,
                                        rate, control_cmd_pub, target_global_pos))
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
                   ros::Rate &rate, ros::Publisher &control_cmd_pub,
                   geometry_msgs::Point &target_global_pos)
{
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(2)
        << "Spiral search. Center: [" << center_x << ", " << center_y
        << "], Max radius: " << std::setprecision(1) << search_radius << " m";
    Logger::print_color(int(LogColor::blue), node_name, oss.str());

    int total_points = 100 * search_circles;  // 总点数
    double k_p = 1.0;
    double spiral_spacing = search_radius / search_circles;  // 螺旋间距

    for (int i = 0; i < total_points && ros::ok(); i++)
    {
        double progress = (double)i / total_points;
        double current_radius = progress * search_radius;
        double theta = progress * search_circles * 2 * M_PI;

        double target_x = center_x + current_radius * cos(theta);
        double target_y = center_y + current_radius * sin(theta);

        std::ostringstream progress_oss;
        progress_oss << std::fixed << std::setprecision(1)
            << "Spiral progress: " << (progress * 100) << "%, radius: "
            << std::setprecision(2) << current_radius << " m";
        Logger::print_color(int(LogColor::blue), node_name, progress_oss.str());

        if (fly_to_point_and_detect(target_x, target_y, search_height, k_p, search_speed,
                                    rate, control_cmd_pub, target_global_pos))
        {
            return true;
        }
    }
    return false;
}

// 栅格搜索
bool search_grid(double center_x, double center_y, double search_height,
                 double search_radius, double search_speed, int grid_lines,
                 ros::Rate &rate, ros::Publisher &control_cmd_pub,
                 geometry_msgs::Point &target_global_pos)
{
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(2)
        << "Grid search. Center: [" << center_x << ", " << center_y
        << "], Size: " << std::setprecision(1) << (search_radius * 2) << " x " << (search_radius * 2) << " m";
    Logger::print_color(int(LogColor::blue), node_name, oss.str());

    double k_p = 1.0;
    double grid_spacing = (search_radius * 2) / (grid_lines - 1);

    // 起始点
    double start_x = center_x - search_radius;
    double start_y = center_y - search_radius;

    for (int row = 0; row < grid_lines && ros::ok(); row++)
    {
        std::ostringstream row_str;
        row_str << "Grid row " << row + 1 << "/" << grid_lines;
        Logger::print_color(int(LogColor::blue), node_name, row_str.str());

        // 蛇形路径：偶数行从左到右，奇数行从右到左
        if (row % 2 == 0)
        {
            for (int col = 0; col < grid_lines && ros::ok(); col++)
            {
                double target_x = start_x + col * grid_spacing;
                double target_y = start_y + row * grid_spacing;

                if (fly_to_point_and_detect(target_x, target_y, search_height, k_p, search_speed,
                                            rate, control_cmd_pub, target_global_pos))
                {
                    return true;
                }
            }
        }
        else
        {
            for (int col = grid_lines - 1; col >= 0 && ros::ok(); col--)
            {
                double target_x = start_x + col * grid_spacing;
                double target_y = start_y + row * grid_spacing;

                if (fly_to_point_and_detect(target_x, target_y, search_height, k_p, search_speed,
                                            rate, control_cmd_pub, target_global_pos))
                {
                    return true;
                }
            }
        }
    }
    return false;
}

int main(int argc, char **argv)
{
    // 设置日志
    Logger::init_default();
    Logger::setPrintLevel(false);
    Logger::setPrintTime(false);

    ros::init(argc, argv, "search_target_and_send_to_ugv");
    ros::NodeHandle nh("~");
    ros::Rate rate(20.0);

    signal(SIGINT, mySigintHandler);

    node_name = ros::this_node::getName();
    node_name = "[" + node_name + "]:";

    // 参数读取
    nh.param<int>("uav_id", uav_id, 1);
    nh.param<string>("uav_name", uav_name, "uav");

    // 搜索参数
    int search_mode;            // 搜索模式
    double search_height;       // 搜索高度
    double search_radius;       // 搜索半径/范围
    double search_speed;        // 搜索速度
    int search_circles;         // 搜索圈数/栅格行数
    double center_x;            // 指定圆心X坐标
    double center_y;            // 指定圆心Y坐标

    nh.param<int>("search_mode", search_mode, 0);
    nh.param<double>("search_height", search_height, 5.0);
    nh.param<double>("search_radius", search_radius, 3.0);
    nh.param<double>("search_speed", search_speed, 0.5);
    nh.param<int>("search_circles", search_circles, 3);
    nh.param<double>("center_x", center_x, 0.0);
    nh.param<double>("center_y", center_y, 0.0);

    uav_name = "/" + uav_name + to_string(uav_id);

    // 打印搜索模式
    string mode_names[] = {"CIRCLE_CURRENT", "CIRCLE_FIXED", "SPIRAL", "GRID"};
    std::ostringstream mode_str;
    mode_str << "Search mode: " << mode_names[search_mode] << " (" << search_mode << ")";
    Logger::print_color(int(LogColor::blue), node_name, mode_str.str());

    // 【订阅】无人机状态
    ros::Subscriber uav_state_sub = nh.subscribe<sunray_msgs::UAVState>(
        uav_name + "/sunray/uav_state", 10, uav_state_callback);

    // 【订阅】二维码检测结果 (下视相机)
    ros::Subscriber qrcode_sub = nh.subscribe<sunray_msgs::TargetsInFrameMsg>(
        uav_name + "/sunray_detect/qrcode_detection_ros", 1, qrcode_callback);

    // 【发布】无人机控制指令
    ros::Publisher control_cmd_pub = nh.advertise<sunray_msgs::UAVControlCMD>(
        uav_name + "/sunray/uav_control_cmd", 1);

    // 【发布】无人机设置指令
    ros::Publisher uav_setup_pub = nh.advertise<sunray_msgs::UAVSetup>(
        uav_name + "/sunray/setup", 1);

    // 【发布】目标全局坐标 (新话题，供无人车使用)
    ros::Publisher target_global_pub = nh.advertise<geometry_msgs::PointStamped>(
        "/target_global_position", 1);

    // 【发布】目标点给UGV EGO规划器
    // 通过 /move_base_simple/goal 发布，goal2swarm节点会转发到 /goal_1
    ros::Publisher ugv_goal_pub = nh.advertise<geometry_msgs::PoseStamped>(
        "/move_base_simple/goal", 1);

    // 控制辅助类初始化
    Control_Utils uav_control_utils;
    uav_control_utils.init(nh, uav_id, node_name);

    // ==================== 1. 等待无人机连接 ====================
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

    // ==================== 2. 自动起飞 ====================
    Logger::print_color(int(LogColor::blue), node_name, "Starting auto takeoff...");
    uav_control_utils.auto_takeoff();
    Logger::print_color(int(LogColor::green), node_name, "Takeoff completed!");

    // 等待稳定
    ros::Duration(3.0).sleep();
    ros::spinOnce();

    // ==================== 3. 上升到搜索高度 ====================
    std::ostringstream height_str;
    height_str << "Ascending to search height: " << search_height << " m";
    Logger::print_color(int(LogColor::blue), node_name, height_str.str());

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

    // ==================== 4. 确定搜索圆心 ====================
    double actual_center_x, actual_center_y;

    if (search_mode == CIRCLE_CURRENT || search_mode == SPIRAL)
    {
        // 以当前位置为圆心
        actual_center_x = uav_state.position[0];
        actual_center_y = uav_state.position[1];
        std::ostringstream center_oss;
        center_oss << std::fixed << std::setprecision(2)
            << "Using current position as center: [" << actual_center_x << ", " << actual_center_y << "]";
        Logger::print_color(int(LogColor::blue), node_name, center_oss.str());
    }
    else
    {
        // 使用指定坐标
        actual_center_x = center_x;
        actual_center_y = center_y;
        std::ostringstream center_oss;
        center_oss << std::fixed << std::setprecision(2)
            << "Using fixed center: [" << actual_center_x << ", " << actual_center_y << "]";
        Logger::print_color(int(LogColor::blue), node_name, center_oss.str());

        // 先飞到指定圆心位置
        Logger::print_color(int(LogColor::blue), node_name, "Flying to search center...");
        geometry_msgs::Point dummy;
        fly_to_point_and_detect(actual_center_x, actual_center_y, search_height,
                                1.0, search_speed, rate, control_cmd_pub, dummy);
    }

    // ==================== 5. 执行搜索 ====================
    bool found_target = false;
    geometry_msgs::Point target_global_pos;

    switch (search_mode)
    {
        case CIRCLE_CURRENT:
        case CIRCLE_FIXED:
            found_target = search_circle(actual_center_x, actual_center_y, search_height,
                                         search_radius, search_speed, search_circles,
                                         rate, control_cmd_pub, target_global_pos);
            break;

        case SPIRAL:
            found_target = search_spiral(actual_center_x, actual_center_y, search_height,
                                         search_radius, search_speed, search_circles,
                                         rate, control_cmd_pub, target_global_pos);
            break;

        case GRID:
            found_target = search_grid(actual_center_x, actual_center_y, search_height,
                                       search_radius, search_speed, search_circles,
                                       rate, control_cmd_pub, target_global_pos);
            break;

        default:
            std::ostringstream unknown_str;
            unknown_str << "Unknown search mode: " << search_mode;
            Logger::print_color(int(LogColor::red), node_name, unknown_str.str());
            break;
    }

    // ==================== 6. 发布目标位置并悬停 ====================
    if (found_target)
    {
        Logger::print_color(int(LogColor::green), node_name, "Publishing target position to UGV...");

        // 发布目标全局坐标 (PointStamped)
        geometry_msgs::PointStamped target_msg;
        target_msg.header.stamp = ros::Time::now();
        target_msg.header.frame_id = "world";
        target_msg.point = target_global_pos;

        // 发布目标点给UGV EGO规划器 (PoseStamped)
        geometry_msgs::PoseStamped ugv_goal_msg;
        ugv_goal_msg.header.stamp = ros::Time::now();
        ugv_goal_msg.header.frame_id = "world";
        ugv_goal_msg.pose.position.x = target_global_pos.x;
        ugv_goal_msg.pose.position.y = target_global_pos.y;
        ugv_goal_msg.pose.position.z = 0.05;  // UGV地面高度
        ugv_goal_msg.pose.orientation.w = 1.0;

        // 持续发布一段时间确保无人车收到，同时保持悬停
        uav_cmd.header.stamp = ros::Time::now();
        uav_cmd.cmd = sunray_msgs::UAVControlCMD::Hover;

        for (int i = 0; i < 50 && ros::ok(); i++)
        {
            target_global_pub.publish(target_msg);
            ugv_goal_pub.publish(ugv_goal_msg);
            control_cmd_pub.publish(uav_cmd);  // 发送悬停指令防止漂移
            ros::spinOnce();
            ros::Duration(0.1).sleep();
        }

        std::ostringstream sent_oss;
        sent_oss << std::fixed << std::setprecision(2)
            << "Target position sent to UGV: [" << target_global_pos.x << ", " << target_global_pos.y << "]";
        Logger::print_color(int(LogColor::green), node_name, sent_oss.str());
    }
    else
    {
        Logger::print_color(int(LogColor::yellow), node_name, "No target found during search.");
    }

    // ==================== 7. 悬停等待 ====================
    Logger::print_color(int(LogColor::blue), node_name, "Hovering...");
    uav_cmd.header.stamp = ros::Time::now();
    uav_cmd.cmd = sunray_msgs::UAVControlCMD::Hover;
    control_cmd_pub.publish(uav_cmd);

    ros::Duration(5.0).sleep();

    // ==================== 8. 返航降落 ====================
    Logger::print_color(int(LogColor::blue), node_name, "Returning to home and landing...");
    uav_control_utils.auto_return();

    Logger::print_color(int(LogColor::green), node_name, "Mission completed!");

    return 0;
}
