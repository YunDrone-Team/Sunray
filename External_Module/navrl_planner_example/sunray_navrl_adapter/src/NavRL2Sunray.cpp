#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Header.h>
#include <sunray_msgs/UAVControlCMD.h>

#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

/*
 * NavRL2Sunray.cpp
 *
 * 这个文件实现了一个 ROS 节点：NavRL2Sunray_node。
 *
 * 它的作用是“协议适配”：
 *   1. NavRL 算法输出 ROS 标准消息，例如：
 *      - geometry_msgs::TwistStamped：速度指令
 *      - geometry_msgs::PoseStamped：位置/朝向指令
 *   2. Sunray无人机控制框架接口需要的是：
 *      - sunray_msgs::UAVControlCMD
 *   3. 本节点订阅 NavRL 的输出话题，把消息字段转换成 Sunray无人机控制框架指令，
 *      再发布到 /uav1/sunray/uav_control_cmd。
 *
 * 对第一次阅读 C++/ROS 的编程者，建议按这个顺序理解本文件：
 *   - 先看全局变量：它们保存参数、话题名和最近收到的数据。
 *   - 再看几个辅助函数：它们负责时间检查、角度换算、填充消息头等小任务。
 *   - 再看 callback：ROS 收到消息后会自动调用这些函数。
 *   - 最后看 main()：它负责初始化节点、读取参数、建立订阅/发布关系。
 */

// ----------------------------- ROS发布器和运行参数 -----------------------------
//
// g_ 前缀表示 global，全局变量。这个文件是一个很小的 ROS 节点，
// 所以直接用全局变量保存状态，避免引入不必要的 class 封装。
//
// ros::Publisher 是 ROS 的“发布器”，后续通过 g_control_cmd_pub.publish(...)
// 把 Sunray无人机控制框架指令发给 uav_control。
ros::Publisher g_control_cmd_pub;

// NavRL 的速度指令只给 vx/vy/vz。Sunray无人机控制框架的 XyVelZPosYaw 控制模式需要：
//   - x/y 使用速度控制
//   - z 使用位置控制
// 因此这里用 fixed_height 作为固定飞行高度。
double g_fixed_height = 1.0;

// 当还没有收到 NavRL setpoint_pose 时，速度指令没有 yaw 字段，
// 这里的 desired_yaw 会作为默认机头朝向。
double g_desired_yaw = 0.0;

// 发布给 Sunray无人机控制框架的控制指令 header.frame_id。
// 它只是消息坐标系标识，具体语义要和上游定位/控制模块保持一致。
std::string g_frame_id = "map";

// 以下字符串是 ROS 话题名。默认值可以被 launch 文件中的私有参数覆盖。
//   g_cmd_vel_topic       : NavRL 输出的速度指令
//   g_setpoint_pose_topic : NavRL 输出的位置/朝向指令
//   g_goal_topic          : RViz 或外部发送的目标点
//   g_odom_topic          : Sunray无人机控制框架当前无人机里程计
//   g_output_topic        : 本节点输出给 Sunray无人机控制框架控制模块的指令
std::string g_cmd_vel_topic = "/CERLAB/quadcopter/cmd_vel";
std::string g_setpoint_pose_topic = "/CERLAB/quadcopter/setpoint_pose";
std::string g_goal_topic = "/move_base_simple/goal";
std::string g_odom_topic = "/uav1/sunray/uav_odom";
std::string g_output_topic = "/uav1/sunray/uav_control_cmd";

// ----------------------------- 最近一次输入状态缓存 -----------------------------
//
// ROS 的 callback 只在收到消息时触发。为了在终端状态面板里显示“最近收到什么”，
// 这里保存每个输入话题的最近一帧数据，以及是否收到过该类型消息。
bool g_has_cmd_vel = false;
bool g_has_setpoint_pose = false;
bool g_has_odom = false;
bool g_has_goal = false;
bool g_has_navrl_yaw = false;

// 每类消息最近一次到达本节点的时间。注意这里记录的是 ros::Time::now()，
// 即本机收到消息的时间，而不是消息 header.stamp。
ros::Time g_last_cmd_vel_time;
ros::Time g_last_setpoint_pose_time;
ros::Time g_last_odom_time;
ros::Time g_last_goal_time;

// 最近一次收到的完整消息。保存完整消息比只保存某几个字段更方便调试和扩展。
geometry_msgs::TwistStamped g_last_cmd_vel;
geometry_msgs::PoseStamped g_last_setpoint_pose;
geometry_msgs::PoseStamped g_last_goal;
nav_msgs::Odometry g_last_odom;

// NavRL 的 setpoint_pose 中带有四元数姿态，这里只提取 yaw 保存下来。
// 后续速度控制指令会沿用这个 yaw。
double g_last_navrl_yaw = 0.0;

// ----------------------------- 终端显示相关常量 -----------------------------
//
// 这些字符串是 ANSI 颜色控制码，只影响终端输出颜色，不影响 ROS 消息。
// 如果终端不支持颜色，最坏情况只是看到一些转义字符。
constexpr const char* kAnsiReset = "\033[0m";
constexpr const char* kAnsiTitle = "\033[1;36m";
constexpr const char* kAnsiGood = "\033[1;32m";

// 从 geometry_msgs::Pose 的四元数中提取 yaw。
//
// ROS 中姿态常用四元数表示，字段是 x/y/z/w。
// 对无人机平面导航来说，常用的是 yaw，也就是绕 z 轴旋转的角度。
// 这里使用标准四元数转欧拉角公式中的 yaw 部分。
double yawFromPose(const geometry_msgs::Pose& pose) {
    const double x = pose.orientation.x;
    const double y = pose.orientation.y;
    const double z = pose.orientation.z;
    const double w = pose.orientation.w;
    return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

// asin() 的输入理论上必须在 [-1, 1]。
// 四元数经过浮点计算后可能出现 1.00000001 这种极小误差，
// 直接传给 asin() 会得到 NaN，所以这里先做夹紧。
double clampUnit(const double value) {
    if (value > 1.0) {
        return 1.0;
    }
    if (value < -1.0) {
        return -1.0;
    }
    return value;
}

// 从四元数中同时提取 roll/pitch/yaw，主要用于状态面板显示。
// 本节点的控制转换只真正使用 yaw，但显示 roll/pitch 可以帮助排查机体姿态异常。
void eulerFromPose(const geometry_msgs::Pose& pose, double& roll, double& pitch, double& yaw) {
    const double qx = pose.orientation.x;
    const double qy = pose.orientation.y;
    const double qz = pose.orientation.z;
    const double qw = pose.orientation.w;

    roll = std::atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy));
    pitch = std::asin(clampUnit(2.0 * (qw * qy - qz * qx)));
    yaw = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
}

// ROS 和 C++ 三角函数通常使用弧度，终端显示时用角度更直观。
double radToDeg(const double rad) {
    return rad * 180.0 / M_PI;
}

// 速度指令 TwistStamped 没有 yaw 字段。
// 如果已经收到过 NavRL 的 setpoint_pose，就使用其中的 yaw；
// 否则使用参数 desired_yaw 作为默认值。
double activeVelocityYaw() {
    return g_has_navrl_yaw ? g_last_navrl_yaw : g_desired_yaw;
}

// 填充 Sunray无人机控制框架控制指令的 header。
//
// header 通常包括：
//   stamp    : 时间戳
//   frame_id : 坐标系名称
//
// 这里先复制输入消息的 header，再把 stamp 改成当前时间，
// 表示这条控制指令是在“现在”生成的。
void fillControlHeader(sunray_msgs::UAVControlCMD& control_cmd, const std_msgs::Header& input_header) {
    control_cmd.header = input_header;
    control_cmd.header.stamp = ros::Time::now();
    if (!g_frame_id.empty()) {
        control_cmd.header.frame_id = g_frame_id;
    }
}

// 获取当前目标点位置，用于状态面板显示。
//
// RViz 发来的 /move_base_simple/goal 可能带有任意 z 值。
// NavRL 的实际控制语义会把目标 z 当作固定飞行高度处理，
// 所以这里也把显示用的 goal.z 改成 g_fixed_height。
geometry_msgs::Point activeGoalPosition() {
    geometry_msgs::Point goal = g_last_goal.pose.position;
    // NavRL 收到 /move_base_simple/goal 后会把目标z覆盖为起飞/定高高度。
    // 这里打印距离时使用相同语义，避免RViz目标点原始z值造成距离判断不一致。
    goal.z = g_fixed_height;
    return goal;
}

// 计算无人机当前位置到目标点的三维距离，仅用于终端显示。
// 如果还没有目标点或里程计，返回 0.0。
double distanceToGoal() {
    if (!g_has_goal || !g_has_odom) {
        return 0.0;
    }

    const geometry_msgs::Point goal = activeGoalPosition();
    const geometry_msgs::Point& pos = g_last_odom.pose.pose.position;
    const double dx = pos.x - goal.x;
    const double dy = pos.y - goal.y;
    const double dz = pos.z - goal.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

// 生成终端状态面板文字。
//
// std::ostringstream 可以理解成“可以不断 << 写入的字符串构造器”。
// 本函数只负责拼接显示文本，不发布任何 ROS 消息。
std::string buildStatusPanel() {
    std::ostringstream ss;

    // fixed + setprecision(2) 表示后续浮点数统一显示两位小数。
    ss << std::fixed << std::setprecision(2);

    ss << kAnsiTitle << "=================== NavRL2Sunray_node ===================" << kAnsiReset << "\n";

    ss << kAnsiGood << " 目标信息 " << kAnsiReset
       << "目标话题（订阅） -> " << g_goal_topic << "\n";
    if (g_has_goal) {
        const geometry_msgs::Point goal = activeGoalPosition();
        ss << "          target = (" << goal.x << ", " << goal.y << ", " << goal.z << ") m";
        if (g_has_odom) {
            ss << "  distance = " << distanceToGoal() << " m";
        } else {
            ss << "  distance = 等待ODOM";
        }
    } else {
        ss << "          target = 等待目标  distance = -";
    }
    ss << "\n";

    ss << kAnsiGood << " 控制输入 " << kAnsiReset
       << "速度话题（订阅） -> " << g_cmd_vel_topic << "\n";
    if (g_has_cmd_vel) {
        ss << "           vx = " << g_last_cmd_vel.twist.linear.x << " m/s"
           << "  vy = " << g_last_cmd_vel.twist.linear.y << " m/s"
           << "  vz输入 = " << g_last_cmd_vel.twist.linear.z << " m/s"
           << "  fixed_height = " << g_fixed_height << " m"
           << "  yaw = " << radToDeg(activeVelocityYaw()) << " deg"
           << "\n";
    }

    ss << kAnsiGood << " 位置输入 " << kAnsiReset
       << "位姿话题（订阅） -> " << g_setpoint_pose_topic << "\n";
    if (g_has_setpoint_pose) {
        ss << "           x = " << g_last_setpoint_pose.pose.position.x << " m"
           << "  y = " << g_last_setpoint_pose.pose.position.y << " m"
           << "  z = " << g_last_setpoint_pose.pose.position.z << " m"
           << "  yaw = " << radToDeg(g_last_navrl_yaw) << " deg"
           << "\n";
    }

    ss << kAnsiGood << " 本机状态 " << kAnsiReset
       << "里程计话题（订阅） -> " << g_odom_topic << "\n";
    if (g_has_odom) {
        double roll = 0.0;
        double pitch = 0.0;
        double yaw = 0.0;
        eulerFromPose(g_last_odom.pose.pose, roll, pitch, yaw);

        ss << "          pos = (" << g_last_odom.pose.pose.position.x << ", "
           << g_last_odom.pose.pose.position.y << ", "
           << g_last_odom.pose.pose.position.z << ") m"
           << "\n"
           << "           vel = (" << g_last_odom.twist.twist.linear.x << ", "
           << g_last_odom.twist.twist.linear.y << ", "
           << g_last_odom.twist.twist.linear.z << ") m/s"
           << "\n"
           << "           euler = (roll " << radToDeg(roll)
           << ", pitch " << radToDeg(pitch)
           << ", yaw " << radToDeg(yaw) << ") deg"
           << "\n";
    } else {
        ss << "          pos = 等待ODOM\n";
    }

    ss << kAnsiGood << " 控制输出 " << kAnsiReset
       << "Sunray控制话题（发布） -> " << g_output_topic;

    return ss.str();
}

// ROS 定时器回调：按照 status_print_hz 的频率打印一次状态面板。
// ros::TimerEvent 是 ROS 定时器传入的事件信息，这里不需要使用，所以参数没有命名。
void printStatusCallback(const ros::TimerEvent&) {
    std::cout << buildStatusPanel() << std::endl;
}

// 目标点回调。
//
// 这个回调只缓存目标点，不直接发布控制指令。
// 原因是目标点主要用于 NavRL 算法自身规划，本适配器只是把它显示出来，
// 方便确认当前导航目标和距离。
void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    g_has_goal = true;
    g_last_goal_time = ros::Time::now();
    g_last_goal = *msg;
}

// NavRL 速度指令回调。
//
// 当 g_cmd_vel_topic 收到 geometry_msgs::TwistStamped 时，ROS 会调用本函数。
// 本函数把 NavRL 的速度指令转换成 Sunray无人机控制框架的 UAVControlCMD。
void navrlCmdVelCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
    // 1. 先缓存输入，供状态面板显示和调试使用。
    g_has_cmd_vel = true;
    g_last_cmd_vel_time = ros::Time::now();
    g_last_cmd_vel = *msg;

    // 2. 创建一条新的 Sunray无人机控制框架控制指令。
    sunray_msgs::UAVControlCMD control_cmd;

    // 3. 填充 header，保留输入坐标系信息，并刷新时间戳。
    fillControlHeader(control_cmd, msg->header);

    // 4. 选择 Sunray无人机控制框架的控制模式：XyVelZPosYaw。
    //
    // 这个模式的含义是：
    //   - x/y 方向按照 desired_vel[0/1] 做速度控制
    //   - z 方向按照 desired_pos[2] 做高度/位置控制
    //   - yaw 按 desired_yaw 控制机头朝向
    //
    // 因此这里不会使用 NavRL 给出的 linear.z 作为 z 速度，
    // 而是把 z 固定到 g_fixed_height。
    control_cmd.cmd = sunray_msgs::UAVControlCMD::XyVelZPosYaw;
    control_cmd.desired_pos[0] = 0.0f;
    control_cmd.desired_pos[1] = 0.0f;
    control_cmd.desired_pos[2] = static_cast<float>(g_fixed_height);

    // static_cast<float>(...) 是 C++ 的显式类型转换。
    // ROS 消息中 desired_vel 是 float32，而 geometry_msgs::Twist 使用 double，
    // 所以这里显式转换，避免隐式转换带来的编译器告警或阅读歧义。
    control_cmd.desired_vel[0] = static_cast<float>(msg->twist.linear.x);
    control_cmd.desired_vel[1] = static_cast<float>(msg->twist.linear.y);
    control_cmd.desired_vel[2] = 0.0f;

    // NavRL 的非 PX4 速度话题 TwistStamped 没有 yaw 字段。
    // control_callback() 在发布速度前会先通过 setpoint_pose 发送 goal_angle，
    // 因此速度阶段沿用最近一次 setpoint_pose 的 yaw，避免把机头重新锁回默认 0 度。
    control_cmd.desired_yaw = static_cast<float>(activeVelocityYaw());
    control_cmd.desired_yaw_rate = 0.0f;

    // 5. 发布转换后的控制指令。Sunray无人机控制框架的 uav_control 节点会订阅该话题并执行控制。
    g_control_cmd_pub.publish(control_cmd);
}

// NavRL 位置/朝向指令回调。
//
// setpoint_pose 通常包含目标位置和目标朝向。
// 与速度回调不同，这里转换成 Sunray无人机控制框架的 XyzPosYaw 模式，即 xyz 都走位置控制。
void navrlSetpointPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    // 1. 缓存输入消息。
    g_has_setpoint_pose = true;
    g_last_setpoint_pose_time = ros::Time::now();
    g_last_setpoint_pose = *msg;

    // 2. 从四元数姿态中提取 yaw，并保存给后续速度指令复用。
    g_last_navrl_yaw = yawFromPose(msg->pose);
    g_has_navrl_yaw = true;

    // 3. 创建并填充 Sunray无人机控制框架控制指令。
    sunray_msgs::UAVControlCMD control_cmd;

    fillControlHeader(control_cmd, msg->header);

    // XyzPosYaw 表示 x/y/z 都使用位置控制，yaw 使用角度控制。
    control_cmd.cmd = sunray_msgs::UAVControlCMD::XyzPosYaw;
    control_cmd.desired_pos[0] = static_cast<float>(msg->pose.position.x);
    control_cmd.desired_pos[1] = static_cast<float>(msg->pose.position.y);
    control_cmd.desired_pos[2] = static_cast<float>(msg->pose.position.z);

    // 位置控制模式下，本适配器不额外给速度前馈，所以 desired_vel 全部置零。
    control_cmd.desired_vel[0] = 0.0f;
    control_cmd.desired_vel[1] = 0.0f;
    control_cmd.desired_vel[2] = 0.0f;
    control_cmd.desired_yaw = static_cast<float>(g_last_navrl_yaw);
    control_cmd.desired_yaw_rate = 0.0f;

    g_control_cmd_pub.publish(control_cmd);
}

// 里程计回调。
//
// 本节点不使用 odom 参与控制转换，只用于状态面板显示当前位置、速度和姿态，
// 以及计算当前位置到目标点的距离。
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    g_has_odom = true;
    g_last_odom_time = ros::Time::now();
    g_last_odom = *msg;
}

// C++ 程序入口。
//
// argc/argv 是命令行参数，ROS 会从中解析 __name、__ns、参数重映射等内容。
int main(int argc, char** argv) {
    // 初始化 ROS 节点。节点名会显示在 rosnode list 中。
    ros::init(argc, argv, "NavRL2Sunray_node");

    // NodeHandle 是 ROS 节点访问系统资源的入口。
    // nh 用于普通话题发布/订阅；
    // private_nh("~") 用于读取私有参数，例如 launch 中的 <param name="cmd_vel_topic" ...>。
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // queue_size 是 ROS 发布/订阅队列长度。
    // 如果回调处理速度跟不上消息到达速度，队列满后旧消息会被丢弃。
    int queue_size = 50;

    // 状态面板打印频率，单位 Hz。
    double status_print_hz = 1.0;

    // 从 ROS 参数服务器读取配置。
    //
    // private_nh.param("参数名", 变量, 默认值) 的含义是：
    //   如果 launch/命令行设置了该参数，就写入变量；
    //   如果没有设置，就使用第三个参数提供的默认值。
    //
    // input_topic 是旧参数名，为了兼容已有 launch 或脚本保留；
    // cmd_vel_topic 是更明确的新参数名，如果两者都设置，后读取的 cmd_vel_topic 生效。
    private_nh.param<std::string>("input_topic", g_cmd_vel_topic, g_cmd_vel_topic);
    private_nh.param<std::string>("cmd_vel_topic", g_cmd_vel_topic, g_cmd_vel_topic);
    private_nh.param<std::string>("setpoint_pose_topic", g_setpoint_pose_topic, g_setpoint_pose_topic);
    private_nh.param<std::string>("goal_topic", g_goal_topic, g_goal_topic);
    private_nh.param<std::string>("odom_topic", g_odom_topic, g_odom_topic);
    private_nh.param<std::string>("output_topic", g_output_topic, g_output_topic);
    private_nh.param<std::string>("frame_id", g_frame_id, g_frame_id);
    private_nh.param("fixed_height", g_fixed_height, g_fixed_height);
    private_nh.param("desired_yaw", g_desired_yaw, g_desired_yaw);
    private_nh.param("queue_size", queue_size, queue_size);
    private_nh.param("status_print_hz", status_print_hz, status_print_hz);

    // 防止用户把状态打印频率设为 0 或负数。
    // 下面会计算 1.0 / status_print_hz，如果不保护会出现除零或负周期。
    if (status_print_hz <= 0.0) {
        status_print_hz = 1.0;
    }

    // 建立发布器：本节点向 g_output_topic 发布 Sunray无人机控制框架控制指令。
    g_control_cmd_pub = nh.advertise<sunray_msgs::UAVControlCMD>(g_output_topic, queue_size);

    // 建立订阅器：每收到一条对应类型的消息，ROS 就调用第三个参数指定的回调函数。
    ros::Subscriber cmd_vel_sub = nh.subscribe(g_cmd_vel_topic, queue_size, navrlCmdVelCallback);
    ros::Subscriber setpoint_pose_sub = nh.subscribe(g_setpoint_pose_topic, queue_size, navrlSetpointPoseCallback);
    ros::Subscriber goal_sub = nh.subscribe(g_goal_topic, queue_size, goalCallback);
    ros::Subscriber odom_sub = nh.subscribe(g_odom_topic, queue_size, odomCallback);

    // 创建定时器。周期是 1 / 频率，例如 1Hz 表示每 1 秒触发一次。
    ros::Timer status_timer = nh.createTimer(ros::Duration(1.0 / status_print_hz), printStatusCallback);

    // 这些变量看起来没有在后面直接使用，但必须保持在 main() 作用域内存活。
    // 如果订阅器/定时器对象被析构，ROS 就会取消订阅或停止定时器。
    // (void)xxx 用来明确告诉编译器：“这是有意保留的变量，不要提示未使用警告”。
    (void)cmd_vel_sub;
    (void)setpoint_pose_sub;
    (void)goal_sub;
    (void)odom_sub;
    (void)status_timer;

    ROS_INFO_STREAM("NavRL2Sunray cmd_vel subscribe:       " << g_cmd_vel_topic);
    ROS_INFO_STREAM("NavRL2Sunray setpoint_pose subscribe: " << g_setpoint_pose_topic);
    ROS_INFO_STREAM("NavRL2Sunray goal subscribe:          " << g_goal_topic);
    ROS_INFO_STREAM("NavRL2Sunray odom subscribe:          " << g_odom_topic);
    ROS_INFO_STREAM("NavRL2Sunray publish:                 " << g_output_topic);
    ROS_INFO_STREAM("NavRL2Sunray fixed_height: " << g_fixed_height);
    ROS_INFO_STREAM("NavRL2Sunray default yaw before setpoint_pose: " << g_desired_yaw);
    ROS_INFO_STREAM("NavRL2Sunray status_print_hz: " << status_print_hz);

    // ros::spin() 会进入 ROS 事件循环。
    // 程序会停在这里，等待订阅消息和定时器事件；
    // 当收到消息时，ROS 自动调用上面注册的 callback。
    ros::spin();
    return 0;
}
