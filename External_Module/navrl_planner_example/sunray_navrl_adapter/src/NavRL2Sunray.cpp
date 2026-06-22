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

ros::Publisher g_control_cmd_pub;
double g_fixed_height = 1.0;
double g_desired_yaw = 0.0;
std::string g_frame_id = "map";
std::string g_cmd_vel_topic = "/CERLAB/quadcopter/cmd_vel";
std::string g_setpoint_pose_topic = "/CERLAB/quadcopter/setpoint_pose";
std::string g_goal_topic = "/move_base_simple/goal";
std::string g_odom_topic = "/uav1/sunray/localization/local_odom";
std::string g_output_topic = "/uav1/sunray/uav_control_cmd";

bool g_has_cmd_vel = false;
bool g_has_setpoint_pose = false;
bool g_has_odom = false;
bool g_has_goal = false;
bool g_has_navrl_yaw = false;
ros::Time g_last_cmd_vel_time;
ros::Time g_last_setpoint_pose_time;
ros::Time g_last_odom_time;
ros::Time g_last_goal_time;
geometry_msgs::TwistStamped g_last_cmd_vel;
geometry_msgs::PoseStamped g_last_setpoint_pose;
geometry_msgs::PoseStamped g_last_goal;
nav_msgs::Odometry g_last_odom;
double g_last_navrl_yaw = 0.0;

constexpr const char* kAnsiReset = "\033[0m";
constexpr const char* kAnsiTitle = "\033[1;36m";
constexpr const char* kAnsiGood = "\033[1;32m";
constexpr const char* kAnsiWarn = "\033[1;33m";
constexpr const char* kAnsiError = "\033[1;31m";
constexpr double kInputTimeoutSec = 0.5;

bool isFresh(const bool has_msg, const ros::Time& stamp) {
    return has_msg && !stamp.isZero() && (ros::Time::now() - stamp).toSec() <= kInputTimeoutSec;
}

std::string inputStateText(const bool has_msg, const ros::Time& stamp) {
    if (!has_msg) {
        return std::string(kAnsiWarn) + "等待" + kAnsiReset;
    }
    if (!isFresh(has_msg, stamp)) {
        return std::string(kAnsiError) + "超时" + kAnsiReset;
    }
    return std::string(kAnsiGood) + "正常" + kAnsiReset;
}

double yawFromPose(const geometry_msgs::Pose& pose) {
    const double x = pose.orientation.x;
    const double y = pose.orientation.y;
    const double z = pose.orientation.z;
    const double w = pose.orientation.w;
    return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

double clampUnit(const double value) {
    if (value > 1.0) {
        return 1.0;
    }
    if (value < -1.0) {
        return -1.0;
    }
    return value;
}

void eulerFromPose(const geometry_msgs::Pose& pose, double& roll, double& pitch, double& yaw) {
    const double qx = pose.orientation.x;
    const double qy = pose.orientation.y;
    const double qz = pose.orientation.z;
    const double qw = pose.orientation.w;

    roll = std::atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy));
    pitch = std::asin(clampUnit(2.0 * (qw * qy - qz * qx)));
    yaw = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
}

double radToDeg(const double rad) {
    return rad * 180.0 / M_PI;
}

double activeVelocityYaw() {
    return g_has_navrl_yaw ? g_last_navrl_yaw : g_desired_yaw;
}

void fillControlHeader(sunray_msgs::UAVControlCMD& control_cmd, const std_msgs::Header& input_header) {
    control_cmd.header = input_header;
    control_cmd.header.stamp = ros::Time::now();
    if (!g_frame_id.empty()) {
        control_cmd.header.frame_id = g_frame_id;
    }
}

geometry_msgs::Point activeGoalPosition() {
    geometry_msgs::Point goal = g_last_goal.pose.position;
    // NavRL 收到 /move_base_simple/goal 后会把目标z覆盖为起飞/定高高度。
    // 这里打印距离时使用相同语义，避免RViz目标点原始z值造成距离判断不一致。
    goal.z = g_fixed_height;
    return goal;
}

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

std::string buildStatusPanel() {
    std::ostringstream ss;
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
       << "速度话题（订阅） -> " << g_cmd_vel_topic
       << "  状态 = " << inputStateText(g_has_cmd_vel, g_last_cmd_vel_time) << "\n";
    if (g_has_cmd_vel) {
        ss << "           vx = " << g_last_cmd_vel.twist.linear.x << " m/s"
           << "  vy = " << g_last_cmd_vel.twist.linear.y << " m/s"
           << "  vz输入 = " << g_last_cmd_vel.twist.linear.z << " m/s"
           << "  fixed_height = " << g_fixed_height << " m"
           << "  yaw = " << radToDeg(activeVelocityYaw()) << " deg"
           << "\n";
    }

    ss << kAnsiGood << " 位置输入 " << kAnsiReset
       << "位姿话题（订阅） -> " << g_setpoint_pose_topic
       << "  状态 = " << inputStateText(g_has_setpoint_pose, g_last_setpoint_pose_time) << "\n";
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

void printStatusCallback(const ros::TimerEvent&) {
    std::cout << buildStatusPanel() << std::endl;
}

void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    g_has_goal = true;
    g_last_goal_time = ros::Time::now();
    g_last_goal = *msg;
}

void navrlCmdVelCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
    g_has_cmd_vel = true;
    g_last_cmd_vel_time = ros::Time::now();
    g_last_cmd_vel = *msg;

    sunray_msgs::UAVControlCMD control_cmd;

    fillControlHeader(control_cmd, msg->header);

    control_cmd.cmd = sunray_msgs::UAVControlCMD::XyVelZPosYaw;
    control_cmd.desired_pos[0] = 0.0f;
    control_cmd.desired_pos[1] = 0.0f;
    control_cmd.desired_pos[2] = static_cast<float>(g_fixed_height);
    control_cmd.desired_vel[0] = static_cast<float>(msg->twist.linear.x);
    control_cmd.desired_vel[1] = static_cast<float>(msg->twist.linear.y);
    control_cmd.desired_vel[2] = 0.0f;

    // NavRL 的非 PX4 速度话题 TwistStamped 没有 yaw 字段。
    // control_callback() 在发布速度前会先通过 setpoint_pose 发送 goal_angle，
    // 因此速度阶段沿用最近一次 setpoint_pose 的 yaw，避免把机头重新锁回默认 0 度。
    control_cmd.desired_yaw = static_cast<float>(activeVelocityYaw());
    control_cmd.desired_yaw_rate = 0.0f;

    g_control_cmd_pub.publish(control_cmd);
}

void navrlSetpointPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    g_has_setpoint_pose = true;
    g_last_setpoint_pose_time = ros::Time::now();
    g_last_setpoint_pose = *msg;
    g_last_navrl_yaw = yawFromPose(msg->pose);
    g_has_navrl_yaw = true;

    sunray_msgs::UAVControlCMD control_cmd;

    fillControlHeader(control_cmd, msg->header);

    control_cmd.cmd = sunray_msgs::UAVControlCMD::XyzPosYaw;
    control_cmd.desired_pos[0] = static_cast<float>(msg->pose.position.x);
    control_cmd.desired_pos[1] = static_cast<float>(msg->pose.position.y);
    control_cmd.desired_pos[2] = static_cast<float>(msg->pose.position.z);
    control_cmd.desired_vel[0] = 0.0f;
    control_cmd.desired_vel[1] = 0.0f;
    control_cmd.desired_vel[2] = 0.0f;
    control_cmd.desired_yaw = static_cast<float>(g_last_navrl_yaw);
    control_cmd.desired_yaw_rate = 0.0f;

    g_control_cmd_pub.publish(control_cmd);
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    g_has_odom = true;
    g_last_odom_time = ros::Time::now();
    g_last_odom = *msg;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "NavRL2Sunray_node");

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    int queue_size = 50;
    double status_print_hz = 1.0;

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

    if (status_print_hz <= 0.0) {
        status_print_hz = 1.0;
    }

    g_control_cmd_pub = nh.advertise<sunray_msgs::UAVControlCMD>(g_output_topic, queue_size);
    ros::Subscriber cmd_vel_sub = nh.subscribe(g_cmd_vel_topic, queue_size, navrlCmdVelCallback);
    ros::Subscriber setpoint_pose_sub = nh.subscribe(g_setpoint_pose_topic, queue_size, navrlSetpointPoseCallback);
    ros::Subscriber goal_sub = nh.subscribe(g_goal_topic, queue_size, goalCallback);
    ros::Subscriber odom_sub = nh.subscribe(g_odom_topic, queue_size, odomCallback);
    ros::Timer status_timer = nh.createTimer(ros::Duration(1.0 / status_print_hz), printStatusCallback);

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

    ros::spin();
    return 0;
}
