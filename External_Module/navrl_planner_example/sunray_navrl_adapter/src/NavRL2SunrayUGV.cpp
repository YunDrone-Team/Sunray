#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

namespace
{
ros::Publisher g_cmd_pub;

std::string g_cmd_vel_topic = "/CERLAB/quadcopter/cmd_vel";
std::string g_setpoint_pose_topic = "/CERLAB/quadcopter/setpoint_pose";
std::string g_goal_topic = "/move_base_simple/goal";
std::string g_odom_topic = "/ugv1/sunray_sim/odom";
std::string g_output_topic = "/ugv1/sunray/ugv_control/cmd_vel";

bool g_has_cmd_vel = false;
bool g_has_setpoint_pose = false;
bool g_has_goal = false;
bool g_has_odom = false;
ros::Time g_last_cmd_vel_time;
ros::Time g_last_setpoint_pose_time;
ros::Time g_last_goal_time;
ros::Time g_last_odom_time;
geometry_msgs::TwistStamped g_last_cmd_vel;
geometry_msgs::PoseStamped g_last_setpoint_pose;
geometry_msgs::PoseStamped g_last_goal;
nav_msgs::Odometry g_last_odom;

double g_max_linear_x = 1.5;
double g_max_linear_y = 1.5;
double g_max_angular_z = 1.5;
double g_yaw_kp = 1.5;
double g_goal_stop_distance = 0.4;
double g_status_print_hz = 1.0;

constexpr const char* kAnsiReset = "\033[0m";
constexpr const char* kAnsiTitle = "\033[1;36m";
constexpr const char* kAnsiGood = "\033[1;32m";
constexpr const char* kAnsiWarn = "\033[1;33m";
constexpr const char* kAnsiError = "\033[1;31m";
constexpr double kInputTimeoutSec = 0.5;

double clampValue(const double value, const double lower, const double upper)
{
    return std::max(lower, std::min(value, upper));
}

double wrapAngle(const double angle)
{
    double wrapped = std::fmod(angle + M_PI, 2.0 * M_PI);
    if (wrapped < 0.0)
    {
        wrapped += 2.0 * M_PI;
    }
    return wrapped - M_PI;
}

double yawFromPose(const geometry_msgs::Pose& pose)
{
    const double x = pose.orientation.x;
    const double y = pose.orientation.y;
    const double z = pose.orientation.z;
    const double w = pose.orientation.w;
    return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

double currentYaw()
{
    return yawFromPose(g_last_odom.pose.pose);
}

double radToDeg(const double rad)
{
    return rad * 180.0 / M_PI;
}

bool isFresh(const bool has_msg, const ros::Time& stamp)
{
    return has_msg && !stamp.isZero() && (ros::Time::now() - stamp).toSec() <= kInputTimeoutSec;
}

std::string inputStateText(const bool has_msg, const ros::Time& stamp)
{
    if (!has_msg)
    {
        return std::string(kAnsiWarn) + "等待" + kAnsiReset;
    }
    if (!isFresh(has_msg, stamp))
    {
        return std::string(kAnsiError) + "超时" + kAnsiReset;
    }
    return std::string(kAnsiGood) + "正常" + kAnsiReset;
}

double distanceToGoal()
{
    if (!g_has_goal || !g_has_odom)
    {
        return 0.0;
    }

    const geometry_msgs::Point& goal = g_last_goal.pose.position;
    const geometry_msgs::Point& pos = g_last_odom.pose.pose.position;
    const double dx = pos.x - goal.x;
    const double dy = pos.y - goal.y;
    return std::sqrt(dx * dx + dy * dy);
}

geometry_msgs::Twist makeStopCmd()
{
    return geometry_msgs::Twist();
}

geometry_msgs::Twist makeYawCmd(const double desired_yaw)
{
    geometry_msgs::Twist cmd;
    if (!g_has_odom)
    {
        return cmd;
    }

    const double yaw_error = wrapAngle(desired_yaw - currentYaw());
    cmd.angular.z = clampValue(g_yaw_kp * yaw_error, -g_max_angular_z, g_max_angular_z);
    return cmd;
}

geometry_msgs::Twist worldVelocityToBodyCmd(const geometry_msgs::TwistStamped& msg)
{
    geometry_msgs::Twist cmd;
    if (!g_has_odom)
    {
        return cmd;
    }

    const double yaw = currentYaw();
    const double cos_yaw = std::cos(yaw);
    const double sin_yaw = std::sin(yaw);
    const double vx_world = msg.twist.linear.x;
    const double vy_world = msg.twist.linear.y;

    cmd.linear.x = clampValue(cos_yaw * vx_world + sin_yaw * vy_world,
                              -g_max_linear_x,
                              g_max_linear_x);
    cmd.linear.y = clampValue(-sin_yaw * vx_world + cos_yaw * vy_world,
                              -g_max_linear_y,
                              g_max_linear_y);

    if (g_has_setpoint_pose)
    {
        const double desired_yaw = yawFromPose(g_last_setpoint_pose.pose);
        const double yaw_error = wrapAngle(desired_yaw - yaw);
        cmd.angular.z = clampValue(g_yaw_kp * yaw_error, -g_max_angular_z, g_max_angular_z);
    }
    return cmd;
}

std::string buildStatusPanel()
{
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(2);

    ss << kAnsiTitle << "=================== NavRL2SunrayUGV_node ===================" << kAnsiReset << "\n";

    ss << kAnsiGood << " 目标信息 " << kAnsiReset
       << "目标话题（订阅） -> " << g_goal_topic << "\n";
    if (g_has_goal)
    {
        ss << "          target = (" << g_last_goal.pose.position.x << ", "
           << g_last_goal.pose.position.y << ") m";
        if (g_has_odom)
        {
            ss << "  distance = " << distanceToGoal() << " m";
        }
        else
        {
            ss << "  distance = 等待ODOM";
        }
    }
    else
    {
        ss << "          target = 等待目标  distance = -";
    }
    ss << "\n";

    ss << kAnsiGood << " 控制输入 " << kAnsiReset
       << "NavRL速度（订阅） -> " << g_cmd_vel_topic
       << "  状态 = " << inputStateText(g_has_cmd_vel, g_last_cmd_vel_time) << "\n";
    if (g_has_cmd_vel)
    {
        ss << "           world_vx = " << g_last_cmd_vel.twist.linear.x << " m/s"
           << "  world_vy = " << g_last_cmd_vel.twist.linear.y << " m/s\n";
    }

    ss << kAnsiGood << " 转向输入 " << kAnsiReset
       << "NavRL位姿（订阅） -> " << g_setpoint_pose_topic
       << "  状态 = " << inputStateText(g_has_setpoint_pose, g_last_setpoint_pose_time) << "\n";
    if (g_has_setpoint_pose)
    {
        ss << "           yaw_target = " << radToDeg(yawFromPose(g_last_setpoint_pose.pose)) << " deg\n";
    }

    ss << kAnsiGood << " 本车状态 " << kAnsiReset
       << "odom（订阅） -> " << g_odom_topic << "\n";
    if (g_has_odom)
    {
        ss << "          pos = (" << g_last_odom.pose.pose.position.x << ", "
           << g_last_odom.pose.pose.position.y << ") m"
           << "  yaw = " << radToDeg(currentYaw()) << " deg"
           << "  body_vel = (" << g_last_odom.twist.twist.linear.x << ", "
           << g_last_odom.twist.twist.linear.y << ") m/s\n";
    }
    else
    {
        ss << "          pos = 等待ODOM\n";
    }

    ss << kAnsiGood << " 控制输出 " << kAnsiReset
       << "车体系cmd_vel（发布） -> " << g_output_topic;

    return ss.str();
}

void printStatusCallback(const ros::TimerEvent&)
{
    std::cout << buildStatusPanel() << std::endl;
}

void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    g_has_goal = true;
    g_last_goal_time = ros::Time::now();
    g_last_goal = *msg;
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    g_has_odom = true;
    g_last_odom_time = ros::Time::now();
    g_last_odom = *msg;
}

void navrlSetpointPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    g_has_setpoint_pose = true;
    g_last_setpoint_pose_time = ros::Time::now();
    g_last_setpoint_pose = *msg;

    if (g_has_goal && g_has_odom && distanceToGoal() <= g_goal_stop_distance)
    {
        g_cmd_pub.publish(makeStopCmd());
        return;
    }

    g_cmd_pub.publish(makeYawCmd(yawFromPose(msg->pose)));
}

void navrlCmdVelCallback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    g_has_cmd_vel = true;
    g_last_cmd_vel_time = ros::Time::now();
    g_last_cmd_vel = *msg;

    if (!g_has_odom)
    {
        g_cmd_pub.publish(makeStopCmd());
        return;
    }
    if (g_has_goal && distanceToGoal() <= g_goal_stop_distance)
    {
        g_cmd_pub.publish(makeStopCmd());
        return;
    }

    g_cmd_pub.publish(worldVelocityToBodyCmd(*msg));
}
}  // namespace

int main(int argc, char** argv)
{
    ros::init(argc, argv, "NavRL2SunrayUGV_node");

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    int queue_size = 50;
    private_nh.param<std::string>("input_topic", g_cmd_vel_topic, g_cmd_vel_topic);
    private_nh.param<std::string>("cmd_vel_topic", g_cmd_vel_topic, g_cmd_vel_topic);
    private_nh.param<std::string>("setpoint_pose_topic", g_setpoint_pose_topic, g_setpoint_pose_topic);
    private_nh.param<std::string>("goal_topic", g_goal_topic, g_goal_topic);
    private_nh.param<std::string>("odom_topic", g_odom_topic, g_odom_topic);
    private_nh.param<std::string>("output_topic", g_output_topic, g_output_topic);
    private_nh.param("queue_size", queue_size, queue_size);
    private_nh.param("max_linear_x", g_max_linear_x, g_max_linear_x);
    private_nh.param("max_linear_y", g_max_linear_y, g_max_linear_y);
    private_nh.param("max_angular_z", g_max_angular_z, g_max_angular_z);
    private_nh.param("yaw_kp", g_yaw_kp, g_yaw_kp);
    private_nh.param("goal_stop_distance", g_goal_stop_distance, g_goal_stop_distance);
    private_nh.param("status_print_hz", g_status_print_hz, g_status_print_hz);

    g_max_linear_x = std::max(0.01, g_max_linear_x);
    g_max_linear_y = std::max(0.01, g_max_linear_y);
    g_max_angular_z = std::max(0.01, g_max_angular_z);
    g_yaw_kp = std::max(0.0, g_yaw_kp);
    g_goal_stop_distance = std::max(0.0, g_goal_stop_distance);
    g_status_print_hz = std::max(0.1, g_status_print_hz);

    g_cmd_pub = nh.advertise<geometry_msgs::Twist>(g_output_topic, queue_size);
    ros::Subscriber cmd_vel_sub = nh.subscribe(g_cmd_vel_topic, queue_size, navrlCmdVelCallback);
    ros::Subscriber setpoint_pose_sub = nh.subscribe(g_setpoint_pose_topic, queue_size, navrlSetpointPoseCallback);
    ros::Subscriber goal_sub = nh.subscribe(g_goal_topic, queue_size, goalCallback);
    ros::Subscriber odom_sub = nh.subscribe(g_odom_topic, queue_size, odomCallback);
    ros::Timer status_timer = nh.createTimer(ros::Duration(1.0 / g_status_print_hz), printStatusCallback);

    (void)cmd_vel_sub;
    (void)setpoint_pose_sub;
    (void)goal_sub;
    (void)odom_sub;
    (void)status_timer;

    ROS_INFO_STREAM("NavRL2SunrayUGV cmd_vel subscribe:       " << g_cmd_vel_topic);
    ROS_INFO_STREAM("NavRL2SunrayUGV setpoint_pose subscribe: " << g_setpoint_pose_topic);
    ROS_INFO_STREAM("NavRL2SunrayUGV goal subscribe:          " << g_goal_topic);
    ROS_INFO_STREAM("NavRL2SunrayUGV odom subscribe:          " << g_odom_topic);
    ROS_INFO_STREAM("NavRL2SunrayUGV body cmd_vel publish:    " << g_output_topic);

    ros::spin();
    return 0;
}
