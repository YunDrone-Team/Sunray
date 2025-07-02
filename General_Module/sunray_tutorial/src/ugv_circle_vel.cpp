#include <ros/ros.h>
#include <sunray_msgs/UGVControlCMD.h>
#include <sunray_msgs/UGVState.h>
#include <cmath>
#include <csignal> // 用于信号处理

// 全局变量声明
ros::Publisher cmd_pub;
sunray_msgs::UGVState current_state;
int ugv_id;
double linear_speed = 0.5;
double circle_radius = 1.0;   // 固定半径为1米
double circle_center_x = 0.0; // 圆心固定为(0,0)
double circle_center_y = 0.0;
double radial_gain = 0.5;                 // 径向校正增益
bool initial_move_completed = false;      // 初始移动完成标志
const double INITIAL_MOVE_DISTANCE = 0.5; // 初始移动距离
bool returning_to_origin = false;         // 返回原点标志
bool shutdown_requested = false;          // 关闭请求标志

// 状态回调函数
void stateCallback(const sunray_msgs::UGVState::ConstPtr &msg)
{
    current_state = *msg;
}

// 计算两点间距离
double distance(double x1, double y1, double x2, double y2)
{
    double dx = x1 - x2;
    double dy = y1 - y2;
    return std::sqrt(dx * dx + dy * dy);
}

// 信号处理函数 (用于捕获Ctrl+C)
void signalHandler(int signum)
{
    ROS_INFO("Interrupt signal (%d) received. Preparing to return to origin...", signum);
    shutdown_requested = true;
}

// 返回原点函数
void return_to_origin()
{
    returning_to_origin = true;
    ROS_INFO("Returning to origin (0,0)...");

    ros::Rate rate(20);
    ros::Time start_time = ros::Time::now();

    while (ros::ok() && shutdown_requested)
    {
        double dx = current_state.position[0] - circle_center_x;
        double dy = current_state.position[1] - circle_center_y;
        double dist = distance(current_state.position[0], current_state.position[1],
                               circle_center_x, circle_center_y);

        // 检查是否到达原点
        if (dist < 0.1)
        {
            ROS_INFO("Reached origin (0,0). Stopping...");
            break;
        }

        // 检查超时（10秒）
        if ((ros::Time::now() - start_time).toSec() > 10.0)
        {
            ROS_WARN("Timeout while returning to origin. Stopping...");
            break;
        }

        // 创建位置控制命令
        sunray_msgs::UGVControlCMD ugv_cmd;
        ugv_cmd.cmd = sunray_msgs::UGVControlCMD::POS_CONTROL_ENU;
        ugv_cmd.desired_pos[0] = circle_center_x;
        ugv_cmd.desired_pos[1] = circle_center_y;
        ugv_cmd.desired_yaw = 0.0;

        // 设置速度（根据距离调整）
        double speed_factor = std::min(1.0, dist / 2.0);
        ugv_cmd.desired_vel[0] = linear_speed * speed_factor * (-dx / dist);
        ugv_cmd.desired_vel[1] = linear_speed * speed_factor * (-dy / dist);

        cmd_pub.publish(ugv_cmd);

        ROS_INFO_THROTTLE(0.5, "Distance to origin: %.2fm", dist);

        ros::spinOnce();
        rate.sleep();
    }

    // 发送停止指令
    sunray_msgs::UGVControlCMD stop_cmd;
    stop_cmd.cmd = sunray_msgs::UGVControlCMD::HOLD;
    cmd_pub.publish(stop_cmd);
    ROS_INFO("UGV stopped at origin.");
}

// 生成圆周运动控制命令
void generate_commands()
{
    sunray_msgs::UGVControlCMD ugv_cmd;

    // 处理返回原点请求
    if (returning_to_origin)
    {
        return; // 不生成命令，由return_to_origin函数处理
    }

    ugv_cmd.cmd = sunray_msgs::UGVControlCMD::VEL_CONTROL_ENU;

    // 计算当前位置与圆心的距离
    double dx = current_state.position[0] - circle_center_x;
    double dy = current_state.position[1] - circle_center_y;
    double dist = distance(current_state.position[0], current_state.position[1],
                           circle_center_x, circle_center_y);

    // 处理原点位置或未完成初始移动的情况
    if ((dist < 0.05) || !initial_move_completed)
    {
        // 初始移动阶段：移动到(0.5, 0)位置
        double target_x = circle_center_x + INITIAL_MOVE_DISTANCE;
        double target_y = circle_center_y;

        // 计算到目标点的向量
        double tx = target_x - current_state.position[0];
        double ty = target_y - current_state.position[1];
        double target_dist = distance(current_state.position[0], current_state.position[1],
                                      target_x, target_y);

        // 归一化向量
        if (target_dist > 1e-3)
        {
            tx /= target_dist;
            ty /= target_dist;
        }

        // 设置速度
        ugv_cmd.desired_vel[0] = linear_speed * tx;
        ugv_cmd.desired_vel[1] = linear_speed * ty;

        // 检查是否到达初始位置
        if (target_dist < 0.1)
        {
            initial_move_completed = true;
            ROS_INFO("Initial move completed. Starting circular motion.");
        }
    }
    else
    {
        // 圆周运动阶段
        // 计算半径误差
        double radius_error = dist - circle_radius;

        // 计算径向单位向量
        double radial_x = 0.0, radial_y = 0.0;
        if (dist > 1e-3)
        {
            radial_x = dx / dist;
            radial_y = dy / dist;
        }

        // 计算切向单位向量 (逆时针方向)
        double tangent_x = -radial_y;
        double tangent_y = radial_x;

        // 计算径向校正速度分量 (指向圆心)
        double v_radial = -radial_gain * radius_error;

        // 组合速度向量 = 切向速度 + 径向校正速度
        double vx = linear_speed * tangent_x + v_radial * radial_x;
        double vy = linear_speed * tangent_y + v_radial * radial_y;

        // 限制最大速度
        double speed = std::sqrt(vx * vx + vy * vy);
        if (speed > 1.5 * linear_speed)
        {
            vx *= (1.5 * linear_speed) / speed;
            vy *= (1.5 * linear_speed) / speed;
        }

        ugv_cmd.desired_vel[0] = vx;
        ugv_cmd.desired_vel[1] = vy;
    }

    ugv_cmd.desired_yaw = 0.0; // 保持当前朝向
    cmd_pub.publish(ugv_cmd);

    // 调试信息
    static ros::Time last_log_time = ros::Time::now();
    if ((ros::Time::now() - last_log_time).toSec() > 1.0)
    {
        ROS_INFO("Position: (%.2f, %.2f), Distance: %.2fm, State: %s",
                 current_state.position[0], current_state.position[1],
                 dist, initial_move_completed ? "Circular" : "Initial Move");
        last_log_time = ros::Time::now();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ugv_circular_motion");
    ros::NodeHandle nh;

    // 注册信号处理函数
    signal(SIGINT, signalHandler);

    // 获取参数
    nh.param<int>("ugv_id", ugv_id, 1);
    nh.param<double>("linear_speed", linear_speed, 0.5);
    nh.param<double>("radial_gain", radial_gain, 0.5);

    // 创建发布器和订阅器
    std::string cmd_topic = "/ugv" + std::to_string(ugv_id) + "/sunray_ugv/ugv_control_cmd";
    std::string state_topic = "/ugv" + std::to_string(ugv_id) + "/sunray_ugv/ugv_state";

    cmd_pub = nh.advertise<sunray_msgs::UGVControlCMD>(cmd_topic, 10);
    ros::Subscriber state_sub = nh.subscribe(state_topic, 1, stateCallback);

    // 等待第一次位置更新
    ROS_INFO("Waiting for initial state...");
    ros::Duration(1.0).sleep();
    ros::spinOnce();

    if (ros::ok())
    {
        ROS_INFO("Starting UGV circular motion controller");
        ROS_INFO("Initial position: (%.2f, %.2f)",
                 current_state.position[0], current_state.position[1]);

        // 检查是否需要在原点开始移动
        double dist = distance(current_state.position[0], current_state.position[1],
                               circle_center_x, circle_center_y);
        if (dist < 0.05)
        {
            ROS_INFO("UGV at center. Performing initial move to (%.1f, 0)", INITIAL_MOVE_DISTANCE);
            initial_move_completed = false;
        }
        else
        {
            initial_move_completed = true;
            ROS_INFO("Starting circular motion directly");
        }

        ros::Rate rate(20); // 20Hz控制频率
        while (ros::ok() && !shutdown_requested)
        {
            generate_commands();
            ros::spinOnce();
            rate.sleep();
        }

        // 程序退出前返回原点
        if (!returning_to_origin)
        {
            return_to_origin();
        }
    }

    // 确保小车停止
    sunray_msgs::UGVControlCMD stop_cmd;
    stop_cmd.cmd = sunray_msgs::UGVControlCMD::HOLD;
    cmd_pub.publish(stop_cmd);
    ROS_INFO("Program exiting.");

    return 0;
}