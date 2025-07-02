/*
程序功能：使用XyzVelYawBody接口，实现无人机自动降落
*/

#include <ros/ros.h>
#include "ros_msg_utils.h"
#include "printf_utils.h"
#include <sunray_msgs/TargetMsg.h>
#include <sunray_msgs/TargetsInFrameMsg.h>
#include "utils.hpp"
#include <printf_format.h>

using namespace std;
using namespace sunray_logger;

int uav_id;
string uav_name;
// 是否自动起飞
bool auto_takeoff = false;
// 降落状态机
enum MISSION_STATE
{
    INIT = 0,           // 初始模式
    TRACK_IN_FIXED_HEIGHT = 1,     // 遥控器控制模式
    LAND_SLOW = 2,    // 外部指令控制模式
    KILL = 3,   // 降落模式
};
MISSION_STATE mission_state;

// 无人机控制指令
sunray_msgs::UAVControlCMD uav_cmd;
// 无人机设置指令
sunray_msgs::UAVSetup uav_setup;
// 无人机状态
sunray_msgs::UAVState uav_state;
Eigen::Vector3d uav_pos;        // 无人机惯性系下的位置

// 停止标志
bool stop_flag{false};
string node_name;

// 可能表示是否收到当前位置信息和目标检测信息的标志位。
bool pos_flag{false};
bool tag_flag{false};
Eigen::Vector3d error_in_fix_height;
struct target_info
{
    Eigen::Vector3d pos_enu;        // 惯性系下的位置
    Eigen::Vector3d vel_enu;        // 惯性系下的速度
    Eigen::Vector3d pos_body;       // 机体系下的位置
    Eigen::Vector3d vel_body;       // 机体系下的速度
    double yaw_body;                // 相对yaw
    double distance;                // 距离无人机的直线距离
    double xy_distance;             // 水平距离
    int lost_times;                 // 位置信息丢失次数
    int regain_times;               // 位置信息获取次数   
};
target_info target;
double land_fixed_height,land_height_min;
double xy_dis_threshold1,xy_dis_threshold2;
struct PIDController 
{
    float Kp;
    float Ki;
    float Kd;
    float Kp_z;
    float Ki_z;
    float Kd_z;
    float integral[3];
    float derivative[3];
    float prev_error[3];
    Eigen::Vector3d control;
    Eigen::Vector3d control_p;
    Eigen::Vector3d control_i;
    Eigen::Vector3d control_d;
    float max_vel[3];
};
PIDController pid;

bool arm_takeoff_flag{false};
ros::Publisher uav_setup_pub,control_cmd_pub;

// 中断信号
void mySigintHandler(int sig)
{
    // 移动到初始位置
    Logger::print_color(int(LogColor::red), node_name, ": exit, send hold cmd.");
    uav_cmd.cmd = sunray_msgs::UAVControlCMD::Hover;
    control_cmd_pub.publish(uav_cmd);
    ros::shutdown();
    exit(0);
}

void target_callback(const sunray_msgs::TargetsInFrameMsg::ConstPtr &msg)
{
    // TargetMsg.msg话题定义：识别程序按照20Hz往外发送目标识别信息
    // 如果没有识别到，则score设置为0，如果识别到，score设置为1
    if(msg->targets[0].score == 0 || msg->targets.size() == 0)
    {
        target.lost_times++;
        target.regain_times = 0;
        return;
    }

    target.lost_times = 0;
    target.regain_times++;

    // 位置信息，注意坐标系！
    // target.pos_body[0] = x_filter.filter(msg->targets[0].px);
    // target.pos_body[1] = y_filter.filter(-msg->targets[0].py);
    // target.pos_body[2] = z_filter.filter(-msg->targets[0].pz);
    // target.yaw_body = yaw_filter.filter(-msg->targets[0].yaw);
}

// 根据已知信息进行视觉模拟
void ugv_odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    if(!pos_flag)
    {
        return;
    }

    target.lost_times = 0;
    target.regain_times++;

    target.pos_enu[0] = msg->pose.pose.position.x;
    target.pos_enu[1] = msg->pose.pose.position.y;
    target.pos_enu[2] = 0.3;                        // 降落板的高度是先验信息

    target.vel_enu[0] = msg->twist.twist.linear.x;
    target.vel_enu[1] = msg->twist.twist.linear.y;
    target.vel_enu[2] = msg->twist.twist.linear.z;

    // 换算得到机体系下的目标相对位置
    target.pos_body[0] = target.pos_enu[0] - uav_pos[0];    
    target.pos_body[1] = target.pos_enu[1] - uav_pos[1];         
    target.pos_body[2] = target.pos_enu[2] - uav_pos[2];

    target.vel_body[0] = target.vel_enu[0] - uav_state.velocity[0];    
    target.vel_body[1] = target.vel_enu[1] - uav_state.velocity[1];         
    target.vel_body[2] = target.vel_enu[2] - uav_state.velocity[2];

    target.yaw_body = 0.0;  //默认无人机和目标偏航角一致

    // 计算距离
    target.distance = (target.pos_enu - uav_pos).norm();
    target.xy_distance = sqrt((target.pos_enu[0] - uav_pos[0])*(target.pos_enu[0] - uav_pos[0])+(target.pos_enu[1] - uav_pos[1])*(target.pos_enu[1] - uav_pos[1]));
}

// 无人机状态回调
void uav_state_callback(const sunray_msgs::UAVState::ConstPtr &msg)
{
    uav_state = *msg;
    uav_pos[0] = uav_state.position[0];
    uav_pos[1] = uav_state.position[1];
    uav_pos[2] = uav_state.position[2];
    
    pos_flag = true;
}

void stop_tutorial_cb(const std_msgs::Empty::ConstPtr &msg)
{
    stop_flag = true;
}

void init(ros::NodeHandle &nh)
{
    mission_state = MISSION_STATE::INIT;
    // 获取节点名字
    node_name = ros::this_node::getName();

    // 变量初始化
    uav_cmd.header.stamp = ros::Time::now();
    uav_cmd.cmd = sunray_msgs::UAVControlCMD::Hover;
    uav_cmd.desired_pos[0] = 0.0;
    uav_cmd.desired_pos[1] = 0.0;
    uav_cmd.desired_pos[2] = 0.0;
    uav_cmd.desired_vel[0] = 0.0;
    uav_cmd.desired_vel[1] = 0.0;
    uav_cmd.desired_vel[2] = 0.0;
    uav_cmd.desired_yaw = 0.0;
    uav_cmd.desired_yaw_rate = 0.0;

    // 【参数】无人机编号
    nh.param<int>("uav_id", uav_id, 1);
    // 【参数】无人机名称
    nh.param<string>("uav_name", uav_name, "uav");
    // 【参数】程序执行时是否自动起飞
    nh.param<bool>("auto_takeoff", auto_takeoff, true);
    // 【参数】降落固定高度
    nh.param<double>("land_fixed_height", land_fixed_height, 0.8);
    // 【参数】降落最低高度
    nh.param<double>("land_height_min", land_height_min, 0.35);
    // 【参数】进入降落程序阈值
    nh.param<double>("xy_dis_threshold1", xy_dis_threshold1, 0.2);
    // 【参数】进入上锁程序阈值
    nh.param<double>("xy_dis_threshold2", xy_dis_threshold2, 0.05);

    uav_name = "/" + uav_name + to_string(uav_id);

    target.pos_enu.setZero();
    target.vel_enu.setZero();
    target.pos_body.setZero();
    target.vel_body.setZero();
    target.yaw_body = 0.0;
    target.distance = 100.0;
    target.xy_distance = 100.0;
    target.lost_times = 0;
    target.regain_times = 0;

    pid.Kp = 0.5;
    pid.Ki = 0.1;
    pid.Kd = 0.05;
    pid.Kp_z = 2.5;
    pid.Ki_z = 1.0;
    pid.Kd_z = 0.05;
    pid.max_vel[0] = 1.0;
    pid.max_vel[1] = 1.0;
    pid.max_vel[2] = 0.3;

    for(int i=0; i<3; i++)
    {
        pid.integral[i] = 0.0;
        pid.prev_error[i] = 0.0;
    }
}

int arm_takeoff()
{
    // 程序运行时自动起飞
    if(!auto_takeoff)
    {
        Logger::print_color(int(LogColor::red), node_name, ": auto_takeoff is set to false, pls arm and takeoff drones by ground station.");
        return 0;
    }

    ros::Duration(1).sleep();

    // 初始化检查：等待飞控与机载电脑建立连接
    int times = 0;
    while (ros::ok() && !uav_state.connected)
    {
        ros::spinOnce();
        ros::Duration(1.0).sleep();
        if (times++ > 5)
            Logger::print_color(int(LogColor::red), node_name, ": Wait for UAV connect...");
    }
    Logger::print_color(int(LogColor::green), node_name, ": UAV connected!");

    // 飞控与机载电脑建立连接后，切换到无人机控制模块至指令控制模式(同时，PX4模式将切换至OFFBOARD模式)
    while (ros::ok() && uav_state.control_mode != sunray_msgs::UAVSetup::CMD_CONTROL)
    {
        uav_setup.cmd = sunray_msgs::UAVSetup::SET_CONTROL_MODE;
        uav_setup.control_mode = "CMD_CONTROL";
        uav_setup_pub.publish(uav_setup);
        Logger::print_color(int(LogColor::green), node_name, ": SET_CONTROL_MODE - [CMD_CONTROL]. ");
        ros::Duration(1.0).sleep();
        ros::spinOnce();
    }
    Logger::print_color(int(LogColor::green), node_name, ": UAV control_mode set to [CMD_CONTROL] successfully!");

    // 解锁无人机倒计时
    if(!uav_state.armed)
    {
        Logger::print_color(int(LogColor::green), node_name, ": Arm UAV in 3 sec...");
        ros::Duration(1.0).sleep();
        Logger::print_color(int(LogColor::green), node_name, ": Arm UAV in 2 sec...");
        ros::Duration(1.0).sleep();
        Logger::print_color(int(LogColor::green), node_name, ": Arm UAV in 1 sec...");
        ros::Duration(1.0).sleep();
    }

    // 解锁无人机
    while (ros::ok() && !uav_state.armed)
    {
        uav_setup.cmd = sunray_msgs::UAVSetup::ARM;
        uav_setup_pub.publish(uav_setup);
        Logger::print_color(int(LogColor::green), node_name, ": Arm UAV now.");
        ros::Duration(1.0).sleep();
        ros::spinOnce();
    }
    Logger::print_color(int(LogColor::green), node_name, ": Arm UAV successfully!");

    // 起飞无人机
    while (ros::ok() && uav_state.landed_state !=2)
    {
        uav_cmd.cmd = sunray_msgs::UAVControlCMD::Takeoff;
        control_cmd_pub.publish(uav_cmd);
        Logger::print_color(int(LogColor::green), node_name, ": Takeoff UAV now.");
        ros::Duration(4.0).sleep();
        ros::spinOnce();
    }
    Logger::print_color(int(LogColor::green), node_name, ": Takeoff UAV successfully!");

    // 移动到初始位置
    Logger::print_color(int(LogColor::green), node_name, ": move to the initial position.");
    uav_cmd.cmd = sunray_msgs::UAVControlCMD::XyzPos;
    uav_cmd.desired_pos[0] = 0.0;
    uav_cmd.desired_pos[1] = 0.0;
    uav_cmd.desired_pos[2] = 2.0;
    control_cmd_pub.publish(uav_cmd);

    ros::Duration(3.0).sleep();
    return 1;
}

void debug_timer_callback(const ros::TimerEvent& e);
void pid_controller(Eigen::Vector3d error_body);
float constrain_function(float data, float Max, float Min);
int main(int argc, char **argv)
{
    // 设置日志
    Logger::init_default();
    Logger::setPrintLevel(false);
    Logger::setPrintTime(false);
    Logger::setPrintToFile(false);
    Logger::setFilename("~/Documents/Sunray_log.txt");

    ros::init(argc, argv, "land_on_a_moving_car");
    ros::NodeHandle nh("~");
    ros::Rate rate(50.0);

    signal(SIGINT, mySigintHandler);

    // 变量及参数初始化
    init(nh);

    // 【订阅】无人机状态
    ros::Subscriber uav_state_sub = nh.subscribe<sunray_msgs::UAVState>(uav_name + "/sunray/uav_state", 10, uav_state_callback);
    // 【订阅】目标odom（暂时使用）
    ros::Subscriber target_odom_sub = nh.subscribe<nav_msgs::Odometry>("/ugv_odom", 10, ugv_odom_callback);
    // 【订阅】目标相对位置（实际使用）
    ros::Subscriber target_pos_sub = nh.subscribe<sunray_msgs::TargetsInFrameMsg>(uav_name + "/sunray_detect/xxx", 1, target_callback);
    // 【订阅】任务结束指令
    ros::Subscriber stop_tutorial_sub = nh.subscribe<std_msgs::Empty>(uav_name + "/sunray/stop_tutorial", 1, stop_tutorial_cb);

    // 【发布】无人机控制指令 （本节点 -> sunray_control_node）
    control_cmd_pub = nh.advertise<sunray_msgs::UAVControlCMD>(uav_name + "/sunray/uav_control_cmd", 1);
    // 【发布】无人机设置指令（本节点 -> sunray_control_node）
    uav_setup_pub = nh.advertise<sunray_msgs::UAVSetup>(uav_name + "/sunray/setup", 1);

    ros::Timer debug_timer = nh.createTimer(ros::Duration(0.05), debug_timer_callback);


    int result = arm_takeoff();
    arm_takeoff_flag = true;

    ros::Duration(0.5).sleep();
    ros::spinOnce();

    geometry_msgs::PoseStamped pose;

    // 等待订阅无人机位置
    while (!pos_flag && ros::ok())
    {
        Logger::print_color(int(LogColor::blue), node_name, "wait for pose!!!");
        ros::spinOnce();
        ros::Duration(1).sleep();
    }

    // 等待订阅到目标位置
    while (target.regain_times==0 && ros::ok())
    {
        Logger::print_color(int(LogColor::blue), node_name, "wait for tag!!!");
        ros::spinOnce();
        ros::Duration(1).sleep();
    }

    double landing_point_num = 0;

    // 视觉引导降落主程序
    while (ros::ok())
    {
        ros::spinOnce();

        // 提前终止任务
        if(stop_flag)
        {
            Logger::print_color(int(LogColor::blue), node_name, "land");
            uav_cmd.cmd = sunray_msgs::UAVControlCMD::Land;
            control_cmd_pub.publish(uav_cmd);
            break;
        }

        switch(mission_state)
        {
            case MISSION_STATE::INIT:
                // 移动到初始位置
                uav_cmd.cmd = sunray_msgs::UAVControlCMD::XyzPos;
                uav_cmd.desired_pos[0] = 0.0;
                uav_cmd.desired_pos[1] = 0.0;
                uav_cmd.desired_pos[2] = 2.0;
                control_cmd_pub.publish(uav_cmd);

                // 检测到目标，开始追踪
                if(target.regain_times > 5)
                {
                    mission_state = MISSION_STATE::TRACK_IN_FIXED_HEIGHT;
                }
            break;

            // 第一阶段:固定高度追踪
            case MISSION_STATE::TRACK_IN_FIXED_HEIGHT:

                // 丢失目标，进入INIT模式，重新去搜寻目标
                if (target.lost_times>10)
                {
                    mission_state = MISSION_STATE::INIT;
                }
                // 距离小于一定阈值之后（这个阈值取决于降落靶标和无人机的尺寸大小），进入下一阶段
                if(target.xy_distance < xy_dis_threshold1)
                {
                    mission_state = MISSION_STATE::LAND_SLOW;
                }

                // 根据相对位置计算期望速度
                pid.Kp = 4.0;
                pid.Ki = 1.0;
                pid.Kd = 0.05;
                pid.Kp_z = 1.0;
                pid.Ki_z = 0.5;
                pid.Kd_z = 0.0;

                // 固定高度追踪
                error_in_fix_height = target.pos_body;
                error_in_fix_height[2] = land_fixed_height - uav_pos[2]; // 固定高度追踪
                pid_controller(error_in_fix_height);

                // 固定高度追踪
                uav_cmd.header.stamp = ros::Time::now();
                uav_cmd.cmd = sunray_msgs::UAVControlCMD::XyzVelYawBody;
                uav_cmd.desired_vel[0] = pid.control[0];
                uav_cmd.desired_vel[1] = pid.control[1];
                uav_cmd.desired_vel[2] = pid.control[2];           
                uav_cmd.desired_yaw = 0.0 - uav_state.attitude[2];
                control_cmd_pub.publish(uav_cmd);
                break;

            // 第二阶段:追踪目标之后，缓慢降落
            case MISSION_STATE::LAND_SLOW:

                // 距离小于阈值（这个阈值可以根据实际情况调节，确保上锁后可以落在降落靶标上），直接停桨
                if (target.xy_distance < xy_dis_threshold2 && uav_pos[2] < 0.35)
                {
                    mission_state = MISSION_STATE::KILL;
                }
                // 丢失目标or目标距离过大，重回上一个追踪状态
                if(target.xy_distance > xy_dis_threshold1 || target.lost_times>5)
                {
                    mission_state = MISSION_STATE::TRACK_IN_FIXED_HEIGHT;
                }

                // 根据相对位置计算期望速度
                pid.Kp = 5.0;
                pid.Ki = 2.0;
                pid.Kd = 0.05;

                pid_controller(target.pos_body);

                uav_cmd.header.stamp = ros::Time::now();
                uav_cmd.cmd = sunray_msgs::UAVControlCMD::XyzVelYawBody;
                uav_cmd.desired_vel[0] = pid.control[0];
                uav_cmd.desired_vel[1] = pid.control[1];
                if(uav_pos[2] < land_height_min)
                {
                    // 降落到最低高度就不在降落了
                    uav_cmd.desired_vel[2] = -0.0;
                }else
                {
                    // 高度稳定下降
                    uav_cmd.desired_vel[2] = -0.2;  
                }
                uav_cmd.desired_yaw = 0.0 - uav_state.attitude[2];
                control_cmd_pub.publish(uav_cmd);
            break;

            // 第三阶段:直接上锁
            case MISSION_STATE::KILL:

                while (ros::ok() && uav_state.armed)
                {
                    Logger::print_color(int(LogColor::red), node_name, "very close to target, arm directly!");
                    uav_setup.header.stamp = ros::Time::now();
                    uav_setup.cmd = sunray_msgs::UAVSetup::EMERGENCY_KILL;
                    uav_setup_pub.publish(uav_setup);
                    ros::spinOnce();
                    rate.sleep();
                }
                // 结束程序
                return 0;
            break;

        }

        rate.sleep();
    }

    return 0;
}

void pid_controller(Eigen::Vector3d error_body)
{   
    Eigen::Vector3d error = error_body;
    float dt = 0.01;

    for(int i=0; i<3; i++)
    {
        // 误差较小时(小于0.2米)才开始积分
        if(abs(error[i]) <= 1.0)
        {
            // 积分项限制幅度，最大产生的速度不超过1.0米每秒
            if(abs(pid.integral[i]) < 1.0)
            {
                pid.integral[i] += error[i] * dt;
            }
        }
        else
        {
            pid.integral[i] = 0;
        }

    }

    for(int i=0; i<3; i++)
    {
        pid.derivative[i] = (error[i] - pid.prev_error[i]) / dt;
        pid.prev_error[i] = error[i];
    }

    for(int i=0; i<2; i++)
    {
        pid.control_p[i] = pid.Kp * error[i];
        pid.control_i[i] = pid.Ki * pid.integral[i];
        pid.control_d[i] = pid.Kd * pid.derivative[i];
        pid.control[i] = pid.control_p[i]  +  pid.control_i[i] + pid.control_d[i];
    }

    pid.control_p[2] = pid.Kp_z * error[2];
    pid.control_i[2] = pid.Ki_z * pid.integral[2];
    pid.control_d[2] = pid.Kd_z * pid.derivative[2];
    pid.control[2] = pid.control_p[2]  +  pid.control_i[2] + pid.control_d[2];

    // limit
    for(int i=0; i<3; i++)
    {
        // 上下limit，死区设置为0
        pid.control[i] = constrain_function(pid.control[i], pid.max_vel[i], 0.0);
    }
}

// 限制幅度函数
float constrain_function(float data, float Max, float Min)
{
    if(abs(data)>Max)
    {
        return (data > 0) ? Max : -Max;
    }else if(abs(data)<Min)
    {
        return 0.0;
    }
    else
    {
        return data;
    }
}


void debug_timer_callback(const ros::TimerEvent& e)
{
    if(!arm_takeoff_flag)
    {
        return;
    }


    if (mission_state == MISSION_STATE::INIT)
    {
        Logger::print_color(int(LogColor::blue), "MISSION_STATE: [ INIT ]");
    }
    else if(mission_state == MISSION_STATE::TRACK_IN_FIXED_HEIGHT)
    {
        Logger::print_color(int(LogColor::blue), "MISSION_STATE: [ TRACK_IN_FIXED_HEIGHT ]");
    }else if(mission_state == MISSION_STATE::LAND_SLOW)
    {
        Logger::print_color(int(LogColor::blue), "MISSION_STATE: [ LAND_SLOW ]");
    }else if(mission_state == MISSION_STATE::KILL)
    {
        Logger::print_color(int(LogColor::blue), "MISSION_STATE: [ KILL ]");
    }

    // debug打印
    Logger::print_color(int(LogColor::blue), "Target INFO:");

    Logger::print_color(int(LogColor::green), "pos_enu [X Y Z]:",
                        target.pos_enu[0],
                        target.pos_enu[1],
                        target.pos_enu[2],
                        "[ m ]");
    Logger::print_color(int(LogColor::green), "vel_enu [X Y Z]:",
                        target.vel_enu[0],
                        target.vel_enu[1],
                        target.vel_enu[2],
                        "[m/s]");
    Logger::print_color(int(LogColor::green), "pos_body [X Y Z]:",
                        target.pos_body[0],
                        target.pos_body[1],
                        target.pos_body[2],
                        "[ m ]");
    Logger::print_color(int(LogColor::green), "vel_body [X Y Z]:",
                        target.vel_enu[0],
                        target.vel_enu[1],
                        target.vel_enu[2],
                        "[m/s]");
    Logger::print_color(int(LogColor::green), "yaw_body [X Y Z]:",
                        target.yaw_body/M_PI*180.0,
                        "[deg]");
    Logger::print_color(int(LogColor::green), "distance :",
                        target.distance,
                        "[ m ]");
    Logger::print_color(int(LogColor::green), "xy_distance :",
                        target.xy_distance,
                        "[ m ]");
    Logger::print_color(int(LogColor::green), "lost_times :",
                        target.lost_times,
                        ", regain_times : ",
                        target.regain_times);  

    Logger::print_color(int(LogColor::blue), "Control INFO:");
    for(int i=0; i<3; i++)
    {
        Logger::print_color(int(LogColor::green), "control [P I D]_",i,": ",
                            pid.control_p[i],
                            pid.control_i[i],
                            pid.control_d[i],
                            "[m/s]");
    }            

}
