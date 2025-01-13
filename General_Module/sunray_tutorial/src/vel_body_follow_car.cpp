#include <ros/ros.h>
#include "ros_msg_utils.h"
#include "printf_utils.h"
#include <csignal>

using namespace  std;

geometry_msgs::PoseStamped current_pose;
sunray_msgs::UAVState uav_state;
sunray_msgs::UAVControlCMD uav_cmd;
std_msgs::Int32 uav_control_state;
geometry_msgs::PoseStamped target_pose;
sunray_msgs::UAVSetup setup;

float target_yaw;
bool stop_flag{false};
// 全局变量存储无人机和车的坐标和朝向
double x_1, y_1, z_1, yaw_1, x_2, y_2, z_2, yaw_2;

void mySigintHandler(int sig)
{
    std::cout<<"[vel_body_follow_car] exit..."<<std::endl;

    ros::shutdown();
    exit(EXIT_SUCCESS); // 或者使用 exit(0)

}

void uav_state_cb(const sunray_msgs::UAVState::ConstPtr &msg)
{
    uav_state = *msg;
}
void uav_control_state_cb(const std_msgs::Int32::ConstPtr &msg)
{
    uav_control_state = *msg;
}

void target_pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    target_pose = *msg;
    tf2::Quaternion quaternion;
    tf2::fromMsg(msg->pose.orientation, quaternion);
    double roll, pitch, yaw;
    tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
    x_2 = msg->pose.position.x;
    y_2 = msg->pose.position.y;
    z_2 = msg->pose.position.z;
    yaw_2 = yaw;
}

void stop_tutorial_cb(const std_msgs::Empty::ConstPtr &msg)
{
    stop_flag = true;
}

void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    current_pose = *msg;
    x_1 = msg->pose.position.x;
    y_1 = msg->pose.position.y;
    z_1 = msg->pose.position.z;
    yaw_1 = tf::getYaw(msg->pose.orientation);
}


// 回调函数，用于获取目标二的位置和朝向
void carOdomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    x_2 = msg->pose.pose.position.x;
    y_2 = msg->pose.pose.position.y;
    z_2 = msg->pose.pose.position.z;
    yaw_2 = tf::getYaw(msg->pose.pose.orientation);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vel_body_follow_car");
    ros::NodeHandle nh("~");

    ros::Rate rate(20.0);

    int uav_id;

    signal(SIGINT, mySigintHandler);

    string uav_name, target_tpoic_name;
    bool sim_mode, flag_printf;
    nh.param<bool>("sim_mode", sim_mode, true);
    nh.param<bool>("flag_printf", flag_printf, true);
    // 【参数】无人机编号
    nh.param<int>("uav_id", uav_id, 1);
    // 【参数】无人机名称
    nh.param<string>("uav_name", uav_name, "uav");
    // 【参数】目标话题名称
    nh.param<string>("target_tpoic_name", target_tpoic_name, "/vrpn_client_node/target/pose");

    uav_name = uav_name + std::to_string(uav_id);
    string topic_prefix = "/" + uav_name;
    // 【订阅】无人机状态 -- from vision_pose
    ros::Subscriber uav_state_sub = nh.subscribe<sunray_msgs::UAVState>(topic_prefix + "/sunray/uav_state", 1, uav_state_cb);
    //【订阅】无人机控制信息
    ros::Subscriber uav_contorl_state_sub = nh.subscribe<std_msgs::Int32>(topic_prefix + "/sunray/control_state", 1, uav_control_state_cb);
    // 【订阅】目标点位置
    ros::Subscriber target_pos_sub;
    if(sim_mode){
        target_pos_sub = nh.subscribe<nav_msgs::Odometry>("/tag_odom", 10, carOdomCallback);
    }
    else{
        target_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>(target_tpoic_name, 1, target_pos_cb);
    }
    // 【订阅】任务结束
    ros::Subscriber stop_tutorial_sub = nh.subscribe<std_msgs::Empty>(topic_prefix + "/sunray/stop_tutorial", 1, stop_tutorial_cb);
    
    // 【发布】无人机控制指令 （本节点 -> sunray_control_node）
    ros::Publisher control_cmd_pub = nh.advertise<sunray_msgs::UAVControlCMD>(topic_prefix + "/sunray/uav_control_cmd", 1);
    // 【发布】无人机设置指令（本节点 -> sunray_control_node）
    ros::Publisher uav_setup_pub = nh.advertise<sunray_msgs::UAVSetup>(topic_prefix + "/sunray/setup", 1);

    ros::Subscriber pose_sub = nh.subscribe(topic_prefix + "/mavros/local_position/pose", 10, pose_cb);

    // 变量初始化
    uav_cmd.header.stamp = ros::Time::now();
    uav_cmd.cmd_id = 0;
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
    uav_cmd.enable_yawRate = false;

    //固定的浮点显示
    cout.setf(ios::fixed);
    // setprecision(n) 设显示小数精度为2位
    cout << setprecision(2);
    //左对齐
    cout.setf(ios::left);
    // 强制显示小数点
    cout.setf(ios::showpoint);
    // 强制显示符号
    cout.setf(ios::showpos);

    // 打印相关信息
    cout << GREEN << ">>>>>>>>>>>>>>>> " << ros::this_node::getName() << " <<<<<<<<<<<<<<<<" << TAIL << endl;
    cout << GREEN << "uav_id                    : " << uav_id << " " << TAIL << endl;
    cout << GREEN << "sim_mode                  : " << sim_mode << " " << TAIL << endl;
    cout << GREEN << "uav_name                  : " << uav_name << " " << TAIL << endl;
    cout << GREEN << "target_tpoic_name         : " << target_tpoic_name << " " << TAIL << endl;
    
    ros::Duration(0.5).sleep();
    // 解锁
    cout<<"arm"<<endl;
    setup.cmd = 1;
    uav_setup_pub.publish(setup);
    ros::Duration(1.0).sleep();

    // 切换到指令控制模式
    cout<<"switch CMD_CONTROL"<<endl;
    setup.cmd = 4;
    setup.control_state = "CMD_CONTROL";
    uav_setup_pub.publish(setup);
    ros::Duration(1.0).sleep();

    // 起飞
    cout<<"takeoff"<<endl;
    uav_cmd.cmd = 1;
    uav_cmd.cmd_id = 0;
    control_cmd_pub.publish(uav_cmd);
    ros::Duration(10).sleep();
    
    // 悬停
    cout<<"hover"<<endl;
    uav_cmd.cmd = 2;
    control_cmd_pub.publish(uav_cmd);
    ros::Duration(3).sleep();

    // Define the proportional gain and maximum velocity
    double k_p = 1.0; // proportional gain
    double yaw_k_p = 0.02; // proportional gain
    double max_vel = 1.0; // maximum velocity (m/s)
    double max_yaw = 3; // maximum velocity (m/s)
    double yaw = 0;
    double hight = 0.8;

    geometry_msgs::PoseStamped pose;

    // Send setpoints until the drone reaches the target point
    while(ros::ok()) {
        if(stop_flag){
                cout<<"land"<<endl;
                uav_cmd.cmd = 3;
                uav_cmd.cmd_id = uav_cmd.cmd_id + 1;
                control_cmd_pub.publish(uav_cmd);
                break;
            }
        // 计算相对位置
        double x_rel = x_2 - x_1;
        double y_rel = y_2 - y_1;
        double z_rel = 1 - (z_1 - z_2);     // 保持1米高度
        double yaw_rel = yaw_2 - yaw_1;

        // 将相对位姿转换为机体系
        double x_rel_ = x_rel * cos(yaw_1) + y_rel * sin(yaw_1);
        double y_rel_ = -x_rel * sin(yaw_1) + y_rel * cos(yaw_1);

        double vx = k_p * x_rel_;
        double vy = k_p * y_rel_;
        double vz = k_p * z_rel;
        yaw = 0.0;
        yaw = yaw_k_p * yaw_rel  / M_PI * 180.0;
        // Limit the velocities to a maximum value
        vx = min(max(vx, -max_vel), max_vel);
        vy = min(max(vy, -max_vel), max_vel);
        vz = min(max(vz, -max_vel), max_vel);
        yaw = min(max(yaw, -max_yaw), max_yaw);
        uav_cmd.header.stamp = ros::Time::now();
        uav_cmd.cmd = sunray_msgs::UAVControlCMD::XYZ_VEL_BODY;
        uav_cmd.desired_vel[0] = vx;
        uav_cmd.desired_vel[1] = vy;
        uav_cmd.desired_vel[2] = vz;
        uav_cmd.desired_pos[0] = 0.0;
        uav_cmd.desired_pos[1] = 0.0;
        uav_cmd.desired_pos[2] = 0.0;
        uav_cmd.desired_yaw = yaw;
        uav_cmd.cmd_id = uav_cmd.cmd_id + 1;
        control_cmd_pub.publish(uav_cmd);

        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }

    // 降落
    cout<<"land"<<endl;
    uav_cmd.cmd = 3;
    uav_cmd.cmd_id = uav_cmd.cmd_id + 1;
    control_cmd_pub.publish(uav_cmd);
    //关键
    ros::Duration(0.5).sleep();

}
