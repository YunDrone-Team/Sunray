#include <ros/ros.h>
#include "ros_msg_utils.h"
#include "printf_utils.h"

sunray_msgs::UAVState uav_state;
sunray_msgs::UAVControlCMD uav_cmd;
std_msgs::Int32 uav_control_state;
geometry_msgs::PoseStamped target_pose;
float target_yaw;
bool stop_flag{false};

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
    target_yaw = yaw;
}
void stop_tutorial_cb(const std_msgs::Empty::ConstPtr &msg)
{
    stop_flag = true;
}

int main(int argc, char **argv)
{
    // ROS初始化,设定节点名
    ros::init(argc, argv, "follow_a_car");
    // 创建句柄
    ros::NodeHandle nh("~");
    ros::Rate rate(20.0);

    int uav_id;
    string uav_name, target_tpoic_name;
    bool sim_mode, flag_printf;
    nh.param<bool>("sim_mode", sim_mode, false);
    nh.param<bool>("flag_printf", flag_printf, true);
    // 【参数】无人机编号
    nh.param<int>("uav_id", uav_id, 0);
    // 【参数】无人机名称
    nh.param<string>("uav_name", uav_name, "uav");
    // 【参数】目标话题名称
    nh.param<string>("target_tpoic_name", target_tpoic_name, "/vrpn_client_node/target/pose");


    string topic_prefix = "/" + uav_name;
    // 【订阅】无人机状态 -- from vision_pose
    ros::Subscriber uav_state_sub = nh.subscribe<sunray_msgs::UAVState>(topic_prefix + "/sunray/uav_state", 1, uav_state_cb);
    //【订阅】无人机控制信息
    ros::Subscriber uav_contorl_state_sub = nh.subscribe<std_msgs::Int32>(topic_prefix + "/sunray/control_state", 1, uav_control_state_cb);
    // 【订阅】目标点位置
    ros::Subscriber target_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/" + uav_name + "/pose", 1, target_pos_cb);
    // 【订阅】任务结束
    ros::Subscriber stop_tutorial_sub = nh.subscribe<std_msgs::Empty>(topic_prefix + "/sunray/stop_tutorial", 1, stop_tutorial_cb);
    
    // 【发布】无人机控制指令 （本节点 -> sunray_control_node）
    ros::Publisher control_cmd_pub = nh.advertise<sunray_msgs::UAVControlCMD>(topic_prefix + "/sunray/uav_control_cmd", 1);
    // 【发布】无人机设置指令（本节点 -> sunray_control_node）
    ros::Publisher uav_setup_pub = nh.advertise<sunray_msgs::UAVSetup>(topic_prefix + "/sunray/setup", 1);

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

    // 等待遥控器切换至COMMAND_CONTROL
    while (ros::ok())
    {
        // 回调函数
        ros::spinOnce();
        if (uav_control_state.data != 2)
        {
            cout << YELLOW << "Please switch to COMMAND_CONTROL mode firstly with RC..." << TAIL << endl;
        }else
        {
            break;
        }
        sleep(5.0);
    }

    int start_flag;
    cout << GREEN << "Please enter 1 to takeoff... " << TAIL << endl;
    cin >> start_flag;

    // 起飞
    while (ros::ok())
    {
        // 回调函数
        ros::spinOnce();
        if(uav_state.position[2] <= 0.5)
        {
            uav_cmd.header.stamp = ros::Time::now();
            uav_cmd.cmd_id = uav_cmd.cmd_id + 1;
            uav_cmd.cmd = sunray_msgs::UAVControlCMD::Takeoff;
            control_cmd_pub.publish(uav_cmd);
            cout << GREEN << "Takeoff... " << TAIL << endl;
        }else
        {
            break;
        }
        sleep(2.0);
    }

    cout << GREEN << "Please enter 1 to start follow target... " << TAIL << endl;
    cin >> start_flag;

    // 追踪
    while (ros::ok() && !stop_flag)
    {
        // 回调函数
        ros::spinOnce();
        uav_cmd.header.stamp = ros::Time::now();
        uav_cmd.cmd_id = uav_cmd.cmd_id + 1;
        uav_cmd.cmd = sunray_msgs::UAVControlCMD::Hover;
        uav_cmd.desired_pos[0] = target_pose.pose.position.x;
        uav_cmd.desired_pos[1] = target_pose.pose.position.y;
        uav_cmd.desired_pos[2] = target_pose.pose.position.z;
        uav_cmd.desired_yaw = target_yaw;
        uav_cmd.enable_yawRate = false;
        control_cmd_pub.publish(uav_cmd);

        rate.sleep();
    }

    cout << GREEN << "Land... " << TAIL << endl;

    // 降落
    while (ros::ok())
    {
        // 回调函数
        ros::spinOnce();
        if(uav_state.position[2] > 0.5)
        {
            uav_cmd.header.stamp = ros::Time::now();
            uav_cmd.cmd_id = uav_cmd.cmd_id + 1;
            uav_cmd.cmd = sunray_msgs::UAVControlCMD::Land;
            control_cmd_pub.publish(uav_cmd);
            cout << GREEN << "Land... " << TAIL << endl;
        }else
        {
            break;
        }
        sleep(2.0);
    }

    return 0;
}
