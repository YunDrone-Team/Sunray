#include <ros/ros.h>
#include "ros_msg_utils.h"
#include "printf_utils.h"

using namespace  std;

/*
current_pose：存储无人机的当前位置。
uav_state：存储无人机的当前状态。
uav_cmd：包含要发送给无人机的控制命令。
target_pose：存储无人机要到达的目标位置。
stop_flag：一个布尔标志，用于指示何时停止无人机的操作。
*/
geometry_msgs::PoseStamped current_pose;
sunray_msgs::UAVState uav_state;
sunray_msgs::UAVControlCMD uav_cmd;
std_msgs::Int32 uav_control_state;
geometry_msgs::PoseStamped target_pose;
sunray_msgs::UAVSetup setup;

float target_yaw;
bool stop_flag{false};
//更新uav_state，接收无人机状态消息。
void uav_state_cb(const sunray_msgs::UAVState::ConstPtr &msg)
{
    uav_state = *msg;
}
//更新uav_control_state，接收控制状态消息。
void uav_control_state_cb(const std_msgs::Int32::ConstPtr &msg)
{
    uav_control_state = *msg;
}
//target_pos_cb：更新target_pose，接收目标位置，并根据方向四元数计算目标偏航角。
void target_pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    target_pose = *msg;
    tf2::Quaternion quaternion;
    tf2::fromMsg(msg->pose.orientation, quaternion);
    double roll, pitch, yaw;
    tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
    target_yaw = yaw;
}
//stop_tutorial_cb：接收到停止命令时，将stop_flag设置为真。
void stop_tutorial_cb(const std_msgs::Empty::ConstPtr &msg)
{
    stop_flag = true;
}

//pose_cb：从MAVROS主题更新current_pose，获取无人机的当前位置。
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    current_pose = *msg;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "circle_z_pos");
    ros::NodeHandle nh("~");

    ros::Rate rate(20.0);

    int uav_id;
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
    if(sim_mode){
        ros::Subscriber target_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>(topic_prefix + "/sunray/gazebo_pose", 1, target_pos_cb);
    }
    else{
        ros::Subscriber target_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/" + uav_name + "/pose", 1, target_pos_cb);
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
    ros::Duration(5).sleep();

    // Define the circle's center and radius
    double center_x = 0;
    double center_y = 0;
    double radius = 1;

    // Define the number of points on the circle
    int num_points = 50;
    // Define the proportional gain and maximum velocity
    double k_p = 1; // proportional gain
    double max_vel = 1.0; // maximum velocity (m/s)

    double hight = 0.6;

    geometry_msgs::PoseStamped pose;

    for (int i = 0; i < num_points; i++) {
        double theta = i * 2 * M_PI / num_points;
        pose.pose.position.x = center_x + radius * cos(theta);
        pose.pose.position.y = center_y + radius * sin(theta);
        pose.pose.position.z = hight; // fixed altitude

        // Send setpoints until the drone reaches the target point
        while(ros::ok()) {
            if(stop_flag){
                cout<<"land"<<endl;
                uav_cmd.cmd = 3;
                uav_cmd.cmd_id = uav_cmd.cmd_id + 1;
                control_cmd_pub.publish(uav_cmd);
                break;
            }
            // Calculate the distance to the target position
            double dx = pose.pose.position.x - current_pose.pose.position.x;
            double dy = pose.pose.position.y - current_pose.pose.position.y;

            // Calculate the desired velocity using a proportional controller
            double vx = k_p * dx;
            double vy = k_p * dy;

            // Limit the velocities to a maximum value
            vx = min(max(vx, -max_vel), max_vel);
            vy = min(max(vy, -max_vel), max_vel);

            uav_cmd.header.stamp = ros::Time::now();
            uav_cmd.cmd = sunray_msgs::UAVControlCMD::XY_VEL_Z_POS;
            uav_cmd.desired_vel[0] = vx;
            uav_cmd.desired_vel[1] = vy;
            uav_cmd.desired_vel[2] = 0.0;
            uav_cmd.desired_pos[0] = 0.0;
            uav_cmd.desired_pos[1] = 0.0;
            uav_cmd.desired_pos[2] = hight;
            uav_cmd.desired_yaw = 0;
            uav_cmd.enable_yawRate = 0;
            uav_cmd.cmd_id = uav_cmd.cmd_id + 1;
            control_cmd_pub.publish(uav_cmd);

            // Check if the drone has reached the target point
            if (fabs(current_pose.pose.position.x - pose.pose.position.x) < 0.2 &&
                fabs(current_pose.pose.position.y - pose.pose.position.y) < 0.2 ) {
                break;
            }

            ros::spinOnce();
            ros::Duration(0.1).sleep();
        }
        
    }

    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = hight;
    while(ros::ok()) {
        if(stop_flag){
                cout<<"land"<<endl;
                uav_cmd.cmd = 3;
                uav_cmd.cmd_id = uav_cmd.cmd_id + 1;
                control_cmd_pub.publish(uav_cmd);
                break;
            }
        // Calculate the distance to the target position
        double dx = pose.pose.position.x - current_pose.pose.position.x;
        double dy = pose.pose.position.y - current_pose.pose.position.y;

        // Calculate the desired velocity using a proportional controller
        double vx = k_p * dx;
        double vy = k_p * dy;

        // Limit the velocities to a maximum value
        vx = min(max(vx, -max_vel), max_vel);
        vy = min(max(vy, -max_vel), max_vel);

        uav_cmd.header.stamp = ros::Time::now();
        uav_cmd.cmd = sunray_msgs::UAVControlCMD::XY_VEL_Z_POS;
        uav_cmd.desired_vel[0] = vx;
        uav_cmd.desired_vel[1] = vy;
        uav_cmd.desired_vel[2] = 0.0;
        uav_cmd.desired_pos[0] = 0.0;
        uav_cmd.desired_pos[1] = 0.0;
        uav_cmd.desired_pos[2] = hight;
        uav_cmd.desired_yaw = 0;
        uav_cmd.enable_yawRate = 0;
        uav_cmd.cmd_id = uav_cmd.cmd_id + 1;
        control_cmd_pub.publish(uav_cmd);

        // Check if the drone has reached the target point
        if (fabs(current_pose.pose.position.x - pose.pose.position.x) < 0.2 &&
            fabs(current_pose.pose.position.y - pose.pose.position.y) < 0.2 ) {
            break;
        }

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
