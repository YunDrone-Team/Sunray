#include <ros/ros.h>
#include "ros_msg_utils.h"
#include "printf_utils.h"

using namespace std;

/*
current_pose：存储无人机的当前位置。
uav_cmd：包含要发送给无人机的控制命令。
stop_flag：一个布尔标志，用于指示何时停止无人机的操作。
*/
geometry_msgs::PoseStamped current_pose;
sunray_msgs::UAVControlCMD uav_cmd;
sunray_msgs::UAVSetup setup;

bool stop_flag{false};
// stop_tutorial_cb：接收到停止命令时，将stop_flag设置为真。
void stop_tutorial_cb(const std_msgs::Empty::ConstPtr &msg)
{
    stop_flag = true;
}

// pose_cb：从MAVROS主题更新current_pose，获取无人机的当前位置。
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    current_pose = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "circle_z_pos");
    ros::NodeHandle nh("~");

    ros::Rate rate(20.0);

    int uav_id;
    string uav_name;
    // 【参数】无人机编号
    nh.param<int>("uav_id", uav_id, 1);
    // 【参数】无人机名称
    nh.param<string>("uav_name", uav_name, "uav");

    uav_name = uav_name + std::to_string(uav_id);
    string topic_prefix = "/" + uav_name;
    // 【订阅】任务结束
    ros::Subscriber stop_tutorial_sub = nh.subscribe<std_msgs::Empty>(topic_prefix + "/sunray/stop_tutorial", 1, stop_tutorial_cb);
    // 【订阅】无人机当前位置
    ros::Subscriber pose_sub = nh.subscribe(topic_prefix + "/mavros/local_position/pose", 10, pose_cb);
    // 【发布】无人机控制指令 （本节点 -> sunray_control_node）
    ros::Publisher control_cmd_pub = nh.advertise<sunray_msgs::UAVControlCMD>(topic_prefix + "/sunray/uav_control_cmd", 1);
    // 【发布】无人机设置指令（本节点 -> sunray_control_node）
    ros::Publisher uav_setup_pub = nh.advertise<sunray_msgs::UAVSetup>(topic_prefix + "/sunray/setup", 1);

    // 变量初始化
    uav_cmd.header.stamp = ros::Time::now();
    uav_cmd.cmd = 102;
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

    // 固定的浮点显示
    cout.setf(ios::fixed);
    // setprecision(n) 设显示小数精度为2位
    cout << setprecision(2);
    // 左对齐
    cout.setf(ios::left);
    // 强制显示小数点
    cout.setf(ios::showpoint);
    // 强制显示符号
    cout.setf(ios::showpos);

    // 打印相关信息
    cout << GREEN << ">>>>>>>>>>>>>>>> " << ros::this_node::getName() << " <<<<<<<<<<<<<<<<" << TAIL << endl;
    cout << GREEN << "uav_id                    : " << uav_id << " " << TAIL << endl;
    cout << GREEN << "uav_name                  : " << uav_name << " " << TAIL << endl;

    ros::Duration(0.5).sleep();
    // 解锁
    cout << "arm" << endl;
    setup.cmd = 1;
    uav_setup_pub.publish(setup);
    ros::Duration(1.0).sleep();

    // 切换到指令控制模式
    cout << "switch CMD_CONTROL" << endl;
    setup.cmd = 4;
    setup.control_state = "CMD_CONTROL";
    uav_setup_pub.publish(setup);
    ros::Duration(1.0).sleep();

    // 起飞
    cout << "takeoff" << endl;
    uav_cmd.cmd = 100;
    control_cmd_pub.publish(uav_cmd);
    ros::Duration(10).sleep();

    // 悬停
    cout << "hover" << endl;
    uav_cmd.cmd = 102;
    control_cmd_pub.publish(uav_cmd);
    ros::Duration(5).sleep();

    // Define the circle's center and radius
    double center_x = 0;
    double center_y = 0;
    double radius = 1;

    // Define the number of points on the circle
    int num_points = 50;
    // Define the proportional gain and maximum velocity
    double k_p = 1;       // proportional gain
    double max_vel = 1.0; // maximum velocity (m/s)

    double hight = 0.6;

    geometry_msgs::PoseStamped pose;
    std::cout << "start circle" << std::endl;
    for (int i = 0; i < num_points; i++)
    {
        double theta = i * 2 * M_PI / num_points;
        pose.pose.position.x = center_x + radius * cos(theta);
        pose.pose.position.y = center_y + radius * sin(theta);
        pose.pose.position.z = hight; // fixed altitude

        // Send setpoints until the drone reaches the target point
        while (ros::ok())
        {
            if (stop_flag)
            {
                cout << "land" << endl;
                uav_cmd.cmd = 101;
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
            uav_cmd.cmd = 3;
            uav_cmd.desired_vel[0] = vx;
            uav_cmd.desired_vel[1] = vy;
            uav_cmd.desired_pos[2] = hight;
            control_cmd_pub.publish(uav_cmd);

            // Check if the drone has reached the target point
            if (fabs(current_pose.pose.position.x - pose.pose.position.x) < 0.15 &&
                fabs(current_pose.pose.position.y - pose.pose.position.y) < 0.15)
            {
                break;
            }
            ros::spinOnce();
            ros::Duration(0.1).sleep();
        }
    }

    // 回到原点
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = hight;
    while (ros::ok())
    {
        if (stop_flag)
        {
            cout << "land" << endl;
            uav_cmd.cmd = 101;
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
        uav_cmd.cmd = 3;
        uav_cmd.desired_vel[0] = vx;
        uav_cmd.desired_vel[1] = vy;
        uav_cmd.desired_vel[2] = 0.0;
        uav_cmd.desired_pos[0] = 0.0;
        uav_cmd.desired_pos[1] = 0.0;
        uav_cmd.desired_pos[2] = hight;
        uav_cmd.desired_yaw = 0;
        control_cmd_pub.publish(uav_cmd);

        // Check if the drone has reached the target point
        if (fabs(current_pose.pose.position.x - pose.pose.position.x) < 0.2 &&
            fabs(current_pose.pose.position.y - pose.pose.position.y) < 0.2)
        {
            // 停下1秒等待无人机速度降下来
            ros::Duration(1.0).sleep();
            break;
        }

        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }

    // 降落
    cout << "land" << endl;
    uav_cmd.cmd = 101;
    control_cmd_pub.publish(uav_cmd);
    ros::Duration(0.5).sleep();
}
