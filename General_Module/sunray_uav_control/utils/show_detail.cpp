// 头文件
#include <ros/ros.h>
#include "math_utils.h"
#include "printf_utils.h"
#include "ros_msg_utils.h"

class SHOW_DETAIL
{
private:
    // 无人机名称
    string uav_name{""};
    // 无人机编号
    int uav_id;
    // 无人机数量
    int uav_num;
    // 循环显示标志
    int idx;
    Eigen::Quaterniond uav_quat; // 无人机四元数
    double uav_yaw;              // 无人机偏航角

    bool sub_state;
    bool sub_state_cmd;
    bool flash_screen;

    std::vector<ros::Time> last_state_time;
    std::vector<ros::Time> last_state_cmd_time;

    geometry_msgs::Quaternion q; // 姿态 - 四元数
    std::vector<Eigen::Vector3d> uav_pos;     // 位置
    std::vector<Eigen::Vector3d> uav_vel;     // 速度
    std::vector<Eigen::Vector3d> uav_att;     // 姿态 - 欧拉角

    std::vector<Eigen::Vector3d> pos_error;
    std::vector<Eigen::Vector3d> vel_error;
    std::vector<Eigen::Vector3d> att_error;

    std::vector<double> yaw_error;

    std::vector<ros::Subscriber> px4_state_sub; // 无人机状态指令
    std::vector<ros::Subscriber> px4_battery_sub; // 无人机电池状态
    std::vector<ros::Subscriber> px4_position_sub; // 无人机当前位置
    std::vector<ros::Subscriber> px4_velocity_sub; // 无人机当前速度
    std::vector<ros::Subscriber> px4_attitude_sub; // 无人机当前姿态
    std::vector<ros::Subscriber> uav_state_sub; // 无人机当前状态
    std::vector<ros::Subscriber> uav_state_cmd_sub; // 无人机当前状态指令

    void px4_state_cb(const mavros_msgs::State::ConstPtr &msg, int uav_idx);
    void px4_battery_cb(const sensor_msgs::BatteryState::ConstPtr &msg, int uav_idx);
    void local_position_ned_cb(const geometry_msgs::PoseStamped::ConstPtr &msg, int uav_idx);
    void local_vel_ned_cb(const geometry_msgs::TwistStamped::ConstPtr &msg, int uav_idx);
    void attitude_cb(const sensor_msgs::Imu::ConstPtr &msg, int uav_idx);

    void uav_state_cb(const sunray_msgs::UAVState::ConstPtr &msg, int uav_idx);
    void uav_state_cmd_cb(const sunray_msgs::UAVState::ConstPtr &msg, int uav_idx);

public:
    SHOW_DETAIL() {};
    ~SHOW_DETAIL() {};
    void printf_debug_info();
    void init(ros::NodeHandle &nh);
    // sunray_msgs::UAVState uav_state;
    std::vector<sunray_msgs::UAVState> uav_state;
};

void SHOW_DETAIL::init(ros::NodeHandle &nh)
{
    // 【参数】无人机编号
    nh.param<int>("uav_id", uav_id, 1);
    // 【参数】无人机名称
    nh.param<string>("uav_name", uav_name, "uav");
    // 【参数】无人机数量
    nh.param<int>("uav_num", uav_num, 1);

    // uav_name = uav_name + std::to_string(uav_id);
    nh.param<bool>("flash_screen", flash_screen, true);
    // string topic_prefix = "/" + uav_name;
    // // 【订阅】无人机模式 - 飞控 -> 本节点
    // px4_state_sub = nh.subscribe<mavros_msgs::State>(topic_prefix + "/mavros/state", 1, &SHOW_DETAIL::px4_state_cb, this);
    // // 【订阅】无人机电池状态 - 飞控 -> 本节点
    // px4_battery_sub = nh.subscribe<sensor_msgs::BatteryState>(topic_prefix + "/mavros/battery", 1, &SHOW_DETAIL::px4_battery_cb, this);
    // // 【订阅】无人机当前位置 - 飞控 -> 本节点
    // px4_position_sub = nh.subscribe<geometry_msgs::PoseStamped>(topic_prefix + "/mavros/local_position/pose", 1, &SHOW_DETAIL::local_position_ned_cb, this);
    // // 【订阅】无人机当前速度 坐标系:ENU系 (PX4 -> 本节点)
    // px4_velocity_sub = nh.subscribe<geometry_msgs::TwistStamped>(topic_prefix + "/mavros/local_position/velocity_local", 1, &SHOW_DETAIL::local_vel_ned_cb, this);
    // // 【订阅】无人机当前欧拉角 - 飞控 -> 本节点
    // px4_attitude_sub = nh.subscribe<sensor_msgs::Imu>(topic_prefix + "/mavros/imu/data", 1, &SHOW_DETAIL::attitude_cb, this);
    // // 【订阅】无人机状态 -- vision_pose -> 本节点
    // uav_state_sub = nh.subscribe<sunray_msgs::UAVState>(topic_prefix + "/sunray/uav_state", 1, &SHOW_DETAIL::uav_state_cb, this);
    // // 【订阅】无人机状态 -- vision_pose -> 本节点
    // uav_state_cmd_sub = nh.subscribe<sunray_msgs::UAVState>(topic_prefix + "/sunray/uav_state_cmd", 1, &SHOW_DETAIL::uav_state_cmd_cb, this);
    int sub_start = 1;
    if(uav_num == 1)
    {
        sub_start = uav_id;
    }
    for (sub_start; sub_start <= uav_num; sub_start++)
    {
        std::string topic_prefix = "/" + uav_name + std::to_string(sub_start);
        // 【订阅】无人机模式 - 飞控 -> 本节点
        px4_state_sub.push_back(nh.subscribe<mavros_msgs::State>(topic_prefix + "/mavros/state", 1, boost::bind(&SHOW_DETAIL::px4_state_cb, this, _1, sub_start-1)));
        // 【订阅】无人机电池状态 - 飞控 -> 本节点
        px4_battery_sub.push_back(nh.subscribe<sensor_msgs::BatteryState>(topic_prefix + "/mavros/battery", 1, boost::bind(&SHOW_DETAIL::px4_battery_cb, this, _1, sub_start-1)));
        // 【订阅】无人机当前位置 - 飞控 -> 本节点
        px4_position_sub.push_back(nh.subscribe<geometry_msgs::PoseStamped>(topic_prefix + "/mavros/local_position/pose", 1, boost::bind(&SHOW_DETAIL::local_position_ned_cb, this, _1, sub_start-1)));
        // 【订阅】无人机当前速度 坐标系:ENU系 (PX4 -> 本节点)
        px4_velocity_sub.push_back(nh.subscribe<geometry_msgs::TwistStamped>(topic_prefix + "/mavros/local_position/velocity_local", 1, boost::bind(&SHOW_DETAIL::local_vel_ned_cb, this, _1, sub_start-1)));
        // 【订阅】无人机当前欧拉角 - 飞控 -> 本节点
        px4_attitude_sub.push_back(nh.subscribe<sensor_msgs::Imu>(topic_prefix + "/mavros/imu/data", 1, boost::bind(&SHOW_DETAIL::attitude_cb, this, _1, sub_start-1)));
        // 【订阅】无人机状态 -- vision_pose -> 本节点
        uav_state_sub.push_back(nh.subscribe<sunray_msgs::UAVState>(topic_prefix + "/sunray/uav_state", 1, boost::bind(&SHOW_DETAIL::uav_state_cb, this, _1, sub_start-1)));
        // 【订阅】无人机状态 -- vision_pose -> 本节点
        uav_state_cmd_sub.push_back(nh.subscribe<sunray_msgs::UAVState>(topic_prefix + "/sunray/uav_state_cmd", 1, boost::bind(&SHOW_DETAIL::uav_state_cmd_cb, this, _1, sub_start-1)));
    }

    // 初始化
    uav_pos = std::vector<Eigen::Vector3d>(uav_num, Eigen::Vector3d::Zero());
    uav_vel = std::vector<Eigen::Vector3d>(uav_num, Eigen::Vector3d::Zero());
    uav_att = std::vector<Eigen::Vector3d>(uav_num, Eigen::Vector3d::Zero());
    pos_error = std::vector<Eigen::Vector3d>(uav_num, Eigen::Vector3d::Zero());
    vel_error = std::vector<Eigen::Vector3d>(uav_num, Eigen::Vector3d::Zero());
    att_error = std::vector<Eigen::Vector3d>(uav_num, Eigen::Vector3d::Zero());
    uav_state = std::vector<sunray_msgs::UAVState>(uav_num, sunray_msgs::UAVState());
    yaw_error = std::vector<double>(uav_num, 0.0);
    last_state_time = std::vector<ros::Time>(uav_num, ros::Time::now());
    last_state_cmd_time = std::vector<ros::Time>(uav_num, ros::Time::now());
    uav_quat = Eigen::Quaterniond(1, 0, 0, 0);
    uav_yaw = 0.0;
    idx = 0;
}

void SHOW_DETAIL::uav_state_cb(const sunray_msgs::UAVState::ConstPtr &msg, int uav_idx)
{
    // uav_state = *msg
    uav_state.at(uav_idx) = *msg;
    if ((ros::Time::now() - last_state_cmd_time.at(uav_idx)).toSec() > 1)
    {
        uav_quat.w() = msg->attitude_q.w;
        uav_quat.x() = msg->attitude_q.x;
        uav_quat.y() = msg->attitude_q.y;
        uav_quat.z() = msg->attitude_q.z;
    }
    tf::Quaternion q(uav_quat.x(), uav_quat.y(), uav_quat.z(), uav_quat.w());
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    uav_yaw = yaw;
    last_state_time.at(uav_idx) = ros::Time::now();
}

void SHOW_DETAIL::uav_state_cmd_cb(const sunray_msgs::UAVState::ConstPtr &msg, int uav_idx)
{
    // uav_state = *msg;
    uav_state.at(uav_idx) = *msg;
    uav_quat.w() = msg->attitude_q.w;
    uav_quat.x() = msg->attitude_q.x;
    uav_quat.y() = msg->attitude_q.y;
    uav_quat.z() = msg->attitude_q.z;

    tf::Quaternion q(uav_quat.x(), uav_quat.y(), uav_quat.z(), uav_quat.w());
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    uav_yaw = yaw;
    last_state_cmd_time.at(uav_idx) = ros::Time::now();
}

void SHOW_DETAIL::px4_state_cb(const mavros_msgs::State::ConstPtr &msg, int uav_idx)
{
    uav_state.at(uav_idx).connected = msg->connected;
    uav_state.at(uav_idx).mode = msg->mode.c_str();
    uav_state.at(uav_idx).armed = msg->armed;
}

void SHOW_DETAIL::px4_battery_cb(const sensor_msgs::BatteryState::ConstPtr &msg, int uav_idx)
{
    uav_state.at(uav_idx).battery_state = msg->voltage;
    uav_state.at(uav_idx).battery_percetage = msg->percentage * 100;
}

void SHOW_DETAIL::local_position_ned_cb(const geometry_msgs::PoseStamped::ConstPtr &msg, int uav_idx)
{
    uav_pos.at(uav_idx) = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    for (int i = 0; i < 3; i++)
    {
        pos_error.at(uav_idx)[i] = uav_state.at(uav_idx).position[i] - uav_pos.at(uav_idx)[i];
    }
}

void SHOW_DETAIL::local_vel_ned_cb(const geometry_msgs::TwistStamped::ConstPtr &msg, int uav_idx)
{
    uav_vel.at(uav_idx) = Eigen::Vector3d(msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z);
    for (int i = 0; i < 3; i++)
    {
        vel_error.at(uav_idx)[i] = uav_state.at(uav_idx).velocity[i] - uav_vel.at(uav_idx)[i];
    }
}

void SHOW_DETAIL::attitude_cb(const sensor_msgs::Imu::ConstPtr &msg, int uav_idx)
{
    tf2::Quaternion quaternion;
    tf2::fromMsg(msg->orientation, quaternion);

    // 转换为欧拉角
    tf2::Matrix3x3 mat(quaternion);
    double roll, pitch, yaw;
    mat.getRPY(roll, pitch, yaw);
    uav_att.at(uav_idx) = Eigen::Vector3d(roll, pitch, yaw);
    for (int i = 0; i < 3; i++)
    {
        att_error.at(uav_idx)[i] = uav_state.at(uav_idx).attitude[i] - uav_att.at(uav_idx)[i];
    }
    yaw_error.at(uav_idx) = uav_state.at(uav_idx).attitude[2] - uav_att.at(uav_idx)[2];
}

void SHOW_DETAIL::printf_debug_info()
{

    if (flash_screen == true)
    {
        system("clear"); // 清屏
    }
    // 固定的浮点显示
    cout.setf(ios::fixed);
    // setprecision(n) 设显示小数精度为n位
    cout << setprecision(2);
    // 左对齐
    cout.setf(ios::left);
    // 强制显示小数点
    cout.setf(ios::showpoint);
    // 强制显示符号
    cout.setf(ios::showpos);

    cout << BLUE << "-------------------------- [ "<< uav_name + std::to_string(idx+1) <<" ]----------------------------------" << TAIL << endl;
    if ((ros::Time::now() - last_state_time.at(idx)).toSec() < 1)
    {
        cout << GREEN << "vision_pose_node: [ Success ] " << TAIL;
    }
    else
    {
        cout << RED << "vision_pose_node: [ Failed  ] " << TAIL;
    }
    if ((ros::Time::now() - last_state_cmd_time.at(idx)).toSec() < 1)
    {
        cout << GREEN << "uav_control: [ Success ] " << TAIL << endl;
    }
    else
    {
        cout << RED << "uav_control: [ Failed  ] " << TAIL << endl;
    }

    // 打印 无人机状态
    if (uav_state.at(idx).connected == true)
    {
        cout << GREEN << "PX4 Status:  [ Connected ] ";
    }
    else
    {
        cout << RED << "PX4 Status:[ Unconnected ] ";
    }
    // 是否上锁
    if (uav_state.at(idx).armed == true)
    {
        cout << GREEN << "[  Armed   ] ";
    }
    else
    {
        cout << RED << "[ DisArmed ] ";
    }

    cout << "[ " << uav_state.at(idx).mode << " ] " << TAIL << endl;

    // 打印外部输入的原始数据
    switch (uav_state.at(idx).location_source)
    {
    case sunray_msgs::ExternalOdom::MOCAP:
        cout << GREEN << "External Odom: [ MOCAP ] ";
        break;
    case sunray_msgs::ExternalOdom::VIOBOT:
        cout << GREEN << "External Odom: [ VIOBOT ] ";
        break;
    case sunray_msgs::ExternalOdom::GAZEBO:
        cout << GREEN << "External Odom: [ GAZEBO ] ";
        break;
    case sunray_msgs::ExternalOdom::VINS:
        cout << GREEN << "External Odom: [ VINS ] ";
        break;
    }

    if (uav_state.at(idx).odom_valid)
    {
        cout << GREEN << " Odom Status : [ Valid ] " << TAIL << endl;
    }
    else
    {
        cout << RED << " Odom Status : [ Invalid ] " << TAIL << endl;
    }

    cout << GREEN << "Battery Voltage : " << uav_state.at(idx).battery_state << " [V] "
         << " Battery Percent : " << uav_state.at(idx).battery_percetage << " [%] " << TAIL << endl;

    // 打印发布的Vision Pose数据
    cout << BLUE << "Pose Send to Autopilot [NED]: " << TAIL << endl;
    cout << GREEN << "Pos [X Y Z] :        " << uav_state.at(idx).position[0] << "  " << uav_state.at(idx).position[1] << "  " << uav_state.at(idx).position[2] << " [ m ] " << TAIL << endl;
    cout << GREEN << "Vel [X Y Z] :        " << uav_state.at(idx).velocity[0] << "  " << uav_state.at(idx).velocity[1] << "  " << uav_state.at(idx).velocity[2] << " [m/s] " << TAIL << endl;
    cout << GREEN << "Att [R P Y] :        " << uav_state.at(idx).attitude[0] * 180 / M_PI << "  " << uav_state.at(idx).attitude[1] * 180 / M_PI << "  " << uav_state.at(idx).attitude[2] * 180 / M_PI << " [deg] " << TAIL << endl;

    // 打印飞控回传的数据
    cout << BLUE << "Pose from Autopilot [NED]: " << TAIL << endl;
    cout << GREEN << "Pos [X Y Z] :        " << uav_pos.at(idx)[0] << "  " << uav_pos.at(idx)[1] << "  " << uav_pos.at(idx)[2] << " [ m ] " << TAIL << endl;
    cout << GREEN << "Vel [X Y Z] :        " << uav_vel.at(idx)[0] << "  " << uav_vel.at(idx)[1] << "  " << uav_vel.at(idx)[2] << " [m/s] " << TAIL << endl;
    cout << GREEN << "Att [R P Y] :        " << uav_att.at(idx)[0] * 180 / M_PI << "  " << uav_att.at(idx)[1] * 180 / M_PI << "  " << uav_att.at(idx)[2] * 180 / M_PI << " [deg] " << TAIL << endl;

    // 打印计算得到的差值
    cout << BLUE << "Pose Error [NED]: " << TAIL << endl;
    cout << GREEN << "Pos [X Y Z] :        " << pos_error.at(idx)[0] << "  " << pos_error.at(idx)[1] << "  " << pos_error.at(idx)[2] << " [ m ] " << TAIL << endl;
    cout << GREEN << "Vel [X Y Z] :        " << vel_error.at(idx)[0] << "  " << vel_error.at(idx)[1] << "  " << vel_error.at(idx)[2] << " [m/s] " << TAIL << endl;
    cout << GREEN << "Att [R P Y] :        " << att_error.at(idx)[0] * 180 / M_PI << "  " << att_error.at(idx)[1] * 180 / M_PI << "  " << att_error.at(idx)[2] * 180 / M_PI << " [deg] " << TAIL << endl;

    if ((ros::Time::now() - last_state_cmd_time.at(idx)).toSec() < 1)
    {
        // 打印目标点
        cout << BLUE << "Target from Autopilot [NED]: " << TAIL << endl;
        cout << GREEN << "Pos_target [X Y Z] : " << uav_state.at(idx).pos_setpoint[0] << "  " << uav_state.at(idx).pos_setpoint[1] << "  " << uav_state.at(idx).pos_setpoint[2] << " [ m ] " << TAIL << endl;
        cout << GREEN << "Vel_target [X Y Z] : " << uav_state.at(idx).vel_setpoint[0] << "  " << uav_state.at(idx).vel_setpoint[1] << "  " << uav_state.at(idx).vel_setpoint[2] << " [m/s] " << TAIL << endl;
        cout << GREEN << "Yaw_target         :true " << yaw_error.at(idx) * 180 / M_PI << " [deg] " << TAIL << endl;

        switch (uav_state.at(idx).control_mode)
        {
        case sunray_msgs::UAVControlCMD::Takeoff:
            cout << GREEN << "Command: [ Takeoff ] " << TAIL << endl;
            break;

        case sunray_msgs::UAVControlCMD::Hover:
            cout << GREEN << "Command: [ Hover ] " << TAIL << endl;
            break;

        case sunray_msgs::UAVControlCMD::Land:
            cout << GREEN << "Command: [ Land ] " << TAIL << endl;
            break;

        case sunray_msgs::UAVControlCMD::XYZ_POS:
            cout << GREEN << "Command: [ Move in XYZ_POS ] " << TAIL << endl;
            break;
        case sunray_msgs::UAVControlCMD::XY_VEL_Z_POS:
            cout << GREEN << "Command: [ Move in XY_VEL_Z_POS ] " << TAIL << endl;
            break;
        case sunray_msgs::UAVControlCMD::XYZ_VEL:
            cout << GREEN << "Command: [ Move in XYZ_VEL ] " << TAIL << endl;
            break;
        case sunray_msgs::UAVControlCMD::XYZ_POS_BODY:
            cout << GREEN << "Command: [ Move in XYZ_POS_BODY ] " << TAIL << endl;
            break;
        case sunray_msgs::UAVControlCMD::XYZ_VEL_BODY:
            cout << GREEN << "Command: [ Move in XYZ_VEL_BODY ] " << TAIL << endl;
            break;
        case sunray_msgs::UAVControlCMD::XY_VEL_Z_POS_BODY:
            cout << GREEN << "Command: [ Move in XY_VEL_Z_POS_BODY ] " << TAIL << endl;
            break;
        default:
            cout << GREEN << "Command: [ Unknown Mode ]. " << TAIL << endl;
            break;
        }
    }
    cout << BLUE << "------------------------------------------------------------\n"
         << TAIL << endl;
    if (++idx > uav_num-1)
        idx = 0;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "show_detail");
    ros::NodeHandle nh("~");
    ros::Rate rate(10.0);

    // 外部定位数据
    SHOW_DETAIL show_detail;
    show_detail.init(nh);

    ros::Time time_now = ros::Time::now();
    ros::Time time_last = ros::Time::now();
    time_last.sec = time_last.sec + 2; // 解决显示问题

    ros::Duration(1.0).sleep();
    // 主循环
    while (ros::ok())
    {
        // 回调函数
        ros::spinOnce();

        // 定时状态打印
        time_now = ros::Time::now();
        if ((time_now - time_last).toSec() > 1.0)
        {
            show_detail.printf_debug_info();
            time_last = time_now;
        }

        rate.sleep();
    }
}