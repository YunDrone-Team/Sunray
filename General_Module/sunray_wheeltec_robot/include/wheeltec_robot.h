#ifndef WHEELTEC_ROBOT_H
#define WHEELTEC_ROBOT_H

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>
#include <sunray_msgs/UGVControlCMD.h>
#include <sunray_msgs/ViobotState.h>

#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Float32.h>
#include <visualization_msgs/Marker.h>

#include "printf_utils.h"
#include "math_utils.h"

#define TRA_WINDOW 40                // 发布轨迹长度

class WheeltecRobot
{
    public:
        // 构造函数
        WheeltecRobot(){};

        // 初始化函数
        void init(ros::NodeHandle& nh, bool if_pritf);
        // 主循环函数
        void mainloop();

        sunray_msgs::UGVControlCMD ugv_control_cmd;         // 无人车外部控制指令
        std_msgs::Float32 battery;                          // 电池电量
        geometry_msgs::Twist cmd_vel;                       // 发送给底层的控制指令
        float k_p,k_yaw;                   // 速度控制参数
        float max_vel;                                      // 无人车最大速度
        bool flag_printf;                                   // 是否打印
        float error_yaw;                                    // 无人车偏航角误差
        sunray_msgs::ViobotState viobot_state;   // VIOBOT状态集合
        Eigen::Vector3d ugv_pos;                            // 无人车位置
        Eigen::Vector3d ugv_vel;                            // 无人车速度
        double ugv_yaw;                                     // 无人车偏航角
        geometry_msgs::Quaternion ugv_q_msg;                // 四元数

    private:
        string node_name;
        int ugv_id;
        string ugv_name{""};

        bool mainloop_start_flag{false};

        struct geo_fence
        {
            float x_min;
            float x_max;
            float y_min;
            float y_max;
        };
        geo_fence ugv_geo_fence;

        std::vector<geometry_msgs::PoseStamped> posehistory_vector_;    // 无人车轨迹容器
        string mesh_resource;

        // 【订阅】无人车控制指令
        ros::Subscriber ugv_control_cmd_sub;
        // 【订阅】VIOBOT state
        ros::Subscriber viobot_state_sub;
        // 【订阅】电池状态(无人车底板电压)
        ros::Subscriber battery_sub;
        // 【发布】底层控制指令
        ros::Publisher cmd_pub;
        // 【发布】RVIZ显示相关话题
        ros::Publisher ugv_mesh_pub;
        // 【发布】RVIZ显示相关话题
        ros::Publisher trajectory_pub;
        // 【定时器】定时打印
        ros::Timer debug_timer;
        ros::Timer timer_rviz_pub;

        int safety_check();
        geometry_msgs::Twist track_point(float enu_x, float enu_y, float yaw);
        geometry_msgs::Twist track_astar_path();
        geometry_msgs::Twist limit_velocity(geometry_msgs::Twist cmd_vel, double max_velocity);
        void viobot_state_cb(const sunray_msgs::ViobotState::ConstPtr &msg);
        void battery_cb(const std_msgs::Float32::ConstPtr &msg);
        void ugv_control_cmd_cb(const sunray_msgs::UGVControlCMD::ConstPtr& msg);
        void debug_timer_cb(const ros::TimerEvent &e);
        void timercb_rviz(const ros::TimerEvent &e);
};

void WheeltecRobot::init(ros::NodeHandle& nh, bool if_pritf)
{
    nh.param<string>("ugv_name", ugv_name, "none");
    nh.param<int>("ugv_id", ugv_id, 0);
    // 【参数】地理围栏
    nh.param<float>("geo_fence/x_min", ugv_geo_fence.x_min, -10.0);
    nh.param<float>("geo_fence/x_max", ugv_geo_fence.x_max, 10.0);
    nh.param<float>("geo_fence/y_min", ugv_geo_fence.y_min, -10.0);
    nh.param<float>("geo_fence/y_max", ugv_geo_fence.y_max, 10.0);

    nh.param("k_p", k_p, 2.0f);
    nh.param("k_yaw", k_yaw, 2.0f);
    nh.param("max_vel", max_vel, 2.0f);
    nh.param("mesh_resource", mesh_resource, std::string("package://sunray_wheeltec_robot/meshes/car.dae"));

    string topic_name = "/"  + ugv_name + std::to_string(ugv_id);

    // 【订阅】无人车控制指令
    ugv_control_cmd_sub = nh.subscribe<sunray_msgs::UGVControlCMD>(topic_name + "/sunray/ugv_control_cmd", 2, &WheeltecRobot::ugv_control_cmd_cb, this);
    // 【订阅】VIOBOT state
    viobot_state_sub = nh.subscribe<sunray_msgs::ViobotState>("/viobot/viobot_state", 1, &WheeltecRobot::viobot_state_cb, this);
    // 【订阅】电池状态(无人车底板电压)
    battery_sub = nh.subscribe<std_msgs::Float32>(topic_name + "/PowerVoltage", 1, &WheeltecRobot::battery_cb, this);
    // 【发布】底层控制指令
    cmd_pub = nh.advertise<geometry_msgs::Twist>(topic_name + "/cmd_vel", 10);
    // 【发布】mesh，用于RVIZ显示
    ugv_mesh_pub =  nh.advertise<visualization_msgs::Marker>(topic_name + "/sunray/ugv_mesh", 1);
    // 【发布】无人机移动轨迹，用于RVIZ显示
    trajectory_pub = nh.advertise<nav_msgs::Path>(topic_name + "/sunray/ugv_trajectory", 10);
    
    // 【定时器】定时打印
    debug_timer = nh.createTimer(ros::Duration(2.0), &WheeltecRobot::debug_timer_cb, this);
    // 【定时器】发布RVIZ显示相关话题，5Hz
    timer_rviz_pub = nh.createTimer(ros::Duration(0.2), &WheeltecRobot::timercb_rviz, this);
    
    flag_printf = if_pritf;

    node_name = ros::this_node::getName();
    cout << GREEN << node_name << " - WheeltecRobot init! " << TAIL << endl;
    cout << GREEN << "ugv_name   : "<< ugv_name << TAIL <<endl; 
    cout << GREEN << "k_yaw      : "<< k_yaw <<"  "<< TAIL <<endl; 
    cout << GREEN << "max_vel    : "<< max_vel <<"  "<< TAIL <<endl; 
    cout << GREEN << "geo_fence_x : "<< ugv_geo_fence.x_min << " [m]  to  "<< ugv_geo_fence.x_max << " [m]"<< TAIL << endl;
    cout << GREEN << "geo_fence_y : "<< ugv_geo_fence.y_min << " [m]  to  "<< ugv_geo_fence.y_max << " [m]"<< TAIL << endl;
}


void WheeltecRobot::mainloop()
{
    mainloop_start_flag = true;

    // 安全检查 怎么处理？
    int safety_flag = safety_check();

    switch (ugv_control_cmd.cmd)
    {
    // 停止模式
    case sunray_msgs::UGVControlCMD::Hold:
        
        cmd_vel.linear.x = 0.0;
        cmd_vel.linear.y = 0.0;
        cmd_vel.linear.z = 0.0;
        cmd_vel.angular.x = 0.0;
        cmd_vel.angular.y = 0.0;
        cmd_vel.angular.z = 0.0;
        cmd_pub.publish(cmd_vel);
        break;

    // 车体系直接控制模式 - 直接使用外部发送过来的速度指令进行控制
    case sunray_msgs::UGVControlCMD::Direct_Control_BODY:  //speed control by vx,vy

        // 注: linear.x与linear.y控制的是无人车车体系下的线速度
        cmd_vel.linear.x = ugv_control_cmd.linear_vel[0];
        cmd_vel.linear.y = ugv_control_cmd.linear_vel[1];
        cmd_vel.linear.z = 0.0;
        cmd_vel.angular.x = 0.0;
        cmd_vel.angular.y = 0.0;
        cmd_vel.angular.z = ugv_control_cmd.angular_vel;
        // 速度限制幅度
        cmd_vel = limit_velocity(cmd_vel, max_vel);
        cmd_pub.publish(cmd_vel);

        break;

    case sunray_msgs::UGVControlCMD::Direct_Control_ENU:  //speed control of yaw angle
        
        error_yaw = ugv_control_cmd.desired_yaw - ugv_yaw;

        if(error_yaw < -M_PI)
        {
            error_yaw = error_yaw + 2*M_PI;
        }else if(error_yaw > M_PI)
        {
            error_yaw = error_yaw - 2*M_PI;
        }

        if( abs(error_yaw) < 5.0/180.0 * M_PI)  //small angle: direct movement
        {
            float body_x, body_y;

            body_x = ugv_control_cmd.linear_vel[0] * cos(ugv_yaw) + ugv_control_cmd.linear_vel[1] * sin(ugv_yaw);
            body_y = -ugv_control_cmd.linear_vel[0] * sin(ugv_yaw) + ugv_control_cmd.linear_vel[1] * cos(ugv_yaw);

            cmd_vel.linear.x = body_x;
            cmd_vel.linear.y = body_y;
            cmd_vel.linear.z = 0.0;
            cmd_vel.angular.x = 0.0;
            cmd_vel.angular.y = 0.0;
            cmd_vel.angular.z = k_yaw*error_yaw;
        }else
        {
            // 先调整yaw
            cmd_vel.linear.x = 0.0;
            cmd_vel.linear.y = 0.0;
            cmd_vel.linear.z = 0.0;
            cmd_vel.angular.x = 0.0;
            cmd_vel.angular.y = 0.0;
            cmd_vel.angular.z = k_yaw*error_yaw;
        }

        cmd_pub.publish(cmd_vel);

        break;

    case sunray_msgs::UGVControlCMD::Point_Control:
        
        cmd_vel = track_point(ugv_control_cmd.desired_pos[0], ugv_control_cmd.desired_pos[1], ugv_control_cmd.desired_yaw);
        cmd_pub.publish(cmd_vel);
        break;

    case sunray_msgs::UGVControlCMD::Point_Control_with_Astar:  //more vel_avoid_nei than point control

        cmd_vel = track_point(ugv_control_cmd.desired_pos[0], ugv_control_cmd.desired_pos[1], ugv_control_cmd.desired_yaw);
        cmd_pub.publish(cmd_vel);
        break;

    case sunray_msgs::UGVControlCMD::Test:  //测试程序：走圆

        break;

    }
}

geometry_msgs::Twist WheeltecRobot::track_point(float enu_x, float enu_y, float yaw)
{
    geometry_msgs::Twist cmd;
    // ugv_control_cmd.desired_yaw = (-180,180]
    // ugv_yaw = (-180,180] not sure
    // error_yaw = (-180,180] 
    error_yaw = yaw - ugv_yaw;

    if(error_yaw < -M_PI)
    {
        error_yaw = error_yaw + 2*M_PI;
    }else if(error_yaw > M_PI)
    {
        error_yaw = error_yaw - 2*M_PI;
    }

    if( abs(error_yaw) < 5.0/180.0 * M_PI)
    {
        float cmd_enu_x, cmd_enu_y;
        cmd_enu_x = k_p*(enu_x - ugv_pos[0]);
        cmd_enu_y = k_p*(enu_y - ugv_pos[1]);
        float cmd_body_x, cmd_body_y;
        cmd_body_x = cmd_enu_x * cos(ugv_yaw) + cmd_enu_y * sin(ugv_yaw);
        cmd_body_y = -cmd_enu_x * sin(ugv_yaw) + cmd_enu_y * cos(ugv_yaw);

        cmd.linear.x = cmd_body_x;
        cmd.linear.y = cmd_body_y;
        cmd.linear.z = 0.0;
        cmd.angular.x = 0.0;
        cmd.angular.y = 0.0;
        cmd.angular.z = k_yaw*error_yaw;
    }else
    {
        // 如果偏航角误差较大，则先调整yaw
        cmd.linear.x = 0.0;
        cmd.linear.y = 0.0;
        cmd.linear.z = 0.0;
        cmd.angular.x = 0.0;
        cmd.angular.y = 0.0;
        cmd.angular.z = k_yaw*error_yaw;
    }

    // 速度限制幅度
    cmd = limit_velocity(cmd, max_vel);

    return cmd;
}

geometry_msgs::Twist WheeltecRobot::limit_velocity(geometry_msgs::Twist cmd_vel, double max_velocity)
{
    if(cmd_vel.linear.x > max_velocity)
    {
        cmd_vel.linear.x = max_velocity;
    }else if(cmd_vel.linear.x < -max_velocity)
    {
        cmd_vel.linear.x = -max_velocity;
    }

    if(cmd_vel.linear.y > max_velocity)
    {
        cmd_vel.linear.y = max_velocity;
    }else if(cmd_vel.linear.y < -max_velocity)
    {
        cmd_vel.linear.y = -max_velocity;
    }

    return cmd_vel;
}


void WheeltecRobot::debug_timer_cb(const ros::TimerEvent &e)
{
    if(!flag_printf)
    {
        return;
    }
    //固定的浮点显示
    cout.setf(ios::fixed);
    // setprecision(n) 设显示小数精度为n位
    cout << setprecision(NUM_POINT);
    //左对齐
    cout.setf(ios::left);
    // 强制显示小数点
    cout.setf(ios::showpoint);
    // 强制显示符号
    cout.setf(ios::showpos);

    cout << BLUE << "-------> /"<< ugv_name+ std::to_string(ugv_id) <<" wheeltec robot: " << TAIL << endl;
    cout << GREEN << "UAV_pos [X Y Z] : " << ugv_pos[0] << " [ m ] "<< ugv_pos[1]<<" [ m ] "<<ugv_pos[2]<<" [ m ] "<< TAIL <<endl; 
    cout << GREEN << "UAV_vel [X Y Z] : " << ugv_vel[0] << " [m/s] "<< ugv_vel[1]<<" [m/s] "<<ugv_vel[2]<<" [m/s] "<< TAIL <<endl; 
    cout << GREEN << "Yaw             : " << ugv_yaw * 180/M_PI<<" [deg] "<< TAIL <<endl; 
    cout << GREEN << "Battery         : " << battery.data<<" [V] "<< TAIL <<endl; 

    switch(ugv_control_cmd.cmd)
    {
        case sunray_msgs::UGVControlCMD::Hold:
            cout << GREEN << "Command: [ Hold ] " << TAIL <<endl; 
            break;

        case sunray_msgs::UGVControlCMD::Direct_Control_BODY:
            cout << GREEN << "Command: [ Direct_Control_BODY ] " << TAIL <<endl; 
            cout << GREEN << "linear_vel [X Y] : " << ugv_control_cmd.linear_vel[0] << " [m/s] "<< ugv_control_cmd.linear_vel[1] << " [m/s] " << TAIL <<endl; 
            cout << GREEN << "angular_vel [YAW]: " << ugv_control_cmd.angular_vel*180.0/M_PI << " [deg/s] "<< TAIL <<endl; 
            break;

        case sunray_msgs::UGVControlCMD::Direct_Control_ENU:
            cout << GREEN << "Command: [ Direct_Control_ENU ] " << TAIL <<endl; 
            cout << GREEN << "linear_vel [X Y] : " << ugv_control_cmd.linear_vel[0] << " [m/s] "<< ugv_control_cmd.linear_vel[1] << " [m/s] " << TAIL <<endl; 
            cout << GREEN << "angular_vel [YAW]: " << ugv_control_cmd.angular_vel*180.0/M_PI << " [deg/s] "<< TAIL <<endl; 
            break;

        case sunray_msgs::UGVControlCMD::Point_Control:
            cout << GREEN << "Command: [ Point_Control ] " << TAIL <<endl; 
            cout << GREEN << "Pos_ref [X Y] : " << ugv_control_cmd.desired_pos[0]  << " [ m ] "<< ugv_control_cmd.desired_pos[1] <<" [ m ] "<< TAIL <<endl; 
            cout << GREEN << "Yaw_ref [YAW] : " << ugv_control_cmd.desired_yaw*180.0/M_PI << " [deg] "<< TAIL <<endl; 
            break;

        case sunray_msgs::UGVControlCMD::Point_Control_with_Astar:
            cout << GREEN << "Command: [ Point_Control_with_Astar ] " << TAIL <<endl; 
            cout << GREEN << "Goal [X Y] : " << ugv_control_cmd.desired_pos[0]  << " [ m ] "<< ugv_control_cmd.desired_pos[1] <<" [ m ] "<< TAIL <<endl; 
            cout << GREEN << "Goal [YAW] : " << ugv_control_cmd.desired_yaw*180.0/M_PI << " [deg] "<< TAIL <<endl; 
            break;

        case sunray_msgs::UGVControlCMD::Test:
            cout << GREEN << "Command: [ Test ] " << TAIL <<endl; 
            break;
    }

    cout << GREEN << "cmd_vel to wheeltec [X Y] : " << cmd_vel.linear.x << " [m/s] "<< cmd_vel.linear.y << " [m/s] " << TAIL <<endl; 
    cout << GREEN << "cmd_vel to wheeltec [YAW] : " << cmd_vel.angular.z*180.0/M_PI << " [deg/s] "<< TAIL <<endl; 
}

void WheeltecRobot::timercb_rviz(const ros::TimerEvent &e)
{
    if(!mainloop_start_flag)
    {
        return;
    }
    // 发布无人车运动轨迹，用于rviz显示
    geometry_msgs::PoseStamped ugv_pos_now;
    ugv_pos_now.header.stamp = ros::Time::now();
    ugv_pos_now.header.frame_id = "world";
    ugv_pos_now.pose.position.x = ugv_pos[0];
    ugv_pos_now.pose.position.y = ugv_pos[1];
    ugv_pos_now.pose.position.z = ugv_pos[2];

    ugv_pos_now.pose.orientation = ugv_q_msg;

    //发布无人车的位姿 和 轨迹 用作rviz中显示
    posehistory_vector_.insert(posehistory_vector_.begin(), ugv_pos_now);
    if (posehistory_vector_.size() > TRA_WINDOW)
    {
        posehistory_vector_.pop_back();
    }

    nav_msgs::Path ugv_trajectory;
    ugv_trajectory.header.stamp = ros::Time::now();
    ugv_trajectory.header.frame_id = "world";
    ugv_trajectory.poses = posehistory_vector_;
    trajectory_pub.publish(ugv_trajectory);

    // 发布mesh
    visualization_msgs::Marker meshROS;
    meshROS.header.frame_id = "world";
    meshROS.header.stamp = ros::Time::now();
    meshROS.ns = "ugv_mesh";
    meshROS.id = 0;
    meshROS.type = visualization_msgs::Marker::MESH_RESOURCE;
    meshROS.action = visualization_msgs::Marker::ADD;
    meshROS.pose.position.x = ugv_pos[0];
    meshROS.pose.position.y = ugv_pos[1];
    meshROS.pose.position.z = ugv_pos[2];
    meshROS.pose.orientation = ugv_q_msg;
    meshROS.scale.x = 0.6/4.5;
    meshROS.scale.y = 0.6/4.5;
    meshROS.scale.z = 0.6/4.5;
    meshROS.color.a = 1.0;
    meshROS.color.r = 0.0;
    meshROS.color.g = 0.0;
    meshROS.color.b = 1.0;
    meshROS.mesh_resource = mesh_resource;
    ugv_mesh_pub.publish(meshROS); 

    // 发布TF用于RVIZ显示（激光雷达与无人车的tf）
    // static tf2_ros::TransformBroadcaster broadcaster;
    // geometry_msgs::TransformStamped tfs;
    // //  |----头设置
    // tfs.header.frame_id = "world";  //相对于世界坐标系
    // tfs.header.stamp = ros::Time::now();  //时间戳
    // //  |----坐标系 ID
    // tfs.child_frame_id = topic_name + "/lidar_link";  //子坐标系，无人车的坐标系
    // //  |----坐标系相对信息设置  偏移量  无人车相对于世界坐标系的坐标
    // tfs.transform.translation.x = ugv_pos[0];
    // tfs.transform.translation.y = ugv_pos[1];
    // tfs.transform.translation.z = ugv_pos[2];
    // //  |--------- 四元数设置  
    // tfs.transform.rotation = ugv_q_msg;
    // //  发布数据
    // broadcaster.sendTransform(tfs);
}

void WheeltecRobot::viobot_state_cb(const sunray_msgs::ViobotState::ConstPtr &msg)
{
    viobot_state = *msg;

    ugv_pos[0] = viobot_state.position[0];
    ugv_pos[1] = viobot_state.position[1];
    ugv_pos[2] = viobot_state.position[2];
    ugv_vel[0] = viobot_state.velocity[0];
    ugv_vel[1] = viobot_state.velocity[1];
    ugv_vel[2] = viobot_state.velocity[2];
    ugv_yaw    = viobot_state.attitude[2];
    ugv_q_msg  = viobot_state.attitude_q;
}


void WheeltecRobot::battery_cb(const std_msgs::Float32::ConstPtr &msg)
{
    battery = *msg;
}

void WheeltecRobot::ugv_control_cmd_cb(const sunray_msgs::UGVControlCMD::ConstPtr& msg)
{
    ugv_control_cmd = *msg;
    // only_rotate = true; // vinson: should be set for initializing.
    // first_arrive = true;
    // stop_flag = false;

    // ugv_control_cmd.desired_yaw 取值范围： (-180,180]
    if(ugv_control_cmd.desired_yaw > M_PI + 0.001f)
    {
        ugv_control_cmd.desired_yaw = 0.0;
        cout << RED << "Wrong ugv_control_cmd.desired_yaw : " <<  ugv_control_cmd.desired_yaw << " , Reset to 0." << TAIL <<endl; 
    }else if(ugv_control_cmd.desired_yaw < -M_PI - 0.001f)
    {
        ugv_control_cmd.desired_yaw = 0.0;
        cout << RED << "Wrong ugv_control_cmd.desired_yaw : " <<  ugv_control_cmd.desired_yaw << " , Reset to 0." << TAIL <<endl; 
    }else if(ugv_control_cmd.desired_yaw == -M_PI)
    {
        ugv_control_cmd.desired_yaw = M_PI;
    }

    // if(ugv_control_cmd.cmd == sunray_msgs::UGVControlCMD::Point_Control_with_Astar)
    // {
    //     goal_pos[0] = ugv_control_cmd.desired_pos[0];
    //     goal_pos[1] = ugv_control_cmd.desired_pos[1];
    // }
}

int WheeltecRobot::safety_check()
{
    if (ugv_pos[0] < ugv_geo_fence.x_min || ugv_pos[0] > ugv_geo_fence.x_max ||
        ugv_pos[1] < ugv_geo_fence.y_min || ugv_pos[1] > ugv_geo_fence.y_max )
    {
        cout << RED << ugv_name << ":----> Out of the geo fence, stop!" << TAIL << endl;
        return 1;
    }

    return 0;
}



#endif
