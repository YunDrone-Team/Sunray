#ifndef TRAJ_GENERATOR_H
#define TRAJ_GENERATOR_H

#include <Eigen/Eigen>
#include <math.h>
#include <math_utils.h>
#include "ros_msg_utils.h"
#include "printf_utils.h"

using namespace std;

#define TRA_WINDOW 2000

class TRAJ_GENERATOR
{
    public:
        TRAJ_GENERATOR() {};
        ~TRAJ_GENERATOR() {};

        void init(ros::NodeHandle &nh);
        void printf_param();
        sunray_msgs::UAVControlCMD Circle_trajectory_generation(float time_from_start);
        void pub_ref_traj();

    private:
        int uav_id;
        string uav_name;
        sunray_msgs::UAVControlCMD uav_cmd;
        std::vector<geometry_msgs::PoseStamped> ref_traj_vector;

        // Circle Parameter
        struct Circle_Params
        {
            Eigen::Vector3f circle_center;  // 圆心坐标
            float circle_radius;            // 半径
            float linear_vel;               // 线速度
            float direction;                // direction = 1 for CCW 逆时针, direction = -1 for CW 顺时针
            bool fixed_yaw;                 // 是否固定yaw角：true代表yaw角恒定为0，false代表yaw角跟随轨迹移动
        };
        Circle_Params circle_params;     

        ros::Publisher ref_trajectory_pub;
};

void TRAJ_GENERATOR::init(ros::NodeHandle &nh)
{
    nh.param<int>("uav_id", uav_id, 1);                 // 【参数】无人机编号
    nh.param<std::string>("uav_name", uav_name, "uav"); // 【参数】无人机名称
    // 圆形轨迹参数
    // 圆心坐标
    nh.param<float>("TRAJ_GENERATOR/Circle/Center_x", circle_params.circle_center[0], 0.0);
    nh.param<float>("TRAJ_GENERATOR/Circle/Center_y", circle_params.circle_center[1], 0.0);
    nh.param<float>("TRAJ_GENERATOR/Circle/Center_z", circle_params.circle_center[2], 1.0);
    // 圆半径
    nh.param<float>("TRAJ_GENERATOR/Circle/circle_radius", circle_params.circle_radius, 2.0);
    // 线速度
    nh.param<float>("TRAJ_GENERATOR/Circle/linear_vel", circle_params.linear_vel, 0.5);
    // 移动方向：1 for CCW 逆时针, -1 for CW 顺时针
    nh.param<float>("TRAJ_GENERATOR/Circle/direction", circle_params.direction, 1.0);
    // 是否固定偏航角
    nh.param<bool>("TRAJ_GENERATOR/Circle/fixed_yaw", circle_params.fixed_yaw, true);

    printf_param();

    uav_name = "/" + uav_name + std::to_string(uav_id);
    //【发布】参考轨迹（RVIZ显示）
    ref_trajectory_pub = nh.advertise<nav_msgs::Path>(uav_name + "/sunray/reference_trajectory", 10);
}

void TRAJ_GENERATOR::pub_ref_traj()
{
    geometry_msgs::PoseStamped reference_pose;

    reference_pose.header.stamp = ros::Time::now();
    reference_pose.header.frame_id = "world";
    reference_pose.pose.position.x = uav_cmd.desired_pos[0];
    reference_pose.pose.position.y = uav_cmd.desired_pos[1];
    reference_pose.pose.position.z = uav_cmd.desired_pos[2];

    ref_traj_vector.insert(ref_traj_vector.begin(), reference_pose);
    if (ref_traj_vector.size() > TRA_WINDOW)
    {
        ref_traj_vector.pop_back();
    }

    nav_msgs::Path reference_trajectory;
    reference_trajectory.header.stamp = ros::Time::now();
    reference_trajectory.header.frame_id = "world";
    reference_trajectory.poses = ref_traj_vector;
    ref_trajectory_pub.publish(reference_trajectory);
}

sunray_msgs::UAVControlCMD TRAJ_GENERATOR::Circle_trajectory_generation(float time_from_start)
{
    uav_cmd.header.stamp = ros::Time::now();
    uav_cmd.cmd = sunray_msgs::UAVControlCMD::CTRL_Traj;

    // 根据线速度和半径计算角速度
    float omega;
    if( circle_params.circle_radius != 0)
    {
        omega = circle_params.direction * fabs(circle_params.linear_vel / circle_params.circle_radius);
    }else
    {
        omega = 0.0;
    }

    // 根据设定的圆参数计算每一时刻的期望位置
    const float angle = time_from_start * omega;
    const float cos_angle = cos(angle);
    const float sin_angle = sin(angle);

    uav_cmd.desired_pos[0] = circle_params.circle_radius * cos_angle + circle_params.circle_center[0];
    uav_cmd.desired_pos[1] = circle_params.circle_radius * sin_angle + circle_params.circle_center[1];
    uav_cmd.desired_pos[2] = circle_params.circle_center[2];

    uav_cmd.desired_vel[0] = - circle_params.circle_radius * omega * sin_angle;
    uav_cmd.desired_vel[1] = circle_params.circle_radius * omega * cos_angle;
    uav_cmd.desired_vel[2] = 0;

    uav_cmd.desired_acc[0] = - circle_params.circle_radius * pow(omega, 2.0) * cos_angle;
    uav_cmd.desired_acc[1] = - circle_params.circle_radius * pow(omega, 2.0) * sin_angle;
    uav_cmd.desired_acc[2] = 0;

    if(circle_params.fixed_yaw)
    {
        uav_cmd.desired_yaw = 0;
    }else
    {
        uav_cmd.desired_yaw = atan2(uav_cmd.desired_vel[1], uav_cmd.desired_vel[0]);
    }
    
    pub_ref_traj();

    return uav_cmd;
}

// 【打印参数函数】
void TRAJ_GENERATOR::printf_param()
{
    cout << GREEN << ">>>>>>>>>>>>>>>>>>>>>>>>> TRAJ_GENERATOR Parameter <<<<<<<<<<<<<<<<<<<<<<" << TAIL <<endl;
    cout << GREEN << "Circle Params :  " << TAIL <<endl;
    cout << GREEN << "origin        :  " << circle_params.circle_center[0] <<" [m] "<< circle_params.circle_center[1] <<" [m] "<< circle_params.circle_center[2] <<" [m] "<< TAIL <<endl;
    cout << GREEN << "radius        :  " << circle_params.circle_radius <<" [m] " << endl;
    cout << GREEN << "linear_vel    :  " << circle_params.linear_vel <<" [m/s] " << endl;
    if(circle_params.direction > 0)
    {
        cout << GREEN << "direction     :  " << "anticlockwise" << endl;
    }else
    {
        cout << GREEN << "fixed_yaw     :  " << "clockwise" << endl;
    }

    if(circle_params.fixed_yaw)
    {
        cout << GREEN << "fixed_yaw     :  " << "true" << endl;
    }else
    {
        cout << GREEN << "fixed_yaw     :  " << "false" << endl;
    }
}
#endif
