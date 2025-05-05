#ifndef TRAJ_GENERATOR_H
#define TRAJ_GENERATOR_H

#include <Eigen/Eigen>
#include <math.h>
#include <math_utils.h>
#include "ros_msg_utils.h"
#include "printf_utils.h"

using namespace std;

class TRAJ_GENERATOR
{
    public:
        //构造函数
        TRAJ_GENERATOR(void):
            nh("~")
        {
            nh.param<float>("TRAJ_GENERATOR/Circle/Center_x", circle_center[0], 0.0);
            nh.param<float>("TRAJ_GENERATOR/Circle/Center_y", circle_center[1], 0.0);
            nh.param<float>("TRAJ_GENERATOR/Circle/Center_z", circle_center[2], 1.0);
            nh.param<float>("TRAJ_GENERATOR/Circle/circle_radius", circle_radius, 2.0);
            nh.param<float>("TRAJ_GENERATOR/Circle/linear_vel", linear_vel, 0.5);
            nh.param<float>("TRAJ_GENERATOR/Circle/direction", direction, 1.0);

            nh.param<float>("TRAJ_GENERATOR/Eight/Center_x", eight_origin_[0], 0.0);
            nh.param<float>("TRAJ_GENERATOR/Eight/Center_y", eight_origin_[1], 0.0);
            nh.param<float>("TRAJ_GENERATOR/Eight/Center_z", eight_origin_[2], 1.0);
            nh.param<float>("TRAJ_GENERATOR/Eight/omega", eight_omega_, 0.5);
            nh.param<float>("TRAJ_GENERATOR/Eight/radial", radial, 2.0);

            nh.param<float>("TRAJ_GENERATOR/Step/step_length", step_length, 0.0);
            nh.param<float>("TRAJ_GENERATOR/Step/step_interval", step_interval, 0.0);
        }

        //Printf the TRAJ_GENERATOR parameter
        void printf_param();

        //TRAJ_GENERATOR Calculation [Input: time_from_start; Output: Trajectory;]
        sunray_msgs::UAVControlCMD Circle_trajectory_generation(float time_from_start);

        sunray_msgs::UAVControlCMD Eight_trajectory_generation(float time_from_start);

        sunray_msgs::UAVControlCMD Step_trajectory_generation(float time_from_start);

        sunray_msgs::UAVControlCMD Line_trajectory_generation(float time_from_start);

    private:

        ros::NodeHandle nh;

        // Circle Parameter
        Eigen::Vector3f circle_center;
        float circle_radius;
        float linear_vel;
        float direction;         //direction = 1 for CCW 逆时针, direction = -1 for CW 顺时针

        // Eight Shape Parameter
        Eigen::Vector3f eight_origin_;
        float radial;
        float eight_omega_;

        // Step
        float step_length;
        float step_interval;
        
};


sunray_msgs::UAVControlCMD TRAJ_GENERATOR::Circle_trajectory_generation(float time_from_start)
{
    sunray_msgs::UAVControlCMD Circle_trajectory;
    float omega;
    if( circle_radius != 0)
    {
        omega = direction * fabs(linear_vel / circle_radius);
    }else
    {
        omega = 0.0;
    }
    const float angle = time_from_start * omega;
    const float cos_angle = cos(angle);
    const float sin_angle = sin(angle);

    // cout << GREEN <<  "omega : " << omega  * 180/M_PI <<" [deg/s] " << TAIL <<endl;
    // cout << GREEN <<  "angle : " << angle  * 180/M_PI <<" [deg] " << TAIL <<endl;

    Circle_trajectory.desired_pos[0] = circle_radius * cos_angle + circle_center[0];
    Circle_trajectory.desired_pos[1] = circle_radius * sin_angle + circle_center[1];
    Circle_trajectory.desired_pos[2] = circle_center[2];

    Circle_trajectory.desired_vel[0] = - circle_radius * omega * sin_angle;
    Circle_trajectory.desired_vel[1] = circle_radius * omega * cos_angle;
    Circle_trajectory.desired_vel[2] = 0;

    Circle_trajectory.desired_acc[0] = - circle_radius * pow(omega, 2.0) * cos_angle;
    Circle_trajectory.desired_acc[1] = - circle_radius * pow(omega, 2.0) * sin_angle;
    Circle_trajectory.desired_acc[2] = 0;

    // Circle_trajectory.jerk_ref[0] = circle_radius * pow(omega, 3.0) * sin_angle;
    // Circle_trajectory.jerk_ref[1] = - circle_radius * pow(omega, 3.0) * cos_angle;
    // Circle_trajectory.jerk_ref[2] = 0;

    // Circle_trajectory.snap_ref[0] = circle_radius * pow(omega, 4.0) * cos_angle;
    // Circle_trajectory.snap_ref[1] = circle_radius * pow(omega, 4.0) * sin_angle;
    // Circle_trajectory.snap_ref[2] = 0;

    Circle_trajectory.desired_yaw = 0;
    // Circle_trajectory.yaw_rate_ref = 0;
    // Circle_trajectory.yaw_acceleration_ref = 0;

    return Circle_trajectory;
}

sunray_msgs::UAVControlCMD TRAJ_GENERATOR::Line_trajectory_generation(float time_from_start)
{
    sunray_msgs::UAVControlCMD Line_trajectory;
    float omega;
    if( circle_radius != 0)
    {
        omega = direction * fabs(linear_vel / circle_radius);
    }else
    {
        omega = 0.0;
    }
    const float angle = time_from_start * omega;
    const float cos_angle = cos(angle);
    const float sin_angle = sin(angle);

    Line_trajectory.desired_pos[0] = 0.0;
    Line_trajectory.desired_pos[1] = circle_radius * sin_angle + circle_center[1];
    Line_trajectory.desired_pos[2] = circle_center[2];

    Line_trajectory.desired_vel[0] = 0.0;
    Line_trajectory.desired_vel[1] = circle_radius * omega * cos_angle;
    Line_trajectory.desired_vel[2] = 0;

    Line_trajectory.desired_acc[0] = 0.0;
    Line_trajectory.desired_acc[1] = - circle_radius * pow(omega, 2.0) * sin_angle;
    Line_trajectory.desired_acc[2] = 0;

    // Line_trajectory.jerk_ref[0] = circle_radius * pow(omega, 3.0) * sin_angle;
    // Line_trajectory.jerk_ref[1] = - circle_radius * pow(omega, 3.0) * cos_angle;
    // Line_trajectory.jerk_ref[2] = 0;

    // Line_trajectory.snap_ref[0] = circle_radius * pow(omega, 4.0) * cos_angle;
    // Line_trajectory.snap_ref[1] = circle_radius * pow(omega, 4.0) * sin_angle;
    // Line_trajectory.snap_ref[2] = 0;

    Line_trajectory.desired_yaw = 0;
    // Line_trajectory.yaw_rate_ref = 0;
    // Line_trajectory.yaw_acceleration_ref = 0;

    return Line_trajectory;
}


sunray_msgs::UAVControlCMD TRAJ_GENERATOR::Eight_trajectory_generation(float time_from_start)
{
    Eigen::Vector3f position;
    Eigen::Vector3f velocity;
    Eigen::Vector3f acceleration;
    
    float angle = eight_omega_* time_from_start;
    const float cos_angle = cos(angle);
    const float sin_angle = sin(angle);
    
    Eigen::Vector3f eight_radial_ ;
    Eigen::Vector3f eight_axis_ ;
    eight_radial_ << radial, 0.0, 0.0;
    eight_axis_ << 0.0, 0.0, 2.0;

    position = cos_angle * eight_radial_ + sin_angle * cos_angle * eight_axis_.cross(eight_radial_)
                 + (1 - cos_angle) * eight_axis_.dot(eight_radial_) * eight_axis_ + eight_origin_;

    velocity = eight_omega_ * (-sin_angle * eight_radial_ + (pow(cos_angle, 2) - pow(sin_angle, 2)) * eight_axis_.cross(eight_radial_)
                 + (sin_angle) * eight_axis_.dot(eight_radial_) * eight_axis_);

    acceleration << 0.0, 0.0, 0.0;

    sunray_msgs::UAVControlCMD Eight_trajectory;

    Eight_trajectory.desired_pos[0] = position[0];
    Eight_trajectory.desired_pos[1] = position[1];
    Eight_trajectory.desired_pos[2] = position[2];

    Eight_trajectory.desired_vel[0] = velocity[0];
    Eight_trajectory.desired_vel[1] = velocity[1];
    Eight_trajectory.desired_vel[2] = velocity[2];

    Eight_trajectory.desired_acc[0] = 0;
    Eight_trajectory.desired_acc[1] = 0;
    Eight_trajectory.desired_acc[2] = 0;

    Eight_trajectory.desired_yaw = 0;

    // to be continued...

    return Eight_trajectory;
}


sunray_msgs::UAVControlCMD TRAJ_GENERATOR::Step_trajectory_generation(float time_from_start)
{
    sunray_msgs::UAVControlCMD Step_trajectory;

    int i = time_from_start / step_interval;

    if( i%2 == 0)
    {
        Step_trajectory.desired_pos[0] = step_length;
    }else 
    {
        Step_trajectory.desired_pos[0] = - step_length;
    }

    Step_trajectory.desired_pos[1] = 0;
    Step_trajectory.desired_pos[2] = 1.0;

    Step_trajectory.desired_vel[0] = 0;
    Step_trajectory.desired_vel[1] = 0;
    Step_trajectory.desired_vel[2] = 0;

    Step_trajectory.desired_acc[0] = 0;
    Step_trajectory.desired_acc[1] = 0;
    Step_trajectory.desired_acc[2] = 0;

    Step_trajectory.desired_yaw = 0;

    return Step_trajectory;
}

// 【打印参数函数】
void TRAJ_GENERATOR::printf_param()
{
    cout << GREEN << ">>>>>>>>>>>>>>>>>>>>>>>>>TRAJ_GENERATOR Parameter <<<<<<<<<<<<<<<<<<<<<<" << TAIL <<endl;
    cout << GREEN << "Circle Shape  :  " << TAIL <<endl;
    cout << GREEN << "origin        :  " << circle_center[0] <<" [m] "<< circle_center[1] <<" [m] "<< circle_center[2] <<" [m] "<< TAIL <<endl;
    cout << GREEN << "radius        :  " << circle_radius <<" [m] " << endl;
    cout << GREEN << "linear_vel    :  " << linear_vel <<" [m/s] " << endl;
    cout << GREEN << "direction     : "<< direction << endl;

    cout << GREEN << "Eight Shape   :  " << TAIL <<endl;
    cout << GREEN << "origin        :  " << eight_origin_[0] <<" [m] "<< eight_origin_[1] <<" [m] "<< eight_origin_[2] <<" [m] "<< TAIL <<endl;
    cout << GREEN << "radial        :  " << radial  <<" [m] "<< endl;
    cout << GREEN << "angular_vel   :  " << eight_omega_  <<" [rad/s] " << endl;

    cout << GREEN << "Step          :  " << TAIL <<endl;
    cout << GREEN << "step_length   :  " << step_length << TAIL <<endl;
    cout << GREEN << "step_interval :  " << step_interval << " [s] "<< TAIL <<endl;
}


#endif
