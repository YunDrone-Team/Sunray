#ifndef UAV_PID_H
#define UAV_PID_H

#include <Eigen/Eigen>
#include <iostream>
#include "printf_utils.h"
/*
无人机PID控制
*/

struct uav_pid
{
    float Kp;
    float Ki;
    float Kd;

    float Integrator; // 积分
    float IntegratorUp;// 积分上限
    float out; // PID输出值
    float outUp; // 输出上限

};

class velocity_ctrl
{
public:
    uav_pid velocity_x; // x轴速度pid
    uav_pid velocity_y; // y轴速度pid
    uav_pid velocity_z; // z轴速度pid

    velocity_ctrl()
    {
        // 初始化
        velocity_x.Kp = 1.2;
        velocity_x.Ki = 0.01;
        velocity_x.Kd = 0.08;
        velocity_x.Integrator = 0;
        velocity_x.IntegratorUp = 100;
        velocity_x.out = 0;
        velocity_x.outUp = 3;

        velocity_y.Kp = 1.0;
        velocity_y.Ki = 0.08;
        velocity_y.Kd = 0.1; 
        velocity_y.Integrator = 0;
        velocity_y.IntegratorUp = 100;
        velocity_y.out = 0;
        velocity_y.outUp = 3;

        velocity_z.Kp = 0.5;
        velocity_z.Ki = 0.01;
        velocity_z.Kd = 0.1;
        velocity_z.Integrator = 0;
        velocity_z.IntegratorUp = 50;
        velocity_z.out = 0;
        velocity_z.outUp = 1;

    }

    Eigen::Vector3d calculate_velocity(Eigen::Vector3d current_pose, Eigen::Vector3d target_pose, double dt, Eigen::Vector3d current_vel)
    {
        // x轴
        Eigen::Vector3d pose_error = target_pose - current_pose; // 位置误差
        // Eigen::Vector3d vel_error = 
        velocity_x.out = velocity_x.Kp * pose_error[0] + velocity_x.Kd * (current_vel[0] - velocity_x.out);//+ velocity_x.Ki * velocity_x.Integrator;
        velocity_x.Integrator += pose_error[0] * dt;

        if(fabs(velocity_x.Integrator) > velocity_x.IntegratorUp)
        {
            if(velocity_x.Integrator >= 0)
            {
                velocity_x.Integrator = velocity_x.IntegratorUp;
            }
            else
                velocity_x.Integrator = -velocity_x.IntegratorUp;
            // std::cout << RED << "velocity_x.Integrator is saturated：" << velocity_x.Integrator << std::endl;
        }
            
        
        if(fabs(velocity_x.out) > velocity_x.outUp)
        {
            if(velocity_x.out >= 0)
            {
                velocity_x.out = velocity_x.outUp;
            }
            else
                velocity_x.out = -velocity_x.outUp;
            // std::cout << RED << "velocity_x.out is saturated：" << velocity_x.out << std::endl;
        }
             
        
        // y轴
        velocity_y.out = velocity_y.Kp * pose_error[1]  + velocity_y.Kd * (current_vel[1] - velocity_y.out);//+ velocity_y.Ki * velocity_y.Integrator;
        velocity_y.Integrator += pose_error[1] * dt;
        if(fabs(velocity_y.Integrator) > velocity_y.IntegratorUp)
        {
            if(velocity_y.Integrator >= 0)
                velocity_y.Integrator = velocity_y.IntegratorUp;
            else
                velocity_y.Integrator = -velocity_y.IntegratorUp;
            // std::cout << RED << "velocity_y.Integrator is saturated：" << velocity_y.Integrator << std::endl;
        }

        if(fabs(velocity_y.out) > velocity_y.outUp)
        {
            if(velocity_y.out >= 0)
                velocity_y.out = velocity_y.outUp;
            else
                velocity_y.out = -velocity_y.outUp;
            // std::cout << RED << "velocity_y.out is saturated：" << velocity_y.out << std::endl;
        }


        // z轴
        velocity_z.out = velocity_z.Kp * pose_error[2]  + velocity_z.Kd * (current_vel[2] - velocity_z.out); //+ velocity_z.Ki * velocity_z.Integrator;
        velocity_z.Integrator += pose_error[2] * dt;
        if(velocity_z.Integrator > velocity_z.IntegratorUp)
        {
            if(velocity_z.Integrator >= 0)
                velocity_z.Integrator = velocity_z.IntegratorUp;
            else
                velocity_z.Integrator = -velocity_z.IntegratorUp;
            // std::cout << RED << "velocity_z.Integrator is saturated：" << velocity_z.Integrator << std::endl;
        }
        
        if(fabs(velocity_z.out) > velocity_z.outUp)
        {
            if(velocity_z.out >= 0)
                velocity_z.out = velocity_z.outUp;
            else
                velocity_z.out = -velocity_z.outUp;
            std::cout << RED << "velocity_z.out is saturated：" << velocity_z.out << std::endl;

        }
        
        Eigen::Vector3d velocity;
        velocity[0] = velocity_x.out;
        velocity[1] = velocity_y.out;
        velocity[2] = velocity_z.out;

        return velocity;
    }
};

#endif