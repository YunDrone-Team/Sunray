#ifndef __CONTROLLER_H
#define __CONTROLLER_H

#include "ros_msg_utils.h"
#include "printf_format.h"
#include "controller_utils.h"
#include "geometry_utils.h"
#include "math_utils.h"
#include "printf_utils.h"

using namespace sunray_logger;
class PosControlPID
{
public:
    PosControlPID(){};

    void init(ros::NodeHandle& nh);

    // 设定位置控制期望信息
    void set_desired_state(const Desired_State_t& des)
    {
        desired_state = des;
    }

    // 设定位置控制当前信息
    void set_current_state(const sunray_msgs::PX4State& state)
    {
        px4_state = state;

        for(int i=0; i<3; i++)
        {
            current_state.pos(i) = px4_state.position[i];
            current_state.vel(i) = px4_state.velocity[i];
        }

        current_state.q.w() = px4_state.attitude_q.w;
        current_state.q.x() = px4_state.attitude_q.x;
        current_state.q.y() = px4_state.attitude_q.y;
        current_state.q.z() = px4_state.attitude_q.z; 

        current_state.yaw = geometry_utils::get_yaw_from_quaternion(current_state.q);
    }

    // pid控制器更新
    Eigen::Vector4d ctrl_update(float controller_hz);
    // 打印参数
    void printf_param();
    // 打印控制器调试信息
    void printf_debug();

  private:
    Ctrl_Param_PID ctrl_param;
    Desired_State_t desired_state;
    Current_State_t current_state;
    sunray_msgs::PX4State px4_state; 
    Eigen::Vector3d F_des;

    Tracking_Error_Evaluation tracking_error;

    Eigen::Vector3d int_e_v;            // 积分
    Eigen::Quaterniond u_q_des;         // 期望姿态角（四元数）
    Eigen::Vector4d u_att;              // 期望姿态角（rad）+期望油门（0-1）
};

// 输入：
// 无人机位置、速度、偏航角
// 期望位置、速度、加速度、偏航角
// 输出：
// 期望姿态 + 期望油门
Eigen::Vector4d PosControlPID::ctrl_update(float controller_hz)
{
    // 位置误差
    Eigen::Vector3d pos_error = desired_state.pos - current_state.pos;
    Eigen::Vector3d vel_error = desired_state.vel - current_state.vel;
    
    tracking_error.input_error(pos_error, vel_error);

    // 限制位置和速度的最大误差
    float max_pos_error = 3.0;
    float max_vel_error = 3.0;

    for (int i=0; i<3; i++)
    {
        if(abs(pos_error[i]) > max_pos_error)
        {            
            pos_error[i] = (pos_error[i] > 0) ? 1.0 : -1.0;
        }
        if(abs(vel_error[i]) > max_vel_error)
        {            
            vel_error[i] = (vel_error[i] > 0) ? 2.0 : -2.0;
        }
    }

    // 积分项 - XY
    for (int i=0; i<2; i++)
    {
        // 只有在pos_error比较小时，才会启动积分
        float int_start_error = 0.2;
        if(abs(pos_error[i]) < int_start_error && px4_state.mode == "OFFBOARD")
        {
            int_e_v[i] += pos_error[i] / controller_hz;
            if(abs(int_e_v[i]) > ctrl_param.int_max[i])
            {
                PCOUT(2, YELLOW, "PosControlPID: int_e_v saturation [ xy ]");
                int_e_v[i] = (int_e_v[i] > 0) ? ctrl_param.int_max[i] : -ctrl_param.int_max[i];
            }
        }else
        {
            int_e_v[i] = 0;
        }
    }

    // 积分项 - Z
    float int_start_error = 1.0;
    if(abs(pos_error[2]) < int_start_error && px4_state.mode == "OFFBOARD")
    {
        int_e_v[2] += pos_error[2] / controller_hz;

        if(abs(int_e_v[2]) > ctrl_param.int_max[2])
        {
            PCOUT(2, YELLOW, "PosControlPID: int_e_v saturation [ z ]");
            int_e_v[2] = (int_e_v[2] > 0) ? ctrl_param.int_max[2] : -ctrl_param.int_max[2];
        }
    }else
    {
        int_e_v[2] = 0;
    }

    // 定点控制的时候才积分，即追踪轨迹或者速度追踪时不进行积分
	if (desired_state.vel(0) != 0.0 || desired_state.vel(1) != 0.0 || desired_state.vel(2) != 0.0) 
    {
        // PCOUT(2, YELLOW, "PosControlPID: Reset XY integration.");
		int_e_v[0] = 0;
        int_e_v[1] = 0;
	}

    // 期望加速度（控制输出） = 期望加速度(前馈) + Kp * 位置误差 + Kv * 速度误差 + Kv * 积分项
    Eigen::Vector3d des_acc = desired_state.acc + ctrl_param.Kp * pos_error + ctrl_param.Kv * vel_error + ctrl_param.Kvi* int_e_v;

	// 期望力 = 质量*控制量 + 重力抵消
    // F_des是基于模型的位置控制器计算得到的三轴期望推力（惯性系），量纲为牛
    // u_att是用于PX4的姿态控制输入，u_att 前三位是roll pitch yaw， 第四位为油门值[0-1]
    F_des = des_acc * ctrl_param.quad_mass + ctrl_param.quad_mass * ctrl_param.g;

	// 如果向上推力小于重力的一半
	// 或者向上推力大于重力的两倍
	if (F_des(2) < 0.5 * ctrl_param.quad_mass * ctrl_param.g(2))
	{
		F_des = F_des / F_des(2) * (0.5 * ctrl_param.quad_mass * ctrl_param.g(2));
	}
	else if (F_des(2) > 2 * ctrl_param.quad_mass * ctrl_param.g(2))
	{
		F_des = F_des / F_des(2) * (2 * ctrl_param.quad_mass * ctrl_param.g(2));
	}

	// 角度限制幅度
	if (std::fabs(F_des(0)/F_des(2)) > std::tan(ctrl_param.tilt_angle_max))
	{
        PCOUT(2, YELLOW, "PosControlPID: pitch too large");
		F_des(0) = F_des(0)/std::fabs(F_des(0)) * F_des(2) * std::tan(ctrl_param.tilt_angle_max);
	}

	// 角度限制幅度
	if (std::fabs(F_des(1)/F_des(2)) > std::tan(ctrl_param.tilt_angle_max))
	{
        PCOUT(2, YELLOW, "PosControlPID: roll too large");
		F_des(1) = F_des(1)/std::fabs(F_des(1)) * F_des(2) * std::tan(ctrl_param.tilt_angle_max);	
	}

    // F_des是位于ENU坐标系的,F_c是FLU
    Eigen::Matrix3d wRc = geometry_utils::rotz(current_state.yaw);
    Eigen::Vector3d F_c = wRc.transpose() * F_des;
    double fx = F_c(0);
    double fy = F_c(1);
    double fz = F_c(2);

    // 期望roll, pitch
    u_att(0)  = std::atan2(-fy, fz);
    u_att(1)  = std::atan2( fx, fz);
    u_att(2)  = desired_state.yaw;

    // 无人机姿态的矩阵形式
    Eigen::Matrix3d wRb_odom = current_state.q.toRotationMatrix();
    // 第三列
    Eigen::Vector3d z_b_curr = wRb_odom.col(2);
    // 机体系下的电机推力 相当于Rb * F_enu 惯性系到机体系
    double u1 = F_des.dot(z_b_curr);
    // 悬停油门与电机参数有关系,也取决于质量
    double full_thrust = ctrl_param.quad_mass * ctrl_param.g(2) / ctrl_param.hov_percent;

    // 油门 = 期望推力/最大推力
    // 这里相当于认为油门是线性的,满足某种比例关系,即认为某个重量 = 悬停油门
    u_att(3) = u1 / full_thrust;

    if(u_att(3) < 0.1)
    {
        u_att(3) = 0.1;
        PCOUT(2, YELLOW, "PosControlPID: throttle too low");
    }

    if(u_att(3) > 1.0)
    {
        u_att(3) = 1.0;
        PCOUT(2, YELLOW, "PosControlPID: throttle too high");
    }

    return u_att;
}

void PosControlPID::init(ros::NodeHandle& nh)
{
    // 【参数】控制参数
    ctrl_param.Kp.setZero();
    ctrl_param.Kv.setZero();
    ctrl_param.Kvi.setZero();
    ctrl_param.Ka.setZero();
    // 【参数】无人机质量
    nh.param<double>("ctrl_param/quad_mass" , ctrl_param.quad_mass, 1.0f);
    // 【参数】悬停油门
    nh.param<double>("ctrl_param/hov_percent" , ctrl_param.hov_percent, 0.5f);
    // 【参数】位置环控制器XYZ积分上限
    nh.param<double>("ctrl_param/pxy_int_max"  , ctrl_param.int_max[0], 10.0f);
    nh.param<double>("ctrl_param/pxy_int_max"  , ctrl_param.int_max[1], 10.0f);
    nh.param<double>("ctrl_param/pz_int_max"   , ctrl_param.int_max[2], 10.0f);
    // 【参数】位置环控制器控制参数
    nh.param<double>("ctrl_param/Kp_xy", ctrl_param.Kp(0,0), 2.0f);
    nh.param<double>("ctrl_param/Kp_xy", ctrl_param.Kp(1,1), 2.0f);
    nh.param<double>("ctrl_param/Kp_z" , ctrl_param.Kp(2,2), 2.0f);
    nh.param<double>("ctrl_param/Kv_xy", ctrl_param.Kv(0,0), 2.0f);
    nh.param<double>("ctrl_param/Kv_xy", ctrl_param.Kv(1,1), 2.0f);
    nh.param<double>("ctrl_param/Kv_z" , ctrl_param.Kv(2,2), 2.0f);
    nh.param<double>("ctrl_param/Kvi_xy", ctrl_param.Kvi(0,0), 0.3f);
    nh.param<double>("ctrl_param/Kvi_xy", ctrl_param.Kvi(1,1), 0.3f);
    nh.param<double>("ctrl_param/Kvi_z" , ctrl_param.Kvi(2,2), 0.3f);
    // 【参数】位置环控制器最大倾斜角
    nh.param<double>("ctrl_param/tilt_angle_max" , ctrl_param.tilt_angle_max, 15.0f);
    ctrl_param.tilt_angle_max = ctrl_param.tilt_angle_max/180.0f*M_PI;
    // 【参数】重力加速度
    ctrl_param.g << 0.0, 0.0, 9.8;

    printf_param();
}

void PosControlPID::printf_debug()
{

    Logger::print_color(int(LogColor::green), "pos_des [X Y Z]:",
                        desired_state.pos[0],
                        desired_state.pos[1],
                        desired_state.pos[2],
                        "[ m ]");
    Logger::print_color(int(LogColor::green), "vel_des [X Y Z]:",
                        desired_state.vel[0],
                        desired_state.vel[1],
                        desired_state.vel[2],
                        "[m/s]");   
    Logger::print_color(int(LogColor::green), "acc_des [X Y Z]:",
                        desired_state.acc[0],
                        desired_state.acc[1],
                        desired_state.acc[2],
                        "[m/s^2]"); 
    Logger::print_color(int(LogColor::green), "yaw_des [X Y Z]:",
                        desired_state.yaw / M_PI * 180,
                        "[deg]");  

    Logger::print_color(int(LogColor::green), "int_e_v [X Y Z]:",
                        int_e_v[0],
                        int_e_v[1],
                        int_e_v[2],
                        "[ N ]"); 

    Logger::print_color(int(LogColor::green), "F_des [X Y Z]:",
                        F_des[0],
                        F_des[1],
                        F_des[2],
                        "[ N ]"); 

    Logger::print_color(int(LogColor::green), "controller_output [R P Y]:",
                        u_att[0] / M_PI * 180,
                        u_att[1] / M_PI * 180,
                        u_att[2] / M_PI * 180,
                        "[deg]"); 


    Logger::print_color(int(LogColor::green), "controller_output [thrust]:",
                        u_att[3] * 100,
                        "[ % ]"); 

    Logger::print_color(int(LogColor::green), "tracking_error.pos_error_mean:",
                        tracking_error.pos_error_mean,
                        "[ m ]"); 
    Logger::print_color(int(LogColor::green), "tracking_error.vel_error_mean:",
                        tracking_error.vel_error_mean,
                        "[m/s]"); 

    // cout << BLUE << "----> pos_now         : " << current_state.pos(0) << " [ m ] " << current_state.pos(1) << " [ m ] " << current_state.pos(2) << " [ m ] "<< TAIL << endl;
    // cout << BLUE << "----> vel_now         : " << current_state.vel(0) << " [ m ] " << current_state.vel(1) << " [ m ] " << current_state.vel(2) << " [ m ] "<< TAIL << endl;
    
    // cout << BLUE << "----> int_e_v         : " << int_e_v(0) << " [N] "<< int_e_v(1) << " [N] "<< int_e_v(2) << " [N] "<< TAIL << endl;
    
    // cout << BLUE << "----> F_des           : " << F_des(0) << " [N] "<< F_des(1) << " [N] "<< F_des(2) << " [N] "<< TAIL << endl;
    
    // cout << BLUE << "----> pos_error_mean  : " << tracking_error.pos_error_mean <<" [m] "<< TAIL <<endl;
    // cout << BLUE << "----> vel_error_mean  : " << tracking_error.vel_error_mean <<" [m/s] "<< TAIL <<endl;
}

// 【打印参数函数】
void PosControlPID::printf_param()
{
    cout << GREEN << ">>>>>>>>>>>>>>>>>>>>>>>>>> PID Parameter <<<<<<<<<<<<<<<<<<<<<<<<<" << TAIL <<endl;
    cout << GREEN <<  "ctrl_param.quad_mass     : "<< ctrl_param.quad_mass<< TAIL <<endl;
    cout << GREEN <<  "ctrl_param.hov_percent   : "<< ctrl_param.hov_percent<< TAIL <<endl;
    cout << GREEN <<  "pxy_int_max              : "<< ctrl_param.int_max[0]<< TAIL <<endl;
    cout << GREEN <<  "pz_int_max               : "<< ctrl_param.int_max[2]<< TAIL <<endl;

    cout << GREEN <<  "Kp_xy         : "<< ctrl_param.Kp(0,0) << TAIL <<endl;
    cout << GREEN <<  "Kp_z          : "<< ctrl_param.Kp(2,2) << TAIL <<endl;
    cout << GREEN <<  "Kv_xy         : "<< ctrl_param.Kv(0,0) << TAIL <<endl;
    cout << GREEN <<  "Kv_z          : "<< ctrl_param.Kv(2,2) << TAIL <<endl;
    cout << GREEN <<  "Kvi_xy        : "<< ctrl_param.Kvi(0,0) << TAIL <<endl;
    cout << GREEN <<  "Kvi_z         : "<< ctrl_param.Kvi(2,2) << TAIL <<endl;
    cout << GREEN <<  "Ka_xy         : "<< ctrl_param.Ka(0,0) << TAIL <<endl;
    cout << GREEN <<  "Ka_z          : "<< ctrl_param.Ka(2,2) << TAIL <<endl;
    cout << GREEN <<  "tilt_angle_max: "<< ctrl_param.tilt_angle_max * 180.0 / M_PI << TAIL <<endl;
}
#endif
