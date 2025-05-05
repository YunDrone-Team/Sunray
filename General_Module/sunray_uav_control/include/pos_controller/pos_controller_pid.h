#ifndef __CONTROLLER_H
#define __CONTROLLER_H

#include "ros_msg_utils.h"
#include "printf_format.h"
#include <queue>

class PosControlPID
{
public:
    PosControlPID(){};

    void init(ros::NodeHandle& nh);

    // 设定位置控制期望信息
    void set_desired_state(const Desired_State& des)
    {
        desired_state = des;
    }

    // 设定位置控制当前信息
    void set_current_state(const sunray_msgs::UAVState& state)
    {
        uav_state = state;

        for(int i=0; i<3; i++)
        {
            current_state.pos(i) = uav_state.position[i];
            current_state.vel(i) = uav_state.velocity[i];
        }

        current_state.q.w() = uav_state.attitude_q.w;
        current_state.q.x() = uav_state.attitude_q.x;
        current_state.q.y() = uav_state.attitude_q.y;
        current_state.q.z() = uav_state.attitude_q.z; 

        current_state.yaw = geometry_utils::get_yaw_from_quaternion(current_state.q);
    }

    void ctrl_update(const Desired_State_t &des,
        const Odom_Data_t &odom,
        const Eigen::Quaterniond &imu_q, 
        Controller_Output_t &u);


    // 打印参数
    void printf_param();
    // 打印控制器调试信息
    void printf_debug();


    

    quadrotor_msgs::Px4ctrlDebug calculateControl(const Desired_State_t &des,
        const Odom_Data_t &odom,
        const Eigen::Quaterniond &imu_q, 
        Controller_Output_t &u);


    bool estimateThrustModel(const Eigen::Vector3d &est_v,
        const Parameter_t &param);
    void resetThrustMapping(void);

    Desired_State_t des;
    Odom_Data_t odom;
    Eigen::Quaterniond imu_q;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  private:
    Parameter_t param_;
    quadrotor_msgs::Px4ctrlDebug debug_msg_;
    std::queue<std::pair<ros::Time, double>> timed_thrust_;
    static constexpr double kMinNormalizedCollectiveThrust_ = 3.0;

    // Thrust-accel mapping params
    const double rho2_ = 0.998; // do not change
    double thr2acc_;
    double P_;

    double computeDesiredCollectiveThrustSignal(const Eigen::Vector3d &des_acc);
    double fromQuaternion2yaw(Eigen::Quaterniond q);
};

// 输入：
// 无人机位置、速度、偏航角
// 期望位置、速度、加速度、偏航角
// 输出：
// 期望姿态 + 期望油门
Eigen::Vector4d pos_controller_PID::update(float controller_hz)
{
    // 定点控制的时候才积分，即追踪轨迹或者速度追踪时不进行积分
	if (desired_state.vel(0) != 0.0 || desired_state.vel(1) != 0.0 || desired_state.vel(2) != 0.0) 
    {
        PCOUT(2, YELLOW, "Reset integration.");
		int_e_v.setZero();
	}

    // 位置误差
    Eigen::Vector3d pos_error = desired_state.pos - current_state.pos;
    Eigen::Vector3d vel_error = desired_state.vel - current_state.vel;
    
    tracking_error.input_error(pos_error,vel_error);

    // 限制最大误差
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
        if(abs(pos_error[i]) < int_start_error && uav_state.mode == "OFFBOARD")
        {
            int_e_v[i] += pos_error[i] / controller_hz;
            if(abs(int_e_v[i]) > ctrl_param.int_max[i])
            {
                PCOUT(2, YELLOW, "int_e_v saturation [ xy ]");
                int_e_v[i] = (int_e_v[i] > 0) ? ctrl_param.int_max[i] : -ctrl_param.int_max[i];
            }
        }else
        {
            int_e_v[i] = 0;
        }
    }

    // 积分项 - Z
    float int_start_error = 0.5;
    if(abs(pos_error[2]) < int_start_error && uav_state.mode == "OFFBOARD")
    {
        int_e_v[2] += pos_error[2] / controller_hz;

        if(abs(int_e_v[2]) > ctrl_param.int_max[2])
        {
            PCOUT(2, YELLOW, "int_e_v saturation [ z ]");
            int_e_v[2] = (int_e_v[2] > 0) ? ctrl_param.int_max[2] : -ctrl_param.int_max[2];
        }
    }else
    {
        int_e_v[2] = 0;
    }

    // 期望加速度 = 期望加速度 + Kp * 位置误差 + Kv * 速度误差 + Kv * 积分项
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
	if (std::fabs(F_des(0)/F_des(2)) > std::tan(geometry_utils::toRad(ctrl_param.tilt_angle_max)))
	{
        PCOUT(2, YELLOW, "pitch too tilt");
		F_des(0) = F_des(0)/std::fabs(F_des(0)) * F_des(2) * std::tan(geometry_utils::toRad(ctrl_param.tilt_angle_max));
	}

	// 角度限制幅度
	if (std::fabs(F_des(1)/F_des(2)) > std::tan(geometry_utils::toRad(ctrl_param.tilt_angle_max)))
	{
        PCOUT(2, YELLOW, "roll too tilt");
		F_des(1) = F_des(1)/std::fabs(F_des(1)) * F_des(2) * std::tan(geometry_utils::toRad(ctrl_param.tilt_angle_max));	
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
        PCOUT(2, YELLOW, "throttle too low");
    }

    if(u_att(3) > 1.0)
    {
        u_att(3) = 1.0;
        PCOUT(2, YELLOW, "throttle too high");
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
    nh.param<float>("pid_gain/quad_mass" , ctrl_param.quad_mass, 1.0f);
    // 【参数】悬停油门
    nh.param<float>("pid_gain/hov_percent" , ctrl_param.hov_percent, 0.5f);
    // 【参数】XYZ积分上限
    nh.param<float>("pid_gain/pxy_int_max"  , ctrl_param.int_max[0], 0.5);
    nh.param<float>("pid_gain/pxy_int_max"  , ctrl_param.int_max[1], 0.5);
    nh.param<float>("pid_gain/pz_int_max"   , ctrl_param.int_max[2], 0.5);
    // 【参数】控制参数
    nh.param<double>("pid_gain/Kp_xy", ctrl_param.Kp(0,0), 2.0f);
    nh.param<double>("pid_gain/Kp_xy", ctrl_param.Kp(1,1), 2.0f);
    nh.param<double>("pid_gain/Kp_z" , ctrl_param.Kp(2,2), 2.0f);
    nh.param<double>("pid_gain/Kv_xy", ctrl_param.Kv(0,0), 2.0f);
    nh.param<double>("pid_gain/Kv_xy", ctrl_param.Kv(1,1), 2.0f);
    nh.param<double>("pid_gain/Kv_z" , ctrl_param.Kv(2,2), 2.0f);
    nh.param<double>("pid_gain/Kvi_xy", ctrl_param.Kvi(0,0), 0.3f);
    nh.param<double>("pid_gain/Kvi_xy", ctrl_param.Kvi(1,1), 0.3f);
    nh.param<double>("pid_gain/Kvi_z" , ctrl_param.Kvi(2,2), 0.3f);
    nh.param<float>("pid_gain/tilt_angle_max" , ctrl_param.tilt_angle_max, 10.0f);
    ctrl_param.g << 0.0, 0.0, 9.8;

    printf_param();
}





double PosControl::fromQuaternion2yaw(Eigen::Quaterniond q)
{
  double yaw = atan2(2 * (q.x()*q.y() + q.w()*q.z()), q.w()*q.w() + q.x()*q.x() - q.y()*q.y() - q.z()*q.z());
  return yaw;
}

PosControl::PosControl(Parameter_t &param) : param_(param)
{
  resetThrustMapping();
}

/* 
  compute u.thrust and u.q, controller gains and other parameters are in param_ 
*/
quadrotor_msgs::Px4ctrlDebug
PosControl::calculateControl(const Desired_State_t &des,
    const Odom_Data_t &odom,
    const Imu_Data_t &imu, 
    Controller_Output_t &u)
{
  /* WRITE YOUR CODE HERE */
      //compute disired acceleration
      Eigen::Vector3d des_acc(0.0, 0.0, 0.0);
      Eigen::Vector3d Kp,Kv;
      Kp << param_.gain.Kp0, param_.gain.Kp1, param_.gain.Kp2;
      Kv << param_.gain.Kv0, param_.gain.Kv1, param_.gain.Kv2;
      des_acc = des.a + Kv.asDiagonal() * (des.v - odom.v) + Kp.asDiagonal() * (des.p - odom.p);
      des_acc += Eigen::Vector3d(0,0,param_.gra);

      u.thrust = computeDesiredCollectiveThrustSignal(des_acc);
      double roll,pitch,yaw,yaw_imu;
      double yaw_odom = fromQuaternion2yaw(odom.q);
      double sin = std::sin(yaw_odom);
      double cos = std::cos(yaw_odom);
      roll = (des_acc(0) * sin - des_acc(1) * cos )/ param_.gra;
      pitch = (des_acc(0) * cos + des_acc(1) * sin )/ param_.gra;
      // yaw = fromQuaternion2yaw(des.q);
      yaw_imu = fromQuaternion2yaw(imu.q);
      // Eigen::Quaterniond q = Eigen::AngleAxisd(yaw,Eigen::Vector3d::UnitZ())
      //   * Eigen::AngleAxisd(roll,Eigen::Vector3d::UnitX())
      //   * Eigen::AngleAxisd(pitch,Eigen::Vector3d::UnitY());
      Eigen::Quaterniond q = Eigen::AngleAxisd(des.yaw,Eigen::Vector3d::UnitZ())
        * Eigen::AngleAxisd(pitch,Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(roll,Eigen::Vector3d::UnitX());
      u.q = imu.q * odom.q.inverse() * q;


  /* WRITE YOUR CODE HERE */

  //used for debug
  // debug_msg_.des_p_x = des.p(0);
  // debug_msg_.des_p_y = des.p(1);
  // debug_msg_.des_p_z = des.p(2);
  
  debug_msg_.des_v_x = des.v(0);
  debug_msg_.des_v_y = des.v(1);
  debug_msg_.des_v_z = des.v(2);
  
  debug_msg_.des_a_x = des_acc(0);
  debug_msg_.des_a_y = des_acc(1);
  debug_msg_.des_a_z = des_acc(2);
  
  debug_msg_.des_q_x = u.q.x();
  debug_msg_.des_q_y = u.q.y();
  debug_msg_.des_q_z = u.q.z();
  debug_msg_.des_q_w = u.q.w();
  
  debug_msg_.des_thr = u.thrust;
  
  // Used for thrust-accel mapping estimation
  timed_thrust_.push(std::pair<ros::Time, double>(ros::Time::now(), u.thrust));
  while (timed_thrust_.size() > 100)
  {
    timed_thrust_.pop();
  }
  return debug_msg_;
}

/*
  compute throttle percentage 
*/
double 
PosControl::computeDesiredCollectiveThrustSignal(
    const Eigen::Vector3d &des_acc)
{
  double throttle_percentage(0.0);
  
  /* compute throttle, thr2acc has been estimated before */
  throttle_percentage = des_acc(2) / thr2acc_;

  return throttle_percentage;
}

bool 
PosControl::estimateThrustModel(
    const Eigen::Vector3d &est_a,
    const Parameter_t &param)
{
  ros::Time t_now = ros::Time::now();
  while (timed_thrust_.size() >= 1)
  {
    // Choose data before 35~45ms ago
    std::pair<ros::Time, double> t_t = timed_thrust_.front();
    double time_passed = (t_now - t_t.first).toSec();
    if (time_passed > 0.045) // 45ms
    {
      // printf("continue, time_passed=%f\n", time_passed);
      timed_thrust_.pop();
      continue;
    }
    if (time_passed < 0.035) // 35ms
    {
      // printf("skip, time_passed=%f\n", time_passed);
      return false;
    }

    /***********************************************************/
    /* Recursive least squares algorithm with vanishing memory */
    /***********************************************************/
    double thr = t_t.second;
    timed_thrust_.pop();
    
    /***********************************/
    /* Model: est_a(2) = thr1acc_ * thr */
    /***********************************/
    double gamma = 1 / (rho2_ + thr * P_ * thr);
    double K = gamma * P_ * thr;
    thr2acc_ = thr2acc_ + K * (est_a(2) - thr * thr2acc_);
    P_ = (1 - K * thr) * P_ / rho2_;
    //printf("%6.3f,%6.3f,%6.3f,%6.3f\n", thr2acc_, gamma, K, P_);
    //fflush(stdout);

    // debug_msg_.thr2acc = thr2acc_;
    return true;
  }
  return false;
}

void 
PosControl::resetThrustMapping(void)
{
  thr2acc_ = param_.gra / param_.thr_map.hover_percentage;
  P_ = 1e6;
}










#endif
