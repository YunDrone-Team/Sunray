
#include "ekf_fileter.hpp"

EkfFilter::EkfFilter() {

    // 配置协方差矩阵
    Q.block<3, 3>(0, 0).diagonal() = cov_gyr;        
    Q.block<3, 3>(3, 3).diagonal() = cov_acc;
    Q.block<3, 3>(6, 6).diagonal() = cov_bias_gyr;
    Q.block<3, 3>(9, 9).diagonal() = cov_bias_acc;

	init_P(6,6) = init_P(7,7) = init_P(8,8) = 0.00001;
    init_P(9,9) = init_P(10,10) = init_P(11,11) = 0.00001;
    init_P(15,15) = init_P(16,16) = init_P(17,17) = 0.0001;
    init_P(18,18) = init_P(19,19) = init_P(20,20) = 0.001;
    init_P(21,21) = init_P(22,22) = init_P(23,23) = 0.00001; 
}


//对应公式(2) 中的f
Eigen::Matrix<double, 24, 1> EkfFilter::Get_F(EkfState x,  ImuData &imu_data)	{

	Eigen::Matrix<double, 24, 1> res = Eigen::Matrix<double, 24, 1>::Zero();
	
	Eigen::Vector3d omega = imu_data.curr_gyr - x.bg;
	Eigen::Vector3d a_inertial = x.rot.matrix() * (imu_data.curr_acc - x.ba);		

	for (int i = 0; i < 3; i++)
	{
		res(i) = x.vel[i];		//速度（对应公式第2行）
		res(i + 3) = omega[i];	//角速度（对应公式第1行）
		res(i + 12) = a_inertial[i] + x.grav[i];		//加速度（对应公式第3行）
	}

	return res;
}

//对应公式(7)的Fx  注意该矩阵没乘dt，没加单位阵
Eigen::Matrix<double, 24, 24> EkfFilter::Df_Dx(EkfState x, input_ikfom in) {

	Eigen::Matrix<double, 24, 24> cov = Eigen::Matrix<double, 24, 24>::Zero();
	cov.block<3, 3>(0, 12) = Eigen::Matrix3d::Identity();	//对应公式(7)第2行第3列   I
	Eigen::Vector3d acc_ = in.acc - s.ba;   	//测量加速度 = a_m - bias	

	cov.block<3, 3>(12, 3) = -s.rot.matrix() * Sophus::SO3::hat(acc_);		//对应公式(7)第3行第1列
	cov.block<3, 3>(12, 18) = -s.rot.matrix(); 				//对应公式(7)第3行第5列 

	cov.template block<3, 3>(12, 21) = Eigen::Matrix3d::Identity();		//对应公式(7)第3行第6列   I
	cov.template block<3, 3>(3, 15) = -Eigen::Matrix3d::Identity();		//对应公式(7)第1行第4列 (简化为-I)
	return cov;
}

//对应公式(7)的Fw  注意该矩阵没乘dt
Eigen::Matrix<double, 24, 12> EkfFilter::Df_Dw(EkfState x, input_ikfom in) {

	Eigen::Matrix<double, 24, 12> cov = Eigen::Matrix<double, 24, 12>::Zero();
	cov.block<3, 3>(12, 3) = -x.rot.matrix();					//对应公式(7)第3行第2列  -R 
	cov.block<3, 3>(3, 0) = -Eigen::Matrix3d::Identity();		//对应公式(7)第1行第1列  -A(w dt)简化为-I
	cov.block<3, 3>(15, 6) = Eigen::Matrix3d::Identity();		//对应公式(7)第4行第3列  I
	cov.block<3, 3>(18, 9) = Eigen::Matrix3d::Identity();		//对应公式(7)第5行第4列  I
	return cov;
}

//广义加法  
EkfState EkfFilter::BoxPlus(EkfState x, Eigen::Matrix<double, 24, 1> f_) {

	EkfState x_r;
	x_r.pos = x.pos + f_.block<3, 1>(0, 0);
	x_r.rot = x.rot * Sophus::SO3::exp(f_.block<3, 1>(3, 0));
	x_r.vel = x.vel + f_.block<3, 1>(12, 0);
	x_r.bg = x.bg + f_.block<3, 1>(15, 0);
	x_r.ba = x.ba + f_.block<3, 1>(18, 0);
	x_r.grav = x.grav + f_.block<3, 1>(21, 0);

	return x_r;
}

//前向传播
void EkfFilter::Predict(const ImuData &imu_data) {

	Eigen::Matrix<double, 24, 1> f_ = Get_F(x_, imu_data);	 
	Eigen::Matrix<double, 24, 24> f_x_ = Df_Dx(x_, imu_data); 
	Eigen::Matrix<double, 24, 12> f_w_ = Df_Dw(x_, imu_data); 

	x_ = BoxPlus(x_, f_ * dt);

	f_x_ = Eigen::Matrix<double, 24, 24>::Identity() + f_x_ * dt;

	P_ = (f_x_)*P_ * (f_x_).transpose() + (dt * f_w_) * Q * (dt * f_w_).transpose(); 
}

void EkfFilter::Update(Eigen::Matrix4d &reg_pose) {

	double N,;

	cov P_updated = P_;

	// 从原本的协方差矩阵截取：位置，姿态，速度，b_g，b_a，g的协方差
	Eigen::Matrix<double, 18, 18> P_needed = Eigen::Matrix<double, 18, 18>::Zero();
	P_needed.block<6, 6>(0,0) = P_updated.block<6, 6>(0, 0);  // 位置，姿态协方差
	P_needed.block<6, 12>(0,6) = P_updated.block<6, 12>(0, 12); // 速度，b_g，b_a，g的协方差
	P_needed.block<12 ,6>(6,0) = P_updated.block<12, 6>(12, 0);
	P_needed.block<12, 12>(6,6) = P_updated.block<12, 12>(12, 12);

	Sophus::SO3 reg_so3 = Sophus::SO3(reg_pose.block<3,3>(0,0));

	// H矩阵 6*18
	Eigen::Matrix<double, 6, 18> H = Eigen::Matrix<double, 6, 18>::Zero();
	H.block<3,3>(0,0) = Eigen::Matrix<double, 3, 3>::Identity(); // P部分
	H.block<3,3>(3,3) = Eigen::Matrix<double, 3, 3>::Identity(); // R部分

	// 观测噪声
	Eigen::Matrix<double, 6, 6> V = Eigen::Matrix<double, 6, 6>::Identity() * N;

	// 卡尔曼增益
	Eigen::Matrix<double, 18, 6> K = P_needed * H.transpose() * (H * P_needed * H.transpose() + V).inverse();

	// 误差状态
	Eigen::Matrix<double, 6, 1> innov = Eigen::Matrix<double, 6, 1>::Zero();
	innov.head<3>() << reg_pose(0,3) - x_.pos(0) , reg_pose(1,3) - x_.pos(1) , reg_pose(2,3) - x_.pos(2);
	innov.tail<3>() = (x_.rot.inverse() * reg_so3).log();

	// 更新误差
	Eigen::Matrix<double, 18, 1> updated_err = K * innov;

	// 更新协方差
	Eigen::Matrix<double, 18, 18> updated_P = Eigen::Matrix<double, 18, 18>::Identity();
	updated_P = (updated_P - K * H) * P_needed;

	// 更新状态量
	x_.pos += updated_err.block<3, 1>(0, 0);    // 更新旋转
	x_.rot  = x_.rot * Sophus::SO3::exp(updated_err.block<3, 1>(3, 0)); // 更新旋转
	x_.vel += updated_err.block<3, 1>(6, 0);    // 更新速度
	x_.bg  += updated_err.block<3, 1>(9, 0);    // 更新bg
	x_.ba  += updated_err.block<3, 1>(12, 0);   // 更新ba
	x_.grav += updated_err.block<3, 1>(15, 0);  // 更新gra

	// 更新协方差
	P_updated.block<6, 6>(0, 0) = updated_P.block<6, 6>(0, 0); // 位置，姿态协方差
	P_updated.block<6, 12>(0, 12) = updated_P.block<6, 12>(0, 6); // 速度，b_g，b_a，g的协方差
	P_updated.block<12, 6>(12, 0) = updated_P.block<12, 6>(6, 0);
	P_updated.block<12, 12>(12, 12) = updated_P.block<12, 12>(6, 6);

	P_ = P_updated;
}



