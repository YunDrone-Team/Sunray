#ifndef EKF_FILTER_HPP
#define EKF_FILTER_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sophus/so3.h>
#include "imu_process.hpp"

//状态量
struct EkfState {

    double timestamp = 0.0;
	Eigen::Vector3d pos(0, 0, 0);
    Sophus::SO3 rot = Sophus::SO3(Eigen::Matrix3d::Identity());
	Eigen::Vector3d vel(0, 0, 0);
	Eigen::Vector3d ba(0, 0, 0);
	Eigen::Vector3d bg(0, 0, 0);
    Eigen::Vector3d grav(0, 0, 0);
};


class EkfFilter {

public:

	EkfFilter();

	~EkfFilter() {};

    EkfState EkfFilter::GetEkfState() { return x_; }

    void EkfFilter::Update(Eigen::Matrix4d& reg_pose);

private:
	
    EkfState x_;
	

    Eigen::Matrix<double, 24, 24> init_P = Eigen::Matrix<double, 24, 24>::Identity(24, 24); 


    Eigen::Vector3d acc_cov_(0.1, 0.1, 0.1);             //加速度协方差
    Eigen::Vector3d gyr_cov_(0.1, 0.1, 0.1);             //角速度协方差
    Eigen::Vector3d bias_acc_cov_(0.0001, 0.0001, 0.0001);        //加速度bias的协方差
    Eigen::Vector3d bias_gyr_cov_(0.0001, 0.0001, 0.0001);        //角速度bias的协方差

};


#endif