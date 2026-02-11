#include "imu_data.hpp"

void ImuProcess::ProcssIMU(const ImuData &input_imu_data, ImuData &output_imu_data) {

    static bool first_frame = true;

    Eigen::Vector3d cur_gyr = input_imu_data.cur_gyr;
    Eigen::Vector3d cur_acc = input_imu_data.cur_acc;
    double cur_timeStamp = input_imu_data.timeStamp.toSec();
  
    if (first_frame) {

        last_gyr_ = cur_gyr;
        last_acc_ = cur_acc;
        last_timeStamp_ = cur_timeStamp;
        return;
    }

    // 中值积分
    output_imu_data.dt = cur_timeStamp - last_timeStamp_;
    output_imu_data.timeStamp = cur_timeStamp;
    output_imu_data.cur_gyr = 0.5 * (cur_gyr + last_gyr_);  
    output_imu_data.cur_acc = 0.5 * (cur_acc + last_acc_);
    output_imu_data.cur_acc  = output_imu_data.cur_acc * G_m_s2 / mean_acc_.norm(); //通过重力数值对加速度进行调整
    output_imu_data.gravity = gravity_;
    
    last_gyr_ = cur_gyr;
    last_acc_ = cur_acc;
    last_timeStamp_ = cur_timeStamp;
}

bool ImuProcess::ImuInit(const ImuData &input_imu_data) {

    static int iter_num = 0;
    
    Eigen::Vector3d cur_gyr = input_imu_data.cur_gyr;
    Eigen::Vector3d cur_acc = input_imu_data.cur_acc;
    double cur_timeStamp = input_imu_data.timeStamp.toSec();

    ++iter_num;

    mean_acc_ += (cur_acc - mean_acc_) / iter_num;
    mean_gyr_ += (cur_gyr - mean_gyr_) / iter_num;

    //累计100帧数据后，完成初始化
    if (iter_num > 99) {         

        gravity_ = - mean_acc_ / mean_acc_.norm() * G_m_s2;

        return true;

    } else {

        return false; 
    }
}

