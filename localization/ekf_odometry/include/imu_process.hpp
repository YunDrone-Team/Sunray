#ifndef IMU_PROCESS_HPP
#define IMU_PROCESS_HPP

#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sensor_msgs/Imu.h>

struct ImuData {

    double dt = 0.0;
    double timeStamp = 0.0;
    Eigen::Vector3d cur_gyr(0, 0, 0);
    Eigen::Vector3d cur_acc(0, 0, 0);
    Eigen::Vector3d gravity(0, 0, 0);
};

class ImuProcess{

public:

    ImuProcess() { };

    ~ImuProcess() { };

    Eigen::Vector3d GetInitGyrBias() { return mean_gyr_; };
 
    void ProcssIMU(const ImuData &input_imu_data, ImuData &output_imu_data);

    bool ImuInit(const ImuData &input_imu_data);

 private:

    const double G_m_s2 = 9.81;                    //重力加速度常数 
    Eigen::Vector3d gravity_(0, 0, 0);             //重力
    Eigen::Vector3d mean_acc_(0, 0,0);         //加速度均值
    Eigen::Vector3d mean_gyr_(0, 0, 0);            //角速度均值

    double last_timeStamp_ = 0.0;
    Eigen::Vector3d last_acc_(0, 0, 0);
    Eigen::Vector3d last_gyr_(0, 0, 0);
};

#ednif // IMU_PROCESS_HPP