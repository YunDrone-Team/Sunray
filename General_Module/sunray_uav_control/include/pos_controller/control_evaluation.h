#ifndef __CONTROLL_EVALUATION_H__
#define __CONTROLL_EVALUATION_H__

#include "ros_msg_utils.h"
#include "printf_format.h"
#include <numeric>
#include <sys/time.h>

using namespace sunray_logger;
using namespace std;

class ControlEvaluation
{
private:
    // 无人机控制信息结构体
    struct ControlINFO
    {
        double time;
        Eigen::Vector3d uav_pos;       // 无人机位置
        Eigen::Vector3d uav_vel;       // 无人机位置
        Eigen::Vector3d uav_att;       // 无人机位置
        Eigen::Vector3d uav_pos_sp;       // 无人机位置
        Eigen::Vector3d uav_vel_sp;       // 无人机位置
        Eigen::Vector3d uav_att_sp;       // 无人机位置
        Eigen::Vector3d pos_error;       // 无人机位置
        Eigen::Vector3d vel_error;       // 无人机位置
    };
    ControlINFO ctrl_info;

    // 无人机控制信息结构体 - 容器
    std::vector<ControlINFO> ctrl_info_vector;

    // 无人机控制状态机
    enum EVA_MODE
    {
        Hover_EVA = 0,           // 悬停精度评估
        Traj_EVA = 1,            // 轨迹追踪精度评估
    };

    int Slide_window;

    // 控制误差信息 - 注意这里面是绝对值
    struct ErrorINFO
    {
        std::vector<double> pos_x;
        std::vector<double> pos_y;
        std::vector<double> pos_z;
        std::vector<double> pos_norm;
        std::vector<double> vel_x;
        std::vector<double> vel_y;
        std::vector<double> vel_z;
        std::vector<double> vel_norm;
    };
    ErrorINFO error_info;

    // 无人机系统参数
    struct ErrorRESULT
    {
        double time;
        Eigen::Vector3d pos_error_mean;       // 无人机位置
        Eigen::Vector3d pos_error_max;       // 无人机位置
        Eigen::Vector3d vel_error_mean;       // 无人机位置
        Eigen::Vector3d vel_error_max;       // 无人机位置
        Eigen::Vector3d att_error_mean;       // 无人机位置
        Eigen::Vector3d att_error_max;       // 无人机位置
    };
    ErrorRESULT error_result;

public:
    ControlEvaluation(){};

    void add_control_info(Eigen::Vector3d pos_sp, Eigen::Vector3d pos_now, Eigen::Vector3d vel_sp, Eigen::Vector3d vel_now, int eva_mode)
    {
        // 获取系统时间
        struct timeval tv;
        gettimeofday(&tv, NULL);
        // 将时间转为 double（秒 + 微秒/1e6）
        ctrl_info.time = (double)tv.tv_sec + (double)tv.tv_usec / 1000000.0;

        // 记录无人机控制数据
        ctrl_info.uav_pos = pos_now;
        ctrl_info.uav_pos_sp = pos_sp;
        ctrl_info.uav_vel = vel_now;
        ctrl_info.uav_vel_sp = vel_sp;
        ctrl_info.pos_error = ctrl_info.uav_pos_sp - ctrl_info.uav_pos;
        ctrl_info.vel_error = ctrl_info.uav_vel_sp - ctrl_info.uav_vel;

        error_info.pos_x.push_back( abs(ctrl_info.pos_error[0]) );
        error_info.pos_y.push_back( abs(ctrl_info.pos_error[1]) );
        error_info.pos_z.push_back( abs(ctrl_info.pos_error[2]) );



        
        // if (pos_error_vector.size() > Slide_window)
        // {
        //     pos_error_vector.pop_back();
        // }
        // if (vel_error_vector.size() > Slide_window)
        // {
        //     vel_error_vector.pop_back();
        // }

        calculate_error();
    }

    void calculate_error()
    {
        // error_info.time = ctrl_info.time;
        // error_info.pos_error_mean[0] = ctrl_info.time;
        // error_info.pos_error_max = ctrl_info.time;
        // error_info.time = ctrl_info.time;

        // 计算平均值
        error_result.pos_error_mean[0] = std::accumulate(error_info.pos_x.begin(), error_info.pos_x.end(), 0.0) / error_info.pos_x.size();
        error_result.pos_error_mean[1] = std::accumulate(error_info.pos_y.begin(), error_info.pos_y.end(), 0.0) / error_info.pos_y.size();
        error_result.pos_error_mean[2] = std::accumulate(error_info.pos_z.begin(), error_info.pos_z.end(), 0.0) / error_info.pos_z.size();

        // 计算最大值
        // TODO

    }

    void show()
    {

        Logger::print_color(int(LogColor::blue), LOG_BOLD, "-------------------- Control Evaluation --------------------");
        Logger::print_color(int(LogColor::green), "error_result.pos_error_mean[ [X Y Z]:",
                        error_result.pos_error_mean[0],
                        error_result.pos_error_mean[1],
                        error_result.pos_error_mean[2],
                        "[ m ]");
    }


    ~ControlEvaluation()
    {
    }
};

#endif