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

    // 无人机控制状态机
    enum EVA_MODE
    {
        Hover_EVA = 0,           // 悬停精度评估
        Traj_EVA = 1,            // 轨迹追踪精度评估
    };

    int Slide_window;
    bool first_enter{true};
    double init_time{0.0};

    // 无人机控制信息结构体（每一帧的信息，变量是容器）
    struct ControlINFO
    {
        std::vector<double> time;
        std::vector<Eigen::Vector3d> uav_pos;       // 无人机当前位置
        std::vector<Eigen::Vector3d> uav_vel;       // 无人机当前速度
        std::vector<Eigen::Vector3d> uav_att;       // 无人机当前姿态
        std::vector<Eigen::Vector3d> uav_pos_sp;    // 无人机期望位置
        std::vector<Eigen::Vector3d> uav_vel_sp;    // 无人机期望速度
        std::vector<Eigen::Vector3d> uav_att_sp;    // 无人机期望姿态
    };
    ControlINFO ctrl_info;

    // 控制误差信息（每一帧的信息，变量是容器） - 注意这里面是绝对值
    struct ErrorINFO
    {
        std::vector<double> time;
        std::vector<double> pos_x;      // X轴位置追踪误差
        std::vector<double> pos_y;      // Y轴位置追踪误差
        std::vector<double> pos_z;      // Z轴位置追踪误差
        std::vector<double> pos_norm;   // 三维位置追踪误差
        std::vector<double> vel_x;      // X轴速度追踪误差
        std::vector<double> vel_y;      // Y轴速度追踪误差
        std::vector<double> vel_z;      // Z轴速度追踪误差
        std::vector<double> vel_norm;   // 三维速度追踪误差
        std::vector<double> yaw;        // YAW追踪误差
    };
    ErrorINFO error_info;

    // 无人机系统参数
    struct ErrorRESULT
    {
        double total_time;             // 总记录时间
        int total_nums;                // 总记录数据
        double pos_x_error_mean;       // X轴位置追踪误差平均值
        double pos_x_error_max;        // X轴位置追踪误差最大值
        double pos_y_error_mean;       // X轴位置追踪误差平均值
        double pos_y_error_max;        // X轴位置追踪误差最大值
        double pos_z_error_mean;       // Z轴位置追踪误差平均值
        double pos_z_error_max;        // Z轴位置追踪误差最大值
        double pos_norm_error_mean;    // 三维位置追踪误差平均值
        double pos_norm_error_max;     // 三维位置追踪误差最大值
        double vel_x_error_mean;       // X轴速度追踪误差平均值
        double vel_x_error_max;        // X轴速度追踪误差最大值
        double vel_y_error_mean;       // X轴速度追踪误差平均值
        double vel_y_error_max;        // X轴速度追踪误差最大值
        double vel_z_error_mean;       // Z轴速度追踪误差平均值
        double vel_z_error_max;        // Z轴速度追踪误差最大值
        double vel_norm_error_mean;    // 三维速度追踪误差平均值
        double vel_norm_error_max;     // 三维速度追踪误差最大值
        double yaw_error_mean;         // YAW追踪误差平均值
        double yaw_error_max;          // YAW追踪误差最大值
    };
    ErrorRESULT error_result;

public:
    ControlEvaluation()
    {
        // 初始化 
        error_result.pos_x_error_mean = 0.0;
        error_result.pos_y_error_mean = 0.0;
        error_result.pos_z_error_mean = 0.0;
        error_result.pos_norm_error_mean = 0.0;
        error_result.vel_x_error_mean = 0.0;
        error_result.vel_y_error_mean = 0.0;
        error_result.vel_z_error_mean = 0.0;
        error_result.vel_norm_error_mean = 0.0;
        error_result.yaw_error_mean = 0.0;
        error_result.pos_x_error_max = 0.0;
        error_result.pos_y_error_max = 0.0;
        error_result.pos_z_error_max = 0.0;
        error_result.pos_norm_error_max = 0.0;
        error_result.vel_x_error_max = 0.0;
        error_result.vel_y_error_max = 0.0;
        error_result.vel_z_error_max = 0.0;
        error_result.vel_norm_error_max = 0.0;
        error_result.yaw_error_max = 0.0;    
    }

    double get_yaw_error(double desired_yaw, double yaw_now)
    {
        double error = desired_yaw - yaw_now;

        if(error > M_PI)
        {
            error = error - 2*M_PI;
        }else if(error < -M_PI)
        {
            error = error + 2*M_PI;
        }

        return error;
    }

    void add_traj_info(Eigen::Vector3d pos_sp, Eigen::Vector3d pos_now, Eigen::Vector3d vel_sp, Eigen::Vector3d vel_now, Eigen::Vector3d att_sp, Eigen::Vector3d att_now)
    {
        // 获取系统时间
        struct timeval tv;
        gettimeofday(&tv, NULL);
        // 将时间转为 double（秒 + 微秒/1e6）
        double time = (double)tv.tv_sec + (double)tv.tv_usec / 1000000.0;

        if(first_enter)
        {
            init_time = time;
            first_enter = false;
        }else
        {
            ctrl_info.time.push_back( time - init_time );
            error_info.time.push_back( time - init_time );
        }

        // 记录无人机控制数据
        ctrl_info.uav_pos.push_back(pos_now);
        ctrl_info.uav_pos_sp.push_back(pos_sp);
        ctrl_info.uav_vel.push_back(vel_now);
        ctrl_info.uav_vel_sp.push_back(vel_sp);
        ctrl_info.uav_att.push_back(att_now);
        ctrl_info.uav_att_sp.push_back(att_sp);

        Eigen::Vector3d pos_error, vel_error;
        double yaw_error; 

        pos_error = pos_sp - pos_now;
        vel_error = vel_sp - vel_now;
        yaw_error = get_yaw_error(att_sp[2], att_now[2]);

        error_info.pos_x.push_back( abs(pos_error[0]) );
        error_info.pos_y.push_back( abs(pos_error[1]) );
        error_info.pos_z.push_back( abs(pos_error[2]) );
        error_info.pos_norm.push_back( pos_error.norm() );
        error_info.vel_x.push_back( abs(vel_error[0]) );
        error_info.vel_y.push_back( abs(vel_error[1]) );
        error_info.vel_z.push_back( abs(vel_error[2]) );
        error_info.vel_norm.push_back( vel_error.norm() );
        error_info.yaw.push_back( abs(yaw_error) );

        // if (pos_error_vector.size() > Slide_window)
        // {
        //     pos_error_vector.pop_back();
        // }
        // if (vel_error_vector.size() > Slide_window)
        // {
        //     vel_error_vector.pop_back();
        // }

        calculate_error_mean();
        log();
    }

    void calculate_error_mean()
    {
        // 计算平均值
        error_result.pos_x_error_mean = std::accumulate(error_info.pos_x.begin(), error_info.pos_x.end(), 0.0) / error_info.pos_x.size();
        error_result.pos_y_error_mean = std::accumulate(error_info.pos_y.begin(), error_info.pos_y.end(), 0.0) / error_info.pos_y.size();
        error_result.pos_z_error_mean = std::accumulate(error_info.pos_z.begin(), error_info.pos_z.end(), 0.0) / error_info.pos_z.size();
        error_result.pos_norm_error_mean = std::accumulate(error_info.pos_norm.begin(), error_info.pos_norm.end(), 0.0) / error_info.pos_norm.size();
        
        error_result.vel_x_error_mean = std::accumulate(error_info.vel_x.begin(), error_info.vel_x.end(), 0.0) / error_info.vel_x.size();
        error_result.vel_y_error_mean = std::accumulate(error_info.vel_y.begin(), error_info.vel_y.end(), 0.0) / error_info.vel_y.size();
        error_result.vel_z_error_mean = std::accumulate(error_info.vel_z.begin(), error_info.vel_z.end(), 0.0) / error_info.vel_z.size();
        error_result.vel_norm_error_mean = std::accumulate(error_info.vel_norm.begin(), error_info.vel_norm.end(), 0.0) / error_info.vel_norm.size();

        error_result.yaw_error_mean = std::accumulate(error_info.yaw.begin(), error_info.yaw.end(), 0.0) / error_info.yaw.size();
    }

    void calculate_error_max()
    {
        error_result.total_time = error_info.time.back();
        error_result.total_nums = error_info.pos_x.size();

        // 计算最大值
        error_result.pos_x_error_max = max(error_result.pos_x_error_max, *max_element(error_info.pos_x.begin(), error_info.pos_x.end()));
        error_result.pos_y_error_max = max(error_result.pos_y_error_max, *max_element(error_info.pos_y.begin(), error_info.pos_y.end()));
        error_result.pos_z_error_max = max(error_result.pos_z_error_max, *max_element(error_info.pos_z.begin(), error_info.pos_z.end()));
        error_result.pos_norm_error_max = max(error_result.pos_norm_error_max, *max_element(error_info.pos_norm.begin(), error_info.pos_norm.end()));

        error_result.vel_x_error_max = max(error_result.vel_x_error_max, *max_element(error_info.vel_x.begin(), error_info.vel_x.end()));
        error_result.vel_y_error_max = max(error_result.vel_y_error_max, *max_element(error_info.vel_y.begin(), error_info.vel_y.end()));
        error_result.vel_z_error_max = max(error_result.vel_z_error_max, *max_element(error_info.vel_z.begin(), error_info.vel_z.end()));
        error_result.vel_norm_error_max = max(error_result.vel_norm_error_max, *max_element(error_info.vel_norm.begin(), error_info.vel_norm.end()));

        error_result.yaw_error_max = max(error_result.yaw_error_max, *max_element(error_info.yaw.begin(), error_info.yaw.end()));
    }


    void log()
    {
        // TODO
        // log ctrl_info\error_info\error_result

    }

    void show_result()
    {
        calculate_error_max();
        Logger::print_color(int(LogColor::blue), LOG_BOLD, "-------------------- Control Evaluation Result --------------------");

        Logger::print_color(int(LogColor::green), "error_result.total_time & total_nums :",
                        error_result.total_time,
                        "[ s ]",
                        error_result.total_nums,
                        "[ groups ]");

        Logger::print_color(int(LogColor::blue), LOG_BOLD, "-------------------- position error --------------------");


        Logger::print_color(int(LogColor::green), "error_result.pos_error_mean[X Y Z]:",
                        error_result.pos_x_error_mean,
                        error_result.pos_y_error_mean,
                        error_result.pos_z_error_mean,
                        "[ m ]");

        Logger::print_color(int(LogColor::green), "error_result.pos_norm_error_mean:",
                        error_result.pos_norm_error_mean,
                        "[ m ]");

        Logger::print_color(int(LogColor::green), "error_result.pos_error_max[X Y Z]:",
                        error_result.pos_x_error_max,
                        error_result.pos_y_error_max,
                        error_result.pos_z_error_max,
                        "[ m ]");

        Logger::print_color(int(LogColor::green), "error_result.pos_norm_error_max:",
                        error_result.pos_norm_error_max,
                        "[ m ]");


        Logger::print_color(int(LogColor::blue), LOG_BOLD, "-------------------- velocity error --------------------");

        Logger::print_color(int(LogColor::green), "error_result.vel_error_mean[X Y Z]:",
                        error_result.vel_x_error_mean,
                        error_result.vel_y_error_mean,
                        error_result.vel_z_error_mean,
                        "[m/s]");

        Logger::print_color(int(LogColor::green), "error_result.vel_norm_error_mean:",
                        error_result.vel_norm_error_mean,
                        "[m/s]");

        Logger::print_color(int(LogColor::green), "error_result.vel_error_max[X Y Z]:",
                        error_result.vel_x_error_max,
                        error_result.vel_y_error_max,
                        error_result.vel_z_error_max,
                        "[m/s]");

        Logger::print_color(int(LogColor::green), "error_result.vel_norm_error_max:",
                        error_result.vel_norm_error_max,
                        "[m/s]");

        Logger::print_color(int(LogColor::blue), LOG_BOLD, "-------------------- yaw error --------------------");

        Logger::print_color(int(LogColor::green), "error_result.yaw_error_mean:",
                        error_result.yaw_error_mean/M_PI*180.0,
                        "[deg]");

        Logger::print_color(int(LogColor::green), "error_result.yaw_error_max:",
                        error_result.yaw_error_max/M_PI*180.0,
                        "[deg]");
    }



    void show_debug()
    {
        Logger::print_color(int(LogColor::blue), LOG_BOLD, "-------------------- Control Evaluation --------------------");
        Logger::print_color(int(LogColor::green), "error_info.time:",
                        error_info.time.back(),
                        "[ s ]");

        Logger::print_color(int(LogColor::blue), LOG_BOLD, "-------------------- position error --------------------");

        Logger::print_color(int(LogColor::green), "error_result.pos_norm_error_mean:",
                        error_result.pos_norm_error_mean,
                        "[ m ]");

        Logger::print_color(int(LogColor::blue), LOG_BOLD, "-------------------- velocity error --------------------");

        Logger::print_color(int(LogColor::green), "error_result.vel_norm_error_mean:",
                        error_result.vel_norm_error_mean,
                        "[m/s]");

        Logger::print_color(int(LogColor::blue), LOG_BOLD, "-------------------- yaw error --------------------");

        Logger::print_color(int(LogColor::green), "error_result.yaw_error_mean:",
                        error_result.yaw_error_mean/M_PI*180.0,
                        "[deg]");
    }


    ~ControlEvaluation()
    {
    }
};

#endif