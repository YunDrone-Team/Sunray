#include <math.h>
#include <vector>
#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include <yaml-cpp/yaml.h>

// 圆形轨迹生成器
class traj_generator
{
public:
    traj_generator(int num)
    {
        this->vehicles_num = num;
    }
    ~traj_generator()
    {
    }
    bool save_trajectory_flag;
    // 圆形轨迹生成器
    std::vector<std::tuple<float, float, float>> circle_trajectory(float radius_x = 1.0,
                                                                   float radius_y = 1.0,
                                                                   float omega = 0.5,
                                                                   float center_x = 0.0,
                                                                   float center_y = 0.0);

    // 八字形轨迹生成器
    std::vector<std::tuple<float, float, float>> figure_eight(float radius_x = 1.0,
                                                              float radius_y = 0.6,
                                                              float omega = 0.5,
                                                              float center_x = 0.0,
                                                              float center_y = 0.0);

    // Lissajous曲线生成器
    std::vector<std::tuple<float, float, float>> generate_lissajous();

    // 保存轨迹
    void save_trajectory(std::vector<std::tuple<float, float, float>> trajectory,
                         std::string filename = "~/Documents/trajectory.txt");

private:
    // Lissajous曲线参数
    double lissa_a;     // x轴频率
    double lissa_b;     // y轴频率
    double lissa_delta; // 相位差
    // 轨迹参数
    Eigen::Vector3d center;           // 圆心
    double radius_x;                  // 半径
    double radius_y;                  // 半径
    float omega;                      // 圆参数：角速度
    float linear_speed;               // 线速度
    Eigen::Vector3d initial_position; // 初始位置
    // 载具数量
    int vehicles_num;
};
