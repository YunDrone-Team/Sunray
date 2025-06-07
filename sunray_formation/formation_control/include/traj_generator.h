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
    traj_generator() {};
    traj_generator(int num)
    {
        this->vehicles_num = num;
    };
    ~traj_generator() {
    };
    bool save_trajectory_flag;
    void init(int num, int id)
    {
        this->vehicles_num = num;
        this->vehicle_id = id;
    };
    void set_param(float lissa_a,
                   float lissa_b,
                   float lissa_delta,
                   float radius_x,
                   float radius_y,
                   float center_x,
                   float center_y,
                   float linear_speed)
    {
        this->lissa_a = lissa_a;
        this->lissa_b = lissa_b;
        this->lissa_delta = lissa_delta;
        this->radius_x = radius_x;
        this->radius_y = radius_y;
        this->center_x = center_x;
        this->center_y = center_y;
        this->linear_speed = linear_speed;
    }
    // 圆形轨迹生成器
    std::vector<std::tuple<float, float, float>> circle_trajectory(float radius_x = 1.0,
                                                                   float radius_y = 1.0,
                                                                   float linear_speed = 0.5,
                                                                   float center_x = 0.0,
                                                                   float center_y = 0.0);

    // 八字形轨迹生成器
    std::vector<std::tuple<float, float, float>> figure_eight(float radius_x = 1.0,
                                                              float radius_y = 0.6,
                                                              float linear_speed = 0.5,
                                                              float center_x = 0.0,
                                                              float center_y = 0.0);

    // Lissajous曲线生成器
    std::vector<std::tuple<float, float, float>> generate_lissajous();

    // 保存轨迹
    void save_trajectory(std::vector<std::tuple<float, float, float>> trajectory,
                         std::string filename = "~/Documents/trajectory.txt");

    std::tuple<float, float> circle_trajectory_online(int t);

    std::tuple<float, float> figure_eight_online(int t);

private:
    // Lissajous曲线参数
    double lissa_a;     // x轴频率
    double lissa_b;     // y轴频率
    double lissa_delta; // 相位差
    // 轨迹参数
    float center_x;                   // 圆心
    float center_y;                   // 圆心
    double radius_x;                  // 半径
    double radius_y;                  // 半径
    float linear_speed;               // 线速度
    Eigen::Vector3d initial_position; // 初始位置
    int vehicles_num;                 // 载具数量
    int vehicle_id;                   // 载具编号: 1~n
};
