#include "traj_generator.h"

// 圆形轨迹生成器
std::vector<std::tuple<float, float, float>> traj_generator::circle_trajectory(float radius_x,
                                                                               float radius_y,
                                                                               float linear_speed,
                                                                               float center_x,
                                                                               float center_y)
{

    std::vector<std::tuple<float, float, float>> trajectory_list; // 生成轨迹列表
    this->lissa_a = 1;
    this->lissa_b = 1;
    this->lissa_delta = M_PI / 2;
    this->linear_speed = linear_speed;
    this->radius_x = radius_x;
    this->radius_y = radius_y;
    trajectory_list = this->generate_lissajous();
    for (auto &point : trajectory_list)
    {
        // 将坐标平移到指定中心点并缩放到指定半径
        std::get<0>(point) = std::get<0>(point) * radius_x + center_x;
        std::get<1>(point) = std::get<1>(point) * radius_y + center_y;
        // std::cout<< "x: " << std::get<0>(point) << " y: " << std::get<1>(point) << std::endl;
    }
    std::string filename = "./circle.yaml";
    std::ofstream fout(filename);
    if (fout.is_open())
    {
        YAML::Emitter out;

        // 开始生成 YAML 内容
        out << YAML::BeginMap;
        out << YAML::Key << "dynamic" << YAML::Value << YAML::BeginMap;
        out << YAML::Key << "radius_x" << YAML::Value << this->radius_x;
        out << YAML::Key << "radius_y" << YAML::Value << this->radius_y;
        out << YAML::Key << "linear_speed" << YAML::Value << linear_speed;
        out << YAML::EndMap;

        out << YAML::Key << "initial_point" << YAML::Value << YAML::BeginMap;
        out << YAML::Key << "num" << YAML::Value << vehicles_num;
        out << YAML::Key << "name" << YAML::Value << "circle";

        for (int i = 0; i < vehicles_num; i++)
        {
            std::string point_key = "ReadyPoint_" + std::to_string(i + 1);
            out << YAML::Key << point_key << YAML::Value << int(trajectory_list.size() / vehicles_num) * i;
        }

        out << YAML::EndMap;
        out << YAML::EndMap;

        fout << out.c_str();
        fout << std::endl;
        fout.close();
        this->save_trajectory(trajectory_list, filename);
    }
    else
    {
        std::cerr << "Unable to open file " << filename << std::endl;
    }
    return trajectory_list;
}

// 八字形轨迹生成器
std::vector<std::tuple<float, float, float>> traj_generator::figure_eight(float radius_x,
                                                                          float radius_y,
                                                                          float linear_speed,
                                                                          float center_x,
                                                                          float center_y)
{

    std::vector<std::tuple<float, float, float>> trajectory_list; // 生成轨迹列表
    this->lissa_a = 1;
    this->lissa_b = 2;
    this->lissa_delta = M_PI / 2;
    this->linear_speed = linear_speed;
    this->radius_x = radius_x;
    this->radius_y = radius_y;
    trajectory_list = this->generate_lissajous();
    for (auto &point : trajectory_list)
    {
        // 将坐标平移到指定中心点并缩放到指定半径
        std::get<0>(point) = std::get<0>(point) * this->radius_x + center_x;
        std::get<1>(point) = std::get<1>(point) * this->radius_y + center_y;
    }
    if (save_trajectory_flag)
    {
        std::string filename = "./figure_eight_trajectory.yaml";
        std::ofstream fout(filename);
        if (fout.is_open())
        {
            YAML::Emitter out;

            // 开始生成 YAML 内容
            out << YAML::BeginMap;
            out << YAML::Key << "dynamic" << YAML::Value << YAML::BeginMap;
            out << YAML::Key << "radius_x" << YAML::Value << this->radius_x;
            out << YAML::Key << "radius_y" << YAML::Value << this->radius_y;
            out << YAML::Key << "linear_speed" << YAML::Value << linear_speed;
            out << YAML::EndMap;

            out << YAML::Key << "initial_point" << YAML::Value << YAML::BeginMap;
            out << YAML::Key << "num" << YAML::Value << vehicles_num;
            out << YAML::Key << "name" << YAML::Value << "figure_eight";

            for (int i = 0; i < vehicles_num; i++)
            {
                std::string point_key = "ReadyPoint_" + std::to_string(i + 1);
                out << YAML::Key << point_key;
                if (i < vehicles_num / 2)
                {
                    out << YAML::Value << int(trajectory_list.size() / vehicles_num) * i + int((trajectory_list.size() / vehicles_num / vehicles_num));
                }
                else
                {
                    out << YAML::Value << int(trajectory_list.size() / vehicles_num) * i - int((trajectory_list.size() / vehicles_num / vehicles_num));
                }
            }
            out << YAML::EndMap;
            out << YAML::EndMap;
            fout << out.c_str();
            fout << std::endl;
            fout.close();
            this->save_trajectory(trajectory_list, filename);
        }
        else
        {
            std::cerr << "Unable to open file " << filename << std::endl;
        }
    }
    return trajectory_list;
}

std::vector<std::tuple<float, float, float>> traj_generator::generate_lissajous()
{
    // int num_points = 2 * M_PI / this->omega * 10;                 // 根据角速度计算点数 时间*频率
    // 根据半径计算点数
    int num_points = 2 * M_PI * std::max(radius_x, radius_y) / this->linear_speed * 10;
    std::vector<std::tuple<float, float, float>> trajectory_list; // 轨迹列表
    for (int i = 0; i < num_points; ++i)
    {
        // 参数t从0到2π
        double t = 2.0 * M_PI * i / num_points;
        // 计算x和y坐标
        double x = this->lissa_a * sin(this->lissa_a * t + this->lissa_delta);
        double y = this->lissa_b * sin(this->lissa_b * t);
        // 将坐标添加到轨迹列表中
        trajectory_list.push_back(std::make_tuple(x, y, 0.0));
    }

    return trajectory_list;
}

void traj_generator::save_trajectory(std::vector<std::tuple<float, float, float>> trajectory,
                                     std::string filename)
{
    std::ofstream fout(filename, std::ios::app);
    if (fout.is_open())
    {
        YAML::Emitter out;
        out << YAML::BeginMap;
        // 生成并添加点
        int i = 0;
        for (const auto &point : trajectory)
        {
            std::string point_key = "point_" + std::to_string(i + 1);
            out << YAML::Key << point_key;
            out << YAML::Value << YAML::Flow << YAML::BeginSeq << std::get<0>(point) << std::get<1>(point) << 0.0 << YAML::EndSeq;
            i++;
        }
        out << YAML::EndMap;
        fout << out.c_str();
        fout.close();

        std::cout << "YAML file generated successfully: " << filename << std::endl;
    }
    else
    {
        std::cerr << "Unable to open file " << filename << std::endl;
    }
}

std::tuple<float, float> traj_generator::circle_trajectory_online(int t)
{
    int num_points = 2 * M_PI * std::max(this->radius_x, this->radius_y) / this->linear_speed * 10;
    int offset = int(num_points / this->vehicles_num) * (vehicle_id - 1);
    t = (t + offset) % num_points;
    float ti = 2.0 * M_PI * t / num_points;
    // 计算x和y坐标
    double x = this->lissa_a * sin(this->lissa_a * ti + this->lissa_delta);
    double y = this->lissa_b * sin(this->lissa_b * ti);
    x = x * this->radius_x + this->center_x;
    y = y * this->radius_y + this->center_y;
    return std::make_tuple(x, y);
}

std::tuple<float, float> traj_generator::figure_eight_online(int t)
{
    int num_points = 2 * M_PI * std::max(this->radius_x, this->radius_y) / this->linear_speed * 10;
    int offset = int(num_points / this->vehicles_num) * (vehicle_id - 1);
    if (vehicle_id <= (this->vehicles_num / 2))
    {
        if(this->vehicles_num > 2)
        {
            t = (t + offset + int(num_points / vehicles_num / vehicles_num)) % num_points;
        }
        else
        {
            t = (t + offset) % num_points;
        }
        
    }
    else
    {
        if(this->vehicles_num >= 2)
        {
            t = (t + offset - int(num_points / vehicles_num / vehicles_num)) % num_points;
        }
        else
        {
            t = (t + offset) % num_points;
        }
    }
    float ti = 2.0 * M_PI * t / num_points;
    // 计算x和y坐标
    double x = this->lissa_a * sin(this->lissa_a * ti + this->lissa_delta);
    double y = this->lissa_b * sin(this->lissa_b * ti);
    x = x * this->radius_x + this->center_x;
    y = y * this->radius_y + this->center_y;
    return std::make_tuple(x, y);
}