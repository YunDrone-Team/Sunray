#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Rand.hh>
#include <vector>
#include <string>

namespace gazebo
{
    class RandomCylinderPlugin : public WorldPlugin
    {
    public:
        void Load(physics::WorldPtr _world, sdf::ElementPtr /*_sdf*/) override
        {
            // 定义坐标组
            std::vector<std::vector<std::string>> coordinate_sets = {
                {"-0.5218 -2.016 1.0 0 0 0", "0.928 -1.284 1.0 0 0 0", "1.9138 -2.2118 1.0 0 0 0"},
                {"-0.6 -2.2 1,0 0 0 0", "0.928 -1.284 1.0 0 0 0", "1.5 -2.2 1.0 0 0 0"},
                {"-0.7 -2.5 1.0 0 0 0", "0.928 -1.284 1.0 0 0 0", "1.7 -2.0 1.0 0 0 0"}
            };

            // 随机选择一组坐标
            int random_index = ignition::math::Rand::IntUniform(0, coordinate_sets.size() - 1);
            std::vector<std::string> selected_set = coordinate_sets[random_index];

            // 模型名称
            std::vector<std::string> model_names = {"cylinder_25cm", "cylinder_25cm_clone_0", "cylinder_25cm_clone"};

            // 更新每个圆柱体的位置
            for (size_t i = 0; i < model_names.size(); ++i)
            {
                physics::ModelPtr model = _world->ModelByName(model_names[i]);
                if (model)
                {
                    // 解析 pose 字符串
                    std::istringstream iss(selected_set[i]);
                    double x, y, z, roll, pitch, yaw;
                    iss >> x >> y >> z >> roll >> pitch >> yaw;
                    model->SetWorldPose(ignition::math::Pose3d(x, y, z, roll, pitch, yaw));
                }
            }
        }
    };
    GZ_REGISTER_WORLD_PLUGIN(RandomCylinderPlugin)
}