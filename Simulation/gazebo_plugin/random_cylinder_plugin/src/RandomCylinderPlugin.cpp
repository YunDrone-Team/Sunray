#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Rand.hh>
#include <vector>
#include <string>
#include <sstream>

namespace gazebo
{
    class RandomCylinderPlugin : public WorldPlugin
    {
    public:
        void Load(physics::WorldPtr _world, sdf::ElementPtr /*_sdf*/) override
        {
            // 定义圆柱体的坐标组
            std::vector<std::vector<std::string>> cylinder_coordinate_sets = {
                // {"-0.5218 -2.016 1.0 0 0 0", "0.928 -1.284 1.0 0 0 0", "1.9138 -2.2118 1.0 0 0 0"},
                // {"-0.6 -2.2 1.0 0 0 0", "0.928 -1.284 1.0 0 0 0", "1.5 -2.2 1.0 0 0 0"},
                // {"-0.7 -2.5 1.0 0 0 0", "0.928 -1.284 1.0 0 0 0", "1.7 -2.0 1.0 0 0 0"},
                {"-0.7 -1.33 1.0 0 0 0", "0.928 -1.284 1.0 0 0 0", "1.03 -2.37 1.0 0 0 0"},
                {"-0.7 -1.33 1.0 0 0 0", "0.928 -1.284 1.0 0 0 0", "0.163 -2.285 1.0 0 0 0"},
                {"-0.24 -1.09 1.0 0 0 0", "0.928 -1.284 1.0 0 0 0", "0.163 -2.285 1.0 0 0 0"}, // 1
                {"-0.305 -1.0 1.0 0 0 0", "0.928 -1.284 1.0 0 0 0", "-0.4 -2.25 1.0 0 0 0"}    // 2
            };

            // 随机选择一组坐标
            int random_index = ignition::math::Rand::IntUniform(0, cylinder_coordinate_sets.size() - 1);
            std::vector<std::string> selected_set = cylinder_coordinate_sets[random_index];

            // 模型名称
            std::vector<std::string> model_names = {"cylinder_25cm", "cylinder_25cm_clone_0", "cylinder_25cm_clone"};

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

            std::vector<std::string> target_zone_poses = {
                "-1.5 1.5 0.27 0 0 0",
                "-1.75 1.8 0.27 0 0 0",
                "-1.25 1.8 0.27 0 0 0",
                "-1.25 1.25 0.27 0 0 0",
                "-1.75 1.25 0.27 0 0 0"};

            int random_zone_index = ignition::math::Rand::IntUniform(0, target_zone_poses.size() - 1);
            std::string selected_zone_pose_str = target_zone_poses[random_zone_index];
            physics::ModelPtr target_model = _world->ModelByName("target_zone");
            if (target_model)
            {
                std::istringstream iss(selected_zone_pose_str);
                double x, y, z, roll, pitch, yaw;
                iss >> x >> y >> z >> roll >> pitch >> yaw;
                target_model->SetWorldPose(ignition::math::Pose3d(x, y, z, roll, pitch, yaw));
            }
            else
            {
                gzerr << "Could not find model: target_zone" << std::endl;
            }
        }
    };
    GZ_REGISTER_WORLD_PLUGIN(RandomCylinderPlugin)
}