#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

namespace gazebo
{
  class WindZonePlugin : public ModelPlugin
  {
  public:
    void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/)
    {
      this->model = _model;
      this->world = this->model->GetWorld();
      this->link = this->model->GetLink("link");

      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&WindZonePlugin::OnUpdate, this));
    }

    void OnUpdate()
    {
      ignition::math::AxisAlignedBox box = this->link->BoundingBox();

      double minX = box.Min().X();
      double maxX = box.Max().X();
      double minY = box.Min().Y();
      double maxY = box.Max().Y();

      double windMinZ = box.Max().Z();
      double windMaxZ = box.Max().Z() + 5.0;

      ignition::math::Vector3d windForce(0, -0.2, 0); //Y轴负方向0.5N的力

      for (auto &m : this->world->Models())
      {
        if (m->GetName() == this->model->GetName())
          continue;

        for (auto &l : m->GetLinks())
        {
          ignition::math::Vector3d pos = l->WorldPose().Pos();

          if (pos.X() >= minX && pos.X() <= maxX &&
              pos.Y() >= minY && pos.Y() <= maxY &&
              pos.Z() >= windMinZ && pos.Z() <= windMaxZ)
          {
            l->AddForce(windForce);
          }
        }
      }
    }

  private:
    physics::ModelPtr model;
    physics::WorldPtr world;
    physics::LinkPtr link;
    event::ConnectionPtr updateConnection;
  };

  GZ_REGISTER_MODEL_PLUGIN(WindZonePlugin)
}
