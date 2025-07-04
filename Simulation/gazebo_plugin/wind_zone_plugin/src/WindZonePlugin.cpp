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
      for (auto &m : this->world->Models())
      {
        if (m->GetName() == this->model->GetName())
          continue;

        for (auto &l : m->GetLinks())
        {
          ignition::math::Vector3d pos = l->WorldPose().Pos();
          ignition::math::AxisAlignedBox box = this->link->BoundingBox();

          if (box.Contains(pos))
          {
            ignition::math::Vector3d windForce(0, -5, 0); // -5N 向Y
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
