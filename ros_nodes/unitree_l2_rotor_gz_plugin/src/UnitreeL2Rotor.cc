#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <gz/plugin/Register.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <sdf/Element.hh>

namespace sverk::sim
{
class UnitreeL2Rotor:
  public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
{
  public: void Configure(
      const gz::sim::Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      gz::sim::EntityComponentManager &_ecm,
      gz::sim::EventManager & /*_eventMgr*/) override
  {
    this->model = gz::sim::Model(_entity);
    if (_sdf && _sdf->HasElement("joint_name"))
      this->jointName = _sdf->Get<std::string>("joint_name");
    if (_sdf && _sdf->HasElement("rotation_hz"))
      this->rotationHz = _sdf->Get<double>("rotation_hz");
    this->ResolveJoint(_ecm);
  }

  public: void PreUpdate(
      const gz::sim::UpdateInfo &_info,
      gz::sim::EntityComponentManager &_ecm) override
  {
    if (_info.paused)
      return;
    if (this->joint == gz::sim::kNullEntity)
      this->ResolveJoint(_ecm);
    if (this->joint == gz::sim::kNullEntity)
      return;

    const double seconds = std::chrono::duration<double>(_info.simTime).count();
    constexpr double kTwoPi = 6.28318530717958647692;
    const double angle = std::remainder(
      kTwoPi * this->rotationHz * seconds, kTwoPi);
    gz::sim::Joint(this->joint).ResetPosition(_ecm, std::vector<double>{angle});
    gz::sim::Joint(this->joint).ResetVelocity(_ecm, std::vector<double>{0.0});
  }

  private: void ResolveJoint(gz::sim::EntityComponentManager &_ecm)
  {
    if (this->model.Valid(_ecm))
      this->joint = this->model.JointByName(_ecm, this->jointName);
  }

  private: gz::sim::Model model{gz::sim::kNullEntity};
  private: gz::sim::Entity joint{gz::sim::kNullEntity};
  private: std::string jointName{"l2_rotor_joint"};
  private: double rotationHz{5.55};
};
}

GZ_ADD_PLUGIN(
  sverk::sim::UnitreeL2Rotor,
  gz::sim::System,
  sverk::sim::UnitreeL2Rotor::ISystemConfigure,
  sverk::sim::UnitreeL2Rotor::ISystemPreUpdate)
