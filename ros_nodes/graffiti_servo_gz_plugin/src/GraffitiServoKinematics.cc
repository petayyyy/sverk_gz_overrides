#include <algorithm>
#include <atomic>
#include <memory>
#include <string>
#include <vector>

#include <gz/msgs/double.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>
#include <sdf/Element.hh>

namespace sverk::sim
{
class GraffitiServoKinematics:
  public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
{
  public: void Configure(
      const gz::sim::Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      gz::sim::EntityComponentManager &_ecm,
      gz::sim::EventManager &/*_eventMgr*/) override
  {
    this->model = gz::sim::Model(_entity);

    this->mode = this->ReadString(_sdf, "mode", "legacy");
    this->simpleCamMode = this->mode == "simple_cam";
    this->gearJointName = this->ReadString(_sdf, "gear_joint", "gear_joint");
    this->leverJointName = this->ReadString(_sdf, "lever_joint", "lever_joint");
    this->camJointName = this->ReadString(_sdf, "cam_joint", "graffiti_cam_joint");
    this->capJointName = this->ReadString(_sdf, "cap_joint", "can_cap_joint");
    this->topic = this->ReadString(_sdf, "topic", "/spray/servo/travel");

    this->gearMaxRad = this->ReadDouble(_sdf, "gear_max_rad", 1.82);
    this->leverMaxRad = this->ReadDouble(_sdf, "lever_max_rad", -0.62);
    this->camPressRad = this->ReadDouble(_sdf, "cam_press_rad", -1.1609930184);
    this->capMaxM = this->ReadDouble(_sdf, "cap_max_m", 0.003);
    this->capStartTravel = std::clamp(
      this->ReadDouble(_sdf, "cap_start_travel", 0.6193103797), 0.0, 1.0);
    this->reverseTravelFraction =
      std::max(0.0, this->ReadDouble(_sdf, "reverse_travel_fraction", 0.6666666667));

    this->ResolveJoints(_ecm);
    this->node.Subscribe(this->topic, &GraffitiServoKinematics::OnTravel, this);
  }

  public: void PreUpdate(
      const gz::sim::UpdateInfo &/*_info*/,
      gz::sim::EntityComponentManager &_ecm) override
  {
    if (!this->jointsResolved)
    {
      this->ResolveJoints(_ecm);
    }

    if (!this->jointsResolved)
    {
      return;
    }

    const double travel = std::clamp(
      this->targetTravel.load(),
      -this->reverseTravelFraction,
      1.0);
    if (this->simpleCamMode)
    {
      const double capTravel = std::max(0.0, travel);
      this->ApplyJointPosition(_ecm, this->camJoint, travel * this->camPressRad);
      this->ApplyJointPosition(_ecm, this->capJoint, capTravel * this->capMaxM);
      return;
    }

    double capTravel = 0.0;
    if (travel > this->capStartTravel && this->capStartTravel < 1.0)
    {
      capTravel = (travel - this->capStartTravel) / (1.0 - this->capStartTravel);
    }
    this->ApplyJointPosition(_ecm, this->gearJoint, travel * this->gearMaxRad);
    this->ApplyJointPosition(_ecm, this->leverJoint, travel * this->leverMaxRad);
    this->ApplyJointPosition(_ecm, this->capJoint, capTravel * this->capMaxM);
  }

  private: void OnTravel(const gz::msgs::Double &_msg)
  {
    this->targetTravel.store(_msg.data());
  }

  private: void ResolveJoints(gz::sim::EntityComponentManager &_ecm)
  {
    if (!this->model.Valid(_ecm))
    {
      return;
    }

    this->capJoint = this->model.JointByName(_ecm, this->capJointName);

    if (this->simpleCamMode)
    {
      this->camJoint = this->model.JointByName(_ecm, this->camJointName);
      this->jointsResolved =
        this->camJoint != gz::sim::kNullEntity &&
        this->capJoint != gz::sim::kNullEntity;
    }
    else
    {
      this->gearJoint = this->model.JointByName(_ecm, this->gearJointName);
      this->leverJoint = this->model.JointByName(_ecm, this->leverJointName);
      this->jointsResolved =
        this->gearJoint != gz::sim::kNullEntity &&
        this->leverJoint != gz::sim::kNullEntity &&
        this->capJoint != gz::sim::kNullEntity;
    }

    if (this->jointsResolved)
    {
      if (this->simpleCamMode)
      {
        gz::sim::Joint(this->camJoint).ResetPosition(_ecm, {0.0});
      }
      else
      {
        gz::sim::Joint(this->gearJoint).ResetPosition(_ecm, {0.0});
        gz::sim::Joint(this->leverJoint).ResetPosition(_ecm, {0.0});
      }
      gz::sim::Joint(this->capJoint).ResetPosition(_ecm, {0.0});
    }
  }

  private: void ApplyJointPosition(
      gz::sim::EntityComponentManager &_ecm,
      gz::sim::Entity _jointEntity,
      double _position)
  {
    gz::sim::Joint joint(_jointEntity);
    joint.ResetPosition(_ecm, std::vector<double>{_position});
    joint.ResetVelocity(_ecm, std::vector<double>{0.0});
  }

  private: std::string ReadString(
      const std::shared_ptr<const sdf::Element> &_sdf,
      const std::string &_name,
      const std::string &_default) const
  {
    if (_sdf && _sdf->HasElement(_name))
    {
      return _sdf->Get<std::string>(_name);
    }
    return _default;
  }

  private: double ReadDouble(
      const std::shared_ptr<const sdf::Element> &_sdf,
      const std::string &_name,
      double _default) const
  {
    if (_sdf && _sdf->HasElement(_name))
    {
      return _sdf->Get<double>(_name);
    }
    return _default;
  }

  private: gz::sim::Model model{gz::sim::kNullEntity};
  private: gz::sim::Entity gearJoint{gz::sim::kNullEntity};
  private: gz::sim::Entity leverJoint{gz::sim::kNullEntity};
  private: gz::sim::Entity camJoint{gz::sim::kNullEntity};
  private: gz::sim::Entity capJoint{gz::sim::kNullEntity};
  private: bool jointsResolved{false};

  private: std::string mode{"legacy"};
  private: std::string gearJointName{"gear_joint"};
  private: std::string leverJointName{"lever_joint"};
  private: std::string camJointName{"graffiti_cam_joint"};
  private: std::string capJointName{"can_cap_joint"};
  private: std::string topic{"/spray/servo/travel"};

  private: double gearMaxRad{1.82};
  private: double leverMaxRad{-0.62};
  private: double camPressRad{-1.1609930184};
  private: double capMaxM{0.003};
  private: double capStartTravel{0.6193103797};
  private: double reverseTravelFraction{0.6666666667};
  private: bool simpleCamMode{false};
  private: std::atomic<double> targetTravel{0.0};
  private: gz::transport::Node node;
};
}

GZ_ADD_PLUGIN(
  sverk::sim::GraffitiServoKinematics,
  gz::sim::System,
  sverk::sim::GraffitiServoKinematics::ISystemConfigure,
  sverk::sim::GraffitiServoKinematics::ISystemPreUpdate)
