#include <algorithm>
#include <chrono>
#include <cmath>
#include <cctype>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <gz/math/Color.hh>
#include <gz/msgs/visual.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/Material.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Visual.hh>
#include <gz/sim/components/VisualCmd.hh>
#include <sdf/Element.hh>
#include <sdf/Material.hh>

#include <led_interfaces/msg/led_state_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

namespace sverk::sim
{
struct Rgb
{
  uint8_t r{0};
  uint8_t g{0};
  uint8_t b{0};

  bool operator==(const Rgb &_other) const
  {
    return this->r == _other.r && this->g == _other.g && this->b == _other.b;
  }
};

struct VisualRef
{
  std::string name;
  gz::sim::Entity entity{gz::sim::kNullEntity};
  std::string poseKey;
};

class LedStripVisualization:
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
    this->ledLinkName = this->ReadString(_sdf, "led_link", "obrik_led_points_link");
    this->visualPrefix = this->ReadString(_sdf, "visual_prefix", "obrik_led_point_");
    this->frameTopic = this->ReadString(_sdf, "frame_topic", "/led/sim/frame");
    this->brightnessTopic = this->ReadString(
      _sdf, "brightness_topic", "/led/sim/brightness");

    if (!rclcpp::ok())
    {
      int argc = 0;
      char **argv = nullptr;
      rclcpp::init(argc, argv);
    }

    std::string nodeName = "led_strip_gz_" + this->model.Name(_ecm);
    std::replace_if(nodeName.begin(), nodeName.end(),
      [](unsigned char c) { return !std::isalnum(c) && c != '_'; }, '_');
    this->rosNode = std::make_shared<rclcpp::Node>(nodeName);
    this->frameSub = this->rosNode->create_subscription<led_interfaces::msg::LEDStateArray>(
      this->frameTopic,
      rclcpp::QoS(10),
      [this](const led_interfaces::msg::LEDStateArray::SharedPtr _msg)
      {
        std::lock_guard<std::mutex> lock(this->frameMutex);
        std::size_t size = 0;
        for (const auto &led : _msg->leds)
        {
          size = std::max(size, static_cast<std::size_t>(led.index) + 1u);
        }
        std::vector<Rgb> next(size);
        for (const auto &led : _msg->leds)
        {
          if (led.index < next.size())
          {
            next[led.index] = Rgb{led.r, led.g, led.b};
          }
        }
        this->logicalFrame = std::move(next);
        this->frameDirty = true;
      });
    this->brightnessSub = this->rosNode->create_subscription<std_msgs::msg::Float32>(
      this->brightnessTopic,
      rclcpp::QoS(10),
      [this](const std_msgs::msg::Float32::SharedPtr _msg)
      {
        std::lock_guard<std::mutex> lock(this->frameMutex);
        this->brightness = std::clamp(static_cast<double>(_msg->data), 0.0, 1.0);
        this->frameDirty = true;
      });
    this->executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    this->executor->add_node(this->rosNode);

    this->ResolveVisuals(_ecm);
  }

  public: ~LedStripVisualization() override
  {
    if (this->executor && this->rosNode)
    {
      this->executor->remove_node(this->rosNode);
    }
  }

  public: void PreUpdate(
      const gz::sim::UpdateInfo &/*_info*/,
      gz::sim::EntityComponentManager &_ecm) override
  {
    if (this->executor)
    {
      this->executor->spin_some(std::chrono::milliseconds(0));
    }
    if (this->visualGroups.empty())
    {
      this->ResolveVisuals(_ecm);
    }

    std::vector<Rgb> frame;
    double frameBrightness = 0.01;
    {
      std::lock_guard<std::mutex> lock(this->frameMutex);
      if (!this->frameDirty || this->logicalFrame.empty() || this->visualGroups.empty())
      {
        return;
      }
      frame = this->logicalFrame;
      frameBrightness = this->brightness;
      this->frameDirty = false;
    }

    std::vector<Rgb> rendered(this->visualGroups.size());
    for (std::size_t i = 0; i < rendered.size(); ++i)
    {
      // The real strip has 112 logical LEDs while CAD previews have a
      // model-specific number of positions. Resample by normalized
      // strip position so gradients and wipe direction stay faithful.
      std::size_t source = 0;
      if (rendered.size() > 1 && frame.size() > 1)
      {
        source = static_cast<std::size_t>(std::llround(
          static_cast<double>(i) * static_cast<double>(frame.size() - 1) /
          static_cast<double>(rendered.size() - 1)));
      }
      rendered[i] = frame[std::min(source, frame.size() - 1)];
    }

    for (std::size_t i = 0; i < this->visualGroups.size(); ++i)
    {
      const auto &rgb = rendered[i];
      // Match perceived low-power LEDs in Gazebo: besides dimming RGB, reduce
      // opacity by the same factor. A 1% low-battery/startup LED is therefore
      // about 99% transparent instead of looking like an opaque black sphere.
      const double alpha = std::clamp(frameBrightness, 0.0, 1.0);
      const gz::math::Color color(
        frameBrightness * static_cast<double>(rgb.r) / 255.0,
        frameBrightness * static_cast<double>(rgb.g) / 255.0,
        frameBrightness * static_cast<double>(rgb.b) / 255.0,
        alpha);
      for (const auto entity : this->visualGroups[i])
      {
        sdf::Material material;
        if (const auto existing = _ecm.ComponentData<gz::sim::components::Material>(entity))
        {
          material = *existing;
        }
        material.SetAmbient(color);
        material.SetDiffuse(color);
        material.SetEmissive(color);
        _ecm.SetComponentData<gz::sim::components::Material>(entity, material);

        // Material is the initial SDF state. VisualCmd is consumed by Gazebo's
        // rendering path for an already-created visual, so it is required for
        // runtime colour changes to reach the GUI.
        gz::msgs::Visual command;
        auto *commandMaterial = command.mutable_material();
        for (auto *commandColor : {
               commandMaterial->mutable_ambient(),
               commandMaterial->mutable_diffuse(),
               commandMaterial->mutable_emissive()})
        {
          commandColor->set_r(static_cast<float>(color.R()));
          commandColor->set_g(static_cast<float>(color.G()));
          commandColor->set_b(static_cast<float>(color.B()));
          commandColor->set_a(static_cast<float>(color.A()));
        }
        _ecm.SetComponentData<gz::sim::components::VisualCmd>(entity, command);
      }
    }
    if (!this->firstFrameApplied)
    {
      this->firstFrameApplied = true;
      RCLCPP_INFO(this->rosNode->get_logger(),
        "Applied first %zu-pixel logical frame at %.1f%% brightness",
        frame.size(), frameBrightness * 100.0);
    }
  }

  private: void ResolveVisuals(gz::sim::EntityComponentManager &_ecm)
  {
    if (!this->model.Valid(_ecm))
    {
      return;
    }
    const auto link = this->model.LinkByName(_ecm, this->ledLinkName);
    if (link == gz::sim::kNullEntity)
    {
      return;
    }

    std::vector<VisualRef> refs;
    _ecm.Each<
      gz::sim::components::Visual,
      gz::sim::components::Name,
      gz::sim::components::ParentEntity,
      gz::sim::components::Pose>(
      [&](const gz::sim::Entity &_visual,
          const gz::sim::components::Visual *,
          const gz::sim::components::Name *_name,
          const gz::sim::components::ParentEntity *_parent,
          const gz::sim::components::Pose *_pose) -> bool
      {
        if (_parent->Data() != link || _name->Data().rfind(this->visualPrefix, 0) != 0)
        {
          return true;
        }
        const auto &pose = _pose->Data();
        std::ostringstream key;
        key << std::fixed << std::setprecision(8)
            << pose.Pos().X() << ',' << pose.Pos().Y() << ',' << pose.Pos().Z() << ','
            << pose.Rot().X() << ',' << pose.Rot().Y() << ','
            << pose.Rot().Z() << ',' << pose.Rot().W();
        refs.push_back(VisualRef{_name->Data(), _visual, key.str()});
        return true;
      });

    std::sort(refs.begin(), refs.end(),
      [](const VisualRef &_a, const VisualRef &_b) { return _a.name < _b.name; });
    std::unordered_map<std::string, std::size_t> groupByPose;
    std::vector<std::vector<gz::sim::Entity>> groups;
    for (const auto &ref : refs)
    {
      auto [it, inserted] = groupByPose.emplace(ref.poseKey, groups.size());
      if (inserted)
      {
        groups.emplace_back();
      }
      groups[it->second].push_back(ref.entity);
    }
    this->visualGroups = std::move(groups);
    if (!this->visualGroups.empty())
    {
      RCLCPP_INFO(this->rosNode->get_logger(),
        "Resolved %zu LED visuals into %zu unique orb positions",
        refs.size(), this->visualGroups.size());
    }
  }

  private: std::string ReadString(
      const std::shared_ptr<const sdf::Element> &_sdf,
      const std::string &_name,
      const std::string &_default) const
  {
    return _sdf && _sdf->HasElement(_name) ? _sdf->Get<std::string>(_name) : _default;
  }

  private: gz::sim::Model model{gz::sim::kNullEntity};
  private: std::string ledLinkName{"obrik_led_points_link"};
  private: std::string visualPrefix{"obrik_led_point_"};
  private: std::string frameTopic{"/led/sim/frame"};
  private: std::string brightnessTopic{"/led/sim/brightness"};
  private: std::vector<std::vector<gz::sim::Entity>> visualGroups;

  private: std::shared_ptr<rclcpp::Node> rosNode;
  private: std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
  private: rclcpp::Subscription<led_interfaces::msg::LEDStateArray>::SharedPtr frameSub;
  private: rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr brightnessSub;
  private: std::mutex frameMutex;
  private: std::vector<Rgb> logicalFrame;
  private: double brightness{0.01};
  private: bool frameDirty{false};
  private: bool firstFrameApplied{false};
};
}

GZ_ADD_PLUGIN(
  sverk::sim::LedStripVisualization,
  gz::sim::System,
  sverk::sim::LedStripVisualization::ISystemConfigure,
  sverk::sim::LedStripVisualization::ISystemPreUpdate)
