#include <chrono>
#include <string>

#include <QCoreApplication>

#include <gz/gui/GuiEvents.hh>
#include <gz/gui/Plugin.hh>
#include <gz/msgs/cameratrack.pb.h>
#include <gz/msgs/pose.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/rendering/Camera.hh>
#include <gz/rendering/Node.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/Scene.hh>
#include <gz/transport/Node.hh>

namespace helsinki_stream
{
class GuiPosePublisher final : public gz::gui::Plugin
{
  public: ~GuiPosePublisher() override
  {
    if (auto *app = QCoreApplication::instance())
      app->removeEventFilter(this);
  }

  protected: void LoadConfig(const tinyxml2::XMLElement *_pluginElem) override
  {
    if (_pluginElem)
    {
      if (const auto *topic = _pluginElem->FirstChildElement("topic"); topic && topic->GetText())
        this->topic = topic->GetText();
      if (const auto *gain = _pluginElem->FirstChildElement("tracking_gain"); gain && gain->GetText())
        this->trackingGain = std::stod(gain->GetText());
    }
    this->posePublisher = this->transport.Advertise<gz::msgs::Pose>(this->topic);
    this->trackingPublisher =
        this->transport.Advertise<gz::msgs::CameraTrack>("/gui/track");
    if (auto *app = QCoreApplication::instance())
      app->installEventFilter(this);
  }

  protected: bool eventFilter(QObject *_watched, QEvent *_event) override
  {
    if (_event->type() == gz::gui::events::Render::kType)
    {
      this->PublishCameraPose();
    }
    return gz::gui::Plugin::eventFilter(_watched, _event);
  }

  private: void PublishCameraPose()
  {
    const auto now = std::chrono::steady_clock::now();
    if (now - this->lastPublish < std::chrono::milliseconds(100))
      return;
    this->lastPublish = now;

    // CameraTracking defaults both gains to 0.01, which can lag behind a
    // moving aircraft. Configure both through its public topic once the
    // plugin has subscribed. USE_LAST preserves the user's selected target.
    if (!this->trackingGainPublished && this->trackingPublisher.HasConnections())
    {
      gz::msgs::CameraTrack message;
      message.set_track_mode(gz::msgs::CameraTrack::USE_LAST);
      message.set_track_pgain(this->trackingGain);
      message.set_follow_pgain(this->trackingGain);
      this->trackingPublisher.Publish(message);
      this->trackingGainPublished = true;
    }

    // MinimalScene creates the interactive camera under this stable name.
    // Render events run in the GUI render thread, where rendering calls are safe.
    const auto scene = gz::rendering::sceneFromFirstRenderEngine();
    if (!scene)
      return;
    auto camera = scene->NodeByName("user-camera");
    if (!camera)
    {
      // Gazebo Sim 8's MinimalScene gives the interactive camera a generated
      // name (for example "scene::Camera(65527)"). It is the only Camera
      // node in the GUI scene, so locate it by type instead of by that name.
      for (unsigned int i = 0; i < scene->NodeCount(); ++i)
      {
        const auto node = scene->NodeByIndex(i);
        if (node && std::dynamic_pointer_cast<gz::rendering::Camera>(node))
        {
          camera = node;
          break;
        }
      }
    }
    if (!camera)
      return;

    const auto pose = camera->WorldPose();
    gz::msgs::Pose message;
    message.mutable_position()->set_x(pose.Pos().X());
    message.mutable_position()->set_y(pose.Pos().Y());
    message.mutable_position()->set_z(pose.Pos().Z());
    message.mutable_orientation()->set_w(pose.Rot().W());
    message.mutable_orientation()->set_x(pose.Rot().X());
    message.mutable_orientation()->set_y(pose.Rot().Y());
    message.mutable_orientation()->set_z(pose.Rot().Z());
    this->posePublisher.Publish(message);
  }

  private: std::string topic{ "/helsinki_stream/gui_pose" };
  private: double trackingGain{0.1};
  private: bool trackingGainPublished{false};
  private: gz::transport::Node transport;
  private: gz::transport::Node::Publisher posePublisher;
  private: gz::transport::Node::Publisher trackingPublisher;
  private: std::chrono::steady_clock::time_point lastPublish{};
};
}

GZ_ADD_PLUGIN(helsinki_stream::GuiPosePublisher, gz::gui::Plugin)
GZ_ADD_PLUGIN_ALIAS(helsinki_stream::GuiPosePublisher, "helsinki_stream::GuiPosePublisher")
