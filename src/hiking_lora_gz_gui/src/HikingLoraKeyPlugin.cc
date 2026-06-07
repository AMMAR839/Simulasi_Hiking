#include <gz/gui/Plugin.hh>
#include <gz/plugin/Register.hh>

#include <QCoreApplication>
#include <QEvent>
#include <QKeyEvent>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <iomanip>
#include <memory>
#include <sstream>
#include <string>

namespace hiking_lora_gz_gui
{
class HikingLoraKeyPlugin : public gz::gui::Plugin
{
  Q_OBJECT

  public: HikingLoraKeyPlugin()
  {
    this->title = "Hiking LoRa Keyboard";
  }

  public: ~HikingLoraKeyPlugin() override
  {
    if (QCoreApplication::instance())
    {
      QCoreApplication::instance()->removeEventFilter(this);
    }
  }

  protected: void LoadConfig(const tinyxml2::XMLElement *_pluginElem) override
  {
    this->ReadConfig(_pluginElem);
    this->InitRos();

    if (QCoreApplication::instance())
    {
      QCoreApplication::instance()->installEventFilter(this);
    }

    if (this->node)
    {
      RCLCPP_INFO(
        this->node->get_logger(),
        "Gazebo GUI keyboard control ready. Click/focus Gazebo, then use 1-9/0, WASD, R, Q, E.");
    }
  }

  public: bool eventFilter(QObject *_obj, QEvent *_event) override
  {
    if (_event->type() != QEvent::KeyPress)
    {
      return QObject::eventFilter(_obj, _event);
    }

    const auto *keyEvent = static_cast<QKeyEvent *>(_event);
    if (this->HandleKey(keyEvent->key()))
    {
      return true;
    }

    return QObject::eventFilter(_obj, _event);
  }

  private: void ReadConfig(const tinyxml2::XMLElement *_pluginElem)
  {
    if (!_pluginElem)
    {
      return;
    }

    this->manualDuration = this->ReadDouble(_pluginElem, "manual_command_duration_s",
      this->manualDuration);
    this->overviewXyStep = this->ReadDouble(_pluginElem, "overview_camera_xy_step",
      this->overviewXyStep);
    this->overviewZStep = this->ReadDouble(_pluginElem, "overview_camera_z_step",
      this->overviewZStep);
    this->overviewAngleStep = this->ReadDouble(_pluginElem, "overview_camera_angle_step_rad",
      this->overviewAngleStep);
  }

  private: static double ReadDouble(
    const tinyxml2::XMLElement *_pluginElem,
    const char *_name,
    double _defaultValue)
  {
    const auto *elem = _pluginElem->FirstChildElement(_name);
    if (!elem || !elem->GetText())
    {
      return _defaultValue;
    }

    try
    {
      return std::stod(elem->GetText());
    }
    catch (...)
    {
      return _defaultValue;
    }
  }

  private: void InitRos()
  {
    if (!rclcpp::ok())
    {
      int argc = 0;
      char **argv = nullptr;
      rclcpp::init(argc, argv);
    }

    rclcpp::NodeOptions options;
    options.start_parameter_services(false);
    options.start_parameter_event_publisher(false);
    this->node = std::make_shared<rclcpp::Node>("gazebo_keyboard_teleop", options);
    this->manualPub = this->node->create_publisher<std_msgs::msg::String>(
      "/hiker/manual_control", 10);
    this->cameraPub = this->node->create_publisher<std_msgs::msg::String>(
      "/hiker/camera_control", 10);
  }

  private: bool HandleKey(int _key)
  {
    if (_key >= Qt::Key_1 && _key <= Qt::Key_9)
    {
      this->activeHikerIndex = _key - Qt::Key_0;
      this->PublishCamera("follow");
      return true;
    }

    switch (_key)
    {
      case Qt::Key_0:
        this->activeHikerIndex = 10;
        this->PublishCamera("follow");
        return true;
      case Qt::Key_Q:
        this->PublishCamera("follow");
        return true;
      case Qt::Key_E:
        this->PublishCamera("overview");
        return true;
      case Qt::Key_R:
        this->PublishManual("auto", 0.0, 0.0);
        return true;
      case Qt::Key_W:
        this->PublishManual("manual", 1.0, 0.0);
        return true;
      case Qt::Key_S:
        this->PublishManual("manual", -1.0, 0.0);
        return true;
      case Qt::Key_A:
        this->PublishManual("manual", 0.0, 1.0);
        return true;
      case Qt::Key_D:
        this->PublishManual("manual", 0.0, -1.0);
        return true;
      case Qt::Key_I:
        this->PublishCameraAdjust("dy", this->overviewXyStep);
        return true;
      case Qt::Key_K:
        this->PublishCameraAdjust("dy", -this->overviewXyStep);
        return true;
      case Qt::Key_J:
        this->PublishCameraAdjust("dx", -this->overviewXyStep);
        return true;
      case Qt::Key_L:
        this->PublishCameraAdjust("dx", this->overviewXyStep);
        return true;
      case Qt::Key_U:
        this->PublishCameraAdjust("dz", -this->overviewZStep);
        return true;
      case Qt::Key_O:
        this->PublishCameraAdjust("dz", this->overviewZStep);
        return true;
      case Qt::Key_BracketLeft:
        this->PublishCameraAdjust("dyaw", -this->overviewAngleStep);
        return true;
      case Qt::Key_BracketRight:
        this->PublishCameraAdjust("dyaw", this->overviewAngleStep);
        return true;
      case Qt::Key_Minus:
        this->PublishCameraAdjust("dpitch", -this->overviewAngleStep);
        return true;
      case Qt::Key_Equal:
      case Qt::Key_Plus:
        this->PublishCameraAdjust("dpitch", this->overviewAngleStep);
        return true;
      default:
        return false;
    }
  }

  private: std::string TargetHikerId() const
  {
    return "hiker_" + std::to_string(this->activeHikerIndex);
  }

  private: void PublishManual(const std::string &_mode, double _forward, double _turn)
  {
    if (!this->manualPub)
    {
      return;
    }

    std_msgs::msg::String msg;
    std::ostringstream out;
    out << "{\"target_hiker_id\":\"" << this->TargetHikerId()
        << "\",\"mode\":\"" << _mode
        << "\",\"forward\":" << this->Format(_forward)
        << ",\"turn\":" << this->Format(_turn)
        << ",\"duration_s\":" << this->Format(this->manualDuration)
        << "}";
    msg.data = out.str();
    this->manualPub->publish(msg);
  }

  private: void PublishCamera(const std::string &_mode)
  {
    if (!this->cameraPub)
    {
      return;
    }

    std_msgs::msg::String msg;
    msg.data = "{\"target_hiker_id\":\"" + this->TargetHikerId()
      + "\",\"mode\":\"" + _mode + "\"}";
    this->cameraPub->publish(msg);
  }

  private: void PublishCameraAdjust(const std::string &_field, double _value)
  {
    if (!this->cameraPub)
    {
      return;
    }

    std_msgs::msg::String msg;
    msg.data = "{\"target_hiker_id\":\"" + this->TargetHikerId()
      + "\",\"mode\":\"overview_adjust\",\"" + _field + "\":"
      + this->Format(_value) + "}";
    this->cameraPub->publish(msg);
  }

  private: static std::string Format(double _value)
  {
    std::ostringstream out;
    out << std::fixed << std::setprecision(4) << _value;
    return out.str();
  }

  private: int activeHikerIndex{1};
  private: double manualDuration{0.15};
  private: double overviewXyStep{8.0};
  private: double overviewZStep{10.0};
  private: double overviewAngleStep{0.10};
  private: std::shared_ptr<rclcpp::Node> node;
  private: rclcpp::Publisher<std_msgs::msg::String>::SharedPtr manualPub;
  private: rclcpp::Publisher<std_msgs::msg::String>::SharedPtr cameraPub;
};
}  // namespace hiking_lora_gz_gui

GZ_ADD_PLUGIN(
  hiking_lora_gz_gui::HikingLoraKeyPlugin,
  gz::gui::Plugin)

GZ_ADD_PLUGIN_ALIAS(
  hiking_lora_gz_gui::HikingLoraKeyPlugin,
  "HikingLoraKeyPlugin")

#include "HikingLoraKeyPlugin.moc"
