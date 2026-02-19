#include <algorithm>
#include <cctype>
#include <memory>
#include <string>
#include <unordered_map>

#include <qi/anyobject.hpp>
#include <qi/session.hpp>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class NaoPostureBridge : public rclcpp::Node
{
public:
  NaoPostureBridge()
  : Node("nao_posture_bridge")
  {
    declare_parameter("posture_command_topic", "/chatbot/posture_command");
    declare_parameter("nao_ip", "10.10.200.169");
    declare_parameter("nao_port", 9559);
    declare_parameter("posture_speed", 0.8);
    declare_parameter("stand_posture_name", "Stand");
    declare_parameter("kneel_posture_name", "Crouch");
    declare_parameter("stand_speed", 0.8);
    declare_parameter("kneel_speed", 0.8);
    declare_parameter("sit_speed", 0.8);
    declare_parameter("command_dedupe_window_sec", 1.5);
    declare_parameter("disable_autonomous_life_on_connect", true);
    declare_parameter("wake_up_on_connect", true);
    declare_parameter("reconnect_on_failure", true);

    posture_command_topic_ = get_parameter("posture_command_topic").as_string();
    nao_ip_ = get_parameter("nao_ip").as_string();
    nao_port_ = get_parameter("nao_port").as_int();
    posture_speed_ = get_parameter("posture_speed").as_double();
    stand_posture_name_ = get_parameter("stand_posture_name").as_string();
    kneel_posture_name_ = get_parameter("kneel_posture_name").as_string();
    stand_speed_ = get_parameter("stand_speed").as_double();
    kneel_speed_ = get_parameter("kneel_speed").as_double();
    sit_speed_ = get_parameter("sit_speed").as_double();
    command_dedupe_window_sec_ = get_parameter("command_dedupe_window_sec").as_double();
    disable_autonomous_life_on_connect_ =
      get_parameter("disable_autonomous_life_on_connect").as_bool();
    wake_up_on_connect_ = get_parameter("wake_up_on_connect").as_bool();
    reconnect_on_failure_ = get_parameter("reconnect_on_failure").as_bool();

    posture_subscription_ = create_subscription<std_msgs::msg::String>(
      posture_command_topic_,
      10,
      std::bind(&NaoPostureBridge::on_posture_command, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "nao_posture_bridge ready | topic:%s nao:%s:%d default_speed:%.2f stand:%s@%.2f kneel:%s@%.2f sit@%.2f dedupe:%.2fs",
      posture_command_topic_.c_str(),
      nao_ip_.c_str(),
      static_cast<int>(nao_port_),
      posture_speed_,
      stand_posture_name_.c_str(),
      stand_speed_,
      kneel_posture_name_.c_str(),
      kneel_speed_,
      sit_speed_,
      command_dedupe_window_sec_);

    (void)connect_session();
  }

private:
  bool connect_session()
  {
    const std::string url = "tcp://" + nao_ip_ + ":" + std::to_string(nao_port_);
    try {
      session_ = qi::makeSession();
      session_->connect(url).value();
      posture_service_ = session_->service("ALRobotPosture").value();

      if (disable_autonomous_life_on_connect_) {
        try {
          auto life = session_->service("ALAutonomousLife").value();
          life.call<void>("setState", std::string("disabled"));
        } catch (const std::exception & e) {
          RCLCPP_WARN(get_logger(), "Could not disable ALAutonomousLife: %s", e.what());
        }
      }
      if (wake_up_on_connect_) {
        try {
          auto motion = session_->service("ALMotion").value();
          motion.call<void>("wakeUp");
        } catch (const std::exception & e) {
          RCLCPP_WARN(get_logger(), "Could not call ALMotion.wakeUp: %s", e.what());
        }
      }

      RCLCPP_INFO(get_logger(), "Connected to NAOqi at %s", url.c_str());
      return true;
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Failed to connect to NAOqi (%s): %s", url.c_str(), e.what());
      return false;
    }
  }

  static std::string normalize(const std::string & text)
  {
    std::string out = text;
    std::transform(
      out.begin(),
      out.end(),
      out.begin(),
      [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    out.erase(std::remove_if(out.begin(), out.end(), [](unsigned char c) { return std::isspace(c); }), out.end());
    return out;
  }

  bool resolve_command(
    const std::string & command,
    std::string & posture_name,
    double & posture_speed) const
  {
    const std::string normalized = normalize(command);
    static const std::unordered_map<std::string, std::string> direct_map = {
      {"sit", "Sit"},
      {"crouch", "Crouch"},
      {"sitrelax", "SitRelax"},
      {"standinit", "StandInit"},
      {"standfull", "Stand"},
      {"standzero", "StandZero"},
      {"lyingback", "LyingBack"},
      {"lyingbelly", "LyingBelly"},
    };
    auto it = direct_map.find(normalized);
    if (it != direct_map.end()) {
      posture_name = it->second;
    } else if (normalized == "kneel") {
      posture_name = kneel_posture_name_;
    } else if (normalized == "stand") {
      posture_name = stand_posture_name_;
    } else {
      return false;
    }

    if (
      normalized == "stand" || normalized == "standinit" || normalized == "standfull" ||
      normalized == "standzero")
    {
      posture_speed = stand_speed_;
      return true;
    }
    if (normalized == "kneel" || normalized == "crouch") {
      posture_speed = kneel_speed_;
      return true;
    }
    if (normalized == "sit" || normalized == "sitrelax") {
      posture_speed = sit_speed_;
      return true;
    }

    posture_speed = posture_speed_;
    return true;
  }

  bool execute_posture(const std::string & posture_name, const double posture_speed)
  {
    auto run_call = [&]() -> bool {
        bool ok = posture_service_.call<bool>("goToPosture", posture_name, posture_speed);
        if (!ok) {
          RCLCPP_WARN(
            get_logger(),
            "ALRobotPosture.goToPosture(%s, %.2f) returned false",
            posture_name.c_str(),
            posture_speed);
        }
        return ok;
      };

    try {
      if (!session_) {
        if (!connect_session()) {
          return false;
        }
      }
      return run_call();
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Posture call failed: %s", e.what());
      if (!reconnect_on_failure_) {
        return false;
      }
    }

    RCLCPP_WARN(get_logger(), "Retrying posture call after reconnect");
    try {
      if (!connect_session()) {
        return false;
      }
      return run_call();
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Posture retry failed: %s", e.what());
      return false;
    }
  }

  void on_posture_command(const std_msgs::msg::String::SharedPtr msg)
  {
    const std::string command = msg->data;
    if (command.empty()) {
      return;
    }
    const std::string normalized_command = normalize(command);
    const auto now = this->get_clock()->now();
    if (!last_command_.empty() && normalized_command == last_command_) {
      const double elapsed_sec = (now - last_command_time_).seconds();
      if (elapsed_sec >= 0.0 && elapsed_sec < command_dedupe_window_sec_) {
        RCLCPP_WARN(
          get_logger(),
          "Ignored duplicate posture command '%s' within %.2fs window",
          command.c_str(),
          command_dedupe_window_sec_);
        return;
      }
    }

    std::string posture_name;
    double posture_speed = posture_speed_;
    if (!resolve_command(command, posture_name, posture_speed)) {
      RCLCPP_WARN(get_logger(), "Unknown posture command: '%s'", command.c_str());
      return;
    }

    const bool ok = execute_posture(posture_name, posture_speed);
    if (ok) {
      last_command_ = normalized_command;
      last_command_time_ = now;
      RCLCPP_INFO(
        get_logger(),
        "Executed posture command '%s' -> goToPosture('%s', %.2f)",
        command.c_str(),
        posture_name.c_str(),
        posture_speed);
    } else {
      RCLCPP_ERROR(get_logger(), "Failed to execute posture command '%s'", command.c_str());
    }
  }

  std::string posture_command_topic_;
  std::string nao_ip_;
  int64_t nao_port_;
  double posture_speed_;
  std::string stand_posture_name_;
  std::string kneel_posture_name_;
  double stand_speed_;
  double kneel_speed_;
  double sit_speed_;
  double command_dedupe_window_sec_;
  bool disable_autonomous_life_on_connect_;
  bool wake_up_on_connect_;
  bool reconnect_on_failure_;
  std::string last_command_;
  rclcpp::Time last_command_time_;

  qi::SessionPtr session_;
  qi::AnyObject posture_service_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr posture_subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NaoPostureBridge>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
