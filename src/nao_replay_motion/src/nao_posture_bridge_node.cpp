#include <algorithm>
#include <cctype>
#include <functional>
#include <memory>
#include <optional>
#include <sstream>
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
    declare_parameter("connect_on_startup", true);
    declare_parameter("posture_command_topic", "/chatbot/posture_command");
    declare_parameter("posture_result_topic", "/chatbot/posture_command_result");
    declare_parameter("nao_ip", "");
    declare_parameter("nao_port", 9559);
    declare_parameter("posture_speed", 0.8);
    declare_parameter("stand_posture_name", "Stand");
    declare_parameter("kneel_posture_name", "Crouch");
    declare_parameter("stand_speed", 0.8);
    declare_parameter("kneel_speed", 0.8);
    declare_parameter("sit_speed", 0.8);
    declare_parameter("command_dedupe_window_sec", 1.5);
    declare_parameter("disable_autonomous_life_on_connect", false);
    declare_parameter("wake_up_on_connect", false);
    declare_parameter("reconnect_on_failure", true);

    connect_on_startup_ = get_parameter("connect_on_startup").as_bool();
    posture_command_topic_ = get_parameter("posture_command_topic").as_string();
    posture_result_topic_ = get_parameter("posture_result_topic").as_string();
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
    posture_result_publisher_ = create_publisher<std_msgs::msg::String>(posture_result_topic_, 10);

    RCLCPP_INFO(
      get_logger(),
      "nao_posture_bridge ready | command_topic:%s result_topic:%s nao:%s:%d default_speed:%.2f stand:%s@%.2f kneel:%s@%.2f sit@%.2f dedupe:%.2fs connect_on_startup:%s disable_life_on_connect:%s wake_up_on_connect:%s reconnect:%s",
      posture_command_topic_.c_str(),
      posture_result_topic_.c_str(),
      nao_ip_.c_str(),
      static_cast<int>(nao_port_),
      posture_speed_,
      stand_posture_name_.c_str(),
      stand_speed_,
      kneel_posture_name_.c_str(),
      kneel_speed_,
      sit_speed_,
      command_dedupe_window_sec_,
      bool_to_string(connect_on_startup_),
      bool_to_string(disable_autonomous_life_on_connect_),
      bool_to_string(wake_up_on_connect_),
      bool_to_string(reconnect_on_failure_));

    if (connect_on_startup_) {
      (void)connect_session("startup");
    }
  }

private:
  struct ResolvedCommand
  {
    std::string posture_name;
    double posture_speed;
  };

  static const char * bool_to_string(const bool value)
  {
    return value ? "true" : "false";
  }

  static std::string json_escape(const std::string & value)
  {
    std::ostringstream escaped;
    for (const unsigned char ch : value) {
      switch (ch) {
        case '\\':
          escaped << "\\\\";
          break;
        case '"':
          escaped << "\\\"";
          break;
        case '\n':
          escaped << "\\n";
          break;
        case '\r':
          escaped << "\\r";
          break;
        case '\t':
          escaped << "\\t";
          break;
        default:
          if (ch < 0x20) {
            escaped << '?';
          } else {
            escaped << static_cast<char>(ch);
          }
          break;
      }
    }
    return escaped.str();
  }

  void publish_result(
    const std::string & command,
    const std::string & normalized_command,
    const std::string & posture_name,
    const bool success,
    const std::string & message)
  {
    if (!posture_result_publisher_) {
      return;
    }

    std_msgs::msg::String result_msg;
    std::ostringstream payload;
    payload << "{"
            << "\"command\":\"" << json_escape(command) << "\","
            << "\"normalized_command\":\"" << json_escape(normalized_command) << "\","
            << "\"posture_name\":\"" << json_escape(posture_name) << "\","
            << "\"success\":" << (success ? "true" : "false") << ","
            << "\"message\":\"" << json_escape(message) << "\""
            << "}";
    result_msg.data = payload.str();
    posture_result_publisher_->publish(result_msg);
  }

  void reset_connection_state()
  {
    posture_service_ = qi::AnyObject();
    session_.reset();
    connected_ = false;
  }

  bool get_autonomous_life_state(std::string & state_name)
  {
    if (!session_) {
      return false;
    }
    try {
      auto life = session_->service("ALAutonomousLife").value();
      state_name = life.call<std::string>("getState");
      return true;
    } catch (const std::exception & e) {
      RCLCPP_WARN(get_logger(), "Could not read ALAutonomousLife state: %s", e.what());
      return false;
    }
  }

  void log_robot_state_snapshot(const std::string & context)
  {
    std::string posture_name = "unknown";
    std::string autonomous_life_state = "unknown";

    std::string current_posture;
    if (get_current_posture(current_posture)) {
      posture_name = current_posture;
    }

    std::string life_state;
    if (get_autonomous_life_state(life_state)) {
      autonomous_life_state = life_state;
    }

    RCLCPP_INFO(
      get_logger(),
      "NAO state snapshot (%s) | posture:%s autonomous_life:%s",
      context.c_str(),
      posture_name.c_str(),
      autonomous_life_state.c_str());
  }

  void apply_connect_policy()
  {
    bool changed_state = false;

    if (disable_autonomous_life_on_connect_) {
      try {
        auto life = session_->service("ALAutonomousLife").value();
        const std::string current_state = life.call<std::string>("getState");
        if (normalize(current_state) == "disabled") {
          RCLCPP_INFO(get_logger(), "ALAutonomousLife already disabled on connect");
        } else {
          RCLCPP_WARN(
            get_logger(),
            "Disabling ALAutonomousLife on connect because disable_autonomous_life_on_connect=true");
          life.call<void>("setState", std::string("disabled"));
          changed_state = true;
        }
      } catch (const std::exception & e) {
        RCLCPP_WARN(get_logger(), "Could not disable ALAutonomousLife: %s", e.what());
      }
    }

    if (wake_up_on_connect_) {
      try {
        auto motion = session_->service("ALMotion").value();
        RCLCPP_WARN(
          get_logger(),
          "Calling ALMotion.wakeUp on connect because wake_up_on_connect=true");
        motion.call<void>("wakeUp");
        changed_state = true;
      } catch (const std::exception & e) {
        RCLCPP_WARN(get_logger(), "Could not call ALMotion.wakeUp: %s", e.what());
      }
    }

    if (changed_state) {
      log_robot_state_snapshot("after connect policy");
    }
  }

  bool connect_session(const std::string & reason)
  {
    if (nao_ip_.empty()) {
      reset_connection_state();
      RCLCPP_ERROR(
        get_logger(),
        "Parameter 'nao_ip' is empty; cannot connect to NAOqi for %s. Set it from the launch file or with --ros-args -p nao_ip:=...",
        reason.c_str());
      return false;
    }
    const std::string url = "tcp://" + nao_ip_ + ":" + std::to_string(nao_port_);
    reset_connection_state();
    try {
      auto session = qi::makeSession();
      session->connect(url).value();
      auto posture_service = session->service("ALRobotPosture").value();

      session_ = session;
      posture_service_ = posture_service;
      connected_ = true;

      RCLCPP_INFO(
        get_logger(),
        "Connected to NAOqi at %s for %s",
        url.c_str(),
        reason.c_str());
      log_robot_state_snapshot("after connect");
      apply_connect_policy();
      return true;
    } catch (const std::exception & e) {
      reset_connection_state();
      RCLCPP_ERROR(
        get_logger(),
        "Failed to connect to NAOqi (%s) for %s: %s",
        url.c_str(),
        reason.c_str(),
        e.what());
      return false;
    }
  }

  bool ensure_session(const std::string & reason)
  {
    if (connected_) {
      return true;
    }
    return connect_session(reason);
  }

  bool get_current_posture(std::string & posture_name)
  {
    try {
      posture_name = posture_service_.call<std::string>("getPosture");
      return true;
    } catch (const std::exception & e) {
      RCLCPP_WARN(get_logger(), "Could not read current posture: %s", e.what());
      return false;
    }
  }

  bool is_already_in_target_posture(const std::string & posture_name)
  {
    std::string current_posture;
    if (!get_current_posture(current_posture)) {
      return false;
    }
    if (normalize(current_posture) != normalize(posture_name)) {
      return false;
    }
    RCLCPP_INFO(
      get_logger(),
      "Skipping goToPosture because NAO is already in '%s'",
      current_posture.c_str());
    return true;
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

  std::optional<std::string> resolve_posture_name(
    const std::string & normalized_command) const
  {
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
    auto it = direct_map.find(normalized_command);
    if (it != direct_map.end()) {
      return it->second;
    }
    if (normalized_command == "kneel") {
      return kneel_posture_name_;
    }
    if (normalized_command == "stand") {
      return stand_posture_name_;
    }
    return std::nullopt;
  }

  double resolve_posture_speed(const std::string & normalized_command) const
  {
    if (
      normalized_command == "stand" || normalized_command == "standinit" ||
      normalized_command == "standfull" || normalized_command == "standzero")
    {
      return stand_speed_;
    }
    if (normalized_command == "kneel" || normalized_command == "crouch") {
      return kneel_speed_;
    }
    if (normalized_command == "sit" || normalized_command == "sitrelax") {
      return sit_speed_;
    }
    return posture_speed_;
  }

  std::optional<ResolvedCommand> resolve_normalized_command(
    const std::string & normalized_command) const
  {
    const auto posture_name = resolve_posture_name(normalized_command);
    if (!posture_name) {
      return std::nullopt;
    }
    return ResolvedCommand{*posture_name, resolve_posture_speed(normalized_command)};
  }

  bool should_ignore_duplicate_command(
    const std::string & normalized_command,
    const rclcpp::Time & now,
    const std::string & original_command) const
  {
    if (last_command_.empty() || normalized_command != last_command_) {
      return false;
    }

    const double elapsed_sec = (now - last_command_time_).seconds();
    if (elapsed_sec < 0.0 || elapsed_sec >= command_dedupe_window_sec_) {
      return false;
    }

    RCLCPP_WARN(
      get_logger(),
      "Ignored duplicate posture command '%s' within %.2fs window",
      original_command.c_str(),
      command_dedupe_window_sec_);
    return true;
  }

  void remember_command(const std::string & normalized_command, const rclcpp::Time & now)
  {
    last_command_ = normalized_command;
    last_command_time_ = now;
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
      if (!ensure_session("posture command")) {
        return false;
      }
      if (is_already_in_target_posture(posture_name)) {
        return true;
      }
      return run_call();
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Posture call failed: %s", e.what());
      reset_connection_state();
      if (!reconnect_on_failure_) {
        return false;
      }
    }

    RCLCPP_WARN(get_logger(), "Retrying posture call after reconnect");
    try {
      if (!connect_session("posture retry")) {
        return false;
      }
      if (is_already_in_target_posture(posture_name)) {
        return true;
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
    const auto now = get_clock()->now();
    if (should_ignore_duplicate_command(normalized_command, now, command)) {
      publish_result(
        command,
        normalized_command,
        "",
        false,
        "Ignored duplicate posture command within the dedupe window");
      return;
    }

    const auto resolved_command = resolve_normalized_command(normalized_command);
    if (!resolved_command) {
      RCLCPP_WARN(get_logger(), "Unknown posture command: '%s'", command.c_str());
      publish_result(command, normalized_command, "", false, "Unknown posture command");
      return;
    }

    remember_command(normalized_command, now);

    if (!execute_posture(resolved_command->posture_name, resolved_command->posture_speed)) {
      RCLCPP_ERROR(
        get_logger(),
        "Failed to execute posture command '%s' -> '%s'",
        command.c_str(),
        resolved_command->posture_name.c_str());
      publish_result(
        command,
        normalized_command,
        resolved_command->posture_name,
        false,
        "Failed to execute posture command");
      return;
    }

    RCLCPP_INFO(
      get_logger(),
      "Executed posture command '%s' -> '%s' @ %.2f",
      command.c_str(),
      resolved_command->posture_name.c_str(),
      resolved_command->posture_speed);
    publish_result(
      command,
      normalized_command,
      resolved_command->posture_name,
      true,
      "Executed posture command");
  }

  bool connect_on_startup_;
  std::string posture_command_topic_;
  std::string posture_result_topic_;
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
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr posture_subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr posture_result_publisher_;
  qi::AnyObject posture_service_;
  qi::SessionPtr session_;
  bool connected_{false};
  std::string last_command_;
  rclcpp::Time last_command_time_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NaoPostureBridge>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
