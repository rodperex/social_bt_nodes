#include "social_bt_nodes/bt_nodes/interaction/speak.hpp"
#include "social_bt_nodes/bt_failure.hpp"

#include <cctype>

namespace
{

std::string trim_copy(const std::string & value)
{
  std::size_t start = 0;
  while (start < value.size() && std::isspace(static_cast<unsigned char>(value[start]))) {
    ++start;
  }
  std::size_t end = value.size();
  while (end > start && std::isspace(static_cast<unsigned char>(value[end - 1]))) {
    --end;
  }
  return value.substr(start, end - start);
}

bool resolve_blackboard_template(
  const std::string & raw,
  const BT::Blackboard::Ptr & blackboard,
  std::string & resolved,
  std::string & error)
{
  resolved.clear();
  bool saw_placeholder = false;
  std::size_t pos = 0;

  while (pos < raw.size()) {
    const std::size_t open = raw.find('{', pos);
    if (open == std::string::npos) {
      resolved += raw.substr(pos);
      break;
    }

    resolved += raw.substr(pos, open - pos);
    const std::size_t close = raw.find('}', open + 1);
    if (close == std::string::npos) {
      error = "unmatched '{' in text template: '" + raw + "'";
      return false;
    }

    const std::string key = trim_copy(raw.substr(open + 1, close - open - 1));
    if (key.empty()) {
      error = "empty blackboard key in text template: '" + raw + "'";
      return false;
    }

    try {
      resolved += blackboard->get<std::string>(key);
    } catch (const std::exception & e) {
      error = "blackboard key '" + key + "' unavailable in text template: " + e.what();
      return false;
    }

    saw_placeholder = true;
    pos = close + 1;
  }

  if (!saw_placeholder) {
    error = "template contains no blackboard placeholders";
    return false;
  }
  return true;
}

}  // namespace

namespace social_bt_nodes
{

Speak::Speak(
  const std::string & name,
  const BT::NodeConfig & conf)
: BT::StatefulActionNode(name, conf),
  timeout_ms_(5000)
{
  // Get ROS node from blackboard
  auto node_any = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  if (!node_any) {
    throw BT::RuntimeError("Speak: 'node' not found in blackboard");
  }
  node_ = node_any;
}

BT::NodeStatus Speak::onStart()
{
  // Read input
  if (!getInput("text", text_)) {
    auto input_it = config().input_ports.find("text");
    if (input_it != config().input_ports.end()) {
      text_ = input_it->second;
    } else {
      RCLCPP_ERROR(node_->get_logger(), "Speak: missing required input 'text'");
      return bt_failure(config(), registrationName(), "missing required input 'text'", "bt_config_error");
    }
  }

  if (!getInput("action_name", action_name_)) {
    action_name_ = "/tts_action";
  }

  if (!getInput("timeout", timeout_ms_)) {
    timeout_ms_ = 5000;
  }

  // Resolve {} placeholders in text
  std::string resolved_text;
  std::string resolve_error;
  if (resolve_blackboard_template(text_, config().blackboard, resolved_text, resolve_error)) {
    text_ = resolved_text;
  } else {
    if (!resolve_error.empty() && resolve_error.find("no blackboard placeholders") == std::string::npos) {
      RCLCPP_WARN(node_->get_logger(), "Speak: template resolution warning: %s", resolve_error.c_str());
    }
  }

  if (!say_.connect(node_, action_name_, std::chrono::milliseconds(timeout_ms_))) {
    RCLCPP_WARN(node_->get_logger(),
      "Speak: Action server '%s' not available yet", action_name_.c_str());
    return bt_failure(config(), registrationName(), "action '" + action_name_ + "' not available");
  }

  RCLCPP_INFO(node_->get_logger(), "Speak: Speaking '%s'", text_.c_str());
  simple_hri_interfaces::action::Say::Goal goal;
  goal.text = text_;
  say_.send(goal);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus Speak::onRunning()
{
  // The Say action finishes when the audio has really finished
  auto result = say_.poll();
  if (!result) {
    return BT::NodeStatus::RUNNING;
  }

  if (result->code != rclcpp_action::ResultCode::SUCCEEDED || !result->result ||
    !result->result->success)
  {
    const std::string message = result->result ? result->result->message : "goal rejected";
    RCLCPP_ERROR(node_->get_logger(), "Speak: Speech failed: %s", message.c_str());
    return bt_failure(config(), registrationName(), "speech action failed: " + message);
  }

  RCLCPP_INFO(node_->get_logger(), "Speak: Speech completed");
  return BT::NodeStatus::SUCCESS;
}

void Speak::onHalted()
{
  RCLCPP_WARN(node_->get_logger(), "Speak: Halted, stopping the audio");
  say_.cancel();
}

}  // namespace social_bt_nodes
