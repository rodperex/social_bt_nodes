#include "social_bt_nodes/bt_nodes/interaction/listen.hpp"
#include "social_bt_nodes/bt_failure.hpp"

namespace social_bt_nodes
{

Listen::Listen(
  const std::string & name,
  const BT::NodeConfig & conf)
: BT::StatefulActionNode(name, conf),
  max_wait_(0.0),
  timeout_ms_(10000)
{
  // Get ROS node from blackboard
  auto node_any = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  if (!node_any) {
    throw BT::RuntimeError("Listen: 'node' not found in blackboard");
  }
  node_ = node_any;
}

BT::NodeStatus Listen::onStart()
{
  // Get input parameters
  if (!getInput("action_name", action_name_)) {
    action_name_ = "/stt_action";
  }

  if (!getInput("max_wait", max_wait_)) {
    max_wait_ = 0.0;
  }

  if (!getInput("timeout", timeout_ms_)) {
    timeout_ms_ = 10000;
  }

  if (!listen_.connect(node_, action_name_, std::chrono::milliseconds(timeout_ms_))) {
    RCLCPP_WARN(node_->get_logger(),
      "Listen: Action server '%s' not available yet", action_name_.c_str());
    return bt_failure(config(), registrationName(), "action '" + action_name_ + "' not available");
  }

  RCLCPP_INFO(node_->get_logger(), "Listen: Starting to listen...");
  simple_hri_interfaces::action::Listen::Goal goal;
  goal.max_wait = static_cast<float>(max_wait_);
  listen_.send(goal);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus Listen::onRunning()
{
  auto result = listen_.poll();
  if (!result) {
    return BT::NodeStatus::RUNNING;
  }

  if (result->code == rclcpp_action::ResultCode::SUCCEEDED && result->result &&
    result->result->success)
  {
    const std::string & transcribed_text = result->result->text;
    RCLCPP_INFO(node_->get_logger(),
      "Listen: Transcribed text: '%s'", transcribed_text.c_str());

    // Set output port with the transcribed text
    setOutput("transcribed_text", transcribed_text);
    return BT::NodeStatus::SUCCESS;
  }

  if (result->result && result->result->timed_out) {
    RCLCPP_WARN(node_->get_logger(), "Listen: Nobody spoke");
    return bt_failure(config(), registrationName(), "nobody spoke (timeout)");
  }

  const std::string message = result->result ? result->result->message : "goal rejected";
  RCLCPP_ERROR(node_->get_logger(), "Listen: Failed to transcribe: %s", message.c_str());
  return bt_failure(config(), registrationName(), "failed to transcribe: " + message);
}

void Listen::onHalted()
{
  RCLCPP_WARN(node_->get_logger(), "Listen: Halted, stopping the recording");
  listen_.cancel();
}

}  // namespace social_bt_nodes
