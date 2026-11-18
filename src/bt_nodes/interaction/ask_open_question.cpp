#include "social_bt_nodes/bt_nodes/interaction/ask_open_question.hpp"

#include <chrono>

#include "social_bt_nodes/bt_failure.hpp"

namespace social_bt_nodes
{

AskOpenQuestion::AskOpenQuestion(
  const std::string & name,
  const BT::NodeConfig & conf)
: BT::StatefulActionNode(name, conf),
  max_wait_(0.0),
  timeout_ms_(10000),
  phase_(Phase::IDLE)
{
  auto node_any = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  if (!node_any) {
    throw BT::RuntimeError("AskOpenQuestion: 'node' not found in blackboard");
  }
  node_ = node_any;
}

BT::NodeStatus AskOpenQuestion::onStart()
{
  if (!getInput("question", question_)) {
    RCLCPP_ERROR(node_->get_logger(), "AskOpenQuestion: missing required input 'question'");
    return bt_failure(config(), registrationName(), "missing required input 'question'", "bt_config_error");
  }
  if (!getInput("tts_action_name", tts_action_name_)) {
    tts_action_name_ = "/tts_action";
  }
  if (!getInput("stt_action_name", stt_action_name_)) {
    stt_action_name_ = "/stt_action";
  }
  if (!getInput("max_wait", max_wait_)) {
    max_wait_ = 0.0;
  }
  if (!getInput("timeout", timeout_ms_)) {
    timeout_ms_ = 10000;
  }

  if (!say_.connect(node_, tts_action_name_, std::chrono::milliseconds(timeout_ms_))) {
    RCLCPP_WARN(
      node_->get_logger(),
      "AskOpenQuestion: TTS action server '%s' not available yet", tts_action_name_.c_str());
    return bt_failure(config(), registrationName(), "action '" + tts_action_name_ + "' not available");
  }

  RCLCPP_INFO(node_->get_logger(), "AskOpenQuestion: Asking question '%s'", question_.c_str());
  simple_hri_interfaces::action::Say::Goal goal;
  goal.text = question_;
  say_.send(goal);
  phase_ = Phase::SPEAKING;
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus AskOpenQuestion::onRunning()
{
  if (phase_ == Phase::SPEAKING) {
    // Say finishes when the audio has really finished, so listening does not
    // record the robot's own voice
    auto said = say_.poll();
    if (!said) {
      return BT::NodeStatus::RUNNING;
    }
    if (said->code != rclcpp_action::ResultCode::SUCCEEDED || !said->result ||
      !said->result->success)
    {
      const std::string message = said->result ? said->result->message : "goal rejected";
      RCLCPP_ERROR(node_->get_logger(), "AskOpenQuestion: TTS failed: %s", message.c_str());
      return bt_failure(config(), registrationName(), "TTS action failed: " + message);
    }

    if (!listen_.connect(node_, stt_action_name_, std::chrono::milliseconds(timeout_ms_))) {
      RCLCPP_WARN(
        node_->get_logger(),
        "AskOpenQuestion: STT action server '%s' not available yet", stt_action_name_.c_str());
      return bt_failure(config(), registrationName(), "action '" + stt_action_name_ + "' not available");
    }
    simple_hri_interfaces::action::Listen::Goal goal;
    goal.max_wait = static_cast<float>(max_wait_);
    listen_.send(goal);
    phase_ = Phase::LISTENING;
    return BT::NodeStatus::RUNNING;
  }

  if (phase_ == Phase::LISTENING) {
    auto heard = listen_.poll();
    if (!heard) {
      return BT::NodeStatus::RUNNING;
    }
    phase_ = Phase::IDLE;
    if (heard->code != rclcpp_action::ResultCode::SUCCEEDED || !heard->result ||
      !heard->result->success)
    {
      const std::string message = !heard->result ? "goal rejected" :
        (heard->result->timed_out ? "nobody spoke (timeout)" : heard->result->message);
      RCLCPP_ERROR(node_->get_logger(), "AskOpenQuestion: STT failed: %s", message.c_str());
      return bt_failure(config(), registrationName(), "failed to transcribe: " + message);
    }
    setOutput("answer", heard->result->text);
    RCLCPP_INFO(
      node_->get_logger(), "AskOpenQuestion: Captured answer '%s'", heard->result->text.c_str());
    return BT::NodeStatus::SUCCESS;
  }

  return bt_failure(config(), registrationName(), "invalid internal state in AskOpenQuestion");
}

void AskOpenQuestion::onHalted()
{
  RCLCPP_WARN(node_->get_logger(), "AskOpenQuestion: Halted");
  say_.cancel();
  listen_.cancel();
  phase_ = Phase::IDLE;
}

}  // namespace social_bt_nodes
