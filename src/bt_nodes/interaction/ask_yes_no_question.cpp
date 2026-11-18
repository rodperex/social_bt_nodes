#include "social_bt_nodes/bt_nodes/interaction/ask_yes_no_question.hpp"
#include <chrono>
#include "social_bt_nodes/bt_failure.hpp"

namespace social_bt_nodes
{

AskYesNoQuestion::AskYesNoQuestion(
  const std::string & name,
  const BT::NodeConfig & conf)
: BT::StatefulActionNode(name, conf),
  max_wait_(0.0),
  timeout_ms_(10000),
  phase_(Phase::IDLE)
{
  // Get ROS node from blackboard
  auto node_any = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  if (!node_any) {
    throw BT::RuntimeError("AskYesNoQuestion: 'node' not found in blackboard");
  }
  node_ = node_any;
}

BT::NodeStatus AskYesNoQuestion::onStart()
{
  // Preferred input is 'question'. Keep 'text' for backward compatibility.
  if (!getInput("question", question_)) {
    if (!getInput("text", question_)) {
      RCLCPP_ERROR(node_->get_logger(), "AskYesNoQuestion: missing required input 'question'");
      return bt_failure(config(), registrationName(), "missing required input 'question'", "bt_config_error");
    }
    RCLCPP_WARN(node_->get_logger(), "AskYesNoQuestion: input port 'text' is deprecated, use 'question'");
  }

  if (!getInput("tts_action_name", tts_action_name_)) {
    tts_action_name_ = "/tts_action";
  }
  if (!getInput("stt_action_name", stt_action_name_)) {
    stt_action_name_ = "/stt_action";
  }
  if (!getInput("service_name", service_name_)) {
    service_name_ = "/yesno_service";
  }
  if (!getInput("max_wait", max_wait_)) {
    max_wait_ = 0.0;
  }
  if (!getInput("timeout", timeout_ms_)) {
    timeout_ms_ = 10000;
  }

  // Speak the question first, Ask-style.
  if (!say_.connect(node_, tts_action_name_, std::chrono::milliseconds(timeout_ms_))) {
    RCLCPP_WARN(node_->get_logger(),
      "AskYesNoQuestion: TTS action server '%s' not available yet", tts_action_name_.c_str());
    return bt_failure(config(), registrationName(), "action '" + tts_action_name_ + "' not available");
  }
  RCLCPP_INFO(node_->get_logger(), "AskYesNoQuestion: Asking question '%s'", question_.c_str());
  simple_hri_interfaces::action::Say::Goal goal;
  goal.text = question_;
  say_.send(goal);
  phase_ = Phase::SPEAKING;

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus AskYesNoQuestion::onRunning()
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
      RCLCPP_ERROR(node_->get_logger(), "AskYesNoQuestion: TTS failed: %s", message.c_str());
      return bt_failure(config(), registrationName(), "TTS action failed: " + message);
    }

    if (!listen_.connect(node_, stt_action_name_, std::chrono::milliseconds(timeout_ms_))) {
      RCLCPP_WARN(node_->get_logger(),
        "AskYesNoQuestion: STT action server '%s' not available yet", stt_action_name_.c_str());
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
    if (heard->code != rclcpp_action::ResultCode::SUCCEEDED || !heard->result ||
      !heard->result->success)
    {
      const std::string message = !heard->result ? "goal rejected" :
        (heard->result->timed_out ? "nobody spoke (timeout)" : heard->result->message);
      RCLCPP_ERROR(node_->get_logger(), "AskYesNoQuestion: STT failed: %s", message.c_str());
      phase_ = Phase::IDLE;
      return bt_failure(config(), registrationName(), "failed to transcribe: " + message);
    }
    user_answer_ = heard->result->text;
    RCLCPP_INFO(node_->get_logger(), "AskYesNoQuestion: Captured answer '%s'", user_answer_.c_str());

    // Interpreting the answer is a query on the text: the YesNo service
    if (!client_ || client_->get_service_name() != service_name_) {
      client_ = node_->create_client<simple_hri_interfaces::srv::YesNo>(service_name_);
    }
    if (!client_->wait_for_service(std::chrono::milliseconds(timeout_ms_))) {
      RCLCPP_WARN(node_->get_logger(),
        "AskYesNoQuestion: Service '%s' not available yet", service_name_.c_str());
      phase_ = Phase::IDLE;
      return bt_failure(config(), registrationName(), "service '" + service_name_ + "' not available");
    }
    auto request = std::make_shared<simple_hri_interfaces::srv::YesNo::Request>();
    request->text = user_answer_;
    future_result_ = std::make_shared<
      rclcpp::Client<simple_hri_interfaces::srv::YesNo>::FutureAndRequestId>(
      client_->async_send_request(request));
    phase_ = Phase::WAITING_YESNO;
    return BT::NodeStatus::RUNNING;
  }

  if (phase_ == Phase::WAITING_YESNO) {
    if (!future_result_) {
      return bt_failure(config(), registrationName(), "no pending yes/no service future");
    }
    auto status = future_result_->wait_for(std::chrono::milliseconds(0));
    if (status != std::future_status::ready) {
      return BT::NodeStatus::RUNNING;
    }
    auto result = future_result_->get();
    std::string confirmation_result = result->result;
    RCLCPP_INFO(node_->get_logger(),
      "AskYesNoQuestion: Result: '%s'", confirmation_result.c_str());
    setOutput("confirmed", confirmation_result == "YES");
    phase_ = Phase::IDLE;

    if (confirmation_result == "YES") {
      return BT::NodeStatus::SUCCESS;
    }
    RCLCPP_INFO(node_->get_logger(),
      "AskYesNoQuestion: User did not confirm (result: '%s')", confirmation_result.c_str());
    return bt_failure(config(), registrationName(), "NO_REAL_FAILURE");
  }

  return bt_failure(config(), registrationName(), "invalid internal state in AskYesNoQuestion");
}

void AskYesNoQuestion::onHalted()
{
  RCLCPP_WARN(node_->get_logger(), "AskYesNoQuestion: Halted");
  say_.cancel();
  listen_.cancel();
  future_result_.reset();
  user_answer_.clear();
  phase_ = Phase::IDLE;
}

}  // namespace social_bt_nodes
