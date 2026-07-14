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
  waiting_for_speech_completion_(false)
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
  
  if (!getInput("service_name", service_name_)) {
    service_name_ = "/tts_service";
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
  
  // Create service client if not already created or if service name changed
  if (!client_ || client_->get_service_name() != service_name_) {
    client_ = node_->create_client<simple_hri_interfaces::srv::Speech>(service_name_);
  }
  
  // Wait for service to be available
  if (!client_->wait_for_service(std::chrono::milliseconds(timeout_ms_))) {
    RCLCPP_WARN(node_->get_logger(), 
      "Speak: Service '%s' not available yet", service_name_.c_str());
    return bt_failure(config(), registrationName(), "service '" + service_name_ + "' not available");
  }
  
  // Prepare and send request
  auto request = std::make_shared<simple_hri_interfaces::srv::Speech::Request>();
  request->text = text_;
  
  RCLCPP_INFO(node_->get_logger(), "Speak: Speaking '%s'", text_.c_str());
  
  future_result_ = std::make_shared<
    rclcpp::Client<simple_hri_interfaces::srv::Speech>::FutureAndRequestId>(
    client_->async_send_request(request));
  
  waiting_for_speech_completion_ = false;
  
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus Speak::onRunning()
{
  // First, wait for service call to complete
  // if (!waiting_for_speech_completion_) {
    if (!future_result_) {
      return bt_failure(config(), registrationName(), "no pending service future");
    }
    
    auto status = future_result_->wait_for(std::chrono::milliseconds(0));
    
    if (status == std::future_status::ready) {
      auto result = future_result_->get();
      
      if (!result->success) {
        RCLCPP_ERROR(node_->get_logger(), 
        "Speak: Speech failed: %s", result->debug.c_str());
        return bt_failure(config(), registrationName(), "speech service failed: " + result->debug);
      }
      
      // Service call succeeded, now calculate speech duration
      // Estimate: ~150 words per minute for Spanish speech
      // Average word length: ~5 characters
      // So roughly 750 characters per minute, or 12.5 chars per second
      // Formula: duration (ms) = text_length * 80 (ms per character)
      // This gives approximately 12.5 chars/sec or 150 wpm
      int text_length = text_.length();
      int duration_ms = text_length * 80 + 500;  // +500ms for padding
      
      speech_duration_ = std::chrono::milliseconds(duration_ms);
      
      std::this_thread::sleep_for(speech_duration_); // todo(juandpenan): remove the sleep
      
      // speech_start_time_ = std::chrono::steady_clock::now();
      // waiting_for_speech_completion_ = true;
      
      // RCLCPP_INFO(node_->get_logger(), 
      //   "Speak: TTS service responded, waiting %d ms for speech completion", 
      //   duration_ms);
      // }
      
      return BT::NodeStatus::SUCCESS;
    }
  
  // // Now wait for the calculated speech duration to elapse
  // auto elapsed = std::chrono::steady_clock::now() - speech_start_time_;
  
  // if (elapsed >= speech_duration_) {
  //   RCLCPP_INFO(node_->get_logger(), "Speak: Speech completed");
  //   return BT::NodeStatus::SUCCESS;
  // }
  
  return BT::NodeStatus::RUNNING; // running
}

void Speak::onHalted()
{
  RCLCPP_WARN(node_->get_logger(), "Speak: Halted");
  future_result_.reset();
}

}  // namespace social_bt_nodes
