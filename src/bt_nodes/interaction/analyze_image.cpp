#include "social_bt_nodes/bt_nodes/interaction/analyze_image.hpp"

#include <cctype>

namespace social_bt_nodes
{

AnalyzeImage::AnalyzeImage(const std::string & name, const BT::NodeConfig & conf)
: BT::StatefulActionNode(name, conf)
{
  auto node_any = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  if (!node_any) {
    throw BT::RuntimeError("AnalyzeImage: 'node' not found in blackboard");
  }
  node_ = node_any;
}

namespace
{

std::string expandBlackboardText(const std::string & input, const BT::Blackboard::Ptr & blackboard)
{
  std::string output;
  output.reserve(input.size());

  for (std::size_t i = 0; i < input.size();) {
    if (input[i] != '{') {
      output.push_back(input[i++]);
      continue;
    }

    const auto end = input.find('}', i + 1);
    if (end == std::string::npos) {
      output.append(input.substr(i));
      break;
    }

    const auto key = input.substr(i + 1, end - i - 1);
    const bool valid_key = !key.empty() &&
      key.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_./-") ==
      std::string::npos;

    if (!valid_key) {
      output.append(input.substr(i, end - i + 1));
      i = end + 1;
      continue;
    }

    try {
      output += blackboard->get<std::string>(key);
    } catch (...) {
      output.append(input.substr(i, end - i + 1));
    }

    i = end + 1;
  }

  return output;
}

}  // namespace

BT::NodeStatus AnalyzeImage::onStart()
{
  if (!getInput("prompt", prompt_)) {
    RCLCPP_ERROR(node_->get_logger(), "AnalyzeImage: missing required input 'prompt'");
    return BT::NodeStatus::FAILURE;
  }

  prompt_ = expandBlackboardText(prompt_, config().blackboard);

  if (!getInput("service_name", service_name_)) {
    service_name_ = "/analyze_image_service";
  }

  if (!getInput("timeout", timeout_ms_)) {
    timeout_ms_ = 15000;
  }

  if (!client_ || client_->get_service_name() != service_name_) {
    client_ = node_->create_client<simple_hri_interfaces::srv::AnalyzeImage>(service_name_);
  }

  if (!client_->wait_for_service(std::chrono::milliseconds(1000))) {
    RCLCPP_WARN(node_->get_logger(), "AnalyzeImage: Service '%s' not available yet", service_name_.c_str());
    return BT::NodeStatus::FAILURE;
  }

  auto request = std::make_shared<simple_hri_interfaces::srv::AnalyzeImage::Request>();
  request->prompt = prompt_;

  RCLCPP_INFO(node_->get_logger(), "AnalyzeImage: requesting image analysis");
  future_result_ = std::make_shared<
    rclcpp::Client<simple_hri_interfaces::srv::AnalyzeImage>::FutureAndRequestId>(
    client_->async_send_request(request));

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus AnalyzeImage::onRunning()
{
  if (!future_result_) {
    return BT::NodeStatus::FAILURE;
  }

  const auto status = future_result_->wait_for(std::chrono::milliseconds(0));
  if (status == std::future_status::ready) {
    auto result = future_result_->get();
    RCLCPP_INFO(node_->get_logger(), "AnalyzeImage: result: '%s'", result->result.c_str());
    setOutput("result", result->result);
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::RUNNING;
}

void AnalyzeImage::onHalted()
{
  RCLCPP_WARN(node_->get_logger(), "AnalyzeImage: Halted");
  future_result_.reset();
}

}  // namespace social_bt_nodes
