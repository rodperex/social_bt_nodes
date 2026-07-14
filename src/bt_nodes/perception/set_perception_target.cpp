#include "social_bt_nodes/bt_nodes/perception/set_perception_target.hpp"
#include "social_bt_nodes/bt_failure.hpp"

namespace social_bt_nodes
{

SetPerceptionTarget::SetPerceptionTarget(
  const std::string & name,
  const BT::NodeConfig & conf)
: BT::StatefulActionNode(name, conf)
{
  // Get ROS node from blackboard
  auto node_any = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  if (!node_any) {
    throw BT::RuntimeError("SetPerceptionTarget: 'node' not found in blackboard");
  }
  node_ = node_any;
}

BT::NodeStatus SetPerceptionTarget::onStart()
{
  service_name_ = "/set_perception_target";
  timeout_ms_ = 2000;

  if (!getInput("service_name", service_name_) || service_name_.empty()) {
    service_name_ = "/set_perception_target";
  }
  
  if (!getInput("target", target_class_)) {
    RCLCPP_ERROR(node_->get_logger(), 
      "SetPerceptionTarget: missing required input 'target'");
    return bt_failure(config(), registrationName(), "missing required input 'target'", "bt_config_error");
  }

  if (!getInput("target_frame", target_frame_)) {
    RCLCPP_ERROR(node_->get_logger(), 
      "SetPerceptionTarget: missing required input 'target_frame'");
    return bt_failure(config(), registrationName(), "missing required input 'target_frame'", "bt_config_error");
  }
  
  RCLCPP_INFO(node_->get_logger(), 
    "SetPerceptionTarget: Attempting to connect to service '%s'", service_name_.c_str());
  
  client_ = node_->create_client<simple_perception_interfaces::srv::SetTargetClass>(
    service_name_);
  
  // Wait for service to be available
  if (!client_->wait_for_service(std::chrono::milliseconds(timeout_ms_))) {
    RCLCPP_ERROR(node_->get_logger(), 
      "SetPerceptionTarget: Service '%s' not available after %d ms", 
      service_name_.c_str(), timeout_ms_);
    return bt_failure(config(), registrationName(), "service '" + service_name_ + "' not available");
  }
  
  RCLCPP_INFO(node_->get_logger(), 
    "SetPerceptionTarget: Service '%s' found", service_name_.c_str());
  
  // Prepare request
  auto request = std::make_shared<simple_perception_interfaces::srv::SetTargetClass::Request>();
  request->target_class = target_class_;
  
  RCLCPP_INFO(node_->get_logger(), 
    "SetPerceptionTarget: Setting target class to '%s'", target_class_.c_str());
  
  // Send request
  future_result_ = std::make_shared<
    rclcpp::Client<simple_perception_interfaces::srv::SetTargetClass>::FutureAndRequestId>(
    client_->async_send_request(request));
  
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SetPerceptionTarget::onRunning()
{
  if (!future_result_) {
    return bt_failure(config(), registrationName(), "no pending service future");
  }
  
  auto status = future_result_->wait_for(std::chrono::milliseconds(0));
  
  if (status == std::future_status::ready) {
    auto result = future_result_->get();
    
    // Check if service call was successful
    if (result->success) {
      RCLCPP_INFO(node_->get_logger(), 
        "SetPerceptionTarget: Successfully set target class to '%s': %s", 
        target_class_.c_str(), result->message.c_str());
      setOutput("frame_id", "target"); // talk to rod
      return BT::NodeStatus::SUCCESS;
    } else {
      RCLCPP_ERROR(node_->get_logger(), 
        "SetPerceptionTarget: Failed to set target class: %s", result->message.c_str());
      return bt_failure(config(), registrationName(), "failed to set target class: " + result->message);
    }
  }
  
  return BT::NodeStatus::RUNNING;
}

void SetPerceptionTarget::onHalted()
{
  RCLCPP_WARN(node_->get_logger(), "SetPerceptionTarget: Halted");
  future_result_.reset();
}

}  // namespace social_bt_nodes
