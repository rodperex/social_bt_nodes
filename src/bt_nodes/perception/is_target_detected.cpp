#include "social_bt_nodes/bt_nodes/perception/is_target_detected.hpp"
#include "social_bt_nodes/bt_failure.hpp"
#include "tf2/exceptions.hpp"

namespace social_bt_nodes
{

IsTargetDetected::IsTargetDetected(
  const std::string & condition_name,
  const BT::NodeConfig & conf)
: BT::ConditionNode(condition_name, conf)
{
  // Get ROS node from blackboard
  auto node_any = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  if (!node_any) {
    throw BT::RuntimeError("IsTargetDetected: 'node' not found in blackboard");
  }
  node_ = node_any;
  
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

IsTargetDetected::~IsTargetDetected()
{
}

BT::NodeStatus IsTargetDetected::tick()
{
  const std::string target_frame = "target";
  std::string base_frame;

  double timeout;

  RCLCPP_DEBUG(node_->get_logger(), "IsTargetDetected ticked");

  if (!getInput("base_frame", base_frame)) {
    RCLCPP_ERROR(node_->get_logger(), "Missing required input [base_frame]");
    return bt_failure(config(), registrationName(), "missing required input 'base_frame'", "bt_config_error");
  }

  if (!getInput("timeout", timeout)) {
    timeout = 0.5;
  }

  try {
    // Try to lookup the transform from base to target
    RCLCPP_DEBUG(node_->get_logger(), "Looking up transform from '%s' to '%s' with timeout %.2f seconds",
      base_frame.c_str(), target_frame.c_str(), timeout);
    auto transform = tf_buffer_->lookupTransform(
      base_frame,
      target_frame,
      tf2::TimePointZero,
      tf2::durationFromSec(timeout));

    // Check if transform is recent enough (not stale)
    auto now = node_->get_clock()->now();
    auto transform_time = rclcpp::Time(transform.header.stamp);
    auto age = (now - transform_time).seconds();
    
    if (age > timeout) {
      RCLCPP_WARN(node_->get_logger(), 
        "Target transform is stale (age: %.2f s > timeout: %.2f s)", age, timeout);
      return bt_failure(config(), registrationName(), "target transform is stale");
    }

    RCLCPP_DEBUG(node_->get_logger(), "Target detected at (%f, %f, %f)",
      transform.transform.translation.x,
      transform.transform.translation.y,
      transform.transform.translation.z);

      double angle_to_target = atan2(
      transform.transform.translation.y,
      transform.transform.translation.x) * 180.0 / M_PI;
    RCLCPP_DEBUG(node_->get_logger(), "Angle to target: %.2fº degrees", angle_to_target);

    // Write the detected frame to the output port for downstream nodes
    setOutput("detected_frame", target_frame);
    return BT::NodeStatus::SUCCESS;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_INFO(node_->get_logger(), "Target not detected: %s", ex.what());
    return bt_failure(config(), registrationName(), "NO_REAL_FAILURE");
  }
}

}  // namespace social_bt_nodes
