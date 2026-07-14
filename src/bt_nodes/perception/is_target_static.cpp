#include "social_bt_nodes/bt_nodes/perception/is_target_static.hpp"

#include <cmath>

#include "social_bt_nodes/bt_failure.hpp"
#include "tf2/exceptions.h"

namespace social_bt_nodes
{

IsTargetStatic::IsTargetStatic(const std::string & name, const BT::NodeConfig & conf)
: BT::ConditionNode(name, conf)
{
  auto node_any = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  if (!node_any) {
    throw BT::RuntimeError("IsTargetStatic: 'node' not found in blackboard");
  }
  node_ = node_any;

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

BT::NodeStatus IsTargetStatic::tick()
{
  std::string target_frame = "target";
  getInput("target_frame", target_frame);

  std::string base_frame = "base_link";
  getInput("base_frame", base_frame);

  double min_static_time_sec = 5.0;
  getInput("min_static_time_sec", min_static_time_sec);

  double position_epsilon = 0.05;
  getInput("position_epsilon", position_epsilon);

  double timeout = 0.5;
  getInput("timeout", timeout);

  if (min_static_time_sec < 0.0) {
    return bt_failure(
      config(), registrationName(),
      "min_static_time_sec must be non-negative",
      "bt_config_error");
  }

  if (position_epsilon < 0.0) {
    return bt_failure(
      config(), registrationName(),
      "position_epsilon must be non-negative",
      "bt_config_error");
  }

  if (timeout <= 0.0) {
    return bt_failure(
      config(), registrationName(),
      "timeout must be greater than zero",
      "bt_config_error");
  }

  try {
    // Current position (latest available transform)
    const auto transform_now = tf_buffer_->lookupTransform(
      base_frame, target_frame,
      tf2::TimePointZero,
      tf2::durationFromSec(timeout));

    const auto now = node_->get_clock()->now();
    const auto tf_time = rclcpp::Time(transform_now.header.stamp);
    const auto age = (now - tf_time).seconds();
    if (age > timeout) {
      return bt_failure(
        config(), registrationName(),
        "target transform is stale",
        "bt_tf_stale");
    }

    // Position min_static_time_sec ago
    const auto past_time = tf_time - rclcpp::Duration::from_seconds(min_static_time_sec);
    const tf2::TimePoint past_tp(std::chrono::nanoseconds(past_time.nanoseconds()));

    geometry_msgs::msg::TransformStamped transform_past;
    try {
      transform_past = tf_buffer_->lookupTransform(
        base_frame, target_frame,
        past_tp,
        tf2::durationFromSec(0.0));
    } catch (const tf2::TransformException &) {
      RCLCPP_INFO(
        node_->get_logger(),
        "[%s] Waiting for TF history to build (need %.2f s of data for '%s')",
        registrationName().c_str(), min_static_time_sec, target_frame.c_str());
      return bt_failure(config(), registrationName(), "NO_REAL_FAILURE");
    }

    // Displacement over the full window
    const double dx = transform_now.transform.translation.x - transform_past.transform.translation.x;
    const double dy = transform_now.transform.translation.y - transform_past.transform.translation.y;
    const double dz = transform_now.transform.translation.z - transform_past.transform.translation.z;
    const double displacement = std::sqrt(dx * dx + dy * dy + dz * dz);

    if (displacement <= position_epsilon) {
      return BT::NodeStatus::SUCCESS;
    }

    RCLCPP_INFO(
      node_->get_logger(),
      "[%s] Target is MOVING (displaced %.4f m over last %.2f s, epsilon: %.4f m)",
      registrationName().c_str(), displacement, min_static_time_sec, position_epsilon);
    return bt_failure(config(), registrationName(), "NO_REAL_FAILURE");

  } catch (const tf2::TransformException & ex) {
    return bt_failure(
      config(), registrationName(),
      "failed TF lookup: " + std::string(ex.what()),
      "bt_tf_lookup_failed");
  }
}

}  // namespace social_bt_nodes
