#include "social_bt_nodes/bt_nodes/motion/move_towards.hpp"

#include <cmath>

#include "social_bt_nodes/bt_failure.hpp"
#include "tf2/exceptions.h"

namespace social_bt_nodes
{

MoveTowards::MoveTowards(const std::string & name, const BT::NodeConfig & conf)
: BT::StatefulActionNode(name, conf),
  linear_pid_(1.0, 0.0, 0.2),
  angular_pid_(1.0, 0.0, 0.3)
{
  // Get ROS node from blackboard
  auto node_any = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  if (!node_any) {
    throw BT::RuntimeError("MoveTowards: 'node' not found in blackboard");
  }
  node_ = node_any;

  // Create TF2 buffer and listener
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

MoveTowards::~MoveTowards()
{
  stop_robot();
}

BT::NodeStatus MoveTowards::onStart()
{
  // Get input parameters
  if (!getInput("target_frame", target_frame_)) {
    target_frame_ = "target";
  }
  if (!getInput("base_frame", base_frame_)) {
    base_frame_ = "base_link";
  }
  if (!getInput("goal_distance", goal_distance_)) {
    goal_distance_ = 1.0;
  }
  if (!getInput("max_linear_speed", max_linear_speed_)) {
    max_linear_speed_ = 0.5;
  }
  if (!getInput("max_angular_speed", max_angular_speed_)) {
    max_angular_speed_ = 1.0;
  }
  if (!getInput("cmd_vel_topic", cmd_vel_topic_)) {
    cmd_vel_topic_ = "/cmd_vel";
  }

  if (goal_distance_ < 0.0) {
    RCLCPP_ERROR(node_->get_logger(), "MoveTowards: 'goal_distance' must be non-negative");
    goal_distance_ = 0.0;
  }
  if (max_linear_speed_ <= 0.0) {
    RCLCPP_ERROR(node_->get_logger(), "MoveTowards: 'max_linear_speed' must be greater than zero");
    max_linear_speed_ = 0.5;
  }
  if (max_angular_speed_ <= 0.0) {
    RCLCPP_ERROR(node_->get_logger(), "MoveTowards: 'max_angular_speed' must be greater than zero");
    max_angular_speed_ = 1.0;
  }

  cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);

  // Get PID gains
  double linear_kp, linear_ki, linear_kd;
  double angular_kp, angular_ki, angular_kd;

  if (!getInput("linear_kp", linear_kp)) {
    linear_kp = 1.0;
  }
  if (!getInput("linear_ki", linear_ki)) {
    linear_ki = 0.0;
  }
  if (!getInput("linear_kd", linear_kd)) {
    linear_kd = 0.2;
  }
  if (!getInput("angular_kp", angular_kp)) {
    angular_kp = 1.0;
  }
  if (!getInput("angular_ki", angular_ki)) {
    angular_ki = 0.0;
  }
  if (!getInput("angular_kd", angular_kd)) {
    angular_kd = 0.3;
  }

  // Set PID gains
  linear_pid_.setGains(linear_kp, linear_ki, linear_kd);
  angular_pid_.setGains(angular_kp, angular_ki, angular_kd);

  // Set output limits for PIDs
  linear_pid_.setOutputLimits(0.0, max_linear_speed_);
  angular_pid_.setOutputLimits(-max_angular_speed_, max_angular_speed_);

  // Reset PID states
  linear_pid_.reset();
  angular_pid_.reset();

  last_time_ = node_->now();

  RCLCPP_INFO(node_->get_logger(),
    "MoveTowards: Starting to move towards '%s' (goal distance: %.2f m)",
    target_frame_.c_str(), goal_distance_);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus MoveTowards::onRunning()
{
  try {
    // Look up transform from base to target
    auto transform = tf_buffer_->lookupTransform(
      base_frame_,
      target_frame_,
      tf2::TimePointZero,
      tf2::durationFromSec(0.5));

    // Extract position
    double target_x = transform.transform.translation.x;
    double target_y = transform.transform.translation.y;

    // Calculate distance and angle
    double distance = std::sqrt(target_x * target_x + target_y * target_y);
    double angle = std::atan2(target_y, target_x);

    RCLCPP_DEBUG(node_->get_logger(),
      "Target at distance: %.2f m, angle: %.2f deg",
      distance, angle * 180.0 / M_PI);

    // Keep action active even at goal distance
    if (distance <= goal_distance_) {
      RCLCPP_INFO(node_->get_logger(),
        "Goal distance reached (distance: %.2f m <= goal: %.2f m), holding position",
        distance, goal_distance_);
      stop_robot();
      return BT::NodeStatus::SUCCESS;
    }

    // Calculate velocities using PID control
    rclcpp::Time current_time = node_->now();
    double dt = (current_time - last_time_).seconds();

    if (dt <= 0.0) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
        "Invalid time step (dt=%.3f), skipping PID computation", dt);
      return BT::NodeStatus::RUNNING;
    }

    // Angular velocity - PID control on angle error
    double vel_ang = angular_pid_.compute(angle, dt);

    // Linear velocity - PID control on distance error
    // Error is positive when we need to move forward (distance > goal_distance_)
    double distance_error = distance - goal_distance_;
    double vel_lin = linear_pid_.compute(distance_error, dt);

    // Ensure linear velocity is positive (only move forward)
    vel_lin = std::max(0.0, vel_lin);

    // Reduce linear speed when turning sharply
    double turn_ratio = std::abs(vel_ang) / max_angular_speed_;
    vel_lin = vel_lin * (1.0 - 0.5 * turn_ratio);

    // Update time
    last_time_ = current_time;

    // Publish velocity command
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = vel_lin;
    cmd.angular.z = vel_ang;
    cmd_vel_pub_->publish(cmd);

    RCLCPP_DEBUG(node_->get_logger(),
      "Publishing velocity: linear=%.3f m/s, angular=%.3f rad/s",
      vel_lin, vel_ang);

    return BT::NodeStatus::RUNNING;

  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(node_->get_logger(), "MoveTowards: TF transform failed: %s", ex.what());
    stop_robot();
    return BT::NodeStatus::RUNNING;
  }
}

void MoveTowards::onHalted()
{
  stop_robot();
  RCLCPP_INFO(node_->get_logger(), "MoveTowards: Halted");
}

void MoveTowards::stop_robot()
{
  if (!cmd_vel_pub_) {
    return;
  }

  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = 0.0;
  cmd.angular.z = 0.0;
  cmd_vel_pub_->publish(cmd);
}

}  // namespace social_bt_nodes
