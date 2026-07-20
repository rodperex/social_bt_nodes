#include "social_bt_nodes/bt_nodes/motion/spin_search.hpp"
#include <cmath>

namespace social_bt_nodes
{

SpinSearch::SpinSearch(
  const std::string & action_name,
  const BT::NodeConfig & conf)
: BT::StatefulActionNode(action_name, conf)
{
  // Get ROS node from blackboard
  auto node_any = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  if (!node_any) {
    throw BT::RuntimeError("SpinSearch: 'node' not found in blackboard");
  }
  node_ = node_any;
  
  std::string cmd_vel_topic;
  if (!getInput("cmd_vel_topic", cmd_vel_topic)) {
    cmd_vel_topic = "/cmd_vel";
  }

  // if not / in the topic name, prepend the node's namespace
  if (cmd_vel_topic[0] != '/') {
    cmd_vel_topic = std::string(node_->get_namespace()) + "/" + cmd_vel_topic;
  }
  
  cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(
    cmd_vel_topic, 10);
}

SpinSearch::~SpinSearch()
{
  stop_robot();
}

BT::NodeStatus SpinSearch::onStart()
{
  if (!getInput("angular_speed", angular_speed_)) {
    angular_speed_ = 0.5;
  }
  
  RCLCPP_INFO(node_->get_logger(), 
    "Starting search: Spinning at %.2f rad/s",
    angular_speed_);
  
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SpinSearch::onRunning()
{
  // Just keep spinning - the BT will halt this when target is detected
  RCLCPP_DEBUG_THROTTLE(
    node_->get_logger(), *node_->get_clock(), 1000,
    "Searching for target...");
    
  RCLCPP_INFO(node_->get_logger(), "Spinning at %.2f rad/s", angular_speed_);
  
  auto twist_msg = geometry_msgs::msg::Twist();
  twist_msg.angular.z = angular_speed_;

  RCLCPP_INFO(node_->get_logger(), "Publishing spin command: %.2f rad/s", angular_speed_);
  
  cmd_vel_pub_->publish(twist_msg);
  
  return BT::NodeStatus::RUNNING;
}

void SpinSearch::onHalted()
{
  RCLCPP_INFO(node_->get_logger(), "Search halted");
  
  auto twist_msg = geometry_msgs::msg::Twist();
  twist_msg.angular.z = angular_speed_;

  cmd_vel_pub_->publish(twist_msg);

  // stop_robot();
}

void SpinSearch::stop_robot()
{
  auto twist_msg = geometry_msgs::msg::Twist();
  cmd_vel_pub_->publish(twist_msg);
}

}  // namespace social_bt_nodes
