#ifndef SOCIAL_BT_NODES__BT_NODES__MOTION__MOVE_TOWARDS_HPP_
#define SOCIAL_BT_NODES__BT_NODES__MOTION__MOVE_TOWARDS_HPP_

#include <memory>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"
#include "social_bt_nodes/bt_nodes/pid_controller.hpp"

namespace social_bt_nodes
{

class MoveTowards : public BT::StatefulActionNode
{
public:
  MoveTowards(const std::string & name, const BT::NodeConfig & conf);

  MoveTowards() = delete;

  ~MoveTowards();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

  static constexpr const char * node_description =
    "Moves the robot towards a target frame by computing linear and angular velocities using PID control. "
    "Publishes velocity commands on cmd_vel topic, keeps the action active, and only returns RUNNING.";

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("target_frame", "target", "Target TF frame to move towards"),
      BT::InputPort<std::string>("base_frame", "base_link", "Base TF frame of the robot"),
      BT::InputPort<double>("goal_distance", 1.0, "Desired distance from target (m)"),
      BT::InputPort<double>("max_linear_speed", 0.5, "Maximum linear speed (m/s)"),
      BT::InputPort<double>("max_angular_speed", 1.0, "Maximum angular speed (rad/s)"),
      BT::InputPort<std::string>("cmd_vel_topic", "/cmd_vel", "Command velocity topic"),
      BT::InputPort<double>("linear_kp", 1.0, "Linear PID proportional gain"),
      BT::InputPort<double>("linear_ki", 0.0, "Linear PID integral gain"),
      BT::InputPort<double>("linear_kd", 0.2, "Linear PID derivative gain"),
      BT::InputPort<double>("angular_kp", 1.0, "Angular PID proportional gain"),
      BT::InputPort<double>("angular_ki", 0.0, "Angular PID integral gain"),
      BT::InputPort<double>("angular_kd", 0.3, "Angular PID derivative gain"),
    };
  }

private:
  void stop_robot();

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Configuration parameters
  std::string target_frame_;
  std::string base_frame_;
  std::string cmd_vel_topic_;
  double goal_distance_;
  double max_linear_speed_;
  double max_angular_speed_;

  // PID controllers
  PIDController linear_pid_;
  PIDController angular_pid_;
  rclcpp::Time last_time_;
};

}  // namespace social_bt_nodes

#endif  // SOCIAL_BT_NODES__BT_NODES__MOTION__MOVE_TOWARDS_HPP_
