#ifndef SOCIAL_BT_NODES__BT_NODES__MOTION__NAO_FOLLOW_HPP_
#define SOCIAL_BT_NODES__BT_NODES__MOTION__NAO_FOLLOW_HPP_

#include <memory>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nao_lola_sensor_msgs/msg/sonar.hpp"
#include "nao_lola_sensor_msgs/msg/touch.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"
#include "social_bt_nodes/bt_nodes/pid_controller.hpp"

namespace social_bt_nodes
{

class NaoFollow : public BT::StatefulActionNode
{
public:
  NaoFollow(
    const std::string & action_name,
    const BT::NodeConfig & conf);

  NaoFollow() = delete;

  ~NaoFollow();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

  static constexpr const char * node_description =
    "Makes the robot follow a person using sensor feedback and PID velocity control.";

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("target_frame", "target", "Target TF frame to follow"),
      BT::InputPort<std::string>("base_frame", "base_link", "Base TF frame"),
      BT::InputPort<double>("min_distance", 1.0, "Desired distance to target (m)"),
      BT::InputPort<double>("avoidance_distance", 0.5, "Minimum distance to obstacles (m)"),
      BT::InputPort<double>("max_linear_speed", 0.5, "Maximum linear speed (m/s)"),
      BT::InputPort<double>("max_angular_speed", 1.0, "Maximum angular speed (rad/s)"),
      BT::InputPort<bool>("succeed_on_reach", false, "Return SUCCESS when target is reached"),
      BT::InputPort<std::string>("cmd_vel_topic", "/cmd_vel", "Command velocity topic"),
      BT::InputPort<std::string>("sonar_topic", "/sensors/sonar", "Sonar topic"),
      BT::InputPort<std::string>("touch_topic", "/sensors/touch", "Touch sensor topic"),
      BT::InputPort<double>("rotation_stop_threshold", 0.087, "Angle threshold to stop rotating (rad, ~5 deg)"),
      BT::InputPort<double>("linear_stop_threshold", 0.26, "Angle threshold to stop linear motion (rad, ~15 deg)")
    };
  }

private:
  void stop_robot();
  void sonar_callback(const nao_lola_sensor_msgs::msg::Sonar::SharedPtr msg);
  void touch_callback(const nao_lola_sensor_msgs::msg::Touch::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<nao_lola_sensor_msgs::msg::Sonar>::SharedPtr sonar_sub_;
  rclcpp::Subscription<nao_lola_sensor_msgs::msg::Touch>::SharedPtr touch_sub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  
  double min_distance_;
  double avoidance_distance_;
  double max_linear_speed_;
  double max_angular_speed_;
  double rotation_stop_threshold_;
  double linear_stop_threshold_;
  std::string target_frame_;
  std::string base_frame_;
  bool succeed_on_reach_;
  
  // Obstacle avoidance state
  double vel_rot_avoidance_;
  bool stop_requested_;
  bool danger_;
  bool last_touch_pressed_;
  bool is_rotating_;  // Track if currently in rotation mode for hysteresis
  
  // PID controller for angular velocity
  PIDController angular_pid_;
  rclcpp::Time last_time_;
};

}  // namespace social_bt_nodes

#endif  // SOCIAL_BT_NODES__BT_NODES__MOTION__NAO_FOLLOW_HPP_
