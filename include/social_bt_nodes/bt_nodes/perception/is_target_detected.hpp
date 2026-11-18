#ifndef SOCIAL_BT_NODES__BT_NODES__PERCEPTION__IS_TARGET_DETECTED_HPP_
#define SOCIAL_BT_NODES__BT_NODES__PERCEPTION__IS_TARGET_DETECTED_HPP_

#include <memory>
#include <string>

#include "behaviortree_cpp/condition_node.h"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

namespace social_bt_nodes
{

class IsTargetDetected : public BT::ConditionNode
{
public:
  IsTargetDetected(
    const std::string & condition_name,
    const BT::NodeConfig & conf);

  IsTargetDetected() = delete;

  ~IsTargetDetected();

  BT::NodeStatus tick() override;

  static constexpr const char * node_description =
    "Checks if the TF frame named 'target' is detected using TF transforms.";

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("base_frame", "base_link", "Base TF frame"),
      BT::InputPort<double>("timeout", 0.5, "Time to wait for detection (seconds)"),
      BT::OutputPort<std::string>("detected_frame", "Writes the detected class/frame (same as 'target_frame') to the blackboard.")
    };
  }

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace social_bt_nodes

#endif  // SOCIAL_BT_NODES__BT_NODES__PERCEPTION__IS_TARGET_DETECTED_HPP_
