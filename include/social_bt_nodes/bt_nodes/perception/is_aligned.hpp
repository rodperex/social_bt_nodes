#ifndef SOCIAL_BT_NODES__BT_NODES__PERCEPTION__IS_ALIGNED_HPP_
#define SOCIAL_BT_NODES__BT_NODES__PERCEPTION__IS_ALIGNED_HPP_

#include <memory>
#include <string>

#include "behaviortree_cpp/condition_node.h"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

namespace social_bt_nodes
{

class IsAligned : public BT::ConditionNode
{
public:
  IsAligned(const std::string & name, const BT::NodeConfig & conf);

  BT::NodeStatus tick() override;

  static constexpr const char * node_description =
    "Checks if the robot is aligned with a target TF frame within a configurable angular threshold and outputs direction ('left' or 'right').";

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("target_frame", "Target TF frame to check if aligned"),
      BT::InputPort<std::string>("base_frame", "base_link", "Base TF frame for reference"),
      BT::InputPort<double>("angle_threshold", 0.5, "Angular threshold in radians for considering aligned"),
      BT::InputPort<double>("timeout", 0.5, "Maximum TF wait/staleness threshold in seconds"),
      BT::OutputPort<std::string>("direction", "Bearing direction to align with target ('left' or 'right')")
    };
  }

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace social_bt_nodes

#endif  // SOCIAL_BT_NODES__BT_NODES__PERCEPTION__IS_ALIGNED_HPP_
