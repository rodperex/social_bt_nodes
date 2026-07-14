#ifndef SOCIAL_BT_NODES__BT_NODES__PERCEPTION__IS_TARGET_STATIC_HPP_
#define SOCIAL_BT_NODES__BT_NODES__PERCEPTION__IS_TARGET_STATIC_HPP_

#include <memory>
#include <string>

#include "behaviortree_cpp/condition_node.h"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace social_bt_nodes
{

class IsTargetStatic : public BT::ConditionNode
{
public:
  IsTargetStatic(const std::string & name, const BT::NodeConfig & conf);

  BT::NodeStatus tick() override;

  static constexpr const char * node_description =
    "Checks whether a target frame has remained static for at least min_static_time_sec. "
    "Returns SUCCESS when static duration requirement is met and FAILURE otherwise.";

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("target_frame", "target", "Target TF frame to evaluate"),
      BT::InputPort<std::string>("base_frame", "base_link", "Base TF frame for reference"),
      BT::InputPort<double>("min_static_time_sec", 5.0, "Required stillness duration in seconds"),
      BT::InputPort<double>("position_epsilon", 0.025, "Maximum displacement in metres consio su dered static"),
      BT::InputPort<double>("timeout", 0.5, "Maximum TF wait/staleness threshold in seconds")
    };
  }

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace social_bt_nodes

#endif  // SOCIAL_BT_NODES__BT_NODES__PERCEPTION__IS_TARGET_STATIC_HPP_
