#ifndef SOCIAL_BT_NODES__BT_NODES__PERCEPTION__GET_BEARING_HPP_
#define SOCIAL_BT_NODES__BT_NODES__PERCEPTION__GET_BEARING_HPP_

#include <memory>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

namespace social_bt_nodes
{

class GetBearing : public BT::SyncActionNode
{
public:
  GetBearing(const std::string & name, const BT::NodeConfig & conf);

  BT::NodeStatus tick() override;

  static constexpr const char * node_description =
    "Calculates the bearing angle to a target TF frame relative to a base frame and writes it to the blackboard.";

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("target_frame", "Target TF frame to calculate bearing to"),
      BT::InputPort<std::string>("base_frame", "base_link", "Base TF frame for reference"),
      BT::InputPort<double>("timeout", 0.5, "Maximum TF wait/staleness threshold in seconds"),
      BT::OutputPort<double>("bearing", "Calculated bearing angle in radians")
    };
  }

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace social_bt_nodes

#endif  // SOCIAL_BT_NODES__BT_NODES__PERCEPTION__GET_BEARING_HPP_