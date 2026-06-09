#ifndef SOCIAL_BT_NODES__BT_NODES__INTERACTION__ANALYZE_IMAGE_HPP_
#define SOCIAL_BT_NODES__BT_NODES__INTERACTION__ANALYZE_IMAGE_HPP_

#include <memory>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "simple_hri_interfaces/srv/analyze_image.hpp"

namespace social_bt_nodes
{

class AnalyzeImage : public BT::StatefulActionNode
{
public:
  AnalyzeImage(const std::string & name, const BT::NodeConfig & conf);
  AnalyzeImage() = delete;
  ~AnalyzeImage() = default;

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("prompt", "Prompt for image analysis"),
      BT::InputPort<std::string>("service_name", "/analyze_image_service", "Analyze image service name"),
      BT::InputPort<int>("timeout", 15000, "Service call timeout (ms)"),
      BT::OutputPort<std::string>("result", "Image analysis result")
    };
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<simple_hri_interfaces::srv::AnalyzeImage>::SharedPtr client_;
  std::shared_ptr<rclcpp::Client<simple_hri_interfaces::srv::AnalyzeImage>::FutureAndRequestId> future_result_;

  std::string prompt_;
  std::string service_name_;
  int timeout_ms_;
};

}  // namespace social_bt_nodes

#endif  // SOCIAL_BT_NODES__BT_NODES__INTERACTION__ANALYZE_IMAGE_HPP_
