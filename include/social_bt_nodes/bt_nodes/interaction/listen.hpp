#ifndef SOCIAL_BT_NODES__BT_NODES__INTERACTION__LISTEN_HPP_
#define SOCIAL_BT_NODES__BT_NODES__INTERACTION__LISTEN_HPP_

#include <memory>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "simple_hri_interfaces/action/listen.hpp"
#include "social_bt_nodes/bt_nodes/interaction/hri_action_call.hpp"

namespace social_bt_nodes
{

/**
 * @brief BehaviorTree node that listens and transcribes with the simple_hri STT action
 *
 * It sends a goal to /stt_action (simple_hri_interfaces/action/Listen): the server waits
 * up to max_wait seconds for the person to start speaking, records until they finish and
 * returns the transcription. Halting the node cancels the goal, which stops the recording.
 *
 * XML Usage:
 *   <Listen transcribed_text="{text_output}" action_name="/stt_action" max_wait="10.0"/>
 *
 * Ports:
 *   Input:
 *     - action_name (string, default: "/stt_action"): STT action name
 *     - max_wait (double, default: 0.0): Seconds to wait for the person to start speaking
 *       (0 = the server default, 10 s)
 *     - timeout (int, default: 10000): Time to wait for the action server (ms)
 *   Output:
 *     - transcribed_text (string): The transcribed text from audio
 */
class Listen : public BT::StatefulActionNode
{
public:
  Listen(
    const std::string & name,
    const BT::NodeConfig & conf);

  Listen() = delete;
  ~Listen() = default;

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

  static constexpr const char * node_description =
    "Listens and transcribes speech to text using the STT action.";

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("action_name", "/stt_action", "STT action name"),
      BT::InputPort<double>("max_wait", 0.0,
        "Seconds to wait for the person to start speaking (0 = server default)"),
      BT::InputPort<int>("timeout", 10000, "Time to wait for the STT action server (ms)"),
      BT::OutputPort<std::string>("transcribed_text", "The transcribed text from audio")
    };
  }

private:
  rclcpp::Node::SharedPtr node_;
  HriActionCall<simple_hri_interfaces::action::Listen> listen_;

  std::string action_name_;
  double max_wait_;
  int timeout_ms_;
};

}  // namespace social_bt_nodes

#endif  // SOCIAL_BT_NODES__BT_NODES__INTERACTION__LISTEN_HPP_
