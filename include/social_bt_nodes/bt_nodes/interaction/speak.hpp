#ifndef SOCIAL_BT_NODES__BT_NODES__INTERACTION__SPEAK_HPP_
#define SOCIAL_BT_NODES__BT_NODES__INTERACTION__SPEAK_HPP_

#include <memory>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "simple_hri_interfaces/action/say.hpp"
#include "social_bt_nodes/bt_nodes/interaction/hri_action_call.hpp"

namespace social_bt_nodes
{

/**
 * @brief BehaviorTree node that speaks a text with the simple_hri TTS action
 *
 * It sends a goal to /tts_action (simple_hri_interfaces/action/Say) and returns
 * SUCCESS when the audio has really finished. Halting the node cancels the goal,
 * which stops the audio. {key} placeholders in the text are read from the blackboard.
 *
 * XML Usage:
 *   <Speak text="Hello world" action_name="/tts_action" timeout="5000"/>
 *
 * Ports:
 *   Input:
 *     - text (string): Text to speak
 *     - action_name (string, default: "/tts_action"): TTS action name
 *     - timeout (int, default: 5000): Time to wait for the action server (ms)
 */
class Speak : public BT::StatefulActionNode
{
public:
  Speak(
    const std::string & name,
    const BT::NodeConfig & conf);

  Speak() = delete;
  ~Speak() = default;

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

  static constexpr const char * node_description =
    "Synthesizes and speaks the specified text using the TTS action; it finishes when the audio ends.";

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("text", "Text to speak"),
      BT::InputPort<std::string>("action_name", "/tts_action", "TTS action name"),
      BT::InputPort<int>("timeout", 5000, "Time to wait for the TTS action server (ms)")
    };
  }

private:
  rclcpp::Node::SharedPtr node_;
  HriActionCall<simple_hri_interfaces::action::Say> say_;

  std::string text_;
  std::string action_name_;
  int timeout_ms_;
};

}  // namespace social_bt_nodes

#endif  // SOCIAL_BT_NODES__BT_NODES__INTERACTION__SPEAK_HPP_
