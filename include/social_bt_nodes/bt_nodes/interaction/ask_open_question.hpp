#ifndef SOCIAL_BT_NODES__BT_NODES__INTERACTION__ASK_OPEN_QUESTION_HPP_
#define SOCIAL_BT_NODES__BT_NODES__INTERACTION__ASK_OPEN_QUESTION_HPP_

#include <memory>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "simple_hri_interfaces/action/listen.hpp"
#include "simple_hri_interfaces/action/say.hpp"
#include "social_bt_nodes/bt_nodes/interaction/hri_action_call.hpp"

namespace social_bt_nodes
{

/**
 * @brief Asks an open-ended question and captures the answer in one node.
 *
 * This node chains two simple_hri actions:
 *  1) /tts_action (Say) asks the question; it finishes when the audio ends.
 *  2) /stt_action (Listen) captures the user answer.
 * Halting the node cancels the running goal (stops the audio or the recording).
 *
 * XML Usage:
 *   <AskOpenQuestion question="¿Cómo te llamas?" answer="{answer}" max_wait="10.0"/>
 */
class AskOpenQuestion : public BT::StatefulActionNode
{
public:
  AskOpenQuestion(
    const std::string & name,
    const BT::NodeConfig & conf);

  AskOpenQuestion() = delete;
  ~AskOpenQuestion() = default;

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

  static constexpr const char * node_description =
    "Asks the user an open-ended question and captures the answer in one node.";

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("question", "Question to ask the user"),
      BT::OutputPort<std::string>("answer", "User answer captured by the node"),
      BT::InputPort<std::string>("tts_action_name", "/tts_action", "TTS action name"),
      BT::InputPort<std::string>("stt_action_name", "/stt_action", "STT action name"),
      BT::InputPort<double>("max_wait", 0.0,
        "Seconds to wait for the user to start answering (0 = server default)"),
      BT::InputPort<int>("timeout", 10000, "Time to wait for each action server (ms)")
    };
  }

private:
  enum class Phase
  {
    IDLE,
    SPEAKING,
    LISTENING
  };

  rclcpp::Node::SharedPtr node_;
  HriActionCall<simple_hri_interfaces::action::Say> say_;
  HriActionCall<simple_hri_interfaces::action::Listen> listen_;

  std::string question_;
  std::string tts_action_name_;
  std::string stt_action_name_;
  double max_wait_;
  int timeout_ms_;
  Phase phase_;
};

}  // namespace social_bt_nodes

#endif  // SOCIAL_BT_NODES__BT_NODES__INTERACTION__ASK_OPEN_QUESTION_HPP_
