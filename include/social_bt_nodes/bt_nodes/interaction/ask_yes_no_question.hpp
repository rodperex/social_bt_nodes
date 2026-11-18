#ifndef SOCIAL_BT_NODES__BT_NODES__INTERACTION__ASK_YES_NO_QUESTION_HPP_
#define SOCIAL_BT_NODES__BT_NODES__INTERACTION__ASK_YES_NO_QUESTION_HPP_

#include <memory>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "simple_hri_interfaces/action/listen.hpp"
#include "simple_hri_interfaces/action/say.hpp"
#include "simple_hri_interfaces/srv/yes_no.hpp"
#include "social_bt_nodes/bt_nodes/interaction/hri_action_call.hpp"

namespace social_bt_nodes
{

/**
 * @brief BehaviorTree node that asks a yes/no question and calls the YesNo service for confirmation
 *
 * It asks the question with the /tts_action action (it finishes when the audio ends),
 * listens to the answer with the /stt_action action and interprets it with the
 * /yesno_service service. Returns SUCCESS if the user confirms (YES), FAILURE otherwise.
 * Halting the node cancels the running goal (stops the audio or the recording).
 *
 * XML Usage:
 *   <AskYesNoQuestion question="{input_text}"
 *                 service_name="/yesno_service" timeout="10000"/>
 *
 * Ports:
 *   Input:
 *     - question (string): The prompt to ask and analyze for yes/no
 *     - tts_action_name (string, default: "/tts_action"): TTS action name
 *     - stt_action_name (string, default: "/stt_action"): STT action name
 *     - service_name (string, default: "/yesno_service"): YesNo service name
 *     - max_wait (double, default: 0.0): Seconds to wait for the user to start answering
 *       (0 = the server default, 10 s)
 *     - timeout (int, default: 10000): Time to wait for each action server or service (ms)
 *   Output:
 *     - confirmed (bool): true if the answer is YES, false if the answer is NO
 * 
 * Returns:
 *   - SUCCESS: User confirmed (result is "YES")
 *   - FAILURE: User did not confirm (result is "NO") or errors occurred
 *   - RUNNING: Speaking, listening or waiting for the YesNo service
 */
class AskYesNoQuestion : public BT::StatefulActionNode
{
public:
  AskYesNoQuestion(
    const std::string & name,
    const BT::NodeConfig & conf);

  AskYesNoQuestion() = delete;

  ~AskYesNoQuestion() = default;

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

  static constexpr const char * node_description =
    "Asks a yes/no question with the TTS action, listens with the STT action and checks the user's response with the YesNo service.";

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("question", "Confirmation prompt to ask the user"),
      BT::OutputPort<bool>("confirmed", "Confirmation result (true or false)"),
      BT::InputPort<std::string>("tts_action_name", "/tts_action", "TTS action name"),
      BT::InputPort<std::string>("stt_action_name", "/stt_action", "STT action name"),
      BT::InputPort<std::string>("service_name", "/yesno_service", "YesNo service name"),
      BT::InputPort<double>("max_wait", 0.0,
        "Seconds to wait for the user to start answering (0 = server default)"),
      BT::InputPort<int>("timeout", 10000, "Time to wait for each action server or service (ms)")
    };
  }

private:
  enum class Phase
  {
    IDLE,
    SPEAKING,
    LISTENING,
    WAITING_YESNO
  };

  rclcpp::Node::SharedPtr node_;
  HriActionCall<simple_hri_interfaces::action::Say> say_;
  HriActionCall<simple_hri_interfaces::action::Listen> listen_;
  rclcpp::Client<simple_hri_interfaces::srv::YesNo>::SharedPtr client_;
  std::shared_ptr<rclcpp::Client<simple_hri_interfaces::srv::YesNo>::FutureAndRequestId> future_result_;

  std::string question_;
  std::string tts_action_name_;
  std::string stt_action_name_;
  std::string service_name_;
  std::string user_answer_;
  double max_wait_;
  int timeout_ms_;
  Phase phase_;
};

}  // namespace social_bt_nodes

#endif  // SOCIAL_BT_NODES__BT_NODES__INTERACTION__ASK_YES_NO_QUESTION_HPP_
