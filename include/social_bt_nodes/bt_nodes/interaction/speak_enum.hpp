#ifndef SOCIAL_BT_NODES__BT_NODES__INTERACTION__SPEAK_ENUM_HPP_
#define SOCIAL_BT_NODES__BT_NODES__INTERACTION__SPEAK_ENUM_HPP_

#include <memory>
#include <string>
#include <vector>

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "simple_hri_interfaces/action/say.hpp"
#include "social_bt_nodes/bt_nodes/interaction/hri_action_call.hpp"

namespace social_bt_nodes
{

/**
 * @brief BehaviorTree node that speaks a list of words with enumeration
 * 
 * This node takes a delimited string of words and speaks them as a list,
 * adding "y" (Spanish) or "and" (English) before the last item.
 * 
 * Example:
 *   Input: "manzana,naranja,plátano" with separator="," and language="es"
 *   Output: "manzana, naranja y plátano"
 * 
 * XML Usage:
 *   <SpeakEnum list="apple,orange,banana" separator="," language="en"
 *              action_name="/tts_action" timeout="5000"/>
 * 
 * Ports:
 *   Input:
 *     - list (string): Delimited list of words
 *     - separator (string, default: ","): Token that separates words
 *     - language (string, default: "es"): Language code ("es" or "en")
 *     - action_name (string, default: "/tts_action"): TTS action name
 *     - timeout (int, default: 5000): Time to wait for the action server (ms)
 *
 * It finishes when the audio has really finished; halting it stops the audio.
 */
class SpeakEnum : public BT::StatefulActionNode
{
public:
  SpeakEnum(
    const std::string & name,
    const BT::NodeConfig & conf);

  SpeakEnum() = delete;

  ~SpeakEnum() = default;

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

  static constexpr const char * node_description =
    "Enumerates and speaks a list of items using TTS with proper conjunction.";

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("list", "Delimited list of words to enumerate"),
      BT::InputPort<std::string>("separator", ",", "Separator token (default: comma)"),
      BT::InputPort<std::string>("language", "es", "Language code: 'es' or 'en'"),
      BT::InputPort<std::string>("action_name", "/tts_action", "TTS action name"),
      BT::InputPort<int>("timeout", 5000, "Time to wait for the TTS action server (ms)")
    };
  }

private:
  std::string build_enumerated_text(
    const std::vector<std::string> & items, 
    const std::string & language);
  
  std::vector<std::string> split_string(
    const std::string & text, 
    const std::string & separator);
  
  std::string trim(const std::string & str);

  rclcpp::Node::SharedPtr node_;
  HriActionCall<simple_hri_interfaces::action::Say> say_;

  std::string enumerated_text_;
  std::string action_name_;
  int timeout_ms_;
};

}  // namespace social_bt_nodes

#endif  // SOCIAL_BT_NODES__BT_NODES__INTERACTION__SPEAK_ENUM_HPP_
