#include "behaviortree_cpp/bt_factory.h"
#include "social_bt_nodes/bt_nodes/motion/nao_follow.hpp"
#include "social_bt_nodes/bt_nodes/motion/nao_follow_dynamic.hpp"
#include "social_bt_nodes/bt_nodes/motion/spin_search.hpp"
#include "social_bt_nodes/bt_nodes/motion/navigate_to.hpp"
#include "social_bt_nodes/bt_nodes/perception/is_target_detected.hpp"
#include "social_bt_nodes/bt_nodes/perception/set_perception_target.hpp"
#include "social_bt_nodes/bt_nodes/interaction/speak.hpp"
#include "social_bt_nodes/bt_nodes/interaction/speak_enum.hpp"
#include "social_bt_nodes/bt_nodes/interaction/listen.hpp"
#include "social_bt_nodes/bt_nodes/interaction/extract.hpp"
#include "social_bt_nodes/bt_nodes/interaction/confirmation.hpp"
#include "social_bt_nodes/bt_nodes/interaction/analyze_image.hpp"
#include "social_bt_nodes/bt_nodes/interaction/nao_position.hpp"
#include "social_bt_nodes/bt_nodes/interaction/nao_set_leds.hpp"
#include "social_bt_nodes/bt_nodes/support/set_ros2_param.hpp"

// Plugin registration with extern "C" linkage for dynamic loading
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<social_bt_nodes::NaoFollow>("Follow");
  factory.registerNodeType<social_bt_nodes::NaoFollowDynamic>("FollowDynamic");
  factory.registerNodeType<social_bt_nodes::SpinSearch>("SpinSearch");
  factory.registerNodeType<social_bt_nodes::NavigateTo>("NavigateTo");
  factory.registerNodeType<social_bt_nodes::IsTargetDetected>("IsTargetDetected");
  factory.registerNodeType<social_bt_nodes::SetPerceptionTarget>("SetPerceptionTarget");
  factory.registerNodeType<social_bt_nodes::Speak>("Speak");
  factory.registerNodeType<social_bt_nodes::SpeakEnum>("SpeakEnum");
  factory.registerNodeType<social_bt_nodes::Listen>("Listen");
  factory.registerNodeType<social_bt_nodes::Extract>("Extract");
  factory.registerNodeType<social_bt_nodes::Confirmation>("Confirmation");
  factory.registerNodeType<social_bt_nodes::AnalyzeImage>("AnalyzeImage");
  factory.registerNodeType<social_bt_nodes::NaoPosition>("NaoPosition");
  factory.registerNodeType<social_bt_nodes::NaoSetLeds>("NaoSetLeds");
  factory.registerNodeType<social_bt_nodes::SetRos2Param>("SetRos2Param");
}
