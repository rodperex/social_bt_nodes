#ifndef SOCIAL_BT_NODES__BT_NODES__INTERACTION__HRI_ACTION_CALL_HPP_
#define SOCIAL_BT_NODES__BT_NODES__INTERACTION__HRI_ACTION_CALL_HPP_

#include <atomic>
#include <chrono>
#include <future>
#include <memory>
#include <optional>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace social_bt_nodes
{

/**
 * @brief Non-blocking tracking of one goal of a simple_hri action (Listen or Say).
 *
 * Listening and speaking last seconds, so simple_hri offers them as actions
 * (/stt_action, /tts_action): the result arrives when the person has finished
 * speaking or when the audio has really finished, and the goal can be canceled.
 *
 * Usage from a BT::StatefulActionNode:
 *   onStart():   connect(node, name, wait) and send(goal)
 *   onRunning(): poll() returns std::nullopt while the goal runs, then its result
 *   onHalted():  cancel() stops the recording or the audio
 *
 * The futures are completed by whoever spins the ROS node, as with the service
 * clients of the other nodes.
 */
template<class ActionT>
class HriActionCall
{
public:
  using Client = rclcpp_action::Client<ActionT>;
  using GoalHandle = rclcpp_action::ClientGoalHandle<ActionT>;
  using WrappedResult = typename GoalHandle::WrappedResult;

  /// Create the client (reused while the name does not change) and wait for the server.
  bool connect(
    const rclcpp::Node::SharedPtr & node, const std::string & action_name,
    std::chrono::milliseconds wait)
  {
    if (!client_ || action_name != action_name_) {
      client_ = rclcpp_action::create_client<ActionT>(node, action_name);
      action_name_ = action_name;
    }
    return client_->wait_for_action_server(wait);
  }

  /// Send a goal; the previous one, if any, is forgotten (call cancel() first to stop it).
  void send(const typename ActionT::Goal & goal)
  {
    state_ = std::make_shared<State>();
    auto state = state_;
    auto client = client_;
    typename Client::SendGoalOptions options;
    // If the goal was canceled before the server accepted it, cancel it as soon as it is
    options.goal_response_callback =
      [state, client](typename GoalHandle::SharedPtr handle) {
        if (handle && state->cancel_requested) {
          client->async_cancel_goal(handle);
        }
      };
    goal_future_ = client_->async_send_goal(goal, options);
    result_future_ = {};
  }

  /// std::nullopt while the goal runs; its result when it finishes.
  /// A goal rejected by the server finishes with ResultCode::ABORTED and a null result.
  std::optional<WrappedResult> poll()
  {
    if (!state_) {
      return std::nullopt;
    }
    if (!result_future_.valid()) {
      if (!ready(goal_future_)) {
        return std::nullopt;
      }
      auto handle = goal_future_.get();
      if (!handle) {
        WrappedResult rejected;
        rejected.code = rclcpp_action::ResultCode::ABORTED;
        state_.reset();
        return rejected;
      }
      result_future_ = client_->async_get_result(handle);
    }
    if (!ready(result_future_)) {
      return std::nullopt;
    }
    state_.reset();
    return result_future_.get();
  }

  /// Ask the server to cancel the goal (it stops the recording or the audio).
  void cancel()
  {
    if (!state_) {
      return;
    }
    state_->cancel_requested = true;
    if (ready(goal_future_)) {
      auto handle = goal_future_.get();
      if (handle) {
        client_->async_cancel_goal(handle);
      }
    }
    // Not accepted yet: goal_response_callback cancels it when it is
    state_.reset();
  }

  const std::string & action_name() const {return action_name_;}

private:
  struct State
  {
    std::atomic<bool> cancel_requested{false};
  };

  template<class FutureT>
  static bool ready(const FutureT & future)
  {
    return future.valid() &&
           future.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
  }

  typename Client::SharedPtr client_;
  std::string action_name_;
  std::shared_ptr<State> state_;
  std::shared_future<typename GoalHandle::SharedPtr> goal_future_;
  std::shared_future<WrappedResult> result_future_;
};

}  // namespace social_bt_nodes

#endif  // SOCIAL_BT_NODES__BT_NODES__INTERACTION__HRI_ACTION_CALL_HPP_
