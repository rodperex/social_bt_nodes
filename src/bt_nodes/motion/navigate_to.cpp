#include "social_bt_nodes/bt_nodes/motion/navigate_to.hpp"
#include "social_bt_nodes/bt_failure.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace social_bt_nodes
{

NavigateTo::NavigateTo(
  const std::string & action_name,
  const BT::NodeConfig & conf)
: BT::StatefulActionNode(action_name, conf),
  goal_accepted_(false),
  goal_completed_(false),
  goal_succeeded_(false)
{
  // Get ROS node from blackboard
  auto node_any = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  if (!node_any) {
    throw BT::RuntimeError("NavigateTo: 'node' not found in blackboard");
  }
  node_ = node_any;

  // Get action name from input port
  std::string nav_action_name;
  if (!getInput("action_name", nav_action_name)) {
    nav_action_name = "navigate_to_pose";
  }

  // Create action client
  action_client_ = rclcpp_action::create_client<NavigateToPose>(
    node_,
    nav_action_name);

  // Create TF buffer and listener for frame-based navigation
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

NavigateTo::~NavigateTo()
{
  if (goal_handle_ && !goal_completed_) {
    RCLCPP_INFO(node_->get_logger(), "NavigateTo: Canceling active goal on destruction");
    auto future_cancel = action_client_->async_cancel_goal(goal_handle_);
  }
}

BT::NodeStatus NavigateTo::onStart()
{
  // Reset state
  goal_accepted_ = false;
  goal_completed_ = false;
  goal_succeeded_ = false;
  error_msg_ = "";
  goal_handle_.reset();

  // Get timeout
  if (!getInput("timeout", timeout_)) {
    timeout_ = 300.0;  // 5 minutes default
  }

  start_steady_time_ = std::chrono::steady_clock::now();

  // Wait for action server
  if (!action_client_->wait_for_action_server(std::chrono::seconds(5))) {
    error_msg_ = "Nav2 action server not available";
    RCLCPP_ERROR(node_->get_logger(), "NavigateTo: %s", error_msg_.c_str());
    setOutput("error_msg", error_msg_);
    return bt_failure(config(), registrationName(), error_msg_);
  }

  // Create goal pose
  geometry_msgs::msg::PoseStamped goal_pose;
  std::string target_frame;
  
  // Check if using TF frame or coordinates
  if (getInput("target", target_frame) && !target_frame.empty()) {
    // Navigate to TF frame
    RCLCPP_INFO(node_->get_logger(), "NavigateTo: Navigating to target frame '%s'", target_frame.c_str());
    std::string frame_id;
    if (!getInput("frame_id", frame_id)) {
      frame_id = "map";
    }

    bool resolve_target_frame = false;
    getInput("resolve_target_frame", resolve_target_frame);

    if (resolve_target_frame) {
      double tf_timeout = 1.0;
      getInput("tf_timeout", tf_timeout);
      try {
        goal_pose = create_goal_pose_from_tf(target_frame, frame_id, tf_timeout);
        RCLCPP_INFO(node_->get_logger(), 
          "NavigateTo: Resolved frame '%s' in '%s' at [%.2f, %.2f, %.2f]",
          target_frame.c_str(),
          frame_id.c_str(),
          goal_pose.pose.position.x,
          goal_pose.pose.position.y,
          goal_pose.pose.position.z);
      } catch (const std::exception & e) {
        error_msg_ = std::string("Failed to get transform: ") + e.what();
        RCLCPP_ERROR(node_->get_logger(), "NavigateTo: %s", error_msg_.c_str());
        setOutput("error_msg", error_msg_);
        return bt_failure(config(), registrationName(), error_msg_);
      }
    } else {
      goal_pose = create_goal_pose_from_target_frame(target_frame);
      RCLCPP_INFO(
        node_->get_logger(),
        "NavigateTo: Sending zero pose in target frame '%s' and letting Nav2 transform it",
        target_frame.c_str());
    }
  } else {
    // Navigate to coordinates
    double x, y, yaw;
    if (!getInput("x", x) || !getInput("y", y)) {
      error_msg_ = "Missing required inputs 'x' and 'y' or 'target_frame'";
      RCLCPP_ERROR(node_->get_logger(), "NavigateTo: %s", error_msg_.c_str());
      setOutput("error_msg", error_msg_);
      return bt_failure(config(), registrationName(), error_msg_);
    }

    if (!getInput("yaw", yaw)) {
      yaw = 0.0;
    }

    std::string frame_id;
    if (!getInput("frame_id", frame_id)) {
      frame_id = "map";
    }

    goal_pose = create_goal_pose_from_coordinates(x, y, yaw, frame_id);
    RCLCPP_INFO(node_->get_logger(), 
      "NavigateTo: Navigating to [%.2f, %.2f] with yaw %.2f rad",
      x, y, yaw);
  }

  // Prepare and send goal
  auto goal_msg = NavigateToPose::Goal();
  goal_msg.pose = goal_pose;
  goal_msg.behavior_tree = resolve_behavior_tree();

  if (!goal_msg.behavior_tree.empty()) {
    RCLCPP_INFO(
      node_->get_logger(),
      "NavigateTo: Using Nav2 behavior tree '%s'",
      goal_msg.behavior_tree.c_str());
  } else {
    RCLCPP_INFO(
      node_->get_logger(),
      "NavigateTo: Using Nav2 navigator default behavior tree");
  }

  auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    std::bind(&NavigateTo::goal_response_callback, this, std::placeholders::_1);
  send_goal_options.feedback_callback =
    std::bind(&NavigateTo::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
  send_goal_options.result_callback =
    std::bind(&NavigateTo::result_callback, this, std::placeholders::_1);

  action_client_->async_send_goal(goal_msg, send_goal_options);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus NavigateTo::onRunning()
{
  // Check timeout
  const auto elapsed = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - start_steady_time_).count();
  if (elapsed > timeout_) {
    error_msg_ = "Navigation timeout exceeded";
    RCLCPP_ERROR(node_->get_logger(), "NavigateTo: %s", error_msg_.c_str());
    setOutput("error_msg", error_msg_);
    
    if (goal_handle_) {
      auto future_cancel = action_client_->async_cancel_goal(goal_handle_);
    }
    
    return bt_failure(config(), registrationName(), error_msg_);
  }

  // Check if goal was rejected
  if (!goal_accepted_ && goal_completed_) {
    RCLCPP_ERROR(node_->get_logger(), "NavigateTo: Goal was rejected");
    error_msg_ = "Goal rejected by Nav2";
    setOutput("error_msg", error_msg_);
    return bt_failure(config(), registrationName(), error_msg_);
  }

  // Check if goal is completed
  if (goal_completed_) {
    if (goal_succeeded_) {
      RCLCPP_INFO(node_->get_logger(), "NavigateTo: Successfully reached goal");
      return BT::NodeStatus::SUCCESS;
    } else {
      RCLCPP_ERROR(node_->get_logger(), "NavigateTo: Failed to reach goal - %s", 
        error_msg_.c_str());
      setOutput("error_msg", error_msg_);
      return bt_failure(config(), registrationName(), "failed to reach goal: " + error_msg_);
    }
  }

  // Still running
  return BT::NodeStatus::RUNNING;
}

void NavigateTo::onHalted()
{
  RCLCPP_WARN(node_->get_logger(), "NavigateTo: Halted, canceling goal");
  
  if (goal_handle_ && !goal_completed_) {
    auto future_cancel = action_client_->async_cancel_goal(goal_handle_);
  }

  goal_handle_.reset();
}

void NavigateTo::goal_response_callback(
  const GoalHandleNavigateToPose::SharedPtr & goal_handle)
{
  if (!goal_handle) {
    RCLCPP_ERROR(node_->get_logger(), "NavigateTo: Goal was rejected by server");
    goal_accepted_ = false;
    goal_completed_ = true;
  } else {
    RCLCPP_INFO(node_->get_logger(), "NavigateTo: Goal accepted by server");
    goal_handle_ = goal_handle;
    goal_accepted_ = true;
  }
}

void NavigateTo::feedback_callback(
  GoalHandleNavigateToPose::SharedPtr,
  const std::shared_ptr<const NavigateToPose::Feedback> feedback)
{
  // Log distance remaining periodically
  RCLCPP_DEBUG(node_->get_logger(), 
    "NavigateTo: Distance remaining: %.2f m",
    feedback->distance_remaining);
}

void NavigateTo::result_callback(
  const GoalHandleNavigateToPose::WrappedResult & result)
{
  goal_completed_ = true;

  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(node_->get_logger(), "NavigateTo: Goal succeeded");
      goal_succeeded_ = true;
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(node_->get_logger(), "NavigateTo: Goal was aborted");
      error_msg_ = "Goal aborted by Nav2";
      goal_succeeded_ = false;
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_WARN(node_->get_logger(), "NavigateTo: Goal was canceled");
      error_msg_ = "Goal canceled";
      goal_succeeded_ = false;
      break;
    default:
      RCLCPP_ERROR(node_->get_logger(), "NavigateTo: Unknown result code");
      error_msg_ = "Unknown result code";
      goal_succeeded_ = false;
      break;
  }
}

geometry_msgs::msg::PoseStamped NavigateTo::create_goal_pose_from_coordinates(
  double x, double y, double yaw, const std::string & frame_id)
{
  geometry_msgs::msg::PoseStamped goal_pose;
  goal_pose.header.frame_id = frame_id;
  goal_pose.header.stamp = node_->now();
  
  goal_pose.pose.position.x = x;
  goal_pose.pose.position.y = y;
  goal_pose.pose.position.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  goal_pose.pose.orientation = tf2::toMsg(q);

  return goal_pose;
}

geometry_msgs::msg::PoseStamped NavigateTo::create_goal_pose_from_target_frame(
  const std::string & target_frame)
{
  geometry_msgs::msg::PoseStamped goal_pose;
  goal_pose.header.frame_id = target_frame;
  goal_pose.header.stamp = builtin_interfaces::msg::Time();

  goal_pose.pose.position.x = 0.0;
  goal_pose.pose.position.y = 0.0;
  goal_pose.pose.position.z = 0.0;
  goal_pose.pose.orientation.x = 0.0;
  goal_pose.pose.orientation.y = 0.0;
  goal_pose.pose.orientation.z = 0.0;
  goal_pose.pose.orientation.w = 1.0;

  return goal_pose;
}

geometry_msgs::msg::PoseStamped NavigateTo::create_goal_pose_from_tf(
  const std::string & target_frame, const std::string & frame_id, double timeout)
{
  geometry_msgs::msg::TransformStamped transform;
  
  try {
    if (timeout <= 0.0) {
      throw std::runtime_error("tf_timeout must be greater than zero");
    }

    // Wait for transform to be available
    if (!tf_buffer_->canTransform(frame_id, target_frame, tf2::TimePointZero, 
                                   tf2::durationFromSec(timeout))) {
      throw std::runtime_error("Transform not available within timeout");
    }

    transform = tf_buffer_->lookupTransform(
      frame_id, target_frame, tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    throw std::runtime_error(std::string("TF lookup failed: ") + ex.what());
  }

  geometry_msgs::msg::PoseStamped goal_pose;
  goal_pose.header = transform.header;
  goal_pose.pose.position.x = transform.transform.translation.x;
  goal_pose.pose.position.y = transform.transform.translation.y;
  goal_pose.pose.position.z = transform.transform.translation.z;
  goal_pose.pose.orientation = transform.transform.rotation;

  return goal_pose;
}

std::string NavigateTo::resolve_behavior_tree()
{
  std::string behavior_tree;
  if (getInput("behavior_tree", behavior_tree) && !behavior_tree.empty()) {
    return behavior_tree;
  }

  bool use_truncated_path = true;
  getInput("use_truncated_path", use_truncated_path);
  if (!use_truncated_path) {
    return "";
  }

  try {
    return ament_index_cpp::get_package_share_directory("social_bt_nodes") +
           "/config/navigate_to_pose_truncated.xml";
  } catch (const std::exception & e) {
    RCLCPP_WARN(
      node_->get_logger(),
      "NavigateTo: Could not resolve default truncated Nav2 BT: %s. "
      "Falling back to Nav2 default BT.",
      e.what());
    return "";
  }
}

}  // namespace social_bt_nodes
