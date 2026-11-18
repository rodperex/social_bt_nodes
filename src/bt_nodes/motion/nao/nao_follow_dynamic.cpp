#include "social_bt_nodes/bt_nodes/motion/nao/nao_follow_dynamic.hpp"
#include "social_bt_nodes/bt_failure.hpp"
#include "tf2/exceptions.hpp"
#include <cmath>

namespace social_bt_nodes
{

NaoFollowDynamic::NaoFollowDynamic(
  const std::string & action_name,
  const BT::NodeConfig & conf)
: BT::StatefulActionNode(action_name, conf),
  vel_rot_avoidance_(0.0),
  stop_requested_(false),
  danger_(false),
  last_touch_pressed_(false),
  angular_pid_(1.0, 0.0, 0.3),  // kp=1.0, ki=0.0, kd=0.3
  linear_pid_(2.0, 0.0, 0.2)    // kp=0.8, ki=0.0, kd=0.2
{
  // Get ROS node from blackboard
  auto node_any = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  if (!node_any) {
    throw BT::RuntimeError("NaoFollowDynamic: 'node' not found in blackboard");
  }
  node_ = node_any;
  
  std::string cmd_vel_topic, sonar_topic, touch_topic;
  
  if (!getInput("cmd_vel_topic", cmd_vel_topic)) {
    cmd_vel_topic = "/cmd_vel";
  }
  if (!getInput("sonar_topic", sonar_topic)) {
    sonar_topic = "/sensors/sonar";
  }
  if (!getInput("touch_topic", touch_topic)) {
    touch_topic = "/sensors/touch";
  }
  
  cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(
    cmd_vel_topic, 10);
  
  sonar_sub_ = node_->create_subscription<nao_lola_sensor_msgs::msg::Sonar>(
    sonar_topic, 10,
    std::bind(&NaoFollowDynamic::sonar_callback, this, std::placeholders::_1));
  
  touch_sub_ = node_->create_subscription<nao_lola_sensor_msgs::msg::Touch>(
    touch_topic, 10,
    std::bind(&NaoFollowDynamic::touch_callback, this, std::placeholders::_1));
  
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

NaoFollowDynamic::~NaoFollowDynamic()
{
  stop_robot();
}

BT::NodeStatus NaoFollowDynamic::onStart()
{
  if (!getInput("min_distance", min_distance_)) {
    min_distance_ = 1.0;
  }
  if (!getInput("avoidance_distance", avoidance_distance_)) {
    avoidance_distance_ = 0.5;
  }
  if (!getInput("danger_distance", danger_distance_)) {
    danger_distance_ = 0.3;
  }
  if (!getInput("max_linear_speed", max_linear_speed_)) {
    max_linear_speed_ = 1.0;
  }
  if (!getInput("max_angular_speed", max_angular_speed_)) {
    max_angular_speed_ = 0.5;
  }
  if (!getInput("target_frame", target_frame_)) {
    target_frame_ = "target";
  }
  if (!getInput("base_frame", base_frame_)) {
    base_frame_ = "base_link";
  }
  if (!getInput("succeed_on_reach", succeed_on_reach_)) {
    succeed_on_reach_ = false;
  }
  if (!getInput("linear_vel_strategy", linear_vel_strategy_)) {
    linear_vel_strategy_ = "proportional";
  }
  
  vel_rot_avoidance_ = 0.0;
  angular_pid_.reset();
  linear_pid_.reset();
  last_time_ = node_->now();
  
  RCLCPP_INFO(node_->get_logger(), 
    "Starting to follow target '%s' (min_dist: %.2f m)",
    target_frame_.c_str(), min_distance_);
  
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus NaoFollowDynamic::onRunning()
{
  // Check if stopped by touch sensor
  if (stop_requested_) {
    RCLCPP_INFO(node_->get_logger(), "Robot stopped by touch sensor");
    stop_robot();
    return BT::NodeStatus::RUNNING;
  }

  if (danger_) {
    RCLCPP_WARN(node_->get_logger(), "In DANGER state due to obstacles");
    stop_robot();
    return BT::NodeStatus::RUNNING;
  }
  
  // Get target position
  try {
    auto transform = tf_buffer_->lookupTransform(
      base_frame_,
      target_frame_,
      tf2::TimePointZero,
      tf2::durationFromSec(0.5));
    
    double target_x = transform.transform.translation.x;
    double target_y = transform.transform.translation.y;
    
    double distance = std::sqrt(target_x * target_x + target_y * target_y);
    double angle = std::atan2(target_y, target_x);
    
    RCLCPP_DEBUG(node_->get_logger(), 
      "Target at distance: %.2f m, angle: %.2f deg",
      distance, angle * 180.0 / M_PI);
    
    // Check if target is too close - stop completely
    if (distance <= min_distance_) {
      RCLCPP_INFO(node_->get_logger(), 
        "Target reached (distance: %.2f m <= min: %.2f m), stopping",
        distance, min_distance_);
      stop_robot();
      return succeed_on_reach_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
    }
    
    // Calculate velocities
    double vel_lin = 0.0;
    double vel_rot = 0.0;
    
    // Angular velocity - PID control
    rclcpp::Time current_time = node_->now();
    double dt = (current_time - last_time_).seconds();
    
    if (dt > 0.0) {
      // Set output limits for angular PID
      angular_pid_.setOutputLimits(-max_angular_speed_, max_angular_speed_);
      
      // Compute PID output
      vel_rot = angular_pid_.compute(angle, dt);
      
      last_time_ = current_time;
    }
    
    // Linear velocity - based on strategy
    if (linear_vel_strategy_ == "pid") {
      // PID control for linear velocity based on distance error
      double distance_error = distance - min_distance_;
      if (dt > 0.0) {
        linear_pid_.setOutputLimits(0.0, max_linear_speed_);
        vel_lin = linear_pid_.compute(distance_error, dt);
      }
    } else {
      // Proportional strategy - reduce speed when turning
      // Scale down linear speed based on angular velocity magnitude
      double turn_ratio = std::abs(vel_rot) / max_angular_speed_;
      vel_lin = max_linear_speed_ * (1.0 - turn_ratio);  // Zero linear speed when turning at max angular speed
    }
    
    // Check for danger (both sides blocked)
    // if (danger_) {
    //   RCLCPP_WARN(node_->get_logger(), "DANGER: Obstacles on both sides, stopping linear motion");
    //   vel_lin = 0.0;  // Stop forward motion
    //   vel_rot = vel_rot_avoidance_;  // Only use avoidance rotation
    // } else {
      // Integrate obstacle avoidance when not in danger
      if (vel_rot * vel_rot_avoidance_ > 0.0) {
        // Same direction, add avoidance
        RCLCPP_INFO(node_->get_logger(), 
          "Integrating avoidance rotation: %.2f rad/s", vel_rot_avoidance_);
        vel_rot += vel_rot_avoidance_;
        vel_rot = std::max(-max_angular_speed_, 
                          std::min(max_angular_speed_, vel_rot));
        vel_lin = vel_lin * 0.8;  // Reduce speed when avoiding
      } else if (std::abs(vel_rot_avoidance_) > 0.01) {
        // Override with avoidance if there's an obstacle
        RCLCPP_INFO(node_->get_logger(), 
          "Overriding rotation with avoidance: %.2f rad/s", vel_rot_avoidance_);
        vel_rot = vel_rot_avoidance_;
        vel_lin = vel_lin * 0.8;  // Reduce speed when avoiding
      }
    // }
    
    RCLCPP_DEBUG(node_->get_logger(), 
      "Following: lin=%.2f m/s, ang=%.2f rad/s", vel_lin, vel_rot);
    
    // Publish velocity command
    auto twist_msg = geometry_msgs::msg::Twist();
    twist_msg.linear.x = vel_lin;
    twist_msg.angular.z = vel_rot;
    cmd_vel_pub_->publish(twist_msg);
    
    return BT::NodeStatus::RUNNING;
    
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(node_->get_logger(), "Lost target: %s", ex.what());
    stop_robot();
    return bt_failure(config(), registrationName(), "lost target: " + std::string(ex.what()));
  }
}

void NaoFollowDynamic::onHalted()
{
  RCLCPP_INFO(node_->get_logger(), "Follow action halted");
  stop_robot();
}

void NaoFollowDynamic::stop_robot()
{
  auto twist_msg = geometry_msgs::msg::Twist();
  twist_msg.linear.x = 0.0;
  twist_msg.linear.y = 0.0;
  twist_msg.linear.z = 0.0;
  twist_msg.angular.x = 0.0;
  twist_msg.angular.y = 0.0;
  twist_msg.angular.z = 0.0;
  
  cmd_vel_pub_->publish(twist_msg);
}

void NaoFollowDynamic::sonar_callback(
  const nao_lola_sensor_msgs::msg::Sonar::SharedPtr msg)
{
  double dis_left = msg->left;
  double dis_right = msg->right;
  
  vel_rot_avoidance_ = 0.0;
  danger_ = false;

  RCLCPP_DEBUG(node_->get_logger(), 
    "Sonar readings - Left: %.2f m, Right: %.2f m", dis_left, dis_right);
  
  // Check for DANGER - any obstacle closer than danger_distance
  if (dis_left < danger_distance_ && dis_right < danger_distance_) {
    danger_ = true;
    RCLCPP_WARN(node_->get_logger(), 
      "DANGER: Obstacle too close (left: %.2f m, right: %.2f m)", 
      dis_left, dis_right);
    return;  // Don't set avoidance rotation, just stop
  }
  
  // Check for obstacle avoidance - obstacles between danger_distance and avoidance_distance
  bool avoid_left = false;
  bool avoid_right = false;
  double vel_rot_avoid_left = 0.0;
  double vel_rot_avoid_right = 0.0;
  vel_rot_avoidance_ = 0.0;
  
  if (dis_left < avoidance_distance_) {
    RCLCPP_INFO(node_->get_logger(), 
      "Obstacle detected on LEFT at %.2f m", dis_left);
    avoid_left = true;
    vel_rot_avoid_left = -max_angular_speed_;
    vel_rot_avoidance_ = vel_rot_avoid_left;
  }
  
  if (dis_right < avoidance_distance_) {
    RCLCPP_INFO(node_->get_logger(), 
      "Obstacle detected on RIGHT at %.2f m", dis_right);
    avoid_right = true;
    vel_rot_avoid_right = max_angular_speed_;
    vel_rot_avoidance_ = vel_rot_avoid_right;
  }
  
  // If obstacles on both sides, turn away from the closer one
  if (avoid_left && avoid_right) {
    RCLCPP_DEBUG(node_->get_logger(), 
      "Obstacles on both sides, turning away from closer one");
    
    if (dis_left < dis_right) {
      vel_rot_avoidance_ = vel_rot_avoid_left;
    } else {
      vel_rot_avoidance_ = vel_rot_avoid_right;
    }
  }
}

void NaoFollowDynamic::touch_callback(
  const nao_lola_sensor_msgs::msg::Touch::SharedPtr msg)
{
  const bool touch_pressed = msg->head_front || msg->head_middle || msg->head_rear;

  if (touch_pressed && !last_touch_pressed_) {
    stop_requested_ = !stop_requested_;
    if (stop_requested_) {
      RCLCPP_INFO(node_->get_logger(), "Touch detected: Stopping robot");
    } else {
      RCLCPP_INFO(node_->get_logger(), "Touch detected: Resuming robot");
    }
  }

  last_touch_pressed_ = touch_pressed;
}

}  // namespace social_bt_nodes
