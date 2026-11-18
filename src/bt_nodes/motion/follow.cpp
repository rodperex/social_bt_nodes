#include "social_bt_nodes/bt_nodes/motion/follow.hpp"
#include "social_bt_nodes/bt_failure.hpp"
#include "tf2/exceptions.hpp"
#include <cmath>
#include <algorithm>

namespace social_bt_nodes
{

Follow::Follow(
  const std::string & action_name,
  const BT::NodeConfig & conf)
: BT::StatefulActionNode(action_name, conf),
  vel_rot_avoidance_(0.0),
  danger_(false),
  is_rotating_(true),
  angular_pid_(1.0, 0.0, 0.3, -1.0, 1.0)
{
  auto node_any = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  if (!node_any) {
    throw BT::RuntimeError("Follow: 'node' not found in blackboard");
  }
  node_ = node_any;

  std::string cmd_vel_topic, scan_topic;

  if (!getInput("cmd_vel_topic", cmd_vel_topic)) {
    cmd_vel_topic = "/cmd_vel";
  }
  if (!getInput("scan_topic", scan_topic)) {
    scan_topic = "/scan";
  }

  cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, 10);

  scan_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
    scan_topic, 10,
    std::bind(&Follow::scan_callback, this, std::placeholders::_1));

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

Follow::~Follow()
{
  stop_robot();
}

BT::NodeStatus Follow::onStart()
{
  if (!getInput("min_distance", min_distance_)) {min_distance_ = 1.0;}
  if (!getInput("avoidance_distance", avoidance_distance_)) {avoidance_distance_ = 0.5;}
  if (!getInput("max_linear_speed", max_linear_speed_)) {max_linear_speed_ = 0.5;}
  if (!getInput("max_angular_speed", max_angular_speed_)) {max_angular_speed_ = 1.0;}
  if (!getInput("target_frame", target_frame_)) {target_frame_ = "target";}
  if (!getInput("base_frame", base_frame_)) {base_frame_ = "base_link";}
  if (!getInput("succeed_on_reach", succeed_on_reach_)) {succeed_on_reach_ = false;}
  if (!getInput("rotation_stop_threshold", rotation_stop_threshold_)) {
    rotation_stop_threshold_ = 0.087;
  }
  if (!getInput("linear_stop_threshold", linear_stop_threshold_)) {
    linear_stop_threshold_ = 0.26;
  }

  vel_rot_avoidance_ = 0.0;
  danger_ = false;
  angular_pid_.reset();
  is_rotating_ = true;
  last_time_ = node_->now();

  RCLCPP_INFO(
    node_->get_logger(),
    "Follow: starting to follow '%s' (min_dist=%.2f m)",
    target_frame_.c_str(), min_distance_);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus Follow::onRunning()
{
  if (danger_) {
    stop_robot();
    return BT::NodeStatus::RUNNING;
  }

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

    RCLCPP_DEBUG(
      node_->get_logger(),
      "Follow: target at %.2f m, angle %.2f deg",
      distance, angle * 180.0 / M_PI);

    if (distance <= min_distance_) {
      stop_robot();
      return succeed_on_reach_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
    }

    double vel_lin = 0.0;
    double vel_rot = 0.0;

    // Two-phase with hysteresis
    if (std::abs(angle) > linear_stop_threshold_) {
      is_rotating_ = true;
    } else if (std::abs(angle) <= rotation_stop_threshold_) {
      is_rotating_ = false;
    }

    if (is_rotating_) {
      vel_lin = 0.0;
      rclcpp::Time now = node_->now();
      double dt = (now - last_time_).seconds();
      if (dt > 0.0) {
        angular_pid_.setOutputLimits(-max_angular_speed_, max_angular_speed_);
        vel_rot = angular_pid_.compute(angle, dt);
        last_time_ = now;
      }
    } else {
      vel_lin = max_linear_speed_;
      vel_rot = 0.0;
      angular_pid_.reset();
      last_time_ = node_->now();
    }

    // Integrate laser obstacle avoidance
    if (vel_rot * vel_rot_avoidance_ > 0.0) {
      vel_rot += vel_rot_avoidance_;
      vel_rot = std::max(-max_angular_speed_, std::min(max_angular_speed_, vel_rot));
    } else if (std::abs(vel_rot_avoidance_) > 0.01) {
      vel_rot = vel_rot_avoidance_;
    }

    RCLCPP_INFO(
      node_->get_logger(),
      "Follow: lin=%.2f m/s, ang=%.2f rad/s", vel_lin, vel_rot);

    auto twist = geometry_msgs::msg::Twist();
    twist.linear.x = vel_lin;
    twist.angular.z = vel_rot;
    cmd_vel_pub_->publish(twist);

    return BT::NodeStatus::RUNNING;

  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(node_->get_logger(), "Follow: lost target: %s", ex.what());
    stop_robot();
    return bt_failure(config(), registrationName(), "lost target: " + std::string(ex.what()));
  }
}

void Follow::onHalted()
{
  stop_robot();
}

void Follow::stop_robot()
{
  cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
}

void Follow::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  vel_rot_avoidance_ = 0.0;
  danger_ = false;

  int n = static_cast<int>(msg->ranges.size());
  if (n == 0) {return;}

  // Divide the scan into left and right halves (front sector only: ±90°)
  // Front-right: indices [0 .. n/4], front-left: indices [3n/4 .. n-1]
  double min_left = std::numeric_limits<double>::infinity();
  double min_right = std::numeric_limits<double>::infinity();

  int front_quarter = n / 4;

  for (int i = 0; i <= front_quarter; ++i) {
    float r = msg->ranges[i];
    if (std::isfinite(r) && r > msg->range_min && r < msg->range_max) {
      min_right = std::min(min_right, static_cast<double>(r));
    }
  }
  for (int i = n - front_quarter; i < n; ++i) {
    float r = msg->ranges[i];
    if (std::isfinite(r) && r > msg->range_min && r < msg->range_max) {
      min_left = std::min(min_left, static_cast<double>(r));
    }
  }

  bool avoid_left = min_left < avoidance_distance_;
  bool avoid_right = min_right < avoidance_distance_;

  if (avoid_left && avoid_right) {
    danger_ = true;
    // Turn away from the closer side
    vel_rot_avoidance_ = (min_left < min_right) ? -max_angular_speed_ : max_angular_speed_;
    RCLCPP_INFO(node_->get_logger(), "Follow: DANGER — obstacles on both sides, stopping");
  } else if (avoid_left) {
    // Obstacle on the left → rotate right (negative z)
    vel_rot_avoidance_ = -max_angular_speed_;
    RCLCPP_DEBUG(node_->get_logger(), "Follow: obstacle LEFT at %.2f m", min_left);
  } else if (avoid_right) {
    // Obstacle on the right → rotate left (positive z)
    vel_rot_avoidance_ = max_angular_speed_;
    RCLCPP_DEBUG(node_->get_logger(), "Follow: obstacle RIGHT at %.2f m", min_right);
  }
}

}  // namespace social_bt_nodes
