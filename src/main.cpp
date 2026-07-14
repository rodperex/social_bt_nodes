#include <memory>
#include <chrono>
#include <algorithm>
#include <cctype>
#include <iostream>
#include <sstream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "behaviortree_cpp/loggers/bt_cout_logger.h"
#include "yaml-cpp/yaml.h"

namespace
{

constexpr int EXIT_SETUP_ERROR = 1;
constexpr int EXIT_BT_FAILURE = 2;

std::string to_lower_copy(std::string value)
{
  std::transform(
    value.begin(), value.end(), value.begin(),
    [](unsigned char c) {return static_cast<char>(std::tolower(c));});
  return value;
}

bool set_typed_blackboard_entry(
  const rclcpp::Logger & logger, BT::Blackboard::Ptr blackboard, const std::string & key,
  const YAML::Node & entry)
{
  if (!entry.IsMap()) {
    RCLCPP_ERROR(
      logger, "Blackboard entry '%s' must be a map with fields 'type' and 'value'", key.c_str());
    return false;
  }

  if (!entry["type"] || !entry["type"].IsScalar()) {
    RCLCPP_ERROR(logger, "Blackboard entry '%s' is missing scalar field 'type'", key.c_str());
    return false;
  }

  if (!entry["value"]) {
    RCLCPP_ERROR(logger, "Blackboard entry '%s' is missing field 'value'", key.c_str());
    return false;
  }

  const std::string type = to_lower_copy(entry["type"].as<std::string>());
  const YAML::Node value = entry["value"];

  try {
    if (type == "bool" || type == "boolean") {
      const bool parsed = value.as<bool>();
      blackboard->set<bool>(key, parsed);
      RCLCPP_INFO(logger, "[BB FEED] %s (bool) = %s", key.c_str(), parsed ? "true" : "false");
      return true;
    }

    if (type == "int" || type == "int64") {
      const int64_t parsed = value.as<int64_t>();
      blackboard->set<int64_t>(key, parsed);
      RCLCPP_INFO(logger, "[BB FEED] %s (int64) = %ld", key.c_str(), static_cast<long>(parsed));
      return true;
    }

    if (type == "double" || type == "float") {
      const double parsed = value.as<double>();
      blackboard->set<double>(key, parsed);
      RCLCPP_INFO(logger, "[BB FEED] %s (double) = %.6f", key.c_str(), parsed);
      return true;
    }

    if (type == "string") {
      const std::string parsed = value.as<std::string>();
      blackboard->set<std::string>(key, parsed);
      RCLCPP_INFO(logger, "[BB FEED] %s (string) = '%s'", key.c_str(), parsed.c_str());
      return true;
    }

    if (type == "yaml") {
      std::stringstream serialized;
      serialized << value;
      const std::string parsed = serialized.str();
      blackboard->set<std::string>(key, parsed);
      RCLCPP_INFO(logger, "[BB FEED] %s (string, serialized YAML)", key.c_str());
      return true;
    }

    RCLCPP_ERROR(
      logger,
      "Blackboard entry '%s' has unsupported type '%s' (supported: bool, int64, double, string, yaml)",
      key.c_str(), type.c_str());
    return false;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      logger, "Blackboard entry '%s' failed type conversion for type '%s': %s", key.c_str(),
      type.c_str(), e.what());
    return false;
  }
}

bool load_blackboard_from_yaml(
  const rclcpp::Logger & logger, BT::Blackboard::Ptr blackboard, const std::string & yaml_path)
{
  YAML::Node root;
  try {
    root = YAML::LoadFile(yaml_path);
  } catch (const YAML::Exception & e) {
    RCLCPP_ERROR(logger, "Failed to read blackboard YAML '%s': %s", yaml_path.c_str(), e.what());
    return false;
  }

  if (!root.IsMap()) {
    RCLCPP_ERROR(logger, "Blackboard YAML root must be a map of entries.");
    return false;
  }

  std::size_t count = 0;
  for (const auto & item : root) {
    if (!item.first.IsScalar()) {
      RCLCPP_ERROR(logger, "Blackboard YAML contains non-scalar key.");
      return false;
    }

    const std::string key = item.first.as<std::string>();
    if (!set_typed_blackboard_entry(logger, blackboard, key, item.second)) {
      return false;
    }
    ++count;
  }

  RCLCPP_INFO(logger, "Loaded %zu blackboard entries from: %s", count, yaml_path.c_str());
  return true;
}

bool wait_for_valid_ros_time(
  const rclcpp::Node::SharedPtr & node,
  double timeout_sec)
{
  const bool use_sim_time = node->get_parameter("use_sim_time").as_bool();
  if (!use_sim_time) {
    return true;
  }

  RCLCPP_INFO(
    node->get_logger(),
    "use_sim_time=true, waiting for first non-zero /clock before ticking the BT");

  const auto deadline =
    std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_sec);
  rclcpp::WallRate rate(20.0);

  while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
    rclcpp::spin_some(node);
    if (node->now().nanoseconds() > 0) {
      RCLCPP_INFO(node->get_logger(), "ROS time is ready");
      return true;
    }
    rate.sleep();
  }

  RCLCPP_ERROR(
    node->get_logger(),
    "Timed out waiting for /clock while use_sim_time=true");
  return false;
}

std::string blackboard_string_or_empty(BT::Blackboard::Ptr blackboard, const std::string & key)
{
  try {
    return blackboard->get<std::string>(key);
  } catch (const std::exception &) {
    return "";
  }
}

void print_failure_metadata(BT::Blackboard::Ptr blackboard)
{
  const std::string code = blackboard_string_or_empty(blackboard, "bt_last_failure_code");
  const std::string reason = blackboard_string_or_empty(blackboard, "bt_last_failure");

  if (!code.empty()) {
    std::cout << "BT_FAILURE_CODE=" << code << std::endl;
  }
  if (!reason.empty()) {
    std::cout << "BT_FAILURE_REASON=" << reason << std::endl;
  }
}

}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<rclcpp::Node>("social_bt_nodes");
  
  // Declare parameters
  node->declare_parameter("bt_xml", "");
  node->declare_parameter("bt_loop_duration", 100);  // ms
  node->declare_parameter("plugin_list", std::vector<std::string>());
  node->declare_parameter("feed_bb", false);
  node->declare_parameter("bb_feed_yaml", "");
  if (!node->has_parameter("use_sim_time")) {
    node->declare_parameter("use_sim_time", false);
  }
  
  // Get parameters
  std::string bt_xml = node->get_parameter("bt_xml").as_string();
  int bt_loop_duration = node->get_parameter("bt_loop_duration").as_int();
  std::vector<std::string> plugin_list = 
    node->get_parameter("plugin_list").as_string_array();
  bool feed_bb = node->get_parameter("feed_bb").as_bool();
  std::string bb_feed_yaml = node->get_parameter("bb_feed_yaml").as_string();
  
  if (bt_xml.empty()) {
    RCLCPP_ERROR(node->get_logger(), "bt_xml parameter is required");
    return EXIT_SETUP_ERROR;
  }
  
  RCLCPP_INFO(node->get_logger(), "Loading behavior tree from: %s", bt_xml.c_str());
  
  // Create BehaviorTree factory
  BT::BehaviorTreeFactory factory;
  
  // Create blackboard and put node in it
  auto blackboard = BT::Blackboard::create();
  blackboard->set("node", node);

  // Optional: preload blackboard entries from YAML for testing
  if (feed_bb) {
    if (bb_feed_yaml.empty()) {
      RCLCPP_ERROR(node->get_logger(), "feed_bb=true but bb_feed_yaml is empty");
      return EXIT_SETUP_ERROR;
    }
    if (!load_blackboard_from_yaml(node->get_logger(), blackboard, bb_feed_yaml)) {
      return EXIT_SETUP_ERROR;
    }
    blackboard->debugMessage();
  }
  
  // Load plugins
  for (const auto& plugin : plugin_list) {
    try {
      RCLCPP_INFO(node->get_logger(), "Loading plugin: %s", plugin.c_str());
      factory.registerFromPlugin(plugin);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(node->get_logger(), 
        "Failed to load plugin %s: %s", plugin.c_str(), e.what());
      return EXIT_SETUP_ERROR;
    }
  }
  
  // Create tree
  BT::Tree tree;
  try {
    tree = factory.createTreeFromFile(bt_xml, blackboard);
    RCLCPP_INFO(node->get_logger(), "Behavior tree created successfully");
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node->get_logger(), 
      "Failed to create tree: %s", e.what());
    return EXIT_SETUP_ERROR;
  }
  
  // StdCout logger: prints node status transitions to stdout
  BT::StdCoutLogger cout_logger(tree);

  // Optional: Enable Groot2 monitoring
  std::unique_ptr<BT::Groot2Publisher> groot_publisher;
  // try {
  //   groot_publisher = std::make_unique<BT::Groot2Publisher>(tree);
  //   RCLCPP_INFO(node->get_logger(), "Groot2 publisher enabled");
  // } catch (const std::exception& e) {
  //   RCLCPP_WARN(node->get_logger(), 
  //     "Groot2 publisher not available: %s", e.what());
  // }
  
  if (!wait_for_valid_ros_time(node, 10.0)) {
    return EXIT_SETUP_ERROR;
  }

  // Tick the tree periodically
  RCLCPP_INFO(node->get_logger(), 
    "Starting behavior tree execution (loop: %d ms)", bt_loop_duration);
  
  rclcpp::WallRate rate{std::chrono::milliseconds(bt_loop_duration)};
  int exit_code = EXIT_SETUP_ERROR;
  
  while (rclcpp::ok()) {
    // Spin ROS callbacks
    rclcpp::spin_some(node);
    
    // Tick the tree
    BT::NodeStatus status = tree.tickOnce();
    
    // Log status changes
    static BT::NodeStatus last_status = BT::NodeStatus::IDLE;
    if (status != last_status) {
      RCLCPP_INFO(node->get_logger(), 
        "Tree status: %s", BT::toStr(status).c_str());
      last_status = status;
    }
    
    // Handle terminal states
    if (status == BT::NodeStatus::SUCCESS) {
      RCLCPP_INFO(node->get_logger(), "Behavior tree succeeded");
      std::cout << "BT_FINAL_STATUS=SUCCESS" << std::endl;
      exit_code = 0;
      break;
    } else if (status == BT::NodeStatus::FAILURE) {
      RCLCPP_WARN(node->get_logger(), "Behavior tree failed");
      print_failure_metadata(blackboard);
      std::cout << "BT_FINAL_STATUS=FAILURE" << std::endl;
      exit_code = EXIT_BT_FAILURE;
      break;
    }
    
    rate.sleep();
  }
  
  RCLCPP_INFO(node->get_logger(), "Shutting down behavior tree");
  rclcpp::shutdown();
  
  return exit_code;
}
