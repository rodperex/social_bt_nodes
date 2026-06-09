cmake_minimum_required(VERSION 3.8)
project(social_bt_nodes)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nao_lola_sensor_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(simple_hri_interfaces REQUIRED)
find_package(simple_perception_interfaces REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nao_pos_interfaces REQUIRED)
find_package(nao_led_interfaces REQUIRED)

set(dependencies
  rclcpp
  rclcpp_action
  geometry_msgs
  vision_msgs
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf2_ros
  tf2_geometry_msgs
  nao_lola_sensor_msgs
  behaviortree_cpp
  simple_hri_interfaces
  simple_perception_interfaces
  nav2_msgs
  nao_pos_interfaces
  nao_led_interfaces
)

include_directories(include)

# BT Nodes plugin library
add_library(social_bt_nodes_plugin SHARED
  src/bt_nodes/perception/is_target_detected.cpp
  src/bt_nodes/perception/set_perception_target.cpp
  src/bt_nodes/motion/spin_search.cpp
  src/bt_nodes/motion/nao_follow.cpp
  src/bt_nodes/motion/nao_follow_dynamic.cpp
  src/bt_nodes/motion/navigate_to.cpp
  src/bt_nodes/interaction/speak.cpp
  src/bt_nodes/interaction/speak_enum.cpp
  src/bt_nodes/interaction/listen.cpp
  src/bt_nodes/interaction/extract.cpp
  src/bt_nodes/interaction/confirmation.cpp
  src/bt_nodes/interaction/analyze_image.cpp
  src/bt_nodes/interaction/nao_position.cpp
  src/bt_nodes/interaction/nao_set_leds.cpp
  src/bt_nodes/support/set_ros2_param.cpp
  src/bt_nodes/bt_plugins.cpp
)

ament_target_dependencies(social_bt_nodes_plugin ${dependencies})

# Required for BT_REGISTER_NODES macro to work properly
target_compile_definitions(social_bt_nodes_plugin PRIVATE BT_PLUGIN_EXPORT)

# Main executable
add_executable(social_bt_nodes_main src/main.cpp)
ament_target_dependencies(social_bt_nodes_main ${dependencies})

# Install
install(TARGETS
  social_bt_nodes_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  social_bt_nodes_main
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include
)

install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install Python scripts
install(PROGRAMS
  social_bt_nodes/entity_tracker_fake_3d.py
  social_bt_nodes/yolo_to_standard.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(social_bt_nodes_plugin)
ament_export_dependencies(${dependencies})
ament_package()
