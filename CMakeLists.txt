cmake_minimum_required(VERSION 3.8)
project(social_bt_nodes)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
# ament_target_dependencies() no longer exists in ROS 2 Lyrical. This links a target to the
# targets that each dependency exports (<pkg>::<pkg>, the <pkg>_TARGETS of interface packages,
# or its include directories and libraries). Works the same on Humble and Jazzy.
function(link_ament_dependencies target)
  foreach(dep ${ARGN})
    if(TARGET ${dep}::${dep})
      target_link_libraries(${target} ${dep}::${dep})
    elseif(DEFINED ${dep}_TARGETS)
      target_link_libraries(${target} ${${dep}_TARGETS})
    else()
      target_include_directories(${target} PUBLIC ${${dep}_INCLUDE_DIRS})
      target_link_libraries(${target} ${${dep}_LIBRARIES})
    endif()
  endforeach()
endfunction()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nao_lola_sensor_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
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
  yaml-cpp
  ament_index_cpp
)

include_directories(include)

# BT Nodes plugin library
add_library(social_bt_nodes_plugin SHARED
  src/bt_nodes/perception/is_target_detected.cpp
  src/bt_nodes/perception/set_perception_target.cpp
  src/bt_nodes/perception/get_bearing.cpp
  src/bt_nodes/perception/get_distance.cpp
  src/bt_nodes/perception/is_aligned.cpp
  src/bt_nodes/perception/is_farther_than.cpp
  src/bt_nodes/perception/is_target_static.cpp
  src/bt_nodes/motion/spin.cpp
  src/bt_nodes/motion/follow.cpp
  src/bt_nodes/motion/nao/nao_follow.cpp
  src/bt_nodes/motion/nao/nao_follow_dynamic.cpp
  src/bt_nodes/motion/navigate_to.cpp
  src/bt_nodes/motion/get_nav_location.cpp
  src/bt_nodes/motion/move_forward.cpp
  src/bt_nodes/motion/move_towards.cpp
  src/bt_nodes/motion/rotate_to_bearing.cpp
  src/bt_nodes/interaction/speak.cpp
  src/bt_nodes/interaction/speak_reactive.cpp
  src/bt_nodes/interaction/speak_enum.cpp
  src/bt_nodes/interaction/ask_open_question.cpp
  src/bt_nodes/interaction/listen.cpp
  src/bt_nodes/interaction/extract.cpp
  src/bt_nodes/interaction/ask_yes_no_question.cpp
  src/bt_nodes/interaction/nao/nao_position.cpp
  src/bt_nodes/interaction/nao/nao_set_leds.cpp
  src/bt_nodes/support/is_available.cpp
  src/bt_nodes/support/set_ros2_param.cpp
  src/bt_nodes/support/is_true.cpp
  src/bt_nodes/support/stop_current_task.cpp
  src/bt_nodes/support/force_plan_fail.cpp
  src/bt_nodes/bt_plugins.cpp
)

link_ament_dependencies(social_bt_nodes_plugin ${dependencies})
target_link_libraries(social_bt_nodes_plugin yaml-cpp)

# Required for BT_REGISTER_NODES macro to work properly
target_compile_definitions(social_bt_nodes_plugin PRIVATE BT_PLUGIN_EXPORT)

# Main executable
add_executable(social_bt_nodes_main src/main.cpp)
link_ament_dependencies(social_bt_nodes_main ${dependencies})
target_link_libraries(social_bt_nodes_main yaml-cpp)

# Utility executable: load key/value YAML into BT blackboard for testing
add_executable(bt_blackboard_from_yaml src/bt_blackboard_from_yaml.cpp)
link_ament_dependencies(bt_blackboard_from_yaml behaviortree_cpp)
target_link_libraries(bt_blackboard_from_yaml yaml-cpp)

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

install(TARGETS
  bt_blackboard_from_yaml
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
  node_descriptions/
  DESTINATION share/${PROJECT_NAME}/node_descriptions
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
