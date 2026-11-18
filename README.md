# social_bt_nodes

ROS 2 package providing reusable BehaviorTree.CPP action and condition nodes for social robotics applications.

The package includes:

- A plugin library: libsocial_bt_nodes_plugin.so
- A standalone executor: social_bt_nodes_main
- Node descriptions for llm_bt_builder under node_descriptions/

## Package contents

```text
social_bt_nodes/
├── include/social_bt_nodes/bt_nodes/
│   ├── interaction/      # Speak, SpeakEnum, Listen, YesNo, Ask, Extract, NaoPosition, NaoSetLeds
│   ├── motion/           # Follow, FollowDynamic, Spin, GetNavLocation, NavigateTo
│   ├── perception/       # IsDetected, SetPerceptionTarget
│   └── support/          # SetRos2Param, StopCurrentTask
├── src/bt_nodes/         # corresponding .cpp implementations + bt_plugins.cpp
├── config/               # example BT XML files
├── launch/               # example launch files
└── node_descriptions/
    ├── social_bt_nodes.yaml
    └── all_social_bt_nodes.yaml
```

## Registered BT nodes (XML names)

The factory registers the following XML node names:

- Interaction: Speak, SpeakEnum, Listen, YesNo, Ask, Extract, NaoPosition, NaoSetLeds
- Motion: Follow, FollowDynamic, Spin, GetNavLocation, NavigateTo
- Perception: IsDetected, SetPerceptionTarget
- Support: SetRos2Param, StopCurrentTask

Note: the C++ class for IsDetected is IsTargetDetected, but the XML tag to use is IsDetected.

## Port reference

### Interaction

| Node | Required input ports | Optional input ports | Output ports |
| --- | --- | --- | --- |
| Speak | text | action_name (/tts_action), timeout (5000 ms) | - |
| SpeakReactive | text | action_name (/tts_action), timeout (5000 ms) | - |
| SpeakEnum | list | separator (,), language (es), action_name (/tts_action), timeout (5000 ms) | - |
| Listen | - | action_name (/stt_action), max_wait (0.0 s = server default), timeout (10000 ms) | transcribed_text |
| AskYesNoQuestion | question | tts_action_name (/tts_action), stt_action_name (/stt_action), service_name (/yesno_service), max_wait (0.0 s), timeout (10000 ms) | confirmed |
| AskOpenQuestion | question | tts_action_name (/tts_action), stt_action_name (/stt_action), max_wait (0.0 s), timeout (10000 ms) | answer |
| Extract | interest, text | service_name (/extract_service), timeout (10000 ms) | extracted_info |
| NaoPosition | action_name | action_server (/nao_pos_server), timeout (10.0 s) | success |
| NaoSetLeds | led_ids | mode (0), color_r (1.0), color_g (1.0), color_b (1.0), intensity (1.0), frequency (2.0), duration (-1.0), action_server (leds_play), timeout (10.0 s) | success |

Speaking and listening use the simple_hri actions (`/tts_action`, `/stt_action`): a speaking
node finishes when the audio has really finished, a listening node when the person has finished
speaking, and halting either node cancels the goal (stops the audio or the recording).
`timeout` is the time to wait for the action server, and `max_wait` the seconds to wait for the
person to start speaking. Extracting information and yes/no detection are queries on a text,
so they use the simple_hri services (`/extract_service`, `/yesno_service`).

### Motion

| Node | Required input ports | Optional input ports | Output ports |
| --- | --- | --- | --- |
| Follow | - | target_frame (target), base_frame (base_link), min_distance (1.0), avoidance_distance (0.5), max_linear_speed (0.5), max_angular_speed (1.0), succeed_on_reach (false), cmd_vel_topic (/cmd_vel), sonar_topic (/sensors/sonar), touch_topic (/sensors/touch), rotation_stop_threshold (0.087), linear_stop_threshold (0.26) | - |
| FollowDynamic | - | target_frame (target), base_frame (base_link), min_distance (1.0), avoidance_distance (0.5), danger_distance (0.3), max_linear_speed (0.5), max_angular_speed (1.0), succeed_on_reach (false), cmd_vel_topic (/cmd_vel), sonar_topic (/sensors/sonar), touch_topic (/sensors/touch), linear_vel_strategy (proportional) | - |
| Spin | - | angular_speed (0.5), cmd_vel_topic (/cmd_vel_muxed) | - |
| GetNavLocation | location_description | - | location_frame |
| NavigateTo | x+y or target_frame | yaw (0.0), frame_id (map), action_name (navigate_to_pose), timeout (300.0 s) | error_msg |

### Perception

| Node | Required input ports | Optional input ports | Output ports |
| --- | --- | --- | --- |
| IsDetected | - | target_frame (target), base_frame (base_link), timeout (0.5 s) | detected_frame |
| SetPerceptionTarget | target | service_name (/set_perception_target) | frame_id |

### Support

| Node | Required input ports | Optional input ports | Output ports |
| --- | --- | --- | --- |
| SetRos2Param | node_name, param_name, param_value | param_type (string), timeout (2000 ms) | success |
| StopCurrentTask | - | event_topic (/stop_current_task), qos_depth (10) | - |

## Runtime behavior notes

- Most service/action based nodes are asynchronous StatefulActionNode implementations and can return RUNNING while waiting.
- Spin is an asynchronous action that keeps rotating and returns RUNNING until halted.
- StopCurrentTask subscribes to an Empty event topic and returns SUCCESS once per received event, then returns FAILURE until a new event arrives.
- Failure details are written with bt_failure(...) to the blackboard key bt_last_failure.

## social_bt_nodes_main executable

Standalone executor that loads BT plugins and runs an XML-defined tree.

Parameters:

- bt_xml (string, required): path to BT XML
- bt_loop_duration (int, default 100): tick period in ms
- plugin_list (string array, default []): plugin .so paths

Example:

```bash
ros2 run social_bt_nodes social_bt_nodes_main \
  --ros-args \
  -p bt_xml:=$(ros2 pkg prefix social_bt_nodes)/share/social_bt_nodes/config/follow_behavior.xml \
  -p plugin_list:=[libsocial_bt_nodes_plugin.so]
```

## Example behavior trees

- follow_behavior.xml
- follow_behavior_dynamic.xml
- follow_change_example.xml
- follow_change_example_2.xml
- spin_search.xml
- restaurant_order.xml
- nao_hello_example.xml
- nao_leds_demo.xml
- nao_set_leds_examples.xml
- navigate_to_example.xml
- test_tree.xml

## Example launches

- bt.launch.py
- follow_behavior.launch.py
- follow_change_example.launch.py
- follow_change_example_2.launch.py
- restaurant_demo.launch.py
- nao_hello_demo.launch.py
- nao_leds_demo.launch.py
- tracker.launch.py

## Building

```bash
colcon build --packages-select social_bt_nodes
source install/setup.bash
```

Key dependencies include rclcpp, rclcpp_action, behaviortree_cpp, tf2_ros, nav2_msgs, simple_hri_interfaces, simple_perception_interfaces, nao_pos_interfaces, and nao_led_interfaces.

## Using the plugin in another package

```cpp
factory.registerFromPlugin("libsocial_bt_nodes_plugin.so");
```

Or in behavior_architecture YAML:

```yaml
plugin_libraries:
  - "libsocial_bt_nodes_plugin.so"
```

The node description files are installed under:

- share/social_bt_nodes/node_descriptions/

And can be selected in llm_bt_builder with:

```yaml
bt_nodes_package: "social_bt_nodes"
```

## License

Apache License 2.0

## Author

Rodrigo Perez-Rodriguez ([rodrigo.perez@urjc.es](mailto:rodrigo.perez@urjc.es))
