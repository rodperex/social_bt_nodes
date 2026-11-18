# social_bt_nodes - Plugin Architecture

## Architecture Overview

The package uses a plugin-based architecture following BehaviorTree.CPP best practices for ROS 2 integration.

### Main Components

1. **main.cpp** - Central orchestrator that:
   - Creates a single ROS 2 node (`social_bt_nodes`)
   - Stores the node in the BehaviorTree blackboard
   - Loads BT nodes as plugins from shared library
   - Ticks the behavior tree periodically
   - Handles ROS callbacks with `rclcpp::spin_some()`
   - Supports optional Groot2 monitoring

2. **BT Node Plugins** - All BT nodes are dynamically loadable plugins:
   - `NaoFollow` (XML: `Follow`) - Basic follow with obstacle avoidance
   - `NaoFollowDynamic` (XML: `FollowDynamic`) - Advanced follow with PID control
   - `SpinSearchAction` (XML: `SpinSearch`) - Spins to search for target
   - `IsTargetDetectedCondition` (XML: `IsTargetDetected`) - Checks if target TF exists

3. **Plugin Library** - `libsocial_bt_nodes_plugin.so`:
   - Contains all BT node implementations
   - Registered via `bt_plugins.cpp` using `BT_REGISTER_NODES` macro
   - Dynamically loadable at runtime

### Key Design Principles

- **Single Shared ROS Node**: All BT nodes share one ROS 2 node via blackboard
- **Blackboard Access**: BT nodes access node via `config().blackboard->get<rclcpp::Node::SharedPtr>("node")`
- **Dynamic Loading**: Nodes loaded dynamically as plugins at runtime
- **Clean Separation**: Clear separation between orchestration (main.cpp) and behavior (plugins)

### Benefits

1. **Resource Efficiency**: Only one ROS node instead of multiple nodes
2. **Modularity**: BT nodes are true plugins, loadable at runtime
3. **Cleaner Code**: Simple `main.cpp` orchestrates, plugins implement behaviors
4. **Better Architecture**: Follows BT.CPP best practices for ROS integration
5. **Reusability**: Plugin can be loaded by other BT executors or applications
6. **Flexibility**: Behavior trees can be modified via XML without recompilation

### File Structure

```
social_bt_nodes/
├── src/
│   ├── main.cpp                                        # Main orchestrator
│   └── bt_nodes/
│       ├── bt_plugins.cpp                              # Plugin registration
│       ├── motion/
│       │   ├── nao_follow.cpp                          # Basic follow
│       │   ├── nao_follow_dynamic.cpp                  # Advanced follow with PID
│       │   ├── spin.cpp                                # Spin search
│       │   └── navigate_to.cpp                         # Nav2 navigation
│       ├── perception/
│       │   └── is_target_detected.cpp                  # Target detection check
│       └── interaction/
│           ├── speak.cpp                               # Text-to-speech (/tts_action)
│           ├── speak_enum.cpp                          # Enumerated speech (/tts_action)
│           ├── listen.cpp                              # Speech recognition (/stt_action)
│           ├── extract.cpp                             # Information extraction (/extract_service)
│           └── ask_yes_no_question.cpp                 # Yes/No question (actions + /yesno_service)
├── include/social_bt_nodes/bt_nodes/
│   ├── motion/
│   │   ├── nao_follow.hpp
│   │   ├── nao_follow_dynamic.hpp
│   │   ├── spin_search.hpp
│   │   └── navigate_to.hpp
│   ├── perception/
│   │   └── is_target_detected.hpp
│   ├── interaction/
│   │   ├── hri_action_call.hpp                     # Non-blocking goal of a simple_hri action
│   │   ├── speak.hpp
│   │   ├── speak_enum.hpp
│   │   ├── listen.hpp
│   │   ├── extract.hpp
│   │   └── ask_yes_no_question.hpp
│   └── pid_controller.hpp                              # PID utility
├── social_bt_nodes/                                    # Python nodes
│   ├── entity_tracker_fake_3d.py                       # TF publisher
│   └── yolo_to_standard.py                             # YOLO converter
├── config/
│   ├── follow_behavior.xml                             # Basic BT config
│   └── follow_behavior_dynamic.xml                     # Advanced BT config
├── launch/
│   ├── follow_behavior.launch.py                       # BT launcher
│   └── tracker.launch.py                               # Detection pipeline
├── CMakeLists.txt                                      # Build configuration
├── package.xml                                         # Package metadata
├── setup.py                                            # Python setup
├── README.md                                           # Main documentation
└── PLUGIN_ARCHITECTURE.md                              # This file
```

## Launch Parameters

### main.cpp Parameters

- `bt_xml` (string, **required**): Path to behavior tree XML file
- `bt_loop_duration` (int, default: 100): Tick rate in milliseconds
- `plugin_list` (string array, default: []): List of plugin library paths to load

### Usage Examples

**Basic usage:**
```bash
ros2 launch social_bt_nodes follow_behavior.launch.py
```

**Manual execution:**
```bash
ros2 run social_bt_nodes social_bt_nodes_main \
  --ros-args \
  -p bt_xml:=/path/to/follow_behavior.xml \
  -p bt_loop_duration:=100 \
  -p plugin_list:=[/path/to/libsocial_bt_nodes_plugin.so]
```

**Complete system:**
```bash
# Terminal 1: Detection and tracking
ros2 launch social_bt_nodes tracker.launch.py

# Terminal 2: Behavior tree execution
ros2 launch social_bt_nodes follow_behavior.launch.py
```

The launch file automatically:
- Locates the plugin library in the installation directory
- Passes the BT XML path from the config directory
- Sets appropriate tick rate (500ms by default)

## Plugin Implementation Details

### Plugin Registration (bt_plugins.cpp)

```cpp
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<social_bt_nodes::NaoFollow>("Follow");
  factory.registerNodeType<social_bt_nodes::NaoFollowDynamic>("FollowDynamic");
  factory.registerNodeType<social_bt_nodes::SpinSearchAction>("SpinSearch");
  factory.registerNodeType<social_bt_nodes::IsTargetDetectedCondition>("IsTargetDetected");
}
```

### Accessing ROS Node in Plugins

All plugins access the shared ROS node from the blackboard:

```cpp
node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
```

This allows:
- Creating publishers/subscribers
- Using TF buffers
- Logging
- Parameter access
- All ROS functionality

### Node Categories

**Motion Nodes** (`bt_nodes/motion/`):
- Control robot movement
- Integrate sonar and touch sensors
- Publish velocity commands

**Perception Nodes** (`bt_nodes/perception/`):
- Process sensor data
- Check TF frames
- Detect conditions

## CMakeLists.txt Configuration

Key sections for plugin architecture:

```cmake
# Build plugin library as SHARED
add_library(social_bt_nodes_plugin SHARED
  src/bt_nodes/perception/is_target_detected.cpp
  src/bt_nodes/motion/spin.cpp
  src/bt_nodes/motion/nao_follow.cpp
  src/bt_nodes/motion/nao_follow_dynamic.cpp
  src/bt_nodes/motion/navigate_to.cpp
  src/bt_nodes/interaction/speak.cpp
  src/bt_nodes/interaction/speak_enum.cpp
  src/bt_nodes/interaction/listen.cpp
  src/bt_nodes/interaction/extract.cpp
  src/bt_nodes/interaction/ask_yes_no_question.cpp
  src/bt_nodes/bt_plugins.cpp
)

# Required for BT_REGISTER_NODES to work
target_compile_definitions(social_bt_nodes_plugin PRIVATE BT_PLUGIN_EXPORT)

# Main executable (separate from plugin)
add_executable(social_bt_nodes_main src/main.cpp)
```

## Adding New BT Nodes

To add a new BehaviorTree node plugin:

1. **Create header file** in `include/social_bt_nodes/bt_nodes/[category]/`
2. **Create implementation** in `src/bt_nodes/[category]/`
3. **Register in bt_plugins.cpp**:
   ```cpp
   factory.registerNodeType<social_bt_nodes::MyNewNode>("MyNodeName");
   ```
4. **Add to CMakeLists.txt** plugin library sources
5. **Use in XML**:
   ```xml
   <MyNodeName param1="value1" param2="value2"/>
   ```

## XML Configuration Examples

### Basic Follow Behavior

```xml
<BehaviorTree ID="FollowBehavior">
  <ReactiveFallback name="MainBehavior">
    <ReactiveSequence name="FollowIfDetected">
      <IsTargetDetected 
        target_frame="target"
        base_frame="base_link"
        timeout="3.0"/>
      <Follow 
        target_frame="target"
        base_frame="base_link"
        min_distance="0.5"
        avoidance_distance="0.2"
        max_linear_speed="1.0"
        max_angular_speed="0.5"
        cmd_vel_topic="/target"
        sonar_topic="/sensors/sonar"
        touch_topic="/sensors/touch"/>
    </ReactiveSequence>
    <SpinSearch 
      angular_speed="1.0"
      cmd_vel_topic="/target"/>
  </ReactiveFallback>
</BehaviorTree>
```

### Advanced Follow with PID

```xml
<BehaviorTree ID="FollowBehavior">
  <ReactiveFallback name="MainBehavior">
    <ReactiveSequence name="FollowIfDetected">
      <IsTargetDetected 
        target_frame="target"
        base_frame="base_link"
        timeout="3.0"/>
      <FollowDynamic 
        target_frame="target"
        base_frame="base_link"
        min_distance="0.5"
        avoidance_distance="1.5"
        danger_distance="0.5"
        max_linear_speed="1.0"
        max_angular_speed="1.0"
        linear_vel_strategy="pid"
        cmd_vel_topic="/target"
        sonar_topic="/sensors/sonar"
        touch_topic="/sensors/touch"/>
    </ReactiveSequence>
    <SpinSearch 
      angular_speed="1.0"
      cmd_vel_topic="/target"/>
  </ReactiveFallback>
</BehaviorTree>
```

## Benefits of This Architecture

### For Development
- Easy to add new BT nodes without modifying main.cpp
- Clear separation of concerns
- Testable components
- Organized by category (motion, perception, etc.)

### For Deployment
- Single executable with plugin loading
- XML files can be modified without recompilation
- Easy to share and reuse nodes across projects
- Minimal runtime overhead

### For Research
- Quick experimentation with different behaviors via XML
- Standard plugin interface for community contributions
- Compatible with BT.CPP ecosystem tools (Groot2, etc.)
- Easy to integrate with other ROS 2 packages

## Debugging and Development

### Enabling Groot2 Monitoring

Uncomment in `main.cpp`:
```cpp
std::unique_ptr<BT::Groot2Publisher> groot_publisher;
groot_publisher = std::make_unique<BT::Groot2Publisher>(tree);
```

### Checking Plugin Loading

```bash
# Verify plugin library exists
ls -l install/social_bt_nodes/lib/libsocial_bt_nodes_plugin.so

# Check symbols
nm -D install/social_bt_nodes/lib/libsocial_bt_nodes_plugin.so | grep BT

# Run with verbose output
ros2 run social_bt_nodes social_bt_nodes_main --ros-args --log-level debug
```

### Common Issues

**Plugin not loading:**
- Ensure library path is correct in `plugin_list` parameter
- Verify `BT_PLUGIN_EXPORT` is defined during compilation
- Check that all dependencies are linked

**Node not found in XML:**
- Verify node name matches registration in `bt_plugins.cpp`
- Check XML syntax is correct
- Ensure plugin is loaded before creating tree

**ROS node not accessible:**
- Confirm node is placed in blackboard in `main.cpp`
- Check blackboard key is "node" (lowercase)
- Verify `config().blackboard` is accessed correctly

## Performance Considerations

- **Single Node**: Reduces IPC overhead compared to multiple nodes
- **Plugin Loading**: One-time cost at startup
- **Tick Rate**: Adjust `bt_loop_duration` based on robot capabilities
- **Spin Some**: Non-blocking ROS callback processing

## Future Enhancements

- [ ] Dynamic plugin reloading without restart
- [ ] Hot-swapping behavior trees at runtime
- [ ] Plugin discovery mechanism
- [ ] BT node performance profiling
- [ ] Additional node categories (HRI, navigation, manipulation)
