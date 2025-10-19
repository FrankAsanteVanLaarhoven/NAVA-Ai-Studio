---
id: terminal-commands
title: ROS Terminal Commands Reference
sidebar_label: Terminal Commands
---

# ROS Terminal Commands Reference

**Complete guide to ROS2 command-line tools with NAVΛ Studio's integrated terminal**

## 🚀 Overview

NAVΛ Studio includes 17+ integrated ROS2 commands that you can execute directly in the browser. This reference provides comprehensive documentation for each command with examples, use cases, and best practices.

## 📋 Command Categories

### Node Management Commands
Commands for managing ROS2 nodes and node lifecycle.

#### `ros2 node list`
List all active ROS2 nodes in the system.

**Syntax:**
```bash
ros2 node list
```

**Example:**
```bash
$ ros2 node list
/talker
/listener
/teleop_turtle
/turtlesim
```

**Use Cases:**
- System debugging and monitoring
- Node relationship verification
- Active process identification

**Related Commands:**
- `ros2 node info` - Get detailed node information
- `ros2 node kill` - Terminate specific nodes

#### `ros2 node info <node_name>`
Display detailed information about a specific node.

**Syntax:**
```bash
ros2 node info <node_name>
```

**Example:**
```bash
$ ros2 node info /turtlesim
Node: /turtlesim
Namespace: /
Type: turtlesim/turtlesim_node
Subscriptions:
  /parameter_events: rcl_interfaces/msg/ParameterEvent
  /turtle1/cmd_vel: geometry_msgs/msg/Twist
Publishers:
  /parameter_events: rcl_interfaces/msg/ParameterEvent
  /rosout: rcl_interfaces/msg/Log
  /turtle1/color_sensor: turtlesim/msg/Color
  /turtle1/pose: turtlesim/msg/Pose
Services:
  /clear: std_srvs/srv/Empty
  /kill: turtlesim/srv/Kill
  /reset: std_srvs/srv/Empty
  /spawn: turtlesim/srv/Spawn
  /turtle1/set_pen: turtlesim/srv/SetPen
  /turtle1/teleport_absolute: turtlesim/srv/TeleportAbsolute
  /turtle1/teleport_relative: turtlesim/srv/TeleportRelative
```

**Use Cases:**
- Node debugging and inspection
- Topic and service relationship analysis
- System architecture understanding

### Topic Management Commands
Commands for working with ROS2 topics and messages.

#### `ros2 topic list`
List all active topics in the system.

**Syntax:**
```bash
ros2 topic list
```

**Options:**
- `-t`: Show topic types

**Example:**
```bash
$ ros2 topic list
/chatter
/parameter_events
/rosout
/turtle1/cmd_vel
/turtle1/color_sensor
/turtle1/pose

$ ros2 topic list -t
/chatter [std_msgs/msg/String]
/parameter_events [rcl_interfaces/msg/ParameterEvent]
/rosout [rcl_interfaces/msg/Log]
/turtle1/cmd_vel [geometry_msgs/msg/Twist]
/turtle1/color_sensor [turtlesim/msg/Color]
/turtle1/pose [turtlesim/msg/Pose]
```

**Use Cases:**
- System communication overview
- Message flow identification
- Topic debugging and monitoring

#### `ros2 topic echo <topic_name>`
Display messages published to a specific topic.

**Syntax:**
```bash
ros2 topic echo <topic_name> [message_field]
```

**Options:**
- `--no-arr`: Don't print array fields
- `--no-str`: Don't print string fields
- `--flow-style`: Print in flow style
- `--no-delim`: Don't print delimiters

**Example:**
```bash
$ ros2 topic echo /chatter
data: 'Hello NAVΛ Studio: 1'
---
data: 'Hello NAVΛ Studio: 2'
---
data: 'Hello NAVΛ Studio: 3'
---

$ ros2 topic echo /turtle1/pose
x: 5.544444561004639
theta: 0.0
y: 5.544444561004639
linear_velocity: 0.0
angular_velocity: 0.0
---
```

**Use Cases:**
- Message content inspection
- Real-time data monitoring
- Debugging communication issues

#### `ros2 topic info <topic_name>`
Display detailed information about a specific topic.

**Syntax:**
```bash
ros2 topic info <topic_name>
```

**Options:**
- `-v`: Verbose output

**Example:**
```bash
$ ros2 topic info /chatter
Type: std_msgs/msg/String
Publisher count: 1
Subscription count: 1

$ ros2 topic info -v /chatter
Type: std_msgs/msg/String
Publisher count: 1
Node name: talker
Node namespace: /
Topic type: std_msgs/msg/String
Endpoint type: PUBLISHER
GID: 01.0f.34.04.7c.59.00.00.01.00.00.00.00.00.00.03.00.00.00.00.00.00.00.00
QoS profile:
  Reliability: RELIABLE
  History (Depth): UNKNOWN
  Durability: VOLATILE
  Lifespan: Infinite
  Deadline: Infinite
  Liveliness: AUTOMATIC
  Liveliness lease duration: Infinite

Subscription count: 1
Node name: listener
Node namespace: /
Topic type: std_msgs/msg/String
Endpoint type: SUBSCRIPTION
GID: 01.0f.34.04.7c.59.00.00.01.00.00.00.00.00.00.04.00.00.00.00.00.00.00.00
QoS profile:
  Reliability: RELIABLE
  History (Depth): UNKNOWN
  Durability: VOLATILE
  Lifespan: Infinite
  Deadline: Infinite
  Liveliness: AUTOMATIC
  Liveliness lease duration: Infinite
```

**Use Cases:**
- Topic debugging and analysis
- QoS profile inspection
- Publisher-subscriber relationship verification

#### `ros2 topic pub <topic_name> <msg_type> <msg_data>`
Publish a message to a topic.

**Syntax:**
```bash
ros2 topic pub <topic_name> <msg_type> <msg_data> [options]
```

**Options:**
- `-r <rate>`: Publishing rate (Hz)
- `-t <count>`: Number of messages to publish
- `-w <seconds>`: Wait time before starting
- `-p`: Print each message

**Example:**
```bash
# Single message
$ ros2 topic pub /chatter std_msgs/msg/String "data: 'Hello from NAVΛ Studio'"

# Continuous publishing at 10 Hz
$ ros2 topic pub -r 10 /chatter std_msgs/msg/String "data: 'Continuous message'"

# Publish 5 messages at 2 Hz
$ ros2 topic pub -r 2 -t 5 /chatter std_msgs/msg/String "data: 'Limited messages'"

# Complex message with YAML syntax
$ ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "
linear:
  x: 1.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.5"
```

**Use Cases:**
- Testing subscriber nodes
- Message format validation
- System stimulation and testing

### Service Management Commands
Commands for working with ROS2 services.

#### `ros2 service list`
List all active services in the system.

**Syntax:**
```bash
ros2 service list
```

**Options:**
- `-t`: Show service types

**Example:**
```bash
$ ros2 service list
/add_two_ints
/clear
/kill
/reset
/spawn
/turtle1/set_pen
/turtle1/teleport_absolute
/turtle1/teleport_relative

$ ros2 service list -t
/add_two_ints [example_interfaces/srv/AddTwoInts]
/clear [std_srvs/srv/Empty]
/kill [turtlesim/srv/Kill]
/reset [std_srvs/srv/Empty]
/spawn [turtlesim/srv/Spawn]
/turtle1/set_pen [turtlesim/srv/SetPen]
/turtle1/teleport_absolute [turtlesim/srv/TeleportAbsolute]
/turtle1/teleport_relative [turtlesim/srv/TeleportRelative]
```

#### `ros2 service call <service_name> <service_type> <request_data>`
Call a service with specific request data.

**Syntax:**
```bash
ros2 service call <service_name> <service_type> <request_data>
```

**Example:**
```bash
# Simple service call
$ ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts '{a: 5, b: 3}'
requester: making request: example_interfaces.srv.AddTwoInts_Request(a=5, b=3)

response:
example_interfaces.srv.AddTwoInts_Response(sum=8)

# Service with empty request
$ ros2 service call /clear std_srvs/srv/Empty {}
requester: making request: std_srvs.srv.Empty_Request()

response:
std_srvs.srv.Empty_Response()

# Complex service with multiple parameters
$ ros2 service call /spawn turtlesim/srv/Spawn '{x: 2.0, y: 2.0, theta: 0.0, name: "turtle2"}'
requester: making request: turtlesim.srv.Spawn_Request(x=2.0, y=2.0, theta=0.0, name='turtle2')

response:
turtlesim.srv.Spawn_Response(name='turtle2')
```

**Use Cases:**
- Service testing and validation
- System interaction and control
- Debugging service implementations

### Parameter Management Commands
Commands for managing ROS2 parameters.

#### `ros2 param list`
List parameters for all nodes or a specific node.

**Syntax:**
```bash
ros2 param list [node_name]
```

**Example:**
```bash
$ ros2 param list
/parameter_demo:
  debug_mode
  max_speed
  robot_name
  use_sim_time
/rosout:
  qos_overrides./parameter_events.publisher.depth
  qos_overrides./parameter_events.publisher.durability
  qos_overrides./parameter_events.publisher.history
  qos_overrides./parameter_events.publisher.reliability
  start_type_description_service
  use_sim_time
```

#### `ros2 param get <node_name> <parameter_name>`
Get the value of a specific parameter.

**Syntax:**
```bash
ros2 param get <node_name> <parameter_name>
```

**Example:**
```bash
$ ros2 param get /parameter_demo robot_name
String value is: NAVΛBot

$ ros2 param get /parameter_demo max_speed
Double value is: 1.0

$ ros2 param get /parameter_demo debug_mode
Boolean value is: True
```

#### `ros2 param set <node_name> <parameter_name> <value>`
Set the value of a specific parameter.

**Syntax:**
```bash
ros2 param set <node_name> <parameter_name> <value>
```

**Example:**
```bash
$ ros2 param set /parameter_demo robot_name "MyRobot"
Set parameter successful

$ ros2 param set /parameter_demo max_speed 2.5
Set parameter successful

$ ros2 param set /parameter_demo debug_mode False
Set parameter successful
```

#### `ros2 param dump <node_name>`
Save node parameters to a YAML file.

**Syntax:**
```bash
ros2 param dump <node_name> [file_name]
```

**Example:**
```bash
$ ros2 param dump /parameter_demo
/parameter_demo:
  ros__parameters:
    debug_mode: true
    max_speed: 1.0
    robot_name: "NAVΛBot"
    use_sim_time: false

$ ros2 param dump /parameter_demo navlambda_params.yaml
```

### Package Management Commands
Commands for managing ROS2 packages.

#### `ros2 pkg list`
List all installed ROS2 packages.

**Syntax:**
```bash
ros2 pkg list
```

**Options:**
- `--dependencies`: Show package dependencies

**Example:**
```bash
$ ros2 pkg list | head -10
action_msgs
action_tutorials_cpp
action_tutorials_interfaces
action_tutorials_py
actionlib_msgs
angles
builtin_interfaces
composition
composition_interfaces
cpp_pubsub
```

#### `ros2 pkg prefix <package_name>`
Get the installation prefix of a package.

**Syntax:**
```bash
ros2 pkg prefix <package_name>
```

**Example:**
```bash
$ ros2 pkg prefix turtlesim
/opt/ros/humble

$ ros2 pkg prefix navlambda_tutorials
/home/user/ros2_ws/install
```

#### `ros2 pkg executables <package_name>`
List executables in a package.

**Syntax:**
```bash
ros2 pkg executables <package_name>
```

**Example:**
```bash
$ ros2 pkg executables turtlesim
turtlesim mimic
turtlesim turtle_teleop_key
turtlesim turtlesim_node
```

### Launch System Commands
Commands for working with ROS2 launch files.

#### `ros2 launch <package_name> <launch_file>`
Launch nodes using a launch file.

**Syntax:**
```bash
ros2 launch <package_name> <launch_file> [arguments]
```

**Example:**
```bash
# Launch turtlesim with teleop
$ ros2 launch turtlesim multisim.launch.py

# Launch with arguments
$ ros2 launch my_package my_launch.py robot_name:="MyRobot" max_speed:=2.0

# Launch Gazebo simulation
$ ros2 launch gazebo_ros empty_world.launch.py

# Launch with debug output
$ ros2 launch my_package my_launch.py --debug
```

**Common Arguments:**
- `--debug`: Enable debug output
- `--show-args`: Show available arguments
- `--show-deprecated`: Show deprecated arguments

### Bag Commands
Commands for recording and playing back ROS2 data.

#### `ros2 bag record <topics>`
Record messages from specified topics.

**Syntax:**
```bash
ros2 bag record <topics> [options]
```

**Options:**
- `-a`: Record all topics
- `-o <bag_name>`: Output bag file name
- `-s <storage_id>`: Storage format (sqlite3, mcap)

**Example:**
```bash
# Record specific topics
$ ros2 bag record /chatter /turtle1/pose

# Record all topics
$ ros2 bag record -a

# Record with custom name
$ ros2 bag record -a -o navlambda_session

# Record for 10 seconds
$ ros2 bag record -a -o test_bag &
$ sleep 10
$ pkill -f ros2 bag record
```

#### `ros2 bag play <bag_file>`
Play back recorded messages from a bag file.

**Syntax:**
```bash
ros2 bag play <bag_file> [options]
```

**Options:**
- `-r <rate>`: Playback rate multiplier
- `--loop`: Loop playback
- `--remap <old_topic>:<new_topic>`: Remap topics

**Example:**
```bash
# Play bag file
$ ros2 bag play navlambda_session

# Play at double speed
$ ros2 bag play navlambda_session -r 2

# Play in loop
$ ros2 bag play navlambda_session --loop

# Play with topic remapping
$ ros2 bag play navlambda_session --remap /chatter:/old_chatter
```

## 🔧 Advanced Usage

### Command Chaining
Combine multiple ROS2 commands for complex operations:

```bash
# Monitor node creation and topic changes
ros2 node list && ros2 topic list

# Check all communication for a specific node
ros2 node info /turtlesim | grep -E "(Publishers|Subscriptions)" | while read line; do echo "$line"; done

# Record all topics from a specific node
ros2 node info /turtlesim | grep "Publishers" -A 20 | grep "/" | cut -d: -f1 | xargs ros2 bag record
```

### Script Integration
Use ROS2 commands in shell scripts:

```bash
#!/bin/bash
# monitor_ros_system.sh

echo "=== ROS2 System Monitor ==="
echo "Active Nodes: $(ros2 node list | wc -l)"
echo "Active Topics: $(ros2 topic list | wc -l)"
echo "Active Services: $(ros2 service list | wc -l)"

# Check if critical nodes are running
if ros2 node list | grep -q "/navigation"; then
    echo "✅ Navigation node is running"
else
    echo "❌ Navigation node is not running"
fi

# Monitor topic rates
if command -v ros2 topic hz &> /dev/null; then
    echo "=== Topic Rates ==="
    for topic in $(ros2 topic list | grep -E "(cmd_vel|odom|scan)"); do
        echo "Topic: $topic"
        timeout 5 ros2 topic hz "$topic" 2>/dev/null | tail -1 || echo "  Rate: unknown"
    done
fi
```

### Debugging Techniques
Advanced debugging with ROS2 commands:

```bash
# Find nodes with specific topics
ros2 node list | while read node; do
    if ros2 node info "$node" | grep -q "/scan"; then
        echo "Node $node publishes/subscribes to /scan"
    fi
done

# Monitor parameter changes
ros2 param list | while read param; do
    echo "=== $param ==="
    ros2 param get "$param" 2>/dev/null || echo "  Unable to get parameter"
done

# Check message compatibility
ros2 topic info /chatter | grep "Type:" | while read type; do
    echo "Topic type: $type"
    ros2 interface show "${type#Type: }" 2>/dev/null || echo "  Interface not found"
done
```

## 📈 Performance Tips

### Efficient Command Usage
- Use specific commands instead of broad listings when possible
- Combine related operations to reduce overhead
- Use command options to filter output
- Leverage shell features for command chaining

### Monitoring Best Practices
- Set up automated monitoring scripts
- Use `timeout` command to prevent hanging
- Implement error handling for robust scripts
- Log command outputs for historical analysis

### Debugging Strategies
- Start with high-level overview commands
- Drill down to specific components as needed
- Use `ros2 doctor` for system health checks
- Cross-reference multiple command outputs

## 🔀 Related Documentation

- [ROS2 Basics Tutorial](basics)
- [Advanced Navigation Course](navigation)
- [Gazebo Simulation Guide](simulation)
- [API Reference](../api/ros/commands)

---

*Master these commands to become proficient with ROS2 system management and debugging. Practice regularly with the integrated terminal in NAVΛ Studio.*