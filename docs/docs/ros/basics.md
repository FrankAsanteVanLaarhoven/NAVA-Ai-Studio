---
id: basics
title: ROS2 Basics Tutorial
sidebar_label: ROS2 Basics
---

# ROS2 Basics Tutorial

**Master the fundamentals of Robot Operating System 2 with interactive lessons and hands-on exercises**

## 🚀 Course Overview

This comprehensive tutorial covers everything you need to know about ROS2 fundamentals. Through interactive lessons and practical exercises, you'll gain hands-on experience with the world's most popular robotics framework.

## 📚 Learning Objectives

By the end of this tutorial, you will be able to:
- Understand ROS2 architecture and core concepts
- Create and manage ROS2 nodes
- Implement publisher-subscriber communication patterns
- Use ROS2 services and actions
- Configure parameters and launch files
- Navigate the ROS2 command-line interface
- Build and manage ROS2 workspaces

## 🔧 Prerequisites

- Basic programming knowledge (Python or C++)
- Familiarity with command-line interfaces
- NAVΛ Studio installed with ROS learning center
- Curiosity about robotics and autonomous systems

## 📃 Course Structure

### Module 1: ROS2 Architecture (45 minutes)
**Understanding the foundation of ROS2**

#### What is ROS2?
ROS2 (Robot Operating System 2) is an open-source framework for robotics development. It provides tools, libraries, and conventions that simplify the task of creating complex and robust robot behavior across a wide variety of robotic platforms.

#### Core Concepts
- **Nodes**: Individual processes that perform computation
- **Topics**: Named buses for message communication
- **Messages**: Data structures passed between nodes
- **Services**: Request-response communication pattern
- **Actions**: Goal-oriented communication with feedback
- **Parameters**: Configuration values for nodes

#### Interactive Exercise: Explore ROS2 Architecture
```bash
# Open the integrated terminal in NAVΛ Studio
# Let's explore the ROS2 architecture together

# First, check if ROS2 is properly installed
ros2 --version

# List available ROS2 commands
ros2 --help

# Explore the ROS2 file structure
ls /opt/ros/humble/
```

#### Knowledge Check
- [ ] What is the difference between ROS and ROS2?
- [ ] Name the five core concepts of ROS2
- [ ] Why is ROS2 important for robotics development?

### Module 2: Creating Your First Node (60 minutes)
**Building your first ROS2 application**

#### Understanding Nodes
A node is a fundamental building block of ROS2. It's an executable that uses ROS2 to communicate with other nodes. Nodes can be publishers, subscribers, service servers, service clients, action servers, or action clients.

#### Creating a Python Node
Let's create your first ROS2 node using Python:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class MyFirstNode(Node):
    def __init__(self):
        super().__init__('my_first_node')
        self.get_logger().info('Hello from NAVΛ Studio ROS Learning Center!')
        
    def destroy_node(self):
        self.get_logger().info('Goodbye from NAVΛ Studio!')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MyFirstNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### Hands-on Exercise: Create and Run Your Node
```bash
# Create a new ROS2 package
ros2 pkg create --build-type ament_python --node-name my_first_node my_first_package

# Navigate to the package directory
cd ~/ros2_ws/src/my_first_package

# Open the node file in NAVΛ Studio editor
# Replace the contents with the code above

# Build the package
cd ~/ros2_ws
colcon build --packages-select my_first_package

# Source the workspace
source install/setup.bash

# Run your node
ros2 run my_first_package my_first_node
```

#### Knowledge Check
- [ ] What is a ROS2 node?
- [ ] How do you create a Python-based ROS2 node?
- [ ] What is the purpose of `rclpy.spin()`?

### Module 3: Publisher-Subscriber Pattern (75 minutes)
**Mastering ROS2 communication**

#### Understanding Publishers and Subscribers
The publisher-subscriber pattern is the most common communication method in ROS2. Publishers send messages to topics, and subscribers receive messages from topics. This creates a decoupled communication system.

#### Creating a Publisher Node
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simple_publisher')
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.i = 0
        
    def timer_callback(self):
        msg = String()
        msg.data = f'Hello NAVΛ Studio: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    publisher = SimplePublisher()
    
    try:
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        pass
    
    publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### Creating a Subscriber Node
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimpleSubscriber(Node):
    def __init__(self):
        super().__init__('simple_subscriber')
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        
    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    subscriber = SimpleSubscriber()
    
    try:
        rclpy.spin(subscriber)
    except KeyboardInterrupt:
        pass
    
    subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### Interactive Exercise: Test Publisher-Subscriber Communication
```bash
# Terminal 1: Run the publisher
ros2 run my_first_package simple_publisher

# Terminal 2: Run the subscriber
ros2 run my_first_package simple_subscriber

# Terminal 3: Monitor the topic
ros2 topic echo /chatter

# Terminal 4: Get topic information
ros2 topic info /chatter

# Terminal 5: List all topics
ros2 topic list
```

#### Knowledge Check
- [ ] What is the publisher-subscriber pattern?
- [ ] How do you create a publisher in ROS2?
- [ ] How do you create a subscriber in ROS2?
- [ ] What is the purpose of the topic name?

### Module 4: ROS2 Services (60 minutes)
**Implementing request-response communication**

#### Understanding Services
Services provide a request-response communication pattern. A client node sends a request to a service server node, which processes the request and sends back a response.

#### Creating a Service Server
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class CalculatorService(Node):
    def __init__(self):
        super().__init__('calculator_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)
        self.get_logger().info('Calculator service ready!')
        
    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Incoming request\na: {request.a} b: {request.b}')
        self.get_logger().info(f'Sending back response: [{response.sum}]')
        return response

def main(args=None):
    rclpy.init(args=args)
    calculator_service = CalculatorService()
    
    try:
        rclpy.spin(calculator_service)
    except KeyboardInterrupt:
        pass
    
    calculator_service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### Creating a Service Client
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts
import sys

class CalculatorClient(Node):
    def __init__(self):
        super().__init__('calculator_client')
        self.client = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = AddTwoInts.Request()
        
    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.client.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    calculator_client = CalculatorClient()
    
    try:
        response = calculator_client.send_request(int(sys.argv[1]), int(sys.argv[2]))
        calculator_client.get_logger().info(f'Result of {sys.argv[1]} + {sys.argv[2]} = {response.sum}')
    except IndexError:
        calculator_client.get_logger().error('Please provide two integers as arguments')
    except Exception as e:
        calculator_client.get_logger().error(f'Service call failed: {e}')
    
    calculator_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### Interactive Exercise: Test Service Communication
```bash
# Terminal 1: Run the service server
ros2 run my_first_package calculator_service

# Terminal 2: Call the service using the client
ros2 run my_first_package calculator_client 5 7

# Terminal 3: Call the service using command line
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts '{a: 10, b: 20}'

# Terminal 4: List all services
ros2 service list

# Terminal 5: Get service information
ros2 service info /add_two_ints
```

#### Knowledge Check
- [ ] What is the difference between topics and services?
- [ ] When should you use services instead of topics?
- [ ] How do you create a service server?
- [ ] How do you create a service client?

### Module 5: ROS2 Parameters (45 minutes)
**Managing configuration data**

#### Understanding Parameters
Parameters provide a way to configure nodes at runtime. They can be integers, floats, booleans, strings, or arrays of these types.

#### Creating a Node with Parameters
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

class ParameterDemo(Node):
    def __init__(self):
        super().__init__('parameter_demo')
        
        # Declare parameters with default values
        self.declare_parameter('robot_name', 'NAVΛBot')
        self.declare_parameter('max_speed', 1.0)
        self.declare_parameter('debug_mode', True)
        
        # Get parameter values
        self.robot_name = self.get_parameter('robot_name').value
        self.max_speed = self.get_parameter('max_speed').value
        self.debug_mode = self.get_parameter('debug_mode').value
        
        self.get_logger().info(f'Robot name: {self.robot_name}')
        self.get_logger().info(f'Max speed: {self.max_speed} m/s')
        self.get_logger().info(f'Debug mode: {self.debug_mode}')
        
        # Create timer to demonstrate parameter usage
        self.timer = self.create_timer(1.0, self.timer_callback)
        
    def timer_callback(self):
        if self.debug_mode:
            self.get_logger().info(f'Robot {self.robot_name} moving at {self.max_speed} m/s')
        else:
            self.get_logger().info('Robot is moving')

def main(args=None):
    rclpy.init(args=args)
    parameter_demo = ParameterDemo()
    
    try:
        rclpy.spin(parameter_demo)
    except KeyboardInterrupt:
        pass
    
    parameter_demo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### Interactive Exercise: Work with Parameters
```bash
# Terminal 1: Run the parameter demo node
ros2 run my_first_package parameter_demo

# Terminal 2: List all parameters
ros2 param list

# Terminal 3: Get parameter values
ros2 param get /parameter_demo robot_name
ros2 param get /parameter_demo max_speed
ros2 param get /parameter_demo debug_mode

# Terminal 4: Set parameter values
ros2 param set /parameter_demo robot_name "MyRobot"
ros2 param set /parameter_demo max_speed 2.5
ros2 param set /parameter_demo debug_mode False

# Terminal 5: Save parameters to file
ros2 param dump /parameter_demo
```

#### Knowledge Check
- [ ] What are ROS2 parameters used for?
- [ ] How do you declare parameters in a node?
- [ ] How do you get and set parameters at runtime?
- [ ] What parameter types are supported?

### Module 6: Launch Files (60 minutes)
**Orchestrating multi-node applications**

#### Understanding Launch Files
Launch files provide a way to start multiple ROS2 nodes with a single command. They can set parameters, remap topics, and configure the entire system.

#### Creating a Launch File
```python
# launch/robot_demo.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_first_package',
            executable='simple_publisher',
            name='talker',
            parameters=[{'topic_name': 'chatter'}]
        ),
        Node(
            package='my_first_package',
            executable='simple_subscriber',
            name='listener',
            parameters=[{'topic_name': 'chatter'}]
        ),
        Node(
            package='my_first_package',
            executable='calculator_service',
            name='calculator'
        ),
        Node(
            package='my_first_package',
            executable='parameter_demo',
            name='param_demo',
            parameters=[
                {'robot_name': 'LaunchBot'},
                {'max_speed': 2.0},
                {'debug_mode': True}
            ]
        )
    ])
```

#### Interactive Exercise: Use Launch Files
```bash
# Terminal 1: Launch multiple nodes
ros2 launch my_first_package robot_demo.launch.py

# Terminal 2: Monitor the system
ros2 node list
ros2 topic list
ros2 service list
ros2 param list

# Terminal 3: Test the communication
ros2 topic echo /chatter
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts '{a: 5, b: 3}'
```

#### Knowledge Check
- [ ] What are launch files used for?
- [ ] How do you create a launch file?
- [ ] What can you configure in a launch file?
- [ ] How do you run a launch file?

## 📋 Practical Assessment

### Project: Build a Complete ROS2 Application
Create a robot monitoring system with the following components:

1. **Sensor Node**: Publishes simulated sensor data
2. **Monitor Node**: Subscribes to sensor data and detects anomalies
3. **Alert Service**: Provides alert service for critical conditions
4. **Config Node**: Uses parameters for system configuration
5. **Launch File**: Orchestrates the entire system

### Solution Guidelines
```python
# sensor_node.py
# TODO: Create a node that publishes sensor data every 2 seconds
# Include temperature, humidity, and battery level
# Use custom message types or standard messages

# monitor_node.py  
# TODO: Create a node that subscribes to sensor data
# Detect anomalies (e.g., temperature > 30°C, battery < 20%)
# Log warnings and call alert service for critical conditions

# alert_service.py
# TODO: Create a service that sends alerts
# Accept alert type and message as parameters
# Log alerts with appropriate severity levels

# config_node.py
# TODO: Create a node with configurable parameters
# Include thresholds for temperature, humidity, battery
# Allow enabling/disabling monitoring features

# monitoring_system.launch.py
# TODO: Create launch file for the complete system
# Set appropriate parameters and remappings
```

## 🎯 Course Completion

### What You've Learned
- ✅ ROS2 architecture and core concepts
- ✅ Creating and managing ROS2 nodes
- ✅ Publisher-subscriber communication
- ✅ Service-based request-response patterns
- ✅ Parameter management and configuration
- ✅ Launch file creation and usage
- ✅ Command-line tools and debugging

### Next Steps
- **Advanced Navigation Course**: Learn Navigation2 stack
- **Gazebo Simulation Course**: Master robotics simulation
- **Real Projects**: Apply your skills to real robotics challenges
- **Community Participation**: Join ROS discussions and projects

### Certification
Complete the practical assessment to earn your ROS2 Basics Certificate. Share your achievement on LinkedIn and join our community of certified ROS developers.

---

*Continue your learning journey with the [Advanced Navigation Course](navigation) or explore [Gazebo Simulation](simulation) for hands-on robotics experience.*