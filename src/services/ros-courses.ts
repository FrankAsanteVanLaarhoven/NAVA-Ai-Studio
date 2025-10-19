// ROS Course Data Structure
// Comprehensive educational content for ROS 2 learning

export interface CodeExample {
  language: string;
  code: string;
  explanation: string;
  executable?: boolean;
  command?: string;
}

export interface QuizQuestion {
  question: string;
  options: string[];
  correctAnswer: number;
  explanation: string;
}

export interface Lesson {
  id: string;
  title: string;
  duration: string;
  description: string;
  content: string;
  codeExamples: CodeExample[];
  quiz?: QuizQuestion[];
  practiceExercises?: string[];
}

export interface Course {
  id: string;
  title: string;
  description: string;
  icon: string;
  level: 'beginner' | 'intermediate' | 'advanced';
  duration: string;
  color: string;
  learningOutcomes: string[];
  lessons: Lesson[];
}

// ROS 2 Fundamentals Course
export const rosFundamentals: Course = {
  id: 'ros2-fundamentals',
  title: 'ROS 2 Fundamentals',
  description: 'Master the basics of Robot Operating System 2',
  icon: '🤖',
  level: 'beginner',
  duration: '6 hours',
  color: '#48bb78',
  learningOutcomes: [
    'Understand ROS 2 architecture and concepts',
    'Create and manage ROS 2 nodes',
    'Work with topics, services, and actions',
    'Build custom messages and packages',
    'Use ROS 2 command-line tools effectively',
    'Debug and troubleshoot ROS systems',
  ],
  lessons: [
    {
      id: 'intro-to-ros2',
      title: 'Introduction to ROS 2',
      duration: '45 min',
      description: 'Learn what ROS 2 is and why it matters',
      content: `# Introduction to ROS 2

## What is ROS 2?

ROS 2 (Robot Operating System 2) is a set of software libraries and tools for building robot applications. It's the next generation of ROS, designed to be more robust, secure, and suitable for production use.

## Key Features

- **Real-time capabilities** - Better support for real-time systems
- **Security** - Built-in security features
- **Multi-platform** - Works on Linux, Windows, and macOS
- **Scalability** - From single robots to robot fleets
- **Quality of Service** - Configurable communication reliability

## ROS 2 vs ROS 1

| Feature | ROS 1 | ROS 2 |
|---------|-------|-------|
| Real-time | Limited | Native support |
| Security | Basic | Built-in |
| Platforms | Linux only | Multi-platform |
| Communication | TCP/UDP | DDS |

## Architecture Overview

ROS 2 uses a distributed architecture where nodes communicate through:
- **Topics** - Publish/subscribe messaging
- **Services** - Request/response communication
- **Actions** - Long-running tasks with feedback`,
      codeExamples: [
        {
          language: 'bash',
          code: 'ros2 --version',
          explanation: 'Check your ROS 2 installation version',
          executable: true,
          command: 'ros2 --version',
        },
        {
          language: 'bash',
          code: 'ros2 --help',
          explanation: 'See all available ROS 2 commands',
          executable: true,
          command: 'ros2 --help',
        },
      ],
      quiz: [
        {
          question: 'What is the main communication protocol used by ROS 2?',
          options: ['TCP/IP', 'DDS', 'HTTP', 'WebSocket'],
          correctAnswer: 1,
          explanation: 'ROS 2 uses DDS (Data Distribution Service) as its communication middleware, which provides better real-time capabilities and quality of service.',
        },
        {
          question: 'Which platforms does ROS 2 support?',
          options: ['Linux only', 'Linux and Windows', 'Linux, Windows, and macOS', 'Any platform'],
          correctAnswer: 2,
          explanation: 'ROS 2 is designed to be multi-platform and officially supports Linux, Windows, and macOS.',
        },
      ],
      practiceExercises: [
        'Install ROS 2 on your system',
        'Run the version command to verify installation',
        'Explore the help system using ros2 --help',
      ],
    },
    {
      id: 'nodes-and-topics',
      title: 'Nodes and Topics',
      duration: '60 min',
      description: 'Understanding the building blocks of ROS 2',
      content: `# Nodes and Topics

## What are Nodes?

A **node** is a single executable that performs a specific task. Nodes are the building blocks of ROS applications.

Examples of nodes:
- Camera driver node
- Motor controller node
- Path planning node
- User interface node

## What are Topics?

**Topics** are named channels used for communication between nodes. They implement a publish-subscribe pattern.

Communication flow:
Publisher Node → Topic → Subscriber Node(s)

## Message Types

Every topic has a specific message type (e.g., std_msgs/String, geometry_msgs/Twist).

## Common Topic Commands

Use these commands to work with topics:
- ros2 topic list - Show all topics
- ros2 topic echo - Listen to messages
- ros2 topic info - Get topic details
- ros2 topic pub - Publish messages`,
      codeExamples: [
        {
          language: 'bash',
          code: 'ros2 topic list',
          explanation: 'List all active topics',
          executable: true,
          command: 'ros2 topic list',
        },
        {
          language: 'bash',
          code: 'ros2 node list',
          explanation: 'List all running nodes',
          executable: true,
          command: 'ros2 node list',
        },
        {
          language: 'bash',
          code: 'ros2 topic echo /cmd_vel',
          explanation: 'Listen to velocity commands',
          executable: true,
          command: 'ros2 topic echo /cmd_vel',
        },
        {
          language: 'bash',
          code: 'ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0}}"',
          explanation: 'Publish a velocity command',
          executable: true,
          command: 'ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0}}"',
        },
      ],
      quiz: [
        {
          question: 'What communication pattern do topics use?',
          options: ['Request-response', 'Publish-subscribe', 'Peer-to-peer', 'Client-server'],
          correctAnswer: 1,
          explanation: 'Topics use a publish-subscribe pattern where publishers send messages to topics and subscribers receive them.',
        },
        {
          question: 'Can multiple nodes subscribe to the same topic?',
          options: ['No, only one subscriber allowed', 'Yes, multiple subscribers are allowed', 'Only if they are the same type', 'Depends on the message type'],
          correctAnswer: 1,
          explanation: 'Multiple nodes can subscribe to the same topic, which is one of the key advantages of the publish-subscribe pattern.',
        },
      ],
      practiceExercises: [
        'List all topics in a running ROS system',
        'Echo messages from a topic',
        'Publish a message to a topic',
        'Get information about a specific topic',
      ],
    },
    {
      id: 'services-and-actions',
      title: 'Services and Actions',
      duration: '60 min',
      description: 'Request-response and long-running task patterns',
      content: `# Services and Actions

## Services

**Services** provide a request-response communication pattern. They're perfect for:
- Getting current state
- Triggering one-time actions
- Configuration changes

Service workflow:
Client → Request → Service Server → Response → Client

## Actions

**Actions** are for long-running tasks that need:
- Progress feedback
- Ability to cancel
- Final result

Action workflow:
Client → Goal → Action Server → Feedback → Result

## When to Use What?

- **Topics**: Continuous data streams (sensor data, commands)
- **Services**: Quick request-response (get status, set parameter)
- **Actions**: Long tasks with feedback (navigation, manipulation)`,
      codeExamples: [
        {
          language: 'bash',
          code: 'ros2 service list',
          explanation: 'List all available services',
          executable: true,
          command: 'ros2 service list',
        },
        {
          language: 'bash',
          code: 'ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 5, b: 3}"',
          explanation: 'Call a service to add two numbers',
          executable: true,
          command: 'ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 5, b: 3}"',
        },
        {
          language: 'bash',
          code: 'ros2 action list',
          explanation: 'List all available actions',
          executable: true,
          command: 'ros2 action list',
        },
        {
          language: 'bash',
          code: 'ros2 action send_goal /fibonacci action_tutorials_interfaces/action/Fibonacci "{order: 5}"',
          explanation: 'Send a goal to calculate Fibonacci sequence',
          executable: true,
          command: 'ros2 action send_goal /fibonacci action_tutorials_interfaces/action/Fibonacci "{order: 5}"',
        },
      ],
      quiz: [
        {
          question: 'What type of communication pattern do services use?',
          options: ['Publish-subscribe', 'Request-response', 'Broadcast', 'Multicast'],
          correctAnswer: 1,
          explanation: 'Services use a request-response pattern where a client sends a request and waits for a response from the server.',
        },
        {
          question: 'What makes actions different from services?',
          options: ['Actions are faster', 'Actions provide feedback and can be cancelled', 'Actions use different protocols', 'Actions are only for robots'],
          correctAnswer: 1,
          explanation: 'Actions are designed for long-running tasks and provide progress feedback and the ability to cancel the task.',
        },
      ],
      practiceExercises: [
        'List all services in the system',
        'Call a simple service',
        'List all actions available',
        'Send a goal to an action server',
      ],
    },
    {
      id: 'parameters-and-launch',
      title: 'Parameters and Launch Files',
      duration: '60 min',
      description: 'Configuration and system orchestration',
      content: `# Parameters and Launch Files

## Parameters

**Parameters** are configuration values that can be set for nodes:
- Runtime configuration
- Tuning values
- Feature toggles
- File paths

Parameter types:
- bool, int, double, string
- Arrays of basic types

## Launch Files

**Launch files** help you:
- Start multiple nodes
- Set parameters
- Configure namespaces
- Set up complex systems

Launch file formats:
- Python (.py) - Most flexible
- XML (.xml) - Declarative
- YAML (.yaml) - Simple configuration`,
      codeExamples: [
        {
          language: 'bash',
          code: 'ros2 param list',
          explanation: 'List all parameters in the system',
          executable: true,
          command: 'ros2 param list',
        },
        {
          language: 'bash',
          code: 'ros2 param get /turtlesim background_r',
          explanation: 'Get the value of a specific parameter',
          executable: true,
          command: 'ros2 param get /turtlesim background_r',
        },
        {
          language: 'bash',
          code: 'ros2 param set /turtlesim background_r 255',
          explanation: 'Set a parameter value',
          executable: true,
          command: 'ros2 param set /turtlesim background_r 255',
        },
        {
          language: 'python',
          code: `from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim',
            parameters=[{'background_r': 255}]
        )
    ])`,
          explanation: 'Simple Python launch file',
          executable: false,
        },
      ],
      quiz: [
        {
          question: 'What are parameters used for in ROS 2?',
          options: ['Sending messages', 'Configuration values', 'Creating topics', 'Starting nodes'],
          correctAnswer: 1,
          explanation: 'Parameters are used to store configuration values that can be set and retrieved at runtime.',
        },
        {
          question: 'Which launch file format is most flexible?',
          options: ['XML', 'YAML', 'Python', 'JSON'],
          correctAnswer: 2,
          explanation: 'Python launch files are the most flexible as they allow for complex logic and dynamic configuration.',
        },
      ],
      practiceExercises: [
        'List parameters of a running node',
        'Change a parameter value',
        'Create a simple launch file',
        'Launch multiple nodes with parameters',
      ],
    },
    {
      id: 'custom-messages',
      title: 'Custom Messages and Packages',
      duration: '45 min',
      description: 'Creating your own message types and packages',
      content: `# Custom Messages and Packages

## Why Custom Messages?

Standard messages don't always fit your needs. Custom messages let you:
- Define specific data structures
- Combine multiple data types
- Create domain-specific interfaces

## Message Definition

Messages are defined in .msg files:

\`\`\`
# PersonInfo.msg
string name
uint8 age
float64 height
bool is_student
\`\`\`

## Package Structure

A typical ROS 2 package contains:
- src/ - Source code
- msg/ - Message definitions
- srv/ - Service definitions
- action/ - Action definitions
- package.xml - Package metadata
- CMakeLists.txt - Build configuration`,
      codeExamples: [
        {
          language: 'bash',
          code: 'ros2 pkg create --build-type ament_cmake my_robot_msgs',
          explanation: 'Create a new package for custom messages',
          executable: true,
          command: 'ros2 pkg create --build-type ament_cmake my_robot_msgs',
        },
        {
          language: 'bash',
          code: 'ros2 pkg list',
          explanation: 'List all available packages',
          executable: true,
          command: 'ros2 pkg list',
        },
        {
          language: 'bash',
          code: 'ros2 interface show geometry_msgs/msg/Twist',
          explanation: 'Show the structure of a message type',
          executable: true,
          command: 'ros2 interface show geometry_msgs/msg/Twist',
        },
      ],
      quiz: [
        {
          question: 'What file extension is used for message definitions?',
          options: ['.msg', '.message', '.def', '.ros'],
          correctAnswer: 0,
          explanation: 'Message definitions use the .msg file extension.',
        },
        {
          question: 'What build system does ROS 2 primarily use?',
          options: ['Make', 'CMake', 'Ninja', 'Bazel'],
          correctAnswer: 1,
          explanation: 'ROS 2 primarily uses CMake as its build system, specifically ament_cmake for C++ packages.',
        },
      ],
      practiceExercises: [
        'Create a new ROS 2 package',
        'Define a custom message type',
        'Build the package',
        'Use the custom message in code',
      ],
    },
    {
      id: 'ros2-tools',
      title: 'ROS 2 Tools and Debugging',
      duration: '60 min',
      description: 'Master the command-line tools and debugging techniques',
      content: `# ROS 2 Tools and Debugging

## Essential Tools

### ros2 doctor
System health checker:
- Verifies installation
- Checks network configuration
- Identifies common issues

### ros2 bag
Data recording and playback:
- Record sensor data
- Replay scenarios
- Debug offline

### ros2 launch
System orchestration:
- Start multiple nodes
- Configure parameters
- Set up complex systems

## Debugging Techniques

1. **Check node status**: ros2 node list
2. **Monitor topics**: ros2 topic echo
3. **Inspect messages**: ros2 interface show
4. **View logs**: ros2 run with --ros-args --log-level debug
5. **Network issues**: ros2 doctor

## Best Practices

- Use meaningful node and topic names
- Add proper error handling
- Log important events
- Test with ros2 bag recordings
- Monitor system resources`,
      codeExamples: [
        {
          language: 'bash',
          code: 'ros2 doctor',
          explanation: 'Check system health and configuration',
          executable: true,
          command: 'ros2 doctor',
        },
        {
          language: 'bash',
          code: 'ros2 bag record /cmd_vel /odom',
          explanation: 'Record data from specific topics',
          executable: true,
          command: 'ros2 bag record /cmd_vel /odom',
        },
        {
          language: 'bash',
          code: 'ros2 bag info rosbag2_2025_01_13-10_30_45',
          explanation: 'Get information about a recorded bag',
          executable: true,
          command: 'ros2 bag info rosbag2_2025_01_13-10_30_45',
        },
        {
          language: 'bash',
          code: 'ros2 topic hz /cmd_vel',
          explanation: 'Monitor the frequency of topic messages',
          executable: true,
          command: 'ros2 topic hz /cmd_vel',
        },
      ],
      quiz: [
        {
          question: 'What does ros2 doctor do?',
          options: ['Fixes ROS issues', 'Checks system health', 'Updates ROS', 'Installs packages'],
          correctAnswer: 1,
          explanation: 'ros2 doctor checks the health of your ROS 2 system and identifies common configuration issues.',
        },
        {
          question: 'What is ros2 bag used for?',
          options: ['Package management', 'Data recording and playback', 'Node debugging', 'Network configuration'],
          correctAnswer: 1,
          explanation: 'ros2 bag is used for recording and playing back ROS 2 message data, which is essential for testing and debugging.',
        },
      ],
      practiceExercises: [
        'Run ros2 doctor to check your system',
        'Record some topic data with ros2 bag',
        'Play back recorded data',
        'Monitor topic frequencies',
        'Debug a failing node',
      ],
    },
  ],
};

// ROS 2 Navigation Stack Course
export const rosNavigation: Course = {
  id: 'ros2-navigation',
  title: 'ROS 2 Navigation Stack',
  description: 'Master autonomous robot navigation',
  icon: '🗺️',
  level: 'intermediate',
  duration: '8 hours',
  color: '#4299e1',
  learningOutcomes: [
    'Understand SLAM and mapping concepts',
    'Implement path planning algorithms',
    'Configure obstacle avoidance',
    'Set up localization systems',
    'Tune navigation parameters',
    'Deploy navigation in real robots',
  ],
  lessons: [
    {
      id: 'navigation-overview',
      title: 'Navigation Stack Overview',
      duration: '60 min',
      description: 'Understanding the ROS 2 navigation architecture',
      content: `# Navigation Stack Overview

## What is Navigation?

Robot navigation involves:
- **Localization** - Where am I?
- **Mapping** - What does the environment look like?
- **Path Planning** - How do I get there?
- **Obstacle Avoidance** - How do I avoid collisions?

## Navigation Stack Components

### Core Components
- **Map Server** - Provides static maps
- **AMCL** - Adaptive Monte Carlo Localization
- **Planner Server** - Global path planning
- **Controller Server** - Local path following
- **Recovery Server** - Handles stuck situations

### Supporting Components
- **Behavior Tree Navigator** - Orchestrates navigation
- **Lifecycle Manager** - Manages component states
- **Waypoint Follower** - Follows predefined paths`,
      codeExamples: [
        {
          language: 'bash',
          code: 'ros2 launch nav2_bringup tb3_simulation_launch.py',
          explanation: 'Launch TurtleBot3 navigation simulation',
          executable: true,
          command: 'ros2 launch nav2_bringup tb3_simulation_launch.py',
        },
        {
          language: 'bash',
          code: 'ros2 topic list | grep nav',
          explanation: 'List navigation-related topics',
          executable: true,
          command: 'ros2 topic list',
        },
      ],
      quiz: [
        {
          question: 'What are the four main aspects of robot navigation?',
          options: ['Speed, accuracy, efficiency, safety', 'Localization, mapping, planning, avoidance', 'Sensors, motors, computers, software', 'Input, processing, output, feedback'],
          correctAnswer: 1,
          explanation: 'The four main aspects are localization (where am I?), mapping (what does the environment look like?), path planning (how do I get there?), and obstacle avoidance (how do I avoid collisions?).',
        },
      ],
      practiceExercises: [
        'Launch a navigation simulation',
        'Explore navigation topics',
        'Understand the component architecture',
      ],
    },
    {
      id: 'slam-basics',
      title: 'SLAM (Simultaneous Localization and Mapping)',
      duration: '90 min',
      description: 'Building maps while localizing',
      content: `# SLAM Basics

## What is SLAM?

SLAM solves two problems simultaneously:
1. **Localization** - Determine robot's position
2. **Mapping** - Build a map of the environment

## SLAM Algorithms

### Grid-based SLAM
- Uses occupancy grids
- Each cell: free, occupied, or unknown
- Good for 2D navigation

### Feature-based SLAM
- Uses landmarks/features
- More memory efficient
- Better for loop closure

## ROS 2 SLAM Tools

- **slam_toolbox** - Most popular SLAM package
- **cartographer** - Google's SLAM solution
- **gmapping** - Classic grid-based SLAM`,
      codeExamples: [
        {
          language: 'bash',
          code: 'ros2 launch slam_toolbox online_async_launch.py',
          explanation: 'Start SLAM mapping',
          executable: true,
          command: 'ros2 launch slam_toolbox online_async_launch.py',
        },
        {
          language: 'bash',
          code: 'ros2 run nav2_map_server map_saver_cli -f my_map',
          explanation: 'Save the generated map',
          executable: true,
          command: 'ros2 run nav2_map_server map_saver_cli -f my_map',
        },
      ],
      quiz: [
        {
          question: 'What does SLAM stand for?',
          options: ['Simple Location and Mapping', 'Simultaneous Localization and Mapping', 'Sensor-based Location and Movement', 'Smart Learning and Memory'],
          correctAnswer: 1,
          explanation: 'SLAM stands for Simultaneous Localization and Mapping, which means determining the robot\'s position while building a map of the environment at the same time.',
        },
      ],
      practiceExercises: [
        'Start a SLAM session',
        'Drive robot to build a map',
        'Save the generated map',
        'Analyze map quality',
      ],
    },
  ],
};

// Gazebo Simulation Course
export const gazeboSimulation: Course = {
  id: 'gazebo-simulation',
  title: 'Gazebo Simulation',
  description: 'Create realistic robot simulations',
  icon: '🌍',
  level: 'intermediate',
  duration: '6 hours',
  color: '#ed8936',
  learningOutcomes: [
    'Set up Gazebo simulation environments',
    'Create robot models (URDF/SDF)',
    'Add sensors to simulated robots',
    'Build custom worlds',
    'Understand physics simulation',
    'Integrate with ROS 2',
  ],
  lessons: [
    {
      id: 'gazebo-basics',
      title: 'Gazebo Basics',
      duration: '60 min',
      description: 'Introduction to robot simulation',
      content: `# Gazebo Basics

## What is Gazebo?

Gazebo is a 3D robot simulation environment that provides:
- **Physics simulation** - Realistic physics
- **Sensor simulation** - Cameras, LiDAR, IMU
- **Rendering** - 3D visualization
- **Plugin system** - Extensible functionality

## Key Concepts

### Worlds
- 3D environments where robots operate
- Include terrain, obstacles, lighting
- Defined in SDF (Simulation Description Format)

### Models
- Robot descriptions
- Can be URDF or SDF format
- Include geometry, physics, sensors

### Plugins
- Add functionality to models
- Control actuators
- Simulate sensors
- Interface with ROS 2`,
      codeExamples: [
        {
          language: 'bash',
          code: 'gazebo worlds/empty.world',
          explanation: 'Launch Gazebo with an empty world',
          executable: false,
        },
        {
          language: 'bash',
          code: 'ros2 launch gazebo_ros gazebo.launch.py',
          explanation: 'Launch Gazebo with ROS 2 integration',
          executable: false,
        },
      ],
      quiz: [
        {
          question: 'What format is primarily used for Gazebo world descriptions?',
          options: ['URDF', 'SDF', 'XML', 'YAML'],
          correctAnswer: 1,
          explanation: 'SDF (Simulation Description Format) is the primary format used for Gazebo world descriptions, though URDF can also be used for robot models.',
        },
      ],
      practiceExercises: [
        'Launch Gazebo with different worlds',
        'Explore the Gazebo interface',
        'Load different robot models',
      ],
    },
  ],
};

// Export all courses
export const allCourses: Course[] = [
  rosFundamentals,
  rosNavigation,
  gazeboSimulation,
];

// Helper functions
export function getCourseById(courseId: string): Course | undefined {
  return allCourses.find(course => course.id === courseId);
}

export function getLessonById(courseId: string, lessonId: string): Lesson | undefined {
  const course = getCourseById(courseId);
  return course?.lessons.find(lesson => lesson.id === lessonId);
}