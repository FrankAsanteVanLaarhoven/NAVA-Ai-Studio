/**
 * ROS Terminal Service
 *
 * Provides terminal emulation and ROS command execution
 * for the ROS Learning Center.
 *
 * Enhanced with 20+ ROS2 commands including:
 * - Core: run, launch, topic, node, pkg
 * - Advanced: service, action, param, interface
 * - Tools: bag, doctor, daemon
 *
 * @author NAVΛ Team
 * @since 2025-01-13
 */

export interface TerminalOutput {
  command: string;
  output: string;
  exitCode: number;
  timestamp: Date;
  type: 'success' | 'error' | 'info';
}

export interface TerminalSession {
  id: string;
  history: TerminalOutput[];
  isActive: boolean;
  workingDirectory: string;
}

/**
 * ROS Terminal Service
 *
 * This service provides:
 * 1. Command execution simulation for ROS commands
 * 2. Terminal history tracking
 * 3. Output formatting and display
 * 4. Integration with web-based terminal emulator
 */
export class ROSTerminalService {
  private sessions: Map<string, TerminalSession>;
  private currentSessionId: string | null;
  private commandHistory: string[];

  constructor() {
    this.sessions = new Map();
    this.currentSessionId = null;
    this.commandHistory = [];
  }

  /**
   * Create a new terminal session
   */
  createSession(): string {
    const sessionId = `ros-terminal-${Date.now()}`;
    const session: TerminalSession = {
      id: sessionId,
      history: [],
      isActive: true,
      workingDirectory: '~/ros2_ws',
    };

    this.sessions.set(sessionId, session);
    this.currentSessionId = sessionId;

    // Add welcome message
    this.addOutput(sessionId, {
      command: '',
      output: this.getWelcomeMessage(),
      exitCode: 0,
      timestamp: new Date(),
      type: 'info',
    });

    return sessionId;
  }

  /**
   * Get a terminal session by ID
   */
  getSession(sessionId: string): TerminalSession | null {
    return this.sessions.get(sessionId) || null;
  }

  /**
   * Execute a ROS command
   */
  async executeCommand(command: string, sessionId?: string): Promise<TerminalOutput> {
    const sid = sessionId || this.currentSessionId;
    if (!sid) {
      throw new Error('No active terminal session');
    }

    const session = this.sessions.get(sid);
    if (!session) {
      throw new Error(`Session ${sid} not found`);
    }

    // Add to command history
    this.commandHistory.push(command);

    // Parse and execute the command
    const output = await this.executeROSCommand(command);

    // Add to session history
    this.addOutput(sid, output);

    return output;
  }

  /**
   * Execute ROS command with simulation/real execution
   */
  private async executeROSCommand(command: string): Promise<TerminalOutput> {
    const trimmedCommand = command.trim();

    // Parse ROS2 commands
    if (trimmedCommand.startsWith('ros2 ')) {
      return await this.handleROS2Command(trimmedCommand);
    }

    // Handle bash commands
    if (trimmedCommand.startsWith('cd ')) {
      return this.handleCdCommand(trimmedCommand);
    }

    if (trimmedCommand === 'pwd') {
      return this.handlePwdCommand();
    }

    if (trimmedCommand === 'ls' || trimmedCommand.startsWith('ls ')) {
      return this.handleLsCommand(trimmedCommand);
    }

    // Unknown command
    return {
      command: trimmedCommand,
      output: `Command not found: ${trimmedCommand}\n\nAvailable commands:\n  - ros2 run <package> <executable>\n  - ros2 launch <package> <launch_file>\n  - ros2 topic list\n  - ros2 node list\n  - cd, pwd, ls`,
      exitCode: 1,
      timestamp: new Date(),
      type: 'error',
    };
  }

  /**
   * Handle ROS2 commands
   */
  private async handleROS2Command(command: string): Promise<TerminalOutput> {
    const parts = command.split(/\s+/);
    const subcommand = parts[1];

    switch (subcommand) {
      case 'run':
        return this.handleROS2Run(parts);

      case 'launch':
        return this.handleROS2Launch(parts);

      case 'topic':
        return this.handleROS2Topic(parts);

      case 'node':
        return this.handleROS2Node(parts);

      case 'pkg':
        return this.handleROS2Pkg(parts);

      case 'service':
        return this.handleROS2Service(parts);

      case 'action':
        return this.handleROS2Action(parts);

      case 'param':
        return this.handleROS2Param(parts);

      case 'interface':
        return this.handleROS2Interface(parts);

      case 'bag':
        return this.handleROS2Bag(parts);

      case 'doctor':
        return this.handleROS2Doctor(parts);

      case 'daemon':
        return this.handleROS2Daemon(parts);

      case '--version':
      case 'version':
        return this.handleROS2Version();

      case '--help':
      case 'help':
        return this.handleROS2Help();

      default:
        return {
          command,
          output: `Unknown ros2 subcommand: ${subcommand}\n\nUsage:\n  ros2 run <package> <executable>\n  ros2 launch <package> <launch_file>\n  ros2 topic [list|echo|info|hz|pub]\n  ros2 node [list|info]\n  ros2 pkg [list|create]\n  ros2 service [list|call|type]\n  ros2 action [list|info|send_goal]\n  ros2 param [list|get|set]\n  ros2 interface [list|show]\n  ros2 bag [record|play|info]\n  ros2 doctor\n  ros2 --help`,
          exitCode: 1,
          timestamp: new Date(),
          type: 'error',
        };
    }
  }

  /**
   * Handle: ros2 run <package> <executable>
   */
  private handleROS2Run(parts: string[]): TerminalOutput {
    if (parts.length < 4) {
      return {
        command: parts.join(' '),
        output: 'Usage: ros2 run <package_name> <executable_name>',
        exitCode: 1,
        timestamp: new Date(),
        type: 'error',
      };
    }

    const packageName = parts[2];
    const executableName = parts[3];

    // Simulate successful execution
    const output = `
[INFO] [${new Date().toISOString()}]: Starting ${executableName} from package ${packageName}
[INFO] [${new Date().toISOString()}]: Initializing node...
[INFO] [${new Date().toISOString()}]: Node started successfully

🤖 ROS2 Node Running: ${executableName}

Package: ${packageName}
Executable: ${executableName}
Status: ✅ Running

Publishing to topics:
  - /cmd_vel (geometry_msgs/Twist)

Subscribing to topics:
  - /odom (nav_msgs/Odometry)

[INFO] Node is active and processing navigation commands
[INFO] Press Ctrl+C to stop

---
💡 This is a simulated execution in NAVΛ Studio Learning Environment
💡 In production, this would launch the actual ROS2 node
    `.trim();

    return {
      command: parts.join(' '),
      output,
      exitCode: 0,
      timestamp: new Date(),
      type: 'success',
    };
  }

  /**
   * Handle: ros2 launch <package> <launch_file>
   */
  private handleROS2Launch(parts: string[]): TerminalOutput {
    if (parts.length < 4) {
      return {
        command: parts.join(' '),
        output: 'Usage: ros2 launch <package_name> <launch_file>',
        exitCode: 1,
        timestamp: new Date(),
        type: 'error',
      };
    }

    const packageName = parts[2];
    const launchFile = parts[3];

    const output = `
[INFO] [${new Date().toISOString()}]: Launching ${launchFile} from package ${packageName}
[INFO] [${new Date().toISOString()}]: Loading launch file...
[INFO] [${new Date().toISOString()}]: Starting nodes...

🚀 Launch File Executing: ${launchFile}

Package: ${packageName}
Launch File: ${launchFile}
Status: ✅ Running

Nodes Started:
  ✓ navigation_publisher
  ✓ energy_visualizer
  ✓ path_optimizer

Topics Active:
  - /cmd_vel
  - /odom
  - /energy_field
  - /optimal_path

Services Available:
  - /compute_geodesic
  - /optimize_trajectory

All nodes launched successfully!

---
💡 This is a simulated execution in NAVΛ Studio Learning Environment
💡 In production, this would launch all nodes defined in the launch file
    `.trim();

    return {
      command: parts.join(' '),
      output,
      exitCode: 0,
      timestamp: new Date(),
      type: 'success',
    };
  }

  /**
   * Handle: ros2 topic list/echo/pub
   */
  private handleROS2Topic(parts: string[]): TerminalOutput {
    const action = parts[2];

    if (action === 'list') {
      const output = `
Available Topics:
  /cmd_vel (geometry_msgs/msg/Twist)
  /odom (nav_msgs/msg/Odometry)
  /scan (sensor_msgs/msg/LaserScan)
  /energy_field (std_msgs/msg/Float64MultiArray)
  /optimal_path (nav_msgs/msg/Path)
  /robot_state (std_msgs/msg/String)
      `.trim();

      return {
        command: parts.join(' '),
        output,
        exitCode: 0,
        timestamp: new Date(),
        type: 'success',
      };
    }

    if (action === 'echo') {
      const topic = parts[3] || '/cmd_vel';
      const output = `
Listening to ${topic}...

linear:
  x: 0.5
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.1
---
(Press Ctrl+C to stop)
      `.trim();

      return {
        command: parts.join(' '),
        output,
        exitCode: 0,
        timestamp: new Date(),
        type: 'success',
      };
    }

    return {
      command: parts.join(' '),
      output: 'Usage: ros2 topic [list|echo|pub] [topic_name]',
      exitCode: 1,
      timestamp: new Date(),
      type: 'error',
    };
  }

  /**
   * Handle: ros2 node list/info
   */
  private handleROS2Node(parts: string[]): TerminalOutput {
    const action = parts[2];

    if (action === 'list') {
      const output = `
Active ROS2 Nodes:
  /navigation_publisher
  /energy_visualizer
  /path_optimizer
  /vnc_global_planner
      `.trim();

      return {
        command: parts.join(' '),
        output,
        exitCode: 0,
        timestamp: new Date(),
        type: 'success',
      };
    }

    return {
      command: parts.join(' '),
      output: 'Usage: ros2 node [list|info] [node_name]',
      exitCode: 1,
      timestamp: new Date(),
      type: 'error',
    };
  }

  /**
   * Handle: ros2 pkg list/create
   */
  private handleROS2Pkg(parts: string[]): TerminalOutput {
    const action = parts[2];

    if (action === 'list') {
      const output = `
Available ROS2 Packages:
  navlambda_navigation
  turtlebot3_gazebo
  nav2_bringup
  geometry_msgs
  nav_msgs
  std_msgs
      `.trim();

      return {
        command: parts.join(' '),
        output,
        exitCode: 0,
        timestamp: new Date(),
        type: 'success',
      };
    }

    if (action === 'create') {
      const pkgName = parts[3];
      if (!pkgName) {
        return {
          command: parts.join(' '),
          output: 'Usage: ros2 pkg create <package_name>',
          exitCode: 1,
          timestamp: new Date(),
          type: 'error',
        };
      }

      const output = `
Creating package: ${pkgName}
✓ Created directory structure
✓ Generated package.xml
✓ Generated CMakeLists.txt
✓ Created src directory
✓ Created include directory

Package '${pkgName}' created successfully!

Next steps:
  cd ${pkgName}
  # Add your source files
  colcon build
      `.trim();

      return {
        command: parts.join(' '),
        output,
        exitCode: 0,
        timestamp: new Date(),
        type: 'success',
      };
    }

    return {
      command: parts.join(' '),
      output: 'Usage: ros2 pkg [list|create] [package_name]',
      exitCode: 1,
      timestamp: new Date(),
      type: 'error',
    };
  }

  /**
   * Handle: ros2 service list/call/type
   */
  private handleROS2Service(parts: string[]): TerminalOutput {
    const action = parts[2];

    if (action === 'list') {
      const output = `
Available Services:
  /add_two_ints (example_interfaces/srv/AddTwoInts)
  /compute_geodesic (navlambda_msgs/srv/ComputeGeodesic)
  /optimize_trajectory (navlambda_msgs/srv/OptimizeTrajectory)
  /get_parameters (rcl_interfaces/srv/GetParameters)
  /set_parameters (rcl_interfaces/srv/SetParameters)
  /describe_parameters (rcl_interfaces/srv/DescribeParameters)
      `.trim();

      return {
        command: parts.join(' '),
        output,
        exitCode: 0,
        timestamp: new Date(),
        type: 'success',
      };
    }

    if (action === 'type') {
      const serviceName = parts[3] || '/add_two_ints';
      const output = `example_interfaces/srv/AddTwoInts`;

      return {
        command: parts.join(' '),
        output,
        exitCode: 0,
        timestamp: new Date(),
        type: 'success',
      };
    }

    if (action === 'call') {
      const serviceName = parts[3] || '/add_two_ints';
      const output = `
Waiting for service '${serviceName}' to become available...
Service is available!

Calling service with request:
  a: 5
  b: 3

Response:
  sum: 8

Service call succeeded!
      `.trim();

      return {
        command: parts.join(' '),
        output,
        exitCode: 0,
        timestamp: new Date(),
        type: 'success',
      };
    }

    return {
      command: parts.join(' '),
      output: 'Usage: ros2 service [list|type|call] [service_name] [args]',
      exitCode: 1,
      timestamp: new Date(),
      type: 'error',
    };
  }

  /**
   * Handle: ros2 action list/info/send_goal
   */
  private handleROS2Action(parts: string[]): TerminalOutput {
    const action = parts[2];

    if (action === 'list') {
      const output = `
Available Actions:
  /fibonacci (example_interfaces/action/Fibonacci)
  /navigate_to_pose (nav2_msgs/action/NavigateToPose)
  /follow_path (nav2_msgs/action/FollowPath)
  /compute_path_to_pose (nav2_msgs/action/ComputePathToPose)
      `.trim();

      return {
        command: parts.join(' '),
        output,
        exitCode: 0,
        timestamp: new Date(),
        type: 'success',
      };
    }

    if (action === 'info') {
      const actionName = parts[3] || '/fibonacci';
      const output = `
Action: ${actionName}
Action clients: 1
  /action_client_node
Action servers: 1
  /fibonacci_server
      `.trim();

      return {
        command: parts.join(' '),
        output,
        exitCode: 0,
        timestamp: new Date(),
        type: 'success',
      };
    }

    if (action === 'send_goal') {
      const actionName = parts[3] || '/fibonacci';
      const output = `
Waiting for action server '${actionName}'...
Sending goal:
  order: 5

Goal accepted with ID: 12345678-1234-5678-1234-567812345678

Feedback:
  sequence: [0, 1, 1]
Feedback:
  sequence: [0, 1, 1, 2, 3]
Feedback:
  sequence: [0, 1, 1, 2, 3, 5]

Result:
  sequence: [0, 1, 1, 2, 3, 5]

Goal finished with status: SUCCEEDED
      `.trim();

      return {
        command: parts.join(' '),
        output,
        exitCode: 0,
        timestamp: new Date(),
        type: 'success',
      };
    }

    return {
      command: parts.join(' '),
      output: 'Usage: ros2 action [list|info|send_goal] [action_name] [args]',
      exitCode: 1,
      timestamp: new Date(),
      type: 'error',
    };
  }

  /**
   * Handle: ros2 param list/get/set
   */
  private handleROS2Param(parts: string[]): TerminalOutput {
    const action = parts[2];

    if (action === 'list') {
      const nodeName = parts[3] || '/navigation_publisher';
      const output = `
Parameters for node '${nodeName}':
  use_sim_time
  qos_overrides./parameter_events.publisher.depth
  qos_overrides./parameter_events.publisher.durability
  qos_overrides./parameter_events.publisher.history
  qos_overrides./parameter_events.publisher.reliability
  update_rate
  robot_base_frame
  global_frame
  transform_tolerance
      `.trim();

      return {
        command: parts.join(' '),
        output,
        exitCode: 0,
        timestamp: new Date(),
        type: 'success',
      };
    }

    if (action === 'get') {
      const nodeName = parts[3] || '/navigation_publisher';
      const paramName = parts[4] || 'use_sim_time';
      const output = `Boolean value is: False`;

      return {
        command: parts.join(' '),
        output,
        exitCode: 0,
        timestamp: new Date(),
        type: 'success',
      };
    }

    if (action === 'set') {
      const nodeName = parts[3] || '/navigation_publisher';
      const paramName = parts[4] || 'use_sim_time';
      const value = parts[5] || 'true';
      const output = `Set parameter successful`;

      return {
        command: parts.join(' '),
        output,
        exitCode: 0,
        timestamp: new Date(),
        type: 'success',
      };
    }

    return {
      command: parts.join(' '),
      output: 'Usage: ros2 param [list|get|set] [node_name] [param_name] [value]',
      exitCode: 1,
      timestamp: new Date(),
      type: 'error',
    };
  }

  /**
   * Handle: ros2 interface list/show
   */
  private handleROS2Interface(parts: string[]): TerminalOutput {
    const action = parts[2];

    if (action === 'list') {
      const output = `
Messages:
  std_msgs/msg/String
  geometry_msgs/msg/Twist
  nav_msgs/msg/Odometry
  sensor_msgs/msg/LaserScan

Services:
  std_srvs/srv/Empty
  example_interfaces/srv/AddTwoInts

Actions:
  example_interfaces/action/Fibonacci
  nav2_msgs/action/NavigateToPose
      `.trim();

      return {
        command: parts.join(' '),
        output,
        exitCode: 0,
        timestamp: new Date(),
        type: 'success',
      };
    }

    if (action === 'show') {
      const interfaceName = parts[3] || 'geometry_msgs/msg/Twist';
      const output = `
# Velocity in free space

Vector3  linear
  float64 x
  float64 y
  float64 z
Vector3  angular
  float64 x
  float64 y
  float64 z
      `.trim();

      return {
        command: parts.join(' '),
        output,
        exitCode: 0,
        timestamp: new Date(),
        type: 'success',
      };
    }

    return {
      command: parts.join(' '),
      output: 'Usage: ros2 interface [list|show] [interface_name]',
      exitCode: 1,
      timestamp: new Date(),
      type: 'error',
    };
  }

  /**
   * Handle: ros2 bag record/play/info
   */
  private handleROS2Bag(parts: string[]): TerminalOutput {
    const action = parts[2];

    if (action === 'record') {
      const topics = parts.slice(3).join(' ') || '/cmd_vel /odom';
      return {
        command: parts.join(' '),
        output: `[INFO] Recording to 'rosbag2_2025_01_13-10_30_45'\n[INFO] Subscribed to topic '${topics}'\nRecording... Press Ctrl+C to stop.`,
        exitCode: 0,
        timestamp: new Date(),
        type: 'success',
      };
    }

    if (action === 'play') {
      return {
        command: parts.join(' '),
        output: '[INFO] Playing bag file...\nPlaying messages:\n  /cmd_vel: 150 messages\n  /odom: 300 messages\nPlayback complete!',
        exitCode: 0,
        timestamp: new Date(),
        type: 'success',
      };
    }

    if (action === 'info') {
      return {
        command: parts.join(' '),
        output: 'Bag size: 2.5 MB\nDuration: 30.5s\nMessages: 450\nTopics:\n  /cmd_vel: 150\n  /odom: 300',
        exitCode: 0,
        timestamp: new Date(),
        type: 'success',
      };
    }

    return {
      command: parts.join(' '),
      output: 'Usage: ros2 bag [record|play|info] [bag_file] [topics...]',
      exitCode: 1,
      timestamp: new Date(),
      type: 'error',
    };
  }

  /**
   * Handle: ros2 doctor
   */
  private handleROS2Doctor(parts: string[]): TerminalOutput {
    const output = `🏥 ROS 2 System Check

✅ ROS 2 Humble Hawksbill installed
✅ Python 3.10.12 found
✅ DDS middleware: Fast-RTPS
✅ Workspace: ~/ros2_ws
✅ 4 nodes running
✅ 6 topics active

System Status: ✅ ALL CHECKS PASSED`;

    return {
      command: parts.join(' '),
      output,
      exitCode: 0,
      timestamp: new Date(),
      type: 'success',
    };
  }

  /**
   * Handle: ros2 daemon start/stop/status
   */
  private handleROS2Daemon(parts: string[]): TerminalOutput {
    const action = parts[2] || 'status';

    if (action === 'start') {
      return {
        command: parts.join(' '),
        output: '✅ Daemon started successfully',
        exitCode: 0,
        timestamp: new Date(),
        type: 'success',
      };
    }

    if (action === 'stop') {
      return {
        command: parts.join(' '),
        output: '✅ Daemon stopped successfully',
        exitCode: 0,
        timestamp: new Date(),
        type: 'success',
      };
    }

    if (action === 'status') {
      return {
        command: parts.join(' '),
        output: '✅ Running (PID: 12345, Uptime: 2h 15m)',
        exitCode: 0,
        timestamp: new Date(),
        type: 'success',
      };
    }

    return {
      command: parts.join(' '),
      output: 'Usage: ros2 daemon [start|stop|status]',
      exitCode: 1,
      timestamp: new Date(),
      type: 'error',
    };
  }

  /**
   * Handle: ros2 --version
   */
  private handleROS2Version(): TerminalOutput {
    return {
      command: 'ros2 --version',
      output: 'ros2 cli version: 0.25.5\nROS 2 Humble Hawksbill (humble)',
      exitCode: 0,
      timestamp: new Date(),
      type: 'success',
    };
  }

  /**
   * Handle: ros2 --help
   */
  private handleROS2Help(): TerminalOutput {
    const output = `usage: ros2 [-h] <command>\n\nROS 2 command line tools\n\nCommands:\n  action     Various action related sub-commands\n  bag        Various rosbag related sub-commands\n  daemon     Various daemon related sub-commands\n  doctor     Check ROS setup and other potential issues\n  interface  Show information about ROS interfaces\n  launch     Run a launch file\n  node       Various node related sub-commands\n  param      Various param related sub-commands\n  pkg        Various package related sub-commands\n  run        Run a package specific executable\n  service    Various service related sub-commands\n  topic      Various topic related sub-commands\n\nCall 'ros2 <command> -h' for more detailed usage.`;

    return {
      command: 'ros2 --help',
      output,
      exitCode: 0,
      timestamp: new Date(),
      type: 'info',
    };
  }

  /**
   * Handle: cd <directory>
   */
  private handleCdCommand(command: string): TerminalOutput {
    const dir = command.substring(3).trim();

    if (!dir) {
      return {
        command,
        output: 'Usage: cd <directory>',
        exitCode: 1,
        timestamp: new Date(),
        type: 'error',
      };
    }

    // Update working directory in current session
    if (this.currentSessionId) {
      const session = this.sessions.get(this.currentSessionId);
      if (session) {
        session.workingDirectory = dir.startsWith('/') ? dir : `${session.workingDirectory}/${dir}`;
      }
    }

    return {
      command,
      output: `Changed directory to: ${dir}`,
      exitCode: 0,
      timestamp: new Date(),
      type: 'success',
    };
  }

  /**
   * Handle: pwd
   */
  private handlePwdCommand(): TerminalOutput {
    const session = this.currentSessionId ? this.sessions.get(this.currentSessionId) : null;
    const pwd = session?.workingDirectory || '~/ros2_ws';

    return {
      command: 'pwd',
      output: pwd,
      exitCode: 0,
      timestamp: new Date(),
      type: 'info',
    };
  }

  /**
   * Handle: ls [directory]
   */
  private handleLsCommand(command: string): TerminalOutput {
    const output = `
build/
install/
log/
src/
  navlambda_navigation/
  turtlebot3_gazebo/
  nav2_bringup/
    `.trim();

    return {
      command,
      output,
      exitCode: 0,
      timestamp: new Date(),
      type: 'success',
    };
  }

  /**
   * Get terminal welcome message
   */
  private getWelcomeMessage(): string {
    return `
╔════════════════════════════════════════════════════════════╗
║                                                            ║
║   🤖 NAVΛ Studio ROS2 Terminal                            ║
║   Interactive ROS Learning Environment                     ║
║                                                            ║
╚════════════════════════════════════════════════════════════╝

Welcome to the NAVΛ Studio ROS Learning Terminal!

This terminal provides:
  • ROS2 command execution and simulation
  • Real-time feedback and output
  • Integration with Van Laarhoven Navigation Calculus
  • Safe learning environment

Available commands:
  ros2 run <package> <executable>  - Run a ROS2 node
  ros2 launch <package> <file>     - Launch multiple nodes
  ros2 topic list                  - List active topics
  ros2 node list                   - List running nodes
  ros2 pkg list                    - List installed packages

Basic commands:
  cd <directory>                   - Change directory
  pwd                              - Print working directory
  ls                               - List files
  clear                            - Clear terminal

💡 Tip: All ROS commands are simulated in this learning environment.
💡 Your code and commands are safe to experiment with!

Current workspace: ~/ros2_ws
Ready to learn! Type a command and press Enter.

    `.trim();
  }

  /**
   * Add output to session history
   */
  private addOutput(sessionId: string, output: TerminalOutput): void {
    const session = this.sessions.get(sessionId);
    if (session) {
      session.history.push(output);
    }
  }

  /**
   * Get session history
   */
  getHistory(sessionId?: string): TerminalOutput[] {
    const sid = sessionId || this.currentSessionId;
    if (!sid) return [];

    const session = this.sessions.get(sid);
    return session?.history || [];
  }

  /**
   * Clear session history
   */
  clearHistory(sessionId?: string): void {
    const sid = sessionId || this.currentSessionId;
    if (!sid) return;

    const session = this.sessions.get(sid);
    if (session) {
      session.history = [];
      this.addOutput(sid, {
        command: 'clear',
        output: this.getWelcomeMessage(),
        exitCode: 0,
        timestamp: new Date(),
        type: 'info',
      });
    }
  }

  /**
   * Get command history
   */
  getCommandHistory(): string[] {
    return this.commandHistory;
  }

  /**
   * Close session
   */
  closeSession(sessionId: string): void {
    const session = this.sessions.get(sessionId);
    if (session) {
      session.isActive = false;
    }
  }
}

// Singleton instance
export const rosTerminalService = new ROSTerminalService();

