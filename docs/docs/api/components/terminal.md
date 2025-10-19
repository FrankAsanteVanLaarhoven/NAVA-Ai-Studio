---
id: terminal
title: Terminal API
sidebar_label: Terminal
---

# Terminal API Reference

The Terminal API provides comprehensive terminal management with specialized ROS integration for NAVΛ Studio.

## 📝 Overview

The Terminal API wraps xterm.js with additional functionality for ROS command execution, session management, and integration with the NAVΛ development environment.

## 🔧 Core Interfaces

### TerminalOptions
```typescript
interface TerminalOptions {
  shell?: string;
  cwd?: string;
  env?: Record<string, string>;
  rows?: number;
  cols?: number;
  fontSize?: number;
  fontFamily?: string;
  theme?: TerminalTheme;
  scrollback?: number;
  allowTransparency?: boolean;
  rosEnabled?: boolean;
  commandWhitelist?: string[];
}
```

### TerminalSession
```typescript
interface TerminalSession {
  id: string;
  terminal: Terminal;
  process: ITerminalChildProcess;
  cwd: string;
  env: Record<string, string>;
  rosEnabled: boolean;
  commandHistory: string[];
  status: 'active' | 'inactive' | 'error';
  createdAt: Date;
  lastActivity: Date;
}
```

### CommandResult
```typescript
interface CommandResult {
  command: string;
  exitCode: number;
  stdout: string;
  stderr: string;
  duration: number;
  timestamp: Date;
  rosCommand?: boolean;
  parsedOutput?: any;
}
```

### ROSCommand
```typescript
interface ROSCommand {
  command: string;
  args: string[];
  options: ROSCommandOptions;
  category: 'topic' | 'node' | 'service' | 'param' | 'launch';
  description: string;
  examples: string[];
}
```

## 📋 API Methods

### createTerminal(options)
Creates a new terminal session with optional ROS support.

```typescript
static createTerminal(options: TerminalOptions): TerminalSession
```

**Parameters:**
- `options`: Terminal configuration options

**Returns:** TerminalSession instance

**Example:**
```typescript
const session = TerminalAPI.createTerminal({
  shell: '/bin/bash',
  cwd: '/workspace/ros-project',
  rows: 24,
  cols: 80,
  rosEnabled: true,
  theme: {
    background: '#1e1e1e',
    foreground: '#d4d4d4',
    cursor: '#ffffff'
  }
});
```

### executeCommand(session, command)
Executes a command in the specified terminal session.

```typescript
static executeCommand(
  session: TerminalSession, 
  command: string, 
  options?: ExecuteOptions
): Promise<CommandResult>
```

**Parameters:**
- `session`: Terminal session instance
- `command`: Command to execute
- `options`: Optional execution parameters

**Returns:** Promise resolving to CommandResult

**Example:**
```typescript
const result = await TerminalAPI.executeCommand(session, 'ros2 topic list');
console.log('Topics:', result.stdout);
console.log('Exit code:', result.exitCode);
```

### executeROSCommand(session, rosCommand)
Executes a ROS-specific command with enhanced parsing and validation.

```typescript
static executeROSCommand(
  session: TerminalSession, 
  rosCommand: ROSCommand
): Promise<CommandResult>
```

**Parameters:**
- `session`: Terminal session instance
- `rosCommand`: ROS command definition

**Returns:** Promise resolving to CommandResult with parsed output

**Example:**
```typescript
const rosCommand = {
  command: 'ros2 topic list',
  args: [],
  category: 'topic',
  description: 'List all ROS topics',
  examples: ['ros2 topic list', 'ros2 topic list -t']
};

const result = await TerminalAPI.executeROSCommand(session, rosCommand);
console.log('Available topics:', result.parsedOutput);
```

### getActiveSessions()
Retrieves all active terminal sessions.

```typescript
static getActiveSessions(): TerminalSession[]
```

**Returns:** Array of active terminal sessions

**Example:**
```typescript
const sessions = TerminalAPI.getActiveSessions();
sessions.forEach(session => {
  console.log(`Session ${session.id}: ${session.status}`);
});
```

### getSessionById(id)
Retrieves a specific terminal session by ID.

```typescript
static getSessionById(id: string): TerminalSession | null
```

**Parameters:**
- `id`: Session identifier

**Returns:** TerminalSession instance or null

**Example:**
```typescript
const session = TerminalAPI.getSessionById('session-123');
if (session) {
  console.log('Session found:', session.cwd);
}
```

### registerCommandHandler(session, command, handler)
Registers a custom command handler for specific commands.

```typescript
static registerCommandHandler(
  session: TerminalSession, 
  command: string, 
  handler: CommandHandler
): void
```

**Parameters:**
- `session`: Terminal session instance
- `command`: Command pattern to handle
- `handler`: Handler function

**Example:**
```typescript
TerminalAPI.registerCommandHandler(session, 'navlambda:*', (command, args) => {
  if (command === 'navlambda:compile') {
    return compileVNCCode(args[0]);
  }
});
```

### sendInput(session, input)
Sends input to the terminal session.

```typescript
static sendInput(session: TerminalSession, input: string): void
```

**Parameters:**
- `session`: Terminal session instance
- `input`: Input to send

**Example:**
```typescript
TerminalAPI.sendInput(session, 'echo "Hello NAVΛ Studio"');
```

### resizeTerminal(session, cols, rows)
Resizes the terminal dimensions.

```typescript
static resizeTerminal(
  session: TerminalSession, 
  cols: number, 
  rows: number
): void
```

**Parameters:**
- `session`: Terminal session instance
- `cols`: Number of columns
- `rows`: Number of rows

**Example:**
```typescript
TerminalAPI.resizeTerminal(session, 120, 30);
```

### disposeSession(session)
Disposes of a terminal session and cleans up resources.

```typescript
static disposeSession(session: TerminalSession): void
```

**Parameters:**
- `session`: Terminal session to dispose

**Example:**
```typescript
TerminalAPI.disposeSession(session);
```

## 🧪 ROS Integration Features

### Built-in ROS Commands
The Terminal API includes pre-defined ROS2 commands:

```typescript
const rosCommands = {
  'ros2:topic:list': {
    command: 'ros2 topic list',
    category: 'topic',
    description: 'List all active ROS topics',
    parser: (output: string) => output.split('\n').filter(line => line.trim())
  },
  'ros2:node:list': {
    command: 'ros2 node list',
    category: 'node',
    description: 'List all ROS nodes',
    parser: (output: string) => output.split('\n').filter(line => line.trim())
  },
  'ros2:service:list': {
    command: 'ros2 service list',
    category: 'service',
    description: 'List all ROS services',
    parser: (output: string) => output.split('\n').filter(line => line.trim())
  },
  'ros2:launch:simulate': {
    command: 'ros2 launch gazebo_ros empty_world.launch.py',
    category: 'launch',
    description: 'Start Gazebo simulation',
    requiresSimulation: true
  }
};
```

### Command Validation
ROS commands are validated before execution:

```typescript
function validateROSCommand(command: string): ValidationResult {
  // Check if ROS2 is installed
  if (!isROS2Installed()) {
    return { valid: false, error: 'ROS2 is not installed' };
  }
  
  // Check if command is in whitelist
  if (!isWhitelistedCommand(command)) {
    return { valid: false, error: 'Command not allowed' };
  }
  
  return { valid: true };
}
```

### Output Parsing
ROS command outputs are automatically parsed:

```typescript
function parseROSOutput(command: string, output: string): ParsedOutput {
  switch (command) {
    case 'ros2 topic list':
      return {
        type: 'topic_list',
        data: output.split('\n').filter(line => line.trim())
      };
    case 'ros2 node list':
      return {
        type: 'node_list',
        data: output.split('\n').filter(line => line.trim())
      };
    case 'ros2 topic echo':
      return {
        type: 'topic_data',
        data: parseTopicMessage(output)
      };
    default:
      return { type: 'raw', data: output };
  }
}
```

## 📋 Code Examples

### Basic Terminal Usage
```typescript
import { TerminalAPI } from '@navlambda/studio';

// Create terminal with ROS support
const session = TerminalAPI.createTerminal({
  shell: '/bin/bash',
  cwd: '/workspace/ros-project',
  rosEnabled: true,
  theme: {
    background: '#1e1e1e',
    foreground: '#d4d4d4',
    cursor: '#ffffff',
    selection: '#264f78'
  }
});

// Execute basic command
const result = await TerminalAPI.executeCommand(session, 'ls -la');
console.log('Files:', result.stdout);
```

### ROS Command Execution
```typescript
// Execute ROS topic list
const topicResult = await TerminalAPI.executeCommand(session, 'ros2 topic list');
if (topicResult.exitCode === 0) {
  const topics = topicResult.stdout.split('\n').filter(t => t.trim());
  console.log('Available topics:', topics);
}

// Execute ROS node list
const nodeResult = await TerminalAPI.executeCommand(session, 'ros2 node list');
if (nodeResult.exitCode === 0) {
  const nodes = nodeResult.stdout.split('\n').filter(n => n.trim());
  console.log('Active nodes:', nodes);
}
```

### Advanced ROS Integration
```typescript
// Register custom ROS command handler
TerminalAPI.registerCommandHandler(session, 'ros2:custom:*', async (command, args) => {
  if (command === 'ros2:custom:navigate') {
    const startPoint = args[0];
    const endPoint = args[1];
    
    // Execute navigation command
    const navCommand = `ros2 run navlambda_navigation navigate --start ${startPoint} --end ${endPoint}`;
    return await TerminalAPI.executeCommand(session, navCommand);
  }
});

// Use custom handler
const result = await TerminalAPI.executeCommand(session, 'ros2:custom:navigate 0,0 10,10');
console.log('Navigation result:', result.stdout);
```

### Session Management
```typescript
// Get all active sessions
const sessions = TerminalAPI.getActiveSessions();
sessions.forEach(session => {
  console.log(`Session ${session.id}:`);
  console.log(`  CWD: ${session.cwd}`);
  console.log(`  ROS Enabled: ${session.rosEnabled}`);
  console.log(`  Status: ${session.status}`);
  console.log(`  Commands: ${session.commandHistory.length}`);
});

// Find specific session
const rosSession = sessions.find(s => s.rosEnabled && s.cwd.includes('ros-project'));
if (rosSession) {
  console.log('Found ROS session:', rosSession.id);
}
```

### Terminal Event Handling
```typescript
// Listen for terminal events
session.terminal.onData((data) => {
  console.log('Terminal output:', data);
});

session.terminal.onExit((exitCode) => {
  console.log('Terminal exited with code:', exitCode);
});

// Handle command completion
session.terminal.onLineFeed(() => {
  const currentLine = session.terminal.buffer.active.getLine(session.terminal.buffer.active.cursorY);
  if (currentLine) {
    console.log('Command executed:', currentLine.translateToString());
  }
});
```

## 🔧 Error Handling

### Common Errors
```typescript
try {
  const session = TerminalAPI.createTerminal({
    shell: '/invalid/shell'
  });
} catch (error) {
  if (error instanceof TerminalAPI.InvalidShellError) {
    console.error('Invalid shell path');
  } else if (error instanceof TerminalAPI.UnsupportedROSError) {
    console.error('ROS not supported on this platform');
  } else {
    console.error('Unknown error:', error);
  }
}
```

### Command Execution Errors
```typescript
try {
  const result = await TerminalAPI.executeCommand(session, 'invalid-command');
  if (result.exitCode !== 0) {
    console.error('Command failed:', result.stderr);
  }
} catch (error) {
  console.error('Execution error:', error);
}
```

## 📈 Performance Considerations

### Session Limits
- Maximum 10 active sessions per workspace
- Sessions auto-dispose after 30 minutes of inactivity
- Command history limited to 1000 entries per session

### Memory Management
```typescript
// Always dispose sessions when done
useEffect(() => {
  const session = TerminalAPI.createTerminal({ rosEnabled: true });
  
  return () => {
    TerminalAPI.disposeSession(session);
  };
}, []);
```

### Command Optimization
- Batch related commands together
- Use ROS command shortcuts when available
- Cache frequently used command outputs
- Limit output parsing for large datasets

## 🔀 Related APIs

- [ROS Commands API](ros/commands) - Detailed ROS command reference
- [Editor API](editor) - Code editor integration
- [Visualizer API](visualizer) - 3D visualization
- [Compiler API](compiler) - Multi-target compilation

---

*For ROS-specific terminal features, see the [ROS Terminal Integration Guide](../../ros/terminal-commands).*