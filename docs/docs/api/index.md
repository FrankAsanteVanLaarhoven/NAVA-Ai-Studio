---
id: index
title: API Reference
sidebar_label: Overview
---

# NAVΛ Studio API Reference

Welcome to the comprehensive API documentation for NAVΛ Studio. This reference covers all public APIs, components, and utilities available in the IDE.

## 📚 API Categories

### Core Components
- **[Editor API](components/editor)** - Monaco Editor integration and customizations
- **[Terminal API](components/terminal)** - Terminal management and command execution
- **[Visualizer API](components/visualizer)** - 3D visualization and rendering
- **[Compiler API](components/compiler)** - Multi-target compilation interfaces
- **[LSP API](components/lsp)** - Language Server Protocol implementation

### ROS Integration
- **[ROS Terminal API](ros/terminal)** - ROS command execution and management
- **[ROS Commands API](ros/commands)** - ROS2 command wrappers and utilities
- **[ROS Simulation API](ros/simulation)** - Gazebo integration and control
- **[ROS Navigation API](ros/navigation)** - Navigation-specific ROS interfaces

### Utility APIs
- **[Parser API](utils/parser)** - VNC syntax parsing and analysis
- **[Validator API](utils/validator)** - Code validation and error checking
- **[Formatter API](utils/formatter)** - Code formatting and beautification
- **[Config API](utils/config)** - Configuration management and settings

## 🔧 Core Interfaces

### Editor Interface
```typescript
interface EditorAPI {
  createEditor(container: HTMLElement, options: EditorOptions): MonacoEditor;
  setVNCMode(editor: MonacoEditor): void;
  registerCompletionProvider(language: string, provider: CompletionProvider): void;
  registerSyntaxHighlighter(language: string, tokens: TokenDefinition[]): void;
}
```

### Terminal Interface
```typescript
interface TerminalAPI {
  createTerminal(options: TerminalOptions): TerminalSession;
  executeCommand(session: TerminalSession, command: string): Promise<CommandResult>;
  registerCommandHandler(command: string, handler: CommandHandler): void;
  getActiveSessions(): TerminalSession[];
}
```

### Visualizer Interface
```typescript
interface VisualizerAPI {
  createScene(container: HTMLElement): Scene3D;
  renderNavigationPath(path: NavigationPath, options: RenderOptions): void;
  visualizeEnergyLandscape(landscape: EnergyLandscape): void;
  setCamera(position: Vector3, target: Vector3): void;
  exportScene(format: 'png' | 'svg' | 'json'): Promise<string>;
}
```

### Compiler Interface
```typescript
interface CompilerAPI {
  compile(source: string, target: CompilationTarget): Promise<CompilationResult>;
  getSupportedTargets(): CompilationTarget[];
  setOptimizationLevel(level: OptimizationLevel): void;
  registerCustomTarget(target: CustomTarget): void;
}
```

## 🔗 Quick Links

### Getting Started
- [Installation Guide](../installation)
- [Quick Start Tutorial](../quick-start)
- [First Project Tutorial](../first-project)

### Component APIs
- [Editor Components](components/editor)
- [Terminal Components](components/terminal)
- [3D Visualizer](components/visualizer)
- [Compiler System](components/compiler)

### ROS APIs
- [ROS Terminal](ros/terminal)
- [ROS Commands](ros/commands)
- [ROS Simulation](ros/simulation)
- [ROS Navigation](ros/navigation)

### Utility APIs
- [Parser Utilities](utils/parser)
- [Validation Tools](utils/validator)
- [Formatting Tools](utils/formatter)
- [Configuration Management](utils/config)

## 📋 Code Examples

### Creating an Editor
```typescript
import { EditorAPI } from '@navlambda/studio';

const editor = EditorAPI.createEditor(document.getElementById('editor'), {
  language: 'navlambda',
  theme: 'vs-dark',
  fontSize: 14,
  minimap: { enabled: true }
});

EditorAPI.setVNCMode(editor);
```

### Executing ROS Commands
```typescript
import { TerminalAPI } from '@navlambda/studio';

const terminal = TerminalAPI.createTerminal({
  shell: '/bin/bash',
  cwd: '/workspace/ros-project'
});

const result = await TerminalAPI.executeCommand(terminal, 'ros2 topic list');
console.log(result.output);
```

### 3D Visualization
```typescript
import { VisualizerAPI } from '@navlambda/studio';

const scene = VisualizerAPI.createScene(document.getElementById('canvas'));
const path = new NavigationPath([
  { x: 0, y: 0, z: 0 },
  { x: 10, y: 5, z: 2 },
  { x: 20, y: 0, z: 4 }
]);

VisualizerAPI.renderNavigationPath(path, {
  color: '#00ff00',
  width: 2,
  animated: true
});
```

### Multi-Target Compilation
```typescript
import { CompilerAPI } from '@navlambda/studio';

const source = `
⋋(x, y) = navigate(x, y, energy_function)
  where energy_function = λ(p) → p.distance + p.obstacle_penalty
`;

const result = await CompilerAPI.compile(source, 'cpp');
console.log(result.output);
console.log(result.errors);
```

## 📚 Type Definitions

All API documentation includes comprehensive TypeScript definitions. For detailed type information, refer to individual API pages.

## 📋 Version Information

- **Current Version**: 1.0.0
- **API Stability**: Stable
- **Breaking Changes**: None planned for v1.x

## 💡 Support

For API support and questions:
- [GitHub Issues](https://github.com/navlambda/studio/issues)
- [Stack Overflow](https://stackoverflow.com/questions/tagged/navlambda)
- [Discord Community](https://discord.gg/navlambda)

---

*This API documentation is automatically generated from source code and updated with each release.*