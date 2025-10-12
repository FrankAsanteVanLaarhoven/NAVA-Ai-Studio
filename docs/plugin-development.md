# NAVΛ Studio Plugin Development Guide

Learn how to extend NAVΛ Studio with custom plugins and integrations.

## Table of Contents

1. [Introduction](#introduction)
2. [Plugin Architecture](#plugin-architecture)
3. [Creating Your First Plugin](#creating-your-first-plugin)
4. [Plugin API](#plugin-api)
5. [Publishing Plugins](#publishing-plugins)

## Introduction

NAVΛ Studio's plugin system allows you to:
- Add custom language features
- Create specialized visualizations
- Integrate external tools
- Extend compilation targets
- Add domain-specific functionality

## Plugin Architecture

Plugins in NAVΛ Studio can extend both the Rust backend and TypeScript frontend.

```
plugin-name/
├── Cargo.toml          # Rust plugin manifest
├── package.json        # Frontend plugin manifest
├── src/
│   ├── lib.rs         # Rust plugin code
│   └── index.ts       # TypeScript plugin code
├── README.md
└── examples/
```

## Creating Your First Plugin

### Step 1: Initialize Plugin

```bash
# Create plugin directory
mkdir my-vnc-plugin
cd my-vnc-plugin

# Initialize Rust plugin
cargo init --lib

# Initialize Node plugin
npm init -y
```

### Step 2: Define Plugin Interface (Rust)

```rust
// src/lib.rs
use navlambda_studio_plugin_api::*;

pub struct MyVncPlugin {
    name: String,
}

impl Plugin for MyVncPlugin {
    fn name(&self) -> &str {
        "My VNC Plugin"
    }

    fn initialize(&mut self) -> Result<(), String> {
        println!("Initializing My VNC Plugin");
        Ok(())
    }

    fn execute(&self, command: &str, args: Vec<String>) -> Result<String, String> {
        match command {
            "custom_navigation" => {
                let result = self.custom_navigation_algorithm(&args);
                Ok(result)
            }
            _ => Err(format!("Unknown command: {}", command))
        }
    }
}

impl MyVncPlugin {
    pub fn new() -> Self {
        Self {
            name: "My VNC Plugin".to_string(),
        }
    }

    fn custom_navigation_algorithm(&self, args: &[String]) -> String {
        // Your custom VNC algorithm here
        format!("Custom navigation result for: {:?}", args)
    }
}

#[no_mangle]
pub extern "C" fn create_plugin() -> Box<dyn Plugin> {
    Box::new(MyVncPlugin::new())
}
```

### Step 3: Add Frontend Extension (TypeScript)

```typescript
// src/index.ts
import { Plugin, PluginContext } from '@navlambda/plugin-api';

export class MyVncPlugin implements Plugin {
  name = 'My VNC Plugin';

  async activate(context: PluginContext): Promise<void> {
    // Register custom commands
    context.registerCommand('myPlugin.customVisualization', () => {
      this.showCustomVisualization();
    });

    // Add custom UI elements
    context.addStatusBarItem({
      text: '⋋ My Plugin',
      command: 'myPlugin.customVisualization',
    });
  }

  private showCustomVisualization(): void {
    // Custom visualization logic
    console.log('Showing custom VNC visualization');
  }

  async deactivate(): Promise<void> {
    // Cleanup
  }
}

export function activate(context: PluginContext): void {
  const plugin = new MyVncPlugin();
  plugin.activate(context);
}
```

### Step 4: Configure Plugin Manifest

```toml
# Cargo.toml
[package]
name = "my-vnc-plugin"
version = "0.1.0"
edition = "2021"

[dependencies]
navlambda-studio-plugin-api = "1.0"

[lib]
crate-type = ["cdylib"]
```

```json
// package.json
{
  "name": "@navlambda/my-vnc-plugin",
  "version": "0.1.0",
  "main": "dist/index.js",
  "types": "dist/index.d.ts",
  "navlambda": {
    "displayName": "My VNC Plugin",
    "description": "Custom navigation visualization plugin",
    "categories": ["Visualization", "Navigation"],
    "activationEvents": [
      "onLanguage:navlambda"
    ]
  },
  "dependencies": {
    "@navlambda/plugin-api": "^1.0.0"
  }
}
```

## Plugin API

### Rust Plugin API

#### Plugin Trait

```rust
pub trait Plugin: Send + Sync {
    /// Plugin name
    fn name(&self) -> &str;
    
    /// Initialize the plugin
    fn initialize(&mut self) -> Result<(), String>;
    
    /// Execute a command
    fn execute(&self, command: &str, args: Vec<String>) -> Result<String, String>;
}
```

#### Accessing VNC Parser

```rust
use navlambda_studio_plugin_api::parser::*;

pub fn custom_parse(&self, code: &str) -> Result<CustomAst, ParseError> {
    let parser = VncParser::new();
    let ast = parser.parse(code)?;
    
    // Transform or analyze the AST
    let custom_ast = self.transform_ast(ast);
    
    Ok(custom_ast)
}
```

#### Custom Compilation Target

```rust
use navlambda_studio_plugin_api::compiler::*;

pub struct MyTargetCompiler;

impl TargetCompiler for MyTargetCompiler {
    fn compile(&self, ast: &NavLambdaAst) -> Result<CompiledOutput, CompilationError> {
        let mut code = String::new();
        
        // Generate code for your target platform
        for statement in &ast.statements {
            code.push_str(&self.generate_statement(statement)?);
        }
        
        Ok(CompiledOutput {
            code,
            dependencies: vec![],
            build_instructions: "my-compiler output.code".to_string(),
            performance_profile: PerformanceProfile::default(),
        })
    }
}
```

### TypeScript Plugin API

#### Plugin Interface

```typescript
export interface Plugin {
  name: string;
  activate(context: PluginContext): Promise<void>;
  deactivate(): Promise<void>;
}

export interface PluginContext {
  // Register commands
  registerCommand(id: string, handler: () => void): void;
  
  // UI extensions
  addStatusBarItem(item: StatusBarItem): void;
  addToolbarButton(button: ToolbarButton): void;
  
  // Editor extensions
  registerCompletionProvider(provider: CompletionProvider): void;
  registerHoverProvider(provider: HoverProvider): void;
  
  // Visualization extensions
  registerVisualization(viz: VisualizationProvider): void;
}
```

#### Custom Visualizations

```typescript
import { VisualizationProvider } from '@navlambda/plugin-api';
import * as THREE from 'three';

export class CustomVisualization implements VisualizationProvider {
  name = 'Custom VNC Visualization';
  
  createScene(data: NavigationData): THREE.Scene {
    const scene = new THREE.Scene();
    
    // Add custom 3D objects
    const geometry = new THREE.BoxGeometry(1, 1, 1);
    const material = new THREE.MeshBasicMaterial({ color: 0x00ff00 });
    const cube = new THREE.Mesh(geometry, material);
    
    scene.add(cube);
    
    return scene;
  }
  
  update(scene: THREE.Scene, data: NavigationData): void {
    // Update visualization with new data
  }
}
```

#### Custom Language Features

```typescript
import { CompletionProvider } from '@navlambda/plugin-api';

export class CustomCompletionProvider implements CompletionProvider {
  provideCompletions(position: number, context: string): Completion[] {
    return [
      {
        label: 'custom_navigate⋋',
        kind: 'Function',
        detail: 'Custom navigation function',
        documentation: 'My custom VNC navigation algorithm',
        insertText: 'custom_navigate⋋($1, $2)',
      },
    ];
  }
}
```

## Plugin Examples

### Example 1: Quantum Navigation Plugin

```rust
// Extends VNC with quantum navigation algorithms
pub struct QuantumNavigationPlugin;

impl Plugin for QuantumNavigationPlugin {
    fn execute(&self, command: &str, args: Vec<String>) -> Result<String, String> {
        match command {
            "quantum_navigate" => {
                let path = self.quantum_navigation_algorithm(&args);
                Ok(serde_json::to_string(&path).unwrap())
            }
            _ => Err("Unknown command".to_string())
        }
    }
}
```

### Example 2: Machine Learning Plugin

```typescript
export class MLNavigationPlugin implements Plugin {
  private model: MLModel;
  
  async activate(context: PluginContext): Promise<void> {
    this.model = await loadModel('vnc-ml-model');
    
    context.registerCommand('ml.predictOptimalPath', async () => {
      const prediction = await this.model.predict(currentState);
      visualizePath(prediction);
    });
  }
}
```

### Example 3: Custom Visualization

```typescript
export class HeatmapVisualization implements VisualizationProvider {
  name = 'Energy Heatmap';
  
  createScene(data: EnergyLandscape): THREE.Scene {
    const scene = new THREE.Scene();
    
    // Create heatmap from energy data
    const heatmap = this.generateHeatmap(data);
    scene.add(heatmap);
    
    return scene;
  }
  
  private generateHeatmap(data: EnergyLandscape): THREE.Mesh {
    // Generate heatmap mesh
    // ...
  }
}
```

## Testing Plugins

### Unit Tests (Rust)

```rust
#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_plugin_initialization() {
        let mut plugin = MyVncPlugin::new();
        assert!(plugin.initialize().is_ok());
    }
    
    #[test]
    fn test_custom_navigation() {
        let plugin = MyVncPlugin::new();
        let result = plugin.execute("custom_navigation", vec!["arg1".to_string()]);
        assert!(result.is_ok());
    }
}
```

### Integration Tests (TypeScript)

```typescript
import { describe, it, expect } from 'vitest';
import { MyVncPlugin } from './index';

describe('MyVncPlugin', () => {
  it('should activate successfully', async () => {
    const plugin = new MyVncPlugin();
    const context = createMockContext();
    
    await expect(plugin.activate(context)).resolves.toBeUndefined();
  });
});
```

## Publishing Plugins

### 1. Package Your Plugin

```bash
# Build Rust plugin
cargo build --release

# Build TypeScript plugin
npm run build
```

### 2. Publish to Registry

```bash
# Publish to crates.io
cargo publish

# Publish to npm
npm publish --access public
```

### 3. Submit to Plugin Marketplace

Create a plugin listing with:
- Name and description
- Screenshots/demos
- Installation instructions
- Documentation link
- Source code repository

## Best Practices

1. **Follow naming conventions**
   - Use clear, descriptive names
   - Prefix with `navlambda-` or `@navlambda/`

2. **Provide comprehensive documentation**
   - API reference
   - Usage examples
   - Configuration options

3. **Test thoroughly**
   - Unit tests for core functionality
   - Integration tests with NAVΛ Studio
   - Performance testing

4. **Keep dependencies minimal**
   - Only include necessary dependencies
   - Specify version constraints

5. **Follow security best practices**
   - Validate all inputs
   - Avoid arbitrary code execution
   - Use secure communication

## Support and Resources

- **API Documentation**: https://docs.navlambda.studio/api
- **Plugin Examples**: https://github.com/navlambda/plugins
- **Community Forum**: https://forum.navlambda.studio
- **Discord**: https://discord.gg/navlambda

---

**Happy Plugin Development!** 🔌⋋💻

