# NAVΛ Language Reference

Complete reference for Van Laarhoven Navigation Calculus programming language.

## Table of Contents

1. [Introduction](#introduction)
2. [Syntax](#syntax)
3. [VNC Symbols](#vnc-symbols)
4. [Operators](#operators)
5. [Functions](#functions)
6. [Types](#types)
7. [Examples](#examples)

## Introduction

NAVΛ is a programming language designed specifically for Van Laarhoven Navigation Calculus, enabling elegant expression of navigation optimization, path planning, and energy minimization problems.

## Syntax

### Comments

```navλ
// Single-line comment

/*
  Multi-line
  comment
*/
```

### Variables

```navλ
let x = 42
let position = [0.0, 0.0, 0.0]
let name = "Navigation Path"
```

### Functions

```navλ
fn optimal_path(start, goal) {
  let path = navigate_to⋋(start, goal)
  return path
}
```

## VNC Symbols

### Core Navigation Symbol: ⋋

The lambda navigation symbol (⋋) is the foundation of VNC programming.

**Keyboard Shortcut**: `Alt+L`

**Usage**:
```navλ
// Basic navigation
let path = navigate_to⋋(start, goal)

// Navigation with constraints
let constrained = navigate_to⋋(start, goal, constraints)
```

### Navigation Operators

| Symbol | Name | Shortcut | Description |
|--------|------|----------|-------------|
| ⋋ | Lambda Navigation | `Alt+L` | Core navigation operator |
| ⊗⋋ | Tensor Product | `Alt+Shift+T` | Combines navigation spaces |
| ⊕⋋ | Navigation Sum | `Alt+Shift+S` | Parallel paths |
| ∪⋋ | Navigation Union | - | Merge navigation spaces |
| ∩⋋ | Navigation Intersection | - | Common paths |
| ↑⋋ | North Navigation | - | Directional constraint |
| ↓⋋ | South Navigation | - | Directional constraint |
| →⋋ | East Navigation | - | Directional constraint |
| ←⋋ | West Navigation | - | Directional constraint |

### Master Operators

| Symbol | Name | Shortcut | Description |
|--------|------|----------|-------------|
| 𝒩ℐ | Master Navigation | `Alt+Shift+M` | Ultimate VNC optimization |
| ℰ | Evolution Operator | - | Time evolution |

## Operators

### Tensor Product: ⊗⋋

Combines multiple navigation spaces into a higher-dimensional space.

```navλ
let space1 = navigation_space([0, 10])
let space2 = navigation_space([0, 10])
let combined = space1 ⊗⋋ space2
```

### Navigation Sum: ⊕⋋

Creates parallel navigation paths.

```navλ
let path1 = navigate_to⋋(start1, goal1)
let path2 = navigate_to⋋(start2, goal2)
let parallel = path1 ⊕⋋ path2
```

### Navigation Union: ∪⋋

Merges navigation spaces.

```navλ
let merged = space1 ∪⋋ space2
```

### Navigation Intersection: ∩⋋

Finds common navigation paths.

```navλ
let common = path1 ∩⋋ path2
```

## Functions

### Navigation Functions

#### `navigate_to⋋(start, goal)`

Navigate from start to goal using VNC optimization.

**Parameters**:
- `start`: Starting position (vector)
- `goal`: Goal position (vector)

**Returns**: Optimized navigation path

**Example**:
```navλ
let path = navigate_to⋋([0, 0, 0], [5, 5, 5])
```

#### `find_optimal_path⋋(start, goal, landscape)`

Find the globally optimal path considering energy landscape.

**Parameters**:
- `start`: Starting position
- `goal`: Goal position  
- `landscape`: Energy landscape function

**Returns**: Globally optimal path

**Example**:
```navλ
let landscape = energy_landscape⋋(terrain_data)
let optimal = find_optimal_path⋋(start, goal, landscape)
```

#### `energy_landscape⋋(data)`

Create an energy landscape from terrain or cost data.

**Parameters**:
- `data`: Terrain or cost data

**Returns**: Energy landscape object

**Example**:
```navλ
let landscape = energy_landscape⋋(terrain_grid)
```

### Visualization Functions

#### `visualize⋋(path)`

Visualize a navigation path in 3D.

**Parameters**:
- `path`: Navigation path to visualize

**Example**:
```navλ
let path = navigate_to⋋(start, goal)
visualize⋋(path)
```

### Master Navigation Operator

#### `𝒩ℐ(expression)`

Apply the master navigation operator for ultimate optimization.

**Parameters**:
- `expression`: Navigation expression to optimize

**Returns**: Maximally optimized result

**Example**:
```navλ
let ultimate = 𝒩ℐ(find_optimal_path⋋(start, goal, landscape))
```

## Types

### Vector

Represents a point in n-dimensional space.

```navλ
let point2d = [x, y]
let point3d = [x, y, z]
let point_nd = [x1, x2, x3, ..., xn]
```

### NavigationPath

Represents a path through navigation space.

```navλ
struct NavigationPath {
  waypoints: [Vector],
  energy: Number,
  optimization_method: String
}
```

### EnergyLandscape

Represents an energy landscape for navigation.

```navλ
struct EnergyLandscape {
  potential: Function,
  dimensions: [Number]
}
```

## Examples

### Basic Navigation

```navλ
// Define start and goal
let start = [0.0, 0.0, 0.0]
let goal = [10.0, 10.0, 0.0]

// Navigate with VNC optimization
let path = navigate_to⋋(start, goal)

// Visualize the result
visualize⋋(path)
```

### Energy-Aware Navigation

```navλ
// Create energy landscape
let terrain = load_terrain("terrain.dat")
let landscape = energy_landscape⋋(terrain)

// Find optimal path
let optimal = find_optimal_path⋋(start, goal, landscape)

// Apply master optimization
let ultimate = 𝒩ℐ(optimal)

// Visualize with energy overlay
visualize⋋(ultimate, show_energy: true)
```

### Multi-Path Navigation

```navλ
// Define multiple goals
let goals = [
  [10.0, 0.0, 0.0],
  [0.0, 10.0, 0.0],
  [10.0, 10.0, 0.0]
]

// Create parallel paths
let paths = []
for goal in goals {
  let path = navigate_to⋋(start, goal)
  paths.append(path)
}

// Combine with navigation sum
let combined = paths[0] ⊕⋋ paths[1] ⊕⋋ paths[2]

// Visualize all paths
visualize⋋(combined)
```

### Constrained Navigation

```navλ
// Define navigation with directional constraints
let path = navigate_to⋋(start, goal)
  .with_constraint(↑⋋)  // Prefer northward movement
  .with_constraint(→⋋)  // Prefer eastward movement
  .avoid(obstacles)

// Optimize with constraints
let optimized = 𝒩ℐ(path)
```

### Tensor Product Navigation

```navλ
// Create 2D navigation spaces
let horizontal = navigation_space([0, 100])
let vertical = navigation_space([0, 100])

// Combine into 2D space
let space2d = horizontal ⊗⋋ vertical

// Navigate in combined space
let path = navigate_to⋋(
  [0, 0],
  [100, 100],
  space: space2d
)
```

## Best Practices

1. **Always use ⋋ operators for navigation tasks**
   - They provide VNC optimization automatically
   - Better performance than classical algorithms

2. **Consider energy landscapes for complex terrains**
   - Provides better paths in difficult environments
   - Enables global optimization

3. **Use master operator 𝒩ℐ for critical paths**
   - Ultimate optimization for important routes
   - Higher computational cost but best results

4. **Visualize paths during development**
   - Helps debug navigation logic
   - Verifies optimization quality

5. **Combine paths with navigation operators**
   - ⊕⋋ for parallel exploration
   - ∩⋋ for finding common routes
   - ∪⋋ for merging alternatives

## Language Extensions

NAVΛ supports custom extensions through the plugin system. See the [Plugin Development Guide](plugin-development.md) for details.

## Compilation Targets

NAVΛ code can compile to:
- **C++** - High performance native code
- **Python** - Data science and prototyping
- **WebAssembly** - Browser and web apps
- **GLSL** - GPU shaders and compute
- **Cloud** - Docker/Kubernetes deployments

See [Multi-Target Compilation](compilation-targets.md) for details.

---

**NAVΛ Language**: Making navigation optimization elegant and powerful 🚀⋋

