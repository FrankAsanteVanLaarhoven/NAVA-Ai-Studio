# NAVΛ SDK API Reference

Complete API documentation for the NAVΛ SDK.

## Core Classes

### NavigationField

Main class for navigation calculus operations.

#### Methods

##### `new()`

Create a new NavigationField instance.

**Rust:**
```rust
let nav = NavigationField::new();
```

**TypeScript:**
```typescript
const nav = new NavigationField();
```

**Python:**
```python
nav = NavigationField()
```

##### `set_manifold(manifold)`

Set the manifold for path planning.

**Parameters:**
- `manifold` (Manifold): The manifold configuration

**Rust:**
```rust
nav.set_manifold(Manifold::euclidean(3));
```

**TypeScript:**
```typescript
await nav.setManifold(Manifold.euclidean(3));
```

**Python:**
```python
nav.set_manifold(3)  # 3D Euclidean
```

##### `find_optimal_path(start, goal, constraints)`

Find an optimal path from start to goal.

**Parameters:**
- `start` (number[]): Starting coordinates
- `goal` (number[]): Goal coordinates
- `constraints` (NavigationConstraints): Navigation constraints

**Returns:**
- `Path`: Optimal path with waypoints

**Rust:**
```rust
let path = nav.find_optimal_path(
    &[0.0, 0.0, 0.0],
    &[5.0, 5.0, 5.0],
    &NavigationConstraints::default()
)?;
```

**TypeScript:**
```typescript
const path = await nav.findOptimalPath(
    [0, 0, 0],
    [5, 5, 5],
    NavigationConstraints.default()
);
```

**Python:**
```python
path = nav.find_optimal_path(
    [0.0, 0.0, 0.0],
    [5.0, 5.0, 5.0],
    max_velocity=2.0
)
```

## Data Structures

### Manifold

Represents a mathematical manifold.

**Types:**
- `Euclidean`: Standard Euclidean space
- `Riemannian`: Riemannian manifold with custom metric
- `Lorentzian`: Lorentzian manifold for spacetime

### Path

Represents a navigation path.

**Properties:**
- `waypoints`: Array of path points
- `totalEnergy`: Total energy of the path
- `optimizationMethod`: Method used (e.g., "vnc")

### NavigationConstraints

Navigation constraints for path planning.

**Properties:**
- `maxVelocity`: Maximum allowed velocity
- `avoidObstacles`: Whether to avoid obstacles
- `custom`: Custom constraint parameters

## Error Handling

All SDK methods return `Result<T>` (Rust) or throw exceptions (TypeScript/Python) on error.

**Error Types:**
- `InvalidManifold`: Invalid manifold configuration
- `PathPlanningFailed`: Path planning algorithm failed
- `InvalidConstraints`: Invalid constraint parameters

## Examples

See the [examples directory](../examples/) for complete working examples.

