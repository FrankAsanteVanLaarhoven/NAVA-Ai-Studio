# NAVΛ Simulation Examples

This directory contains example world configurations for the NAVΛ Simulation Platform.

## Available Worlds

### 1. Warehouse World (`warehouse_world.json`)

A typical warehouse environment with shelving, pallets, and support columns.

**Features:**
- 2 large shelving units
- Pallets on the ground
- Support columns
- Concrete floor
- Industrial lighting

**Best For:**
- Logistics robots
- Forklift simulations
- Indoor navigation
- Obstacle avoidance testing

**Usage:**
```bash
cargo run --release -- --config examples/warehouse_world.json
```

---

### 2. Outdoor World (`outdoor_world.json`)

Natural outdoor environment with trees and rocks.

**Features:**
- Trees with realistic dimensions
- Large rocks and boulders
- Grass ground plane
- Natural daylight simulation

**Best For:**
- All-terrain vehicles
- Outdoor navigation
- Path planning in natural environments
- Ackermann steering robots

**Usage:**
```bash
cargo run --release -- --config examples/outdoor_world.json
```

---

### 3. Obstacle Course (`obstacle_course.json`)

Challenging obstacle course for testing advanced navigation.

**Features:**
- Barriers at various angles
- Tunnel passage
- Pillars to navigate around
- Ramp for elevation changes
- Rubber floor for traction testing

**Best For:**
- Algorithm testing
- Navigation challenges
- Path planning validation
- Agility demonstrations

**Usage:**
```bash
cargo run --release -- --config examples/obstacle_course.json
```

---

## Creating Custom Worlds

You can create your own world configurations using this JSON structure:

```json
{
  "world": {
    "name": "my_world",
    "gravity": [0.0, -9.81, 0.0],
    "ground_plane": {
      "enabled": true,
      "height": 0.0,
      "size": 50.0,
      "material": "concrete"
    },
    "obstacles": [
      {
        "id": "obstacle1",
        "shape": "box",
        "position": [x, y, z],
        "rotation": [roll, pitch, yaw],
        "scale": [x_scale, y_scale, z_scale]
      }
    ],
    "lighting": {
      "ambient": 0.5,
      "directional_lights": [
        {
          "direction": [x, y, z],
          "intensity": 0.8,
          "color": [r, g, b]
        }
      ]
    }
  },
  "robots": [
    {
      "id": "robot1",
      "name": "My Robot",
      "model": "differential_drive",
      "max_linear_velocity": 1.0,
      "max_angular_velocity": 1.0,
      "battery_capacity": 100.0
    }
  ]
}
```

### Obstacle Shapes

- `box`: Rectangular box
- `sphere`: Perfect sphere
- `cylinder`: Cylindrical shape
- `cone`: Conical shape

### Robot Models

- `differential_drive`: Two-wheeled mobile robot
- `ackermann`: Car-like steering
- `legged`: Multi-legged robot
- `aerial`: Drone/quadcopter
- `marine`: Water-based vehicle

### Ground Materials

- `concrete`: Hard, smooth surface
- `grass`: Outdoor terrain
- `rubber`: High traction surface
- `wood`: Wooden flooring
- `metal`: Metallic surface

---

## Tips

1. **Start Simple**: Begin with the warehouse world to familiarize yourself with controls
2. **Test Navigation**: Use obstacle course for algorithm validation
3. **Scale Appropriately**: Keep obstacle scales realistic (1 unit = 1 meter)
4. **Lighting Matters**: Adjust ambient and directional light for visualization
5. **Battery Management**: Set appropriate battery capacity for your use case

---

## API Integration

These worlds can also be loaded via the web API:

```bash
curl -X POST http://localhost:3030/api/load_world \
  -H "Content-Type: application/json" \
  -d @examples/warehouse_world.json
```

---

## Contributing

Feel free to create and share your own world configurations!
Just add a new `.json` file to this directory following the structure above.

Happy Simulating! 🤖✨

