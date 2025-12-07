# 🤖 ROS 2 Integration - Complete Guide

## 🌟 Overview

NAVΛ Studio IDE now includes **complete ROS 2 integration** with:
- 🦀 **Kobuki/TurtleBot2** bringup packages
- 🤖 **Helix Figure3** humanoid robot skeleton
- 📡 **gRPC Path Emitter** (C++) with custom messages
- 🐍 **Python Drive Bridge** to Univarm backend
- 🎯 **Curvature-based path following**
- 📊 **Custom ROS 2 messages** for Univarm

---

## 📦 ROS 2 Packages

### 1. **univarm_kobuki_bringup** 🦀
- Kobuki (TurtleBot2 base) URDF & launch files
- Differential drive controller configuration
- C++ path follower with curvature tracking
- RViz visualization support

### 2. **univarm_tb2_bringup** 🤖
- TurtleBot2 complete platform (Kobuki + sensors)
- Same controller architecture as Kobuki
- Extended URDF with camera and LiDAR
- Drop-in replacement for Kobuki bringup

### 3. **helix_figure3** 🦾
- Helix humanoid robot URDF skeleton
- Joint groups and controller config
- IK/pose bridge stub for integration
- FollowJointTrajectory action server

### 4. **univarm_msgs** 📨
Custom ROS 2 message types:
- `IntentConstraint.msg` - Planning constraints
- `Metrics.msg` - Performance metrics
- `PathWithMeta.msg` - Path with metadata
- `SolvePath.srv` - Path planning service

### 5. **univarm_path_emitter_cpp** 📡
- C++ node with gRPC client
- Calls Drive.SolvePath on your planner
- Publishes `nav_msgs/Path` and `PathWithMeta`
- Service interface: `/univarm/solve_path`

### 6. **univarm_drive_bridge_py** 🐍
- Python bridge to Univarm backend
- Calls `/api/solve` and `/api/drive/path`
- Publishes to ROS 2 topics
- Subscribes to backend SSE stream

---

## 🚀 Quick Start

### Prerequisites

**System Requirements**:
- ROS 2 Humble or newer
- Protobuf compiler and gRPC libraries
- Python 3.8+
- colcon build tools

**Install Dependencies** (Ubuntu/Debian):
```bash
# ROS 2 (if not installed)
# Follow: https://docs.ros.org/en/humble/Installation.html

# Build tools
sudo apt-get install -y python3-colcon-common-extensions

# gRPC & Protobuf
sudo apt-get install -y protobuf-compiler libprotobuf-dev \
  libgrpc++-dev protobuf-compiler-grpc

# rosdep
sudo apt-get install python3-rosdep
sudo rosdep init  # If not done before
rosdep update
```

### Build ROS 2 Workspace

```bash
cd "/Users/frankvanlaarhoven/Downloads/NAVΛ STUDIO IDE"
./build-ros2.sh
```

This will:
1. ✅ Detect your ROS 2 distro
2. ✅ Source ROS 2 environment
3. ✅ Install dependencies with rosdep
4. ✅ Build all 6 packages with colcon
5. ✅ Generate gRPC stubs from proto files

---

## 🎮 Usage Scenarios

### Scenario 1: Kobuki Robot with RViz

**What it does**: Launches Kobuki robot simulation with visualization

```bash
# Terminal 1: Start Univarm backend
cd "/Users/frankvanlaarhoven/Downloads/NAVΛ STUDIO IDE"
./start-univarm-backend.sh

# Terminal 2: Launch Kobuki + RViz
./start-kobuki-demo.sh
```

**Or manually**:
```bash
cd ros2_ws
source install/setup.bash
ros2 launch univarm_kobuki_bringup bringup.launch.py use_rviz:=true
```

**What you'll see**:
- RViz opens with Kobuki robot model
- Robot appears at origin
- TF tree published (`odom` → `base_link`)
- Ready to receive path commands

---

### Scenario 2: Drive Bridge (Backend → ROS 2)

**What it does**: Bridges Univarm backend API to ROS 2 topics

```bash
# Terminal 1: Univarm backend
./start-univarm-backend.sh

# Terminal 2: ROS 2 workspace
cd ros2_ws
source install/setup.bash

# Terminal 3: Drive bridge
ros2 run univarm_drive_bridge_py drive_bridge \
  --ros-args -p backend_url:=http://localhost:8080
```

**Topics published**:
- `/univarm/planned_path` (`nav_msgs/Path`)
- `/cmd_vel` (`geometry_msgs/Twist`)

**Backend calls**:
- `POST /api/solve` - Get path from backend
- `POST /api/drive/path` - Execute path
- `GET /api/drive/sse` - Subscribe to telemetry

---

### Scenario 3: gRPC Path Planning Service

**What it does**: ROS 2 service that calls gRPC planner

```bash
# Terminal 1: Start your gRPC planner
# (Must implement Drive.SolvePath from proto/univarm/drive.proto)
# Should run on 127.0.0.1:50051

# Terminal 2: Source workspace
cd ros2_ws
source install/setup.bash

# Terminal 3: Start path emitter
ros2 run univarm_path_emitter_cpp path_emitter \
  --ros-args -p planner_addr:=127.0.0.1:50051

# Terminal 4: Call the service
ros2 service call /univarm/solve_path univarm_msgs/srv/SolvePath \
  "{start: {header:{frame_id: map}, pose:{position:{x:0.0, y:0.0, z:0.0}}}, \
    goal: {header:{frame_id: map}, pose:{position:{x:2.0, y:0.0, z:0.0}}}, \
    objectives: ['shortest']}"
```

**Response**:
```yaml
ok: true
path:
  header:
    frame_id: "map"
  poses:
    - pose:
        position: {x: 0.0, y: 0.0, z: 0.0}
    - pose:
        position: {x: 1.0, y: 0.0, z: 0.0}
    - pose:
        position: {x: 2.0, y: 0.0, z: 0.0}
meta:
  cost: 2.0
  samples: 48
  solver: "grpc"
```

**Topics published**:
- `/univarm/drive/path` (`nav_msgs/Path`)
- `/univarm/drive/path_meta` (`univarm_msgs/PathWithMeta`)

---

### Scenario 4: TurtleBot2 Full Platform

```bash
cd ros2_ws
source install/setup.bash
ros2 launch univarm_tb2_bringup bringup.launch.py use_rviz:=true
```

Same as Kobuki but includes:
- Camera sensors
- LiDAR (if configured)
- Extended URDF

---

### Scenario 5: Helix Humanoid

```bash
cd ros2_ws
source install/setup.bash
ros2 launch helix_figure3 helix_bringup.launch.py use_rviz:=true
```

**Features**:
- Humanoid robot URDF
- Joint state publisher
- Controller manager
- Ready for pose/IK integration

---

## 🔧 Package Details

### univarm_kobuki_bringup

**Path**: `ros2_ws/src/univarm_kobuki_bringup/`

**Files**:
- `urdf/kobuki.urdf.xacro` - Robot description
- `config/controllers.yaml` - Diff-drive controller config
- `launch/bringup.launch.py` - Launch file
- `src/path_follower.cpp` - C++ path following node

**Nodes**:
- `path_follower` - Subscribes to `/planned_path`, publishes `/cmd_vel`

**Topics**:
- Subscribe: `/planned_path` (`nav_msgs/Path`)
- Publish: `/cmd_vel` (`geometry_msgs/Twist`)
- Publish: `/odom` (`nav_msgs/Odometry`)

**Parameters**:
- `wheelbase` - Wheel separation (default: 0.23m)
- `max_linear_vel` - Max speed (default: 0.7 m/s)
- `max_angular_vel` - Max rotation (default: 2.84 rad/s)

---

### univarm_drive_bridge_py

**Path**: `ros2_ws/src/univarm_drive_bridge_py/`

**Files**:
- `univarm_drive_bridge_py/drive_bridge.py` - Main bridge node

**Parameters**:
- `backend_url` - Univarm backend (default: http://localhost:8080)
- `sse_token` - Authentication token (default: demo-token-123)

**Workflow**:
1. Calls `/api/solve` with start/goal
2. Publishes path to `/univarm/planned_path`
3. Calls `/api/drive/path` to execute
4. Subscribes to `/api/drive/sse` for telemetry
5. Publishes `/cmd_vel` from SSE updates

---

### univarm_msgs

**Path**: `ros2_ws/src/univarm_msgs/`

**Messages**:

**IntentConstraint.msg**:
```
string constraint_type
float64 value
float64 weight
string[] affected_joints
```

**Metrics.msg**:
```
float64 cost
float64 constraint_satisfaction
int32 samples
string solver_type
duration planning_time
```

**PathWithMeta.msg**:
```
nav_msgs/Path path
Metrics metrics
IntentConstraint[] constraints
```

**Services**:

**SolvePath.srv**:
```
# Request
geometry_msgs/PoseStamped start
geometry_msgs/PoseStamped goal
string[] objectives

---
# Response
bool ok
nav_msgs/Path path
PathWithMeta meta
```

---

### univarm_path_emitter_cpp

**Path**: `ros2_ws/src/univarm_path_emitter_cpp/`

**Files**:
- `src/path_emitter.cpp` - C++ node with gRPC client
- `CMakeLists.txt` - Builds with protobuf/gRPC

**Service Offered**:
- `/univarm/solve_path` (`univarm_msgs/srv/SolvePath`)

**Topics Published**:
- `/univarm/drive/path` (`nav_msgs/Path`)
- `/univarm/drive/path_meta` (`univarm_msgs/PathWithMeta`)

**gRPC Integration**:
Calls `Drive.SolvePath` defined in `proto/univarm/drive.proto`:
```protobuf
service Drive {
  rpc SolvePath(SolveRequest) returns (SolveResponse);
}
```

**CMake automatically**:
- Generates C++ gRPC stubs at build time
- Links protobuf and gRPC libraries
- Includes generated headers

---

## 🔌 Integration Architecture

```
┌─────────────────────────────────────────────────────┐
│  NAVΛ Studio IDE                                    │
│  ├── Univarm Advanced Frontend                     │
│  └── Univarm Backend (Rust/Actix) :8080            │
└─────────────────────────────────────────────────────┘
                      ↓ HTTP/SSE
┌─────────────────────────────────────────────────────┐
│  univarm_drive_bridge_py                           │
│  ├── Calls /api/solve                              │
│  ├── Subscribes to /api/drive/sse                  │
│  └── Publishes to ROS 2 topics                     │
└─────────────────────────────────────────────────────┘
                      ↓ ROS 2 Topics
┌─────────────────────────────────────────────────────┐
│  ROS 2 Ecosystem                                    │
│  ├── /planned_path → path_follower → /cmd_vel      │
│  ├── /cmd_vel → diff_drive_controller → wheels     │
│  └── /odom → Published by controller               │
└─────────────────────────────────────────────────────┘
                      ↓
┌─────────────────────────────────────────────────────┐
│  Robot (Simulation or Real)                        │
│  ├── Kobuki/TurtleBot2                            │
│  ├── Helix Figure3                                 │
│  └── Your Custom Robot                             │
└─────────────────────────────────────────────────────┘
```

**Alternative gRPC Path**:
```
Your gRPC Planner :50051
         ↓ gRPC Drive.SolvePath
univarm_path_emitter_cpp
         ↓ ROS 2 Service
ROS 2 Service Client (/univarm/solve_path)
         ↓ nav_msgs/Path
Navigation Stack (nav2, move_base)
```

---

## 🏗️ Build Instructions

### First Time Setup

```bash
cd "/Users/frankvanlaarhoven/Downloads/NAVΛ STUDIO IDE"

# Install ROS 2 if not already installed
# https://docs.ros.org/en/humble/Installation.html

# Install system dependencies
sudo apt-get install -y \
  protobuf-compiler \
  libprotobuf-dev \
  libgrpc++-dev \
  protobuf-compiler-grpc \
  python3-colcon-common-extensions \
  python3-rosdep

# Initialize rosdep (if first time)
sudo rosdep init
rosdep update

# Build workspace
./build-ros2.sh
```

### Rebuild After Changes

```bash
cd ros2_ws
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --packages-select <package_name>
source install/setup.bash
```

---

## 🎯 Launch Files

### Kobuki Bringup

```bash
source ros2_ws/install/setup.bash
ros2 launch univarm_kobuki_bringup bringup.launch.py use_rviz:=true
```

**Parameters**:
- `use_rviz` - Launch RViz (default: false)
- `robot_name` - Namespace (default: kobuki)
- `use_sim_time` - Simulation mode (default: false)

**Nodes Launched**:
- `robot_state_publisher` - Publishes TF tree
- `joint_state_publisher` - Joint states
- `diff_drive_controller` - Velocity control
- `path_follower` - Path following node
- `rviz2` (if use_rviz:=true)

---

### TurtleBot2 Bringup

```bash
source ros2_ws/install/setup.bash
ros2 launch univarm_tb2_bringup bringup.launch.py use_rviz:=true
```

Same parameters and nodes as Kobuki, plus:
- Camera transforms
- LiDAR mount points (if configured)

---

### Helix Figure3 Bringup

```bash
source ros2_ws/install/setup.bash
ros2 launch helix_figure3 helix_bringup.launch.py use_rviz:=true
```

**Nodes Launched**:
- `robot_state_publisher` - Humanoid TF tree
- `joint_state_publisher_gui` - Manual joint control
- `controller_manager` - ros2_control manager
- `trajectory_controller` - Joint trajectory action server

---

## 🔗 Integration Workflows

### Workflow 1: Web UI → ROS 2

**Complete flow from Univarm Advanced to real/simulated robot**:

```bash
# Terminal 1: Start Univarm backend
cd "/Users/frankvanlaarhoven/Downloads/NAVΛ STUDIO IDE"
./start-univarm-backend.sh

# Terminal 2: Start ROS 2 Kobuki
./start-kobuki-demo.sh

# Terminal 3: Start drive bridge
cd ros2_ws
source install/setup.bash
ros2 run univarm_drive_bridge_py drive_bridge \
  --ros-args -p backend_url:=http://localhost:8080

# Terminal 4: Open Web UI
npm run dev:vite
# Click 🦀 in workspace, solve path, click "▶ Execute Drive"
```

**Flow**:
1. User clicks "⋋ Solve Path" in Univarm Advanced
2. Frontend calls `POST /api/solve`
3. Backend returns path
4. User clicks "▶ Execute Drive"
5. Frontend calls `POST /api/drive/path`
6. Backend starts SSE streaming
7. Drive bridge subscribes to SSE
8. Bridge publishes to `/univarm/planned_path`
9. Path follower converts to `/cmd_vel`
10. Diff-drive controller moves robot!

---

### Workflow 2: ROS 2 Service → gRPC Planner

**Use ROS 2 services with external gRPC planner**:

```bash
# Terminal 1: Start your gRPC planner on port 50051
# (Must implement Drive.SolvePath)

# Terminal 2: Start path emitter
cd ros2_ws
source install/setup.bash
ros2 run univarm_path_emitter_cpp path_emitter \
  --ros-args -p planner_addr:=127.0.0.1:50051

# Terminal 3: Call service
ros2 service call /univarm/solve_path univarm_msgs/srv/SolvePath \
  "{start: {header:{frame_id: map}, pose:{position:{x:0.0, y:0.0, z:0.0}}}, \
    goal: {header:{frame_id: map}, pose:{position:{x:2.0, y:0.0, z:0.0}}}, \
    objectives: ['shortest']}"
```

**Flow**:
1. ROS 2 client calls `/univarm/solve_path` service
2. Path emitter receives request
3. Emitter calls gRPC planner via `Drive.SolvePath`
4. Planner returns path
5. Emitter publishes to `/univarm/drive/path` and `/univarm/drive/path_meta`
6. Emitter returns service response
7. Client receives path!

---

### Workflow 3: Full Navigation Stack

**Integrate with nav2 or move_base**:

```bash
# Terminal 1: Launch Kobuki
ros2 launch univarm_kobuki_bringup bringup.launch.py

# Terminal 2: Launch nav2 (or move_base)
ros2 launch nav2_bringup navigation_launch.py

# Terminal 3: Path emitter with nav2 integration
ros2 run univarm_path_emitter_cpp path_emitter \
  --ros-args -p planner_addr:=127.0.0.1:50051 \
               -p publish_to_nav2:=true
```

The emitter can feed paths directly into nav2's path following.

---

## 📡 gRPC Protocol

### Drive.proto

```protobuf
syntax = "proto3";
package univarm;

service Drive {
  rpc SolvePath(SolveRequest) returns (SolveResponse);
}

message Point2D {
  double x = 1;
  double y = 2;
}

message Pose {
  double x = 1;
  double y = 2;
  double theta = 3;
}

message SolveRequest {
  repeated Point2D start = 1;
  repeated Point2D goal = 2;
  int32 samples = 3;
  repeated string objectives = 4;
}

message SolveResponse {
  bool ok = 1;
  repeated Pose path = 2;
  double cost = 3;
  int32 samples = 4;
}
```

**Location**: `ros2_ws/proto/univarm/drive.proto`

---

## 🔧 Customization

### Replace URDF Models

**Kobuki**:
```bash
# Edit: ros2_ws/src/univarm_kobuki_bringup/urdf/kobuki.urdf.xacro
# Add your meshes, sensors, etc.
```

**Helix**:
```bash
# Edit: ros2_ws/src/helix_figure3/urdf/helix_figure3.urdf.xacro
# Replace with your humanoid model
```

### Adjust Controller Parameters

**Kobuki/TB2**:
```yaml
# ros2_ws/src/univarm_kobuki_bringup/config/controllers.yaml
diff_drive_controller:
  ros__parameters:
    left_wheel_names: ["wheel_left_joint"]
    right_wheel_names: ["wheel_right_joint"]
    
    wheel_separation: 0.230  # Adjust for your robot
    wheel_radius: 0.035
    
    # Velocity limits
    linear:
      x:
        max_velocity: 0.7
        max_acceleration: 1.0
    angular:
      z:
        max_velocity: 2.84
        max_acceleration: 2.0
```

### Configure Path Follower

**Edit C++ node**: `src/path_follower.cpp`

```cpp
// Curvature parameters
double lookahead_distance_ = 0.5;  // meters
double k_p_ = 1.0;                 // Proportional gain
double max_linear_vel_ = 0.7;
double max_angular_vel_ = 2.84;
```

---

## 🐛 Troubleshooting

### Build Fails - Missing gRPC

**Error**: `Could not find grpc++`

**Solution**:
```bash
sudo apt-get install -y libgrpc++-dev protobuf-compiler-grpc
```

### Build Fails - Missing ROS 2 Dependencies

**Error**: `Package X not found`

**Solution**:
```bash
cd ros2_ws
rosdep install --from-paths src --ignore-src -y
```

### Service Call Fails

**Error**: `Service /univarm/solve_path not available`

**Solution**:
1. Check emitter is running: `ros2 node list`
2. Check planner is on port 50051: `lsof -i :50051`
3. Restart emitter with correct planner address

### Robot Doesn't Move

**Error**: Kobuki stays at origin

**Solutions**:
1. Check `/planned_path` topic: `ros2 topic echo /planned_path`
2. Check `/cmd_vel` output: `ros2 topic echo /cmd_vel`
3. Verify path_follower is running: `ros2 node list`
4. Check RViz - is robot visible?

### Drive Bridge Not Connecting

**Error**: Bridge can't reach backend

**Solutions**:
1. Verify backend is running: `curl http://localhost:8080/api/solve`
2. Check backend_url parameter is correct
3. Ensure SSE token matches: `demo-token-123`

---

## 📊 Topic Map

### Published Topics

| Topic | Type | Publisher | Description |
|-------|------|-----------|-------------|
| `/univarm/planned_path` | `nav_msgs/Path` | drive_bridge_py | Planned path from backend |
| `/univarm/drive/path` | `nav_msgs/Path` | path_emitter_cpp | Path from gRPC planner |
| `/univarm/drive/path_meta` | `PathWithMeta` | path_emitter_cpp | Path + metadata |
| `/cmd_vel` | `geometry_msgs/Twist` | path_follower | Velocity commands |
| `/odom` | `nav_msgs/Odometry` | diff_drive_controller | Odometry |
| `/joint_states` | `sensor_msgs/JointState` | joint_state_publisher | Joint positions |
| `/tf` | `tf2_msgs/TFMessage` | robot_state_publisher | Transform tree |

### Subscribed Topics

| Topic | Type | Subscriber | Description |
|-------|------|------------|-------------|
| `/planned_path` | `nav_msgs/Path` | path_follower | Path to follow |
| `/cmd_vel` | `geometry_msgs/Twist` | diff_drive_controller | Velocity control |

---

## 🚀 Advanced Features

### Curvature-Based Path Following

**Reference implementation**: `backend/rust/src/follower/curvature.rs`

Key features:
- Curvature calculation from path segments
- Dynamic velocity limits based on curvature
- Smooth velocity transitions
- Constraint satisfaction monitoring

**To use in ROS 2**:
- Already implemented in `path_follower.cpp`
- Adjust `lookahead_distance` parameter
- Tune `k_p` gain for your robot

### Multi-Robot Support

**Launch multiple robots**:
```bash
# Robot 1
ros2 launch univarm_kobuki_bringup bringup.launch.py robot_name:=kobuki1

# Robot 2
ros2 launch univarm_kobuki_bringup bringup.launch.py robot_name:=kobuki2
```

Each robot gets its own namespace and topics.

---

## 📚 Documentation Files

### In This Repo
- **[ROS2_INTEGRATION_COMPLETE.md](./ROS2_INTEGRATION_COMPLETE.md)** - This file
- **[Kobuki README](./ros2_ws/src/univarm_kobuki_bringup/README.md)**
- **[Helix README](./ros2_ws/src/helix_figure3/README.md)**
- **[Messages README](./ros2_ws/src/univarm_msgs/)**
- **[Path Emitter README](./ros2_ws/src/univarm_path_emitter_cpp/)**

### Related Docs
- [Univarm Advanced Integration](./UNIVARM_ADVANCED_INTEGRATION.md)
- [Univarm Complete Architecture](./UNIVARM_COMPLETE_ARCHITECTURE.md)
- [ROBOTIS Integration](./ROBOTIS_INTEGRATION_COMPLETE.md)

---

## ✅ Integration Checklist

- [x] ✅ ROS 2 workspace created (`ros2_ws/`)
- [x] ✅ Kobuki bringup package integrated
- [x] ✅ TurtleBot2 bringup package integrated
- [x] ✅ Helix Figure3 humanoid integrated
- [x] ✅ Custom messages package (univarm_msgs)
- [x] ✅ gRPC path emitter (C++)
- [x] ✅ Python drive bridge
- [x] ✅ Build script (`build-ros2.sh`)
- [x] ✅ Launch scripts (`start-kobuki-demo.sh`, `start-ros2-emitter.sh`)
- [x] ✅ Proto files for gRPC
- [x] ✅ Complete documentation

---

## 🎯 Quick Commands Reference

```bash
# Build everything
./build-ros2.sh

# Launch Kobuki with RViz
./start-kobuki-demo.sh

# Start path emitter (gRPC)
./start-ros2-emitter.sh

# Manual launches
cd ros2_ws && source install/setup.bash

# Kobuki
ros2 launch univarm_kobuki_bringup bringup.launch.py use_rviz:=true

# TB2
ros2 launch univarm_tb2_bringup bringup.launch.py use_rviz:=true

# Helix
ros2 launch helix_figure3 helix_bringup.launch.py use_rviz:=true

# Drive bridge
ros2 run univarm_drive_bridge_py drive_bridge \
  --ros-args -p backend_url:=http://localhost:8080

# Path emitter
ros2 run univarm_path_emitter_cpp path_emitter \
  --ros-args -p planner_addr:=127.0.0.1:50051
```

---

## 🌟 What's Next?

### Immediate Enhancements
1. **Add your WASM engine** to backend
2. **Customize URDF** with your robot models
3. **Configure sensors** (LiDAR, camera, IMU)
4. **Tune controller parameters** for your hardware

### Future Features
- SLAM integration (gmapping, cartographer)
- Sensor fusion with GTSAM
- Multi-robot coordination
- Cloud deployment with K8s

---

**🎊 ROS 2 integration complete! Launch Kobuki now: `./start-kobuki-demo.sh`** 🤖🚀

