# Real-Time Simulation Engine - Technical Specification

## Overview
The Real-Time Simulation Engine is the core component responsible for photorealistic rendering, physics simulation, and sensor modeling. It integrates CARLA, NVIDIA Isaac Sim, and Gazebo to provide a comprehensive simulation environment for autonomous vehicles and robotics.

## Architecture

### High-Level Design
```
┌─────────────────────────────────────────────────────────────────┐
│                   Simulation Engine Layer                        │
├─────────────────────────────────────────────────────────────────┤
│                                                                   │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐          │
│  │    CARLA     │  │  Isaac Sim   │  │   Gazebo     │          │
│  │   Adapter    │  │   Adapter    │  │   Adapter    │          │
│  └──────┬───────┘  └──────┬───────┘  └──────┬───────┘          │
│         │                  │                  │                   │
│         └──────────────────┴──────────────────┘                   │
│                            │                                      │
│                   ┌────────▼────────┐                            │
│                   │  Sim Manager    │                            │
│                   │  - Orchestrate  │                            │
│                   │  - Synchronize  │                            │
│                   │  - Control      │                            │
│                   └────────┬────────┘                            │
│                            │                                      │
│         ┌──────────────────┼──────────────────┐                  │
│         │                  │                  │                   │
│  ┌──────▼───────┐  ┌──────▼───────┐  ┌──────▼───────┐          │
│  │   Physics    │  │   Sensor     │  │   Renderer   │          │
│  │   Engine     │  │   Simulator  │  │              │          │
│  └──────────────┘  └──────────────┘  └──────────────┘          │
│                                                                   │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐          │
│  │  Digital     │  │   Scene      │  │   Vehicle    │          │
│  │   Twin       │  │   Manager    │  │   Dynamics   │          │
│  └──────────────┘  └──────────────┘  └──────────────┘          │
│                                                                   │
└─────────────────────────────────────────────────────────────────┘
```

## Simulation Engines

### 1. CARLA (Autonomous Driving)
**Version**: 0.9.15+
**Purpose**: Urban driving scenarios, traffic simulation
**Capabilities**:
- Photorealistic rendering with Unreal Engine 4
- Traffic simulation with pedestrians and vehicles
- Weather and lighting conditions
- Multiple sensor types (cameras, LiDAR, RADAR, GNSS, IMU)

**Implementation**:
```rust
use carla_client::{Client, World, Actor, Sensor};

pub struct CARLAAdapter {
    client: Arc<Client>,
    world: Arc<RwLock<World>>,
    actors: HashMap<ActorId, Actor>,
    sensors: HashMap<SensorId, Sensor>,
}

impl CARLAAdapter {
    pub async fn initialize(&mut self, host: &str, port: u16) -> Result<(), Error> {
        self.client = Arc::new(Client::connect(host, port).await?);
        self.world = Arc::new(RwLock::new(self.client.get_world().await?));
        
        // Load default map
        self.load_map("Town01").await?;
        
        Ok(())
    }
    
    pub async fn spawn_vehicle(&mut self, blueprint: &str, transform: Transform) -> Result<ActorId, Error> {
        let world = self.world.read();
        let bp_library = world.get_blueprint_library();
        let vehicle_bp = bp_library.find(blueprint)?;
        
        let actor = world.spawn_actor(vehicle_bp, transform).await?;
        let actor_id = actor.id();
        
        self.actors.insert(actor_id, actor);
        Ok(actor_id)
    }
    
    pub async fn attach_sensor(&mut self, 
                                 sensor_type: SensorType, 
                                 parent_id: ActorId,
                                 transform: Transform) -> Result<SensorId, Error> {
        let world = self.world.read();
        let bp_library = world.get_blueprint_library();
        
        let sensor_bp = match sensor_type {
            SensorType::Camera => bp_library.find("sensor.camera.rgb")?,
            SensorType::LiDAR => bp_library.find("sensor.lidar.ray_cast")?,
            SensorType::RADAR => bp_library.find("sensor.other.radar")?,
            SensorType::IMU => bp_library.find("sensor.other.imu")?,
        };
        
        let parent = self.actors.get(&parent_id).ok_or(Error::ActorNotFound)?;
        let sensor = world.spawn_actor(sensor_bp, transform)
            .attach_to(parent)
            .await?;
        
        let sensor_id = sensor.id();
        self.sensors.insert(sensor_id, sensor);
        
        Ok(sensor_id)
    }
    
    pub async fn step_simulation(&mut self, delta_time: f64) -> Result<(), Error> {
        let mut world = self.world.write();
        world.tick(delta_time).await?;
        Ok(())
    }
    
    pub async fn set_weather(&mut self, weather: WeatherParameters) -> Result<(), Error> {
        let mut world = self.world.write();
        world.set_weather(weather).await?;
        Ok(())
    }
}

pub struct WeatherParameters {
    pub cloudiness: f32,        // 0-100
    pub precipitation: f32,     // 0-100
    pub sun_altitude_angle: f32, // -90 to 90
    pub fog_density: f32,       // 0-100
}
```

### 2. NVIDIA Isaac Sim (Robotics)
**Version**: 2023.1+
**Purpose**: Robotic manipulation, warehouse automation
**Capabilities**:
- Photorealistic ray-traced rendering (RTX)
- Accurate physics simulation (PhysX 5)
- ROS/ROS2 integration
- Synthetic data generation for AI training

**Implementation**:
```python
from omni.isaac.kit import SimulationApp
from omni.isaac.core import World
from omni.isaac.core.robots import Robot
from omni.isaac.core.utils.stage import add_reference_to_stage

class IsaacSimAdapter:
    def __init__(self):
        self.simulation_app = SimulationApp({"headless": False})
        self.world = World()
        self.robots = {}
        self.sensors = {}
    
    async def initialize(self):
        """Initialize Isaac Sim environment"""
        await self.world.initialize_simulation_context_async()
        self.world.scene.add_default_ground_plane()
        
    async def load_robot(self, robot_type: str, position: tuple) -> str:
        """Load a robot into the scene"""
        if robot_type == "franka":
            robot_path = "/Isaac/Robots/Franka/franka.usd"
        elif robot_type == "ur5":
            robot_path = "/Isaac/Robots/UniversalRobots/ur5e/ur5e.usd"
        else:
            raise ValueError(f"Unknown robot type: {robot_type}")
        
        robot_prim = add_reference_to_stage(robot_path, f"/World/{robot_type}")
        robot = Robot(prim_path=f"/World/{robot_type}")
        robot.set_world_pose(position=position)
        
        robot_id = f"{robot_type}_{len(self.robots)}"
        self.robots[robot_id] = robot
        
        return robot_id
    
    async def attach_camera(self, parent_id: str, resolution: tuple) -> str:
        """Attach a camera sensor to a robot"""
        from omni.isaac.sensor import Camera
        
        camera = Camera(
            prim_path=f"/World/{parent_id}/camera",
            resolution=resolution,
            frequency=30
        )
        
        camera_id = f"camera_{len(self.sensors)}"
        self.sensors[camera_id] = camera
        
        return camera_id
    
    async def step_simulation(self, delta_time: float):
        """Step the simulation forward"""
        await self.world.step_async(render=True)
    
    async def get_sensor_data(self, sensor_id: str):
        """Retrieve data from a sensor"""
        sensor = self.sensors.get(sensor_id)
        if sensor is None:
            raise ValueError(f"Sensor not found: {sensor_id}")
        
        return sensor.get_current_frame()
    
    def shutdown(self):
        """Clean shutdown of Isaac Sim"""
        self.simulation_app.close()
```

### 3. Gazebo (ROS Integration)
**Version**: Gazebo 11 / Ignition Gazebo
**Purpose**: ROS-based robotics simulation
**Capabilities**:
- Native ROS/ROS2 integration
- Plugin system for custom sensors and actuators
- Multi-robot simulation
- Distributed simulation support

**Implementation**:
```rust
use gazebo_client::{GazeboClient, Model, Plugin};

pub struct GazeboAdapter {
    client: Arc<GazeboClient>,
    models: HashMap<String, Model>,
    world_name: String,
}

impl GazeboAdapter {
    pub async fn initialize(&mut self, world_file: &str) -> Result<(), Error> {
        self.client = Arc::new(GazeboClient::connect().await?);
        self.client.load_world(world_file).await?;
        self.world_name = self.client.get_world_name().await?;
        
        Ok(())
    }
    
    pub async fn spawn_model(&mut self, 
                              model_name: &str, 
                              sdf_file: &str,
                              pose: Pose) -> Result<(), Error> {
        let sdf_content = std::fs::read_to_string(sdf_file)?;
        let model = self.client.spawn_model(model_name, &sdf_content, pose).await?;
        
        self.models.insert(model_name.to_string(), model);
        Ok(())
    }
    
    pub async fn apply_force(&mut self, 
                              model_name: &str, 
                              link_name: &str,
                              force: Vector3) -> Result<(), Error> {
        let model = self.models.get(model_name)
            .ok_or(Error::ModelNotFound)?;
        
        model.apply_force(link_name, force).await?;
        Ok(())
    }
    
    pub async fn get_model_state(&self, model_name: &str) -> Result<ModelState, Error> {
        let model = self.models.get(model_name)
            .ok_or(Error::ModelNotFound)?;
        
        model.get_state().await
    }
    
    pub async fn pause_simulation(&mut self) -> Result<(), Error> {
        self.client.pause().await
    }
    
    pub async fn resume_simulation(&mut self) -> Result<(), Error> {
        self.client.unpause().await
    }
}
```

## Physics Engine

### Custom Physics Implementation
```rust
use rapier3d::prelude::*;

pub struct PhysicsEngine {
    rigid_body_set: RigidBodySet,
    collider_set: ColliderSet,
    integration_parameters: IntegrationParameters,
    physics_pipeline: PhysicsPipeline,
    island_manager: IslandManager,
    broad_phase: BroadPhase,
    narrow_phase: NarrowPhase,
    impulse_joint_set: ImpulseJointSet,
    multibody_joint_set: MultibodyJointSet,
    ccd_solver: CCDSolver,
}

impl PhysicsEngine {
    pub fn new() -> Self {
        let mut integration_parameters = IntegrationParameters::default();
        integration_parameters.dt = 1.0 / 60.0; // 60 Hz
        
        PhysicsEngine {
            rigid_body_set: RigidBodySet::new(),
            collider_set: ColliderSet::new(),
            integration_parameters,
            physics_pipeline: PhysicsPipeline::new(),
            island_manager: IslandManager::new(),
            broad_phase: BroadPhase::new(),
            narrow_phase: NarrowPhase::new(),
            impulse_joint_set: ImpulseJointSet::new(),
            multibody_joint_set: MultibodyJointSet::new(),
            ccd_solver: CCDSolver::new(),
        }
    }
    
    pub fn add_rigid_body(&mut self, 
                           position: Vector3<f32>,
                           mass: f32,
                           shape: ColliderShape) -> RigidBodyHandle {
        let rigid_body = RigidBodyBuilder::dynamic()
            .translation(vector![position.x, position.y, position.z])
            .build();
        
        let handle = self.rigid_body_set.insert(rigid_body);
        
        let collider = ColliderBuilder::new(shape)
            .density(mass)
            .build();
        
        self.collider_set.insert_with_parent(collider, handle, &mut self.rigid_body_set);
        
        handle
    }
    
    pub fn step(&mut self, delta_time: f32) {
        self.integration_parameters.dt = delta_time;
        
        self.physics_pipeline.step(
            &vector![0.0, -9.81, 0.0], // Gravity
            &self.integration_parameters,
            &mut self.island_manager,
            &mut self.broad_phase,
            &mut self.narrow_phase,
            &mut self.rigid_body_set,
            &mut self.collider_set,
            &mut self.impulse_joint_set,
            &mut self.multibody_joint_set,
            &mut self.ccd_solver,
            None,
            &(),
            &(),
        );
    }
    
    pub fn get_body_transform(&self, handle: RigidBodyHandle) -> Option<Isometry3<f32>> {
        self.rigid_body_set.get(handle).map(|body| *body.position())
    }
}
```

## Sensor Simulation

### LiDAR Simulation
```rust
pub struct LiDARSimulator {
    range: f32,
    horizontal_fov: f32,
    vertical_fov: f32,
    horizontal_resolution: usize,
    vertical_resolution: usize,
    noise_model: NoiseModel,
}

impl LiDARSimulator {
    pub fn simulate(&self, scene: &Scene, pose: &Pose) -> PointCloud {
        let mut points = Vec::new();
        
        let h_step = self.horizontal_fov / self.horizontal_resolution as f32;
        let v_step = self.vertical_fov / self.vertical_resolution as f32;
        
        for v in 0..self.vertical_resolution {
            let vertical_angle = -self.vertical_fov / 2.0 + v as f32 * v_step;
            
            for h in 0..self.horizontal_resolution {
                let horizontal_angle = -self.horizontal_fov / 2.0 + h as f32 * h_step;
                
                let direction = self.compute_ray_direction(horizontal_angle, vertical_angle);
                let ray = Ray::new(pose.position, direction);
                
                if let Some(hit) = scene.raycast(&ray, self.range) {
                    let distance = hit.distance + self.noise_model.sample();
                    let point = pose.position + direction * distance;
                    points.push(point);
                }
            }
        }
        
        PointCloud { points }
    }
    
    fn compute_ray_direction(&self, h_angle: f32, v_angle: f32) -> Vector3<f32> {
        let x = h_angle.cos() * v_angle.cos();
        let y = h_angle.sin() * v_angle.cos();
        let z = v_angle.sin();
        
        Vector3::new(x, y, z).normalize()
    }
}
```

### Camera Simulation
```rust
pub struct CameraSimulator {
    width: u32,
    height: u32,
    fov: f32,
    near_clip: f32,
    far_clip: f32,
    distortion_model: DistortionModel,
}

impl CameraSimulator {
    pub fn render(&self, scene: &Scene, pose: &Pose) -> Image {
        let projection_matrix = self.compute_projection_matrix();
        let view_matrix = self.compute_view_matrix(pose);
        
        let mut image = Image::new(self.width, self.height);
        
        // Render scene using ray tracing or rasterization
        for y in 0..self.height {
            for x in 0..self.width {
                let ray = self.compute_camera_ray(x, y, &projection_matrix, &view_matrix);
                let color = scene.trace_ray(&ray);
                image.set_pixel(x, y, color);
            }
        }
        
        // Apply distortion
        self.distortion_model.apply(&mut image);
        
        image
    }
    
    fn compute_projection_matrix(&self) -> Matrix4<f32> {
        let aspect = self.width as f32 / self.height as f32;
        Matrix4::new_perspective(aspect, self.fov, self.near_clip, self.far_clip)
    }
}
```

## Digital Twin Integration

### Bidirectional Synchronization
```rust
pub struct DigitalTwin {
    virtual_state: Arc<RwLock<RobotState>>,
    physical_state: Arc<RwLock<RobotState>>,
    sync_interval: Duration,
    sync_thread: Option<JoinHandle<()>>,
}

impl DigitalTwin {
    pub fn start_synchronization(&mut self) {
        let virtual_state = Arc::clone(&self.virtual_state);
        let physical_state = Arc::clone(&self.physical_state);
        let interval = self.sync_interval;
        
        self.sync_thread = Some(tokio::spawn(async move {
            let mut ticker = tokio::time::interval(interval);
            
            loop {
                ticker.tick().await;
                
                // Sync virtual -> physical
                let v_state = virtual_state.read();
                let mut p_state = physical_state.write();
                
                // Apply state from virtual to physical
                p_state.joint_positions = v_state.joint_positions.clone();
                p_state.joint_velocities = v_state.joint_velocities.clone();
                
                // Sync physical -> virtual (sensor feedback)
                v_state.sensor_data = p_state.sensor_data.clone();
            }
        }));
    }
    
    pub fn stop_synchronization(&mut self) {
        if let Some(handle) = self.sync_thread.take() {
            handle.abort();
        }
    }
}
```

## Performance Optimization

### Multi-Threading
```rust
pub struct SimulationManager {
    thread_pool: ThreadPool,
    simulation_tasks: Vec<SimulationTask>,
}

impl SimulationManager {
    pub fn run_parallel_simulations(&self, scenarios: Vec<Scenario>) -> Vec<SimulationResult> {
        scenarios.par_iter()
            .map(|scenario| self.run_simulation(scenario))
            .collect()
    }
}
```

### GPU Acceleration
```rust
use wgpu::{Device, Queue, Buffer};

pub struct GPUAcceleratedRenderer {
    device: Device,
    queue: Queue,
    render_pipeline: RenderPipeline,
}

impl GPUAcceleratedRenderer {
    pub fn render_frame(&self, scene: &Scene) -> Image {
        // Use GPU for parallel rendering
        // Leverage compute shaders for physics calculations
        // Implement ray tracing on GPU
    }
}
```

## Performance Requirements

### Target Metrics
- **Frame Rate**: 60+ FPS
- **Physics Update Rate**: 1000 Hz
- **Sensor Update Rate**: 30 Hz (cameras), 10 Hz (LiDAR)
- **Latency**: <16ms per frame
- **Memory Usage**: <8GB per simulation instance

### Optimization Techniques
- Level of Detail (LOD) for distant objects
- Frustum culling for rendering
- Spatial partitioning (octree/BVH) for collision detection
- Asynchronous sensor simulation
- GPU-accelerated ray tracing

## Testing Strategy

### Unit Tests
- Test each simulation engine adapter
- Validate physics calculations
- Test sensor simulation accuracy

### Integration Tests
- Test multi-engine coordination
- Verify sensor data synchronization
- Test digital twin bidirectional sync

### Performance Tests
- Measure frame rate under load
- Profile CPU/GPU usage
- Test with multiple concurrent simulations

## Deployment

### Hardware Requirements
- **GPU**: NVIDIA RTX 3080 or better (for Isaac Sim)
- **CPU**: 16+ cores for parallel simulation
- **RAM**: 32GB minimum, 64GB recommended
- **Storage**: 500GB SSD for simulation assets

### Software Dependencies
```toml
[dependencies]
carla-client = "0.9.15"
rapier3d = "0.17"
wgpu = "0.18"
tokio = { version = "1.35", features = ["full"] }
rayon = "1.8"
```

## Timeline

### Week 1-3: CARLA Integration
- Set up CARLA server
- Implement adapter
- Test vehicle spawning and sensor attachment

### Week 4-6: Isaac Sim Integration
- Set up Isaac Sim environment
- Implement Python/Rust bridge
- Test robot manipulation scenarios

### Week 7-9: Gazebo Integration
- Set up Gazebo with ROS2
- Implement adapter
- Test multi-robot scenarios

### Week 10-12: Physics and Sensors
- Implement custom physics engine
- Build sensor simulators
- Optimize performance

### Week 13-16: Digital Twin and Testing
- Implement digital twin synchronization
- Comprehensive testing
- Performance optimization
- Documentation

## Success Criteria

- ✅ All three simulation engines integrated
- ✅ Achieve 60+ FPS in standard scenarios
- ✅ Accurate sensor simulation (LiDAR, cameras, IMU)
- ✅ Digital twin synchronization with <10ms latency
- ✅ Support for 10+ concurrent simulation instances

---

**Document Version**: 1.0  
**Last Updated**: 2025  
**Status**: Specification Complete - Ready for Implementation
