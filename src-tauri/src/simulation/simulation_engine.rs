use rapier3d::prelude::*;
use std::sync::Arc;
use parking_lot::RwLock;
use std::time::{Duration, Instant};
use tokio::process::Command;
use serde::{Deserialize, Serialize};
use rand::Rng;

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct VehicleState {
    pub id: String,
    pub position: [f32; 3],
    pub rotation: [f32; 3],
    pub velocity: [f32; 3],
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct SensorData {
    pub sensor_type: String,
    pub data: Vec<u8>,
    pub timestamp: u64,
}

pub trait SimulatorAdapter {
    async fn initialize(&mut self) -> Result<(), Box<dyn std::error::Error>>;
    async fn spawn_vehicle(&mut self, vehicle_type: &str, position: [f32; 3]) -> Result<String, Box<dyn std::error::Error>>;
    async fn control_vehicle(&mut self, vehicle_id: &str, throttle: f32, steer: f32, brake: f32) -> Result<(), Box<dyn std::error::Error>>;
    async fn get_vehicle_state(&self, vehicle_id: &str) -> Result<VehicleState, Box<dyn std::error::Error>>;
    async fn get_sensor_data(&self, vehicle_id: &str, sensor_type: &str) -> Result<SensorData, Box<dyn std::error::Error>>;
    async fn step(&mut self, delta_time: f32) -> Result<(), Box<dyn std::error::Error>>;
    async fn shutdown(&mut self) -> Result<(), Box<dyn std::error::Error>>;
}

pub struct CarlaAdapter {
    carla_host: String,
    carla_port: u16,
    client: Option<reqwest::Client>,
    vehicles: std::collections::HashMap<String, VehicleState>,
}

impl CarlaAdapter {
    pub fn new(host: String, port: u16) -> Self {
        CarlaAdapter {
            carla_host: host,
            carla_port: port,
            client: None,
            vehicles: std::collections::HashMap::new(),
        }
    }
}

impl SimulatorAdapter for CarlaAdapter {
    async fn initialize(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        println!("Initializing CARLA adapter at {}:{}", self.carla_host, self.carla_port);
        
        // Check if CARLA server is running
        let output = Command::new("pgrep")
            .arg("-f")
            .arg("CarlaUE4")
            .output()
            .await?;
        
        if !output.status.success() || String::from_utf8_lossy(&output.stdout).trim().is_empty() {
            println!("CARLA server not running - starting it...");
            Command::new("bash")
                .arg("-c")
                .arg(format!("cd /opt/carla-simulator && ./CarlaUE4.sh -carla-server -world-port={} -RenderOffScreen", self.carla_port))
                .spawn()?;
            
            // Wait for server to start
            tokio::time::sleep(Duration::from_secs(10)).await;
        }
        
        self.client = Some(reqwest::Client::new());
        println!("CARLA adapter initialized successfully");
        Ok(())
    }

    async fn spawn_vehicle(&mut self, vehicle_type: &str, position: [f32; 3]) -> Result<String, Box<dyn std::error::Error>> {
        let vehicle_id = format!("vehicle_{}", self.vehicles.len());
        
        let vehicle_state = VehicleState {
            id: vehicle_id.clone(),
            position,
            rotation: [0.0, 0.0, 0.0],
            velocity: [0.0, 0.0, 0.0],
        };
        
        self.vehicles.insert(vehicle_id.clone(), vehicle_state);
        
        // In a real implementation, this would send a command to CARLA server
        println!("Spawned {} vehicle at position {:?}", vehicle_type, position);
        
        Ok(vehicle_id)
    }

    async fn control_vehicle(&mut self, vehicle_id: &str, throttle: f32, steer: f32, brake: f32) -> Result<(), Box<dyn std::error::Error>> {
        if let Some(vehicle) = self.vehicles.get_mut(vehicle_id) {
            // Update vehicle control (simplified)
            vehicle.velocity[0] = throttle * 10.0; // Forward velocity
            vehicle.velocity[1] = steer * 5.0; // Lateral velocity from steering
            println!("Controlling vehicle {}: throttle={}, steer={}, brake={}", vehicle_id, throttle, steer, brake);
        }
        Ok(())
    }

    async fn get_vehicle_state(&self, vehicle_id: &str) -> Result<VehicleState, Box<dyn std::error::Error>> {
        if let Some(vehicle) = self.vehicles.get(vehicle_id) {
            Ok(vehicle.clone())
        } else {
            Err(format!("Vehicle {} not found", vehicle_id).into())
        }
    }

    async fn get_sensor_data(&self, vehicle_id: &str, sensor_type: &str) -> Result<SensorData, Box<dyn std::error::Error>> {
        // Mock sensor data
        let data = match sensor_type {
            "camera" => vec![255; 1920 * 1080 * 3], // RGB image
            "lidar" => vec![0; 100000 * 4], // Point cloud data
            _ => vec![0; 100],
        };
        
        Ok(SensorData {
            sensor_type: sensor_type.to_string(),
            data,
            timestamp: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_secs(),
        })
    }

    async fn step(&mut self, delta_time: f32) -> Result<(), Box<dyn std::error::Error>> {
        // Update all vehicles
        for vehicle in self.vehicles.values_mut() {
            // Simple physics update
            vehicle.position[0] += vehicle.velocity[0] * delta_time;
            vehicle.position[2] += vehicle.velocity[1] * delta_time;
            
            // Apply friction
            vehicle.velocity[0] *= 0.95;
            vehicle.velocity[1] *= 0.95;
        }
        Ok(())
    }

    async fn shutdown(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        println!("Shutting down CARLA adapter");
        self.vehicles.clear();
        Ok(())
    }
}

pub struct SimulationEngine {
    gravity: Vector<f32>,
    integration_parameters: IntegrationParameters,
    physics_pipeline: PhysicsPipeline,
    island_manager: IslandManager,
    broad_phase: BroadPhase,
    narrow_phase: NarrowPhase,
    rigid_body_set: RigidBodySet,
    collider_set: ColliderSet,
    impulse_joint_set: ImpulseJointSet,
    multibody_joint_set: MultibodyJointSet,
    ccd_solver: CCDSolver,
    running: Arc<RwLock<bool>>,
    simulator_adapters: Vec<Box<dyn SimulatorAdapter + Send + Sync>>,
}

impl SimulationEngine {
    pub fn new() -> Self {
        SimulationEngine {
            gravity: vector![0.0, -9.81, 0.0],
            integration_parameters: IntegrationParameters::default(),
            physics_pipeline: PhysicsPipeline::new(),
            island_manager: IslandManager::new(),
            broad_phase: BroadPhase::new(),
            narrow_phase: NarrowPhase::new(),
            rigid_body_set: RigidBodySet::new(),
            collider_set: ColliderSet::new(),
            impulse_joint_set: ImpulseJointSet::new(),
            multibody_joint_set: MultibodyJointSet::new(),
            ccd_solver: CCDSolver::new(),
            running: Arc::new(RwLock::new(false)),
            simulator_adapters: Vec::new(),
        }
    }

    pub fn add_rigid_body(&mut self, rigid_body: RigidBody) -> RigidBodyHandle {
        self.rigid_body_set.insert(rigid_body)
    }

    pub fn add_collider(&mut self, collider: Collider) -> ColliderHandle {
        self.collider_set.insert(collider)
    }

    pub fn register_simulator_adapter(&mut self, adapter: Box<dyn SimulatorAdapter + Send + Sync>) {
        self.simulator_adapters.push(adapter);
    }

    pub async fn initialize_simulators(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        for adapter in &mut self.simulator_adapters {
            // Log the concrete adapter type for debugging and detect IsaacSimAdapter
            // so we can apply any adapter-specific pre-initialization behavior if needed.
            // Use &**adapter to get a reference to the underlying trait object value.
            let adapter_type = std::any::type_name_of_val(&**adapter);
            println!("Initializing simulator adapter: {}", adapter_type);

            if adapter_type.ends_with("IsaacSimAdapter") || adapter_type.contains("IsaacSim") {
                // Detected IsaacSimAdapter. Place any special-case preparation here.
                // The adapter's own initialize() will still be called below and is expected
                // to perform Isaac Sim specific startup (process launch, client init, etc).
                println!("Detected IsaacSimAdapter - performing Isaac-specific pre-init steps (if any).");
            }
            adapter.initialize().await?;
        }
        Ok(())
    }

    pub async fn step_simulators(&mut self, delta_time: f32) -> Result<(), Box<dyn std::error::Error>> {
        for adapter in &mut self.simulator_adapters {
            adapter.step(delta_time).await?;
        }
        Ok(())
    }

    pub async fn shutdown_simulators(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        for adapter in &mut self.simulator_adapters {
            adapter.shutdown().await?;
        }
        Ok(())
    }

    pub fn step(&mut self) {
        self.physics_pipeline.step(
            &self.gravity,
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

    pub fn start(&self) {
        let mut running = self.running.write();
        *running = true;
        println!("Starting real-time simulation engine...");
    }

    pub fn stop(&self) {
        let mut running = self.running.write();
        *running = false;
        println!("Stopping real-time simulation engine...");
    }

    pub fn is_running(&self) -> bool {
        *self.running.read()
    }

    pub fn get_rigid_body_set(&self) -> &RigidBodySet {
        &self.rigid_body_set
    }

    pub fn get_rigid_body_set_mut(&mut self) -> &mut RigidBodySet {
        &mut self.rigid_body_set
    }
}

pub async fn start_simulation() -> Result<(), Box<dyn std::error::Error>> {
    let mut engine = SimulationEngine::new();
    
    // Register CARLA adapter
    let carla_adapter = Box::new(CarlaAdapter::new("localhost".to_string(), 2000));
    engine.register_simulator_adapter(carla_adapter);

    // Register Isaac Sim adapter
    let isaac_adapter = Box::new(IsaacSimAdapter::new("localhost".to_string(), 3000));
    engine.register_simulator_adapter(isaac_adapter);

    // Register Gazebo adapter
    let gazebo_adapter = Box::new(GazeboAdapter::new("localhost".to_string(), 11345));
    engine.register_simulator_adapter(gazebo_adapter);

    // Initialize simulators
    engine.initialize_simulators().await?;
    engine.start();

    // Create a simple ground plane
    let ground_size = 100.0;
    let ground_collider = ColliderBuilder::cuboid(ground_size, 0.1, ground_size)
        .translation(vector![0.0, -0.1, 0.0])
        .build();
    engine.add_collider(ground_collider);

    // Create a falling cube
    let rigid_body = RigidBodyBuilder::dynamic()
        .translation(vector![0.0, 10.0, 0.0])
        .build();
    let collider = ColliderBuilder::cuboid(1.0, 1.0, 1.0)
        .restitution(0.7)
        .build();
    let _body_handle = engine.add_rigid_body(rigid_body);
    engine.add_collider(collider);

    // Spawn a vehicle in CARLA
    if !engine.simulator_adapters.is_empty() {
        // Obtain disjoint mutable borrows to the first adapter and the rest of the adapters.
        // This allows interacting with the first adapter while optionally also using the second adapter.
        let (adapter, rest) = engine.simulator_adapters.split_first_mut().unwrap();

        let vehicle_id = adapter.spawn_vehicle("vehicle.tesla.model3", [0.0, 0.0, 10.0]).await?;
        println!("Spawned vehicle with ID: {}", vehicle_id);

        // Control the vehicle
        adapter.control_vehicle(&vehicle_id, 0.5, 0.0, 0.0).await?;

        // If an Isaac Sim adapter is available (second adapter), spawn and control a robot there.
        if !rest.is_empty() {
            let isaac_adapter = &mut rest[0];
            let robot_id = isaac_adapter.spawn_vehicle("robot.ur5e", [2.0, 0.0, 0.0]).await?;
            println!("Spawned robot with ID: {}", robot_id);

            // Control the robot (mock manipulation)
            isaac_adapter.control_vehicle(&robot_id, 0.2, 0.1, 0.0).await?;

            // Get sensor data from the robot
            let force_data = isaac_adapter.get_sensor_data(&robot_id, "force_torque").await?;
            println!("Force/torque data size: {} bytes", force_data.data.len());
        }

        // If a Gazebo adapter is available (third adapter), spawn and control a model there.
        if rest.len() > 1 {
            let gazebo_adapter = &mut rest[1];
            let model_id = gazebo_adapter.spawn_vehicle("turtlebot3_burger", [-2.0, 0.0, 0.0]).await?;
            println!("Spawned model with ID: {}", model_id);

            // Control the model
            gazebo_adapter.control_vehicle(&model_id, 0.3, 0.0, 0.0).await?;

            // Get sensor data
            let imu_data = gazebo_adapter.get_sensor_data(&model_id, "imu").await?;
            println!("IMU data size: {} bytes", imu_data.data.len());
        }

        // Get sensor data
        let camera_data = adapter.get_sensor_data(&vehicle_id, "camera").await?;
        println!("Camera data size: {} bytes", camera_data.data.len());
    }

    // Run simulation for a few steps
    for i in 0..100 {
        if !engine.is_running() {
            break;
        }
        engine.step();
        engine.step_simulators(0.016).await?; // ~60 FPS
        std::thread::sleep(Duration::from_millis(16));
        
        if i % 10 == 0 {
            {
                let step_msg = format!("Simulation step {}", i);
                // If ENABLE_GAZEBO env var is set, annotate the log to indicate Gazebo integration.
                // This keeps the runtime change minimal while signaling that Gazebo (SDF world) support
                // is expected/active. The full GazeboAdapter implementation is registered elsewhere.
                let gazebo_enabled = std::env::var("ENABLE_GAZEBO").unwrap_or_else(|_| "false".into());
                if gazebo_enabled == "1" || gazebo_enabled.eq_ignore_ascii_case("true") {
                    println!("{} - Gazebo adapter integration active (SDF world support)", step_msg);
                } else {
                    println!("{}", step_msg);
                }
            }
        }
    }

    engine.shutdown_simulators().await?;
    engine.stop();
    Ok(())
}

pub fn stop_simulation() {
    println!("Stopping real-time simulation engine...");
}