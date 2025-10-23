
use std::sync::Arc;
use tokio::sync::RwLock;

#[derive(Clone, Debug)]
pub struct SimulationState {
    pub is_running: bool,
    pub vehicles: Vec<VehicleState>,
    pub robots: Vec<RobotState>,
    pub models: Vec<ModelState>,
}

#[derive(Clone, Debug)]
pub struct VehicleState {
    pub id: String,
    pub position: [f32; 3],
    pub velocity: [f32; 3],
    pub sensors: Vec<SensorData>,
}

#[derive(Clone, Debug)]
pub struct RobotState {
    pub id: String,
    pub joint_positions: Vec<f32>,
    pub joint_velocities: Vec<f32>,
    pub sensors: Vec<SensorData>,
}

#[derive(Clone, Debug)]
pub struct ModelState {
    pub id: String,
    pub position: [f32; 3],
    pub orientation: [f32; 4], // quaternion
    pub sensors: Vec<SensorData>,
}

#[derive(Clone, Debug)]
pub struct SensorData {
    pub sensor_type: String,
    pub data: Vec<u8>,
    pub timestamp: u64,
}

pub struct SimulationEngine {
    state: Arc<RwLock<SimulationState>>,
}

impl SimulationEngine {
    pub fn new() -> Self {
        SimulationEngine {
            state: Arc::new(RwLock::new(SimulationState {
                is_running: false,
                vehicles: Vec::new(),
                robots: Vec::new(),
                models: Vec::new(),
            })),
        }
    }

    pub async fn start_simulation(&self) {
        let mut state = self.state.write().await;
        if state.is_running {
            println!("Simulation already running.");
            return;
        }
        state.is_running = true;
        println!("Starting real-time simulation engine...");
    }

    pub async fn stop_simulation(&self) {
        let mut state = self.state.write().await;
        if !state.is_running {
            println!("Simulation already stopped.");
            return;
        }
        state.is_running = false;
        println!("Stopping real-time simulation engine...");
    }

    pub async fn get_simulation_state(&self) -> SimulationState {
        self.state.read().await.clone()
    }

    pub async fn add_vehicle(&self, vehicle: VehicleState) {
        let mut state = self.state.write().await;
        // avoid duplicates by id
        if state.vehicles.iter().any(|v| v.id == vehicle.id) {
            println!("Vehicle with id '{}' already exists; skipping add.", vehicle.id);
            return;
        }
        state.vehicles.push(vehicle);
    }

    pub async fn add_robot(&self, robot: RobotState) {
        let mut state = self.state.write().await;
        if state.robots.iter().any(|r| r.id == robot.id) {
            println!("Robot with id '{}' already exists; skipping add.", robot.id);
            return;
        }
        state.robots.push(robot);
    }

    pub async fn add_model(&self, model: ModelState) {
        let mut state = self.state.write().await;
        if state.models.iter().any(|m| m.id == model.id) {
            println!("Model with id '{}' already exists; skipping add.", model.id);
            return;
        }
        state.models.push(model);
    }

    pub async fn update_vehicle_state(&self, vehicle_id: &str, new_state: VehicleState) {
        let mut state = self.state.write().await;
        if let Some(vehicle) = state.vehicles.iter_mut().find(|v| v.id == vehicle_id) {
            *vehicle = new_state;
        } else {
            println!("Vehicle with id '{}' not found; update ignored.", vehicle_id);
        }
    }

    pub async fn update_robot_state(&self, robot_id: &str, new_state: RobotState) {
        let mut state = self.state.write().await;
        if let Some(robot) = state.robots.iter_mut().find(|r| r.id == robot_id) {
            *robot = new_state;
        } else {
            println!("Robot with id '{}' not found; update ignored.", robot_id);
        }
    }

    pub async fn update_model_state(&self, model_id: &str, new_state: ModelState) {
        let mut state = self.state.write().await;
        if let Some(model) = state.models.iter_mut().find(|m| m.id == model_id) {
            *model = new_state;
        } else {
            println!("Model with id '{}' not found; update ignored.", model_id);
        }
    }
}

// Provide simple async wrappers preserving the original exported function names,
// but now asynchronous to work with the runtime and the SimulationEngine.
pub async fn start_simulation() {
    let engine = SimulationEngine::new();
    engine.start_simulation().await;
}

pub async fn stop_simulation() {
    let engine = SimulationEngine::new();
    engine.stop_simulation().await;
}
