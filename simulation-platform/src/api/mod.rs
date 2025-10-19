//! Web API for the NAVΛ simulation platform
//!
//! This module provides a REST API for controlling the simulation from the web frontend.

use anyhow::Result;
use std::sync::{Arc, Mutex};
use serde::{Serialize, Deserialize};
use warp::{Filter, Reply};

use crate::core::{SimulationEngine, Obstacle, ObstacleShape};
use crate::robotics::{Robot, RobotConfig, RobotModel};

/// API server for the simulation
pub struct ApiServer {
    /// Simulation engine
    simulation: Arc<Mutex<SimulationEngine>>,
    
    /// Server port
    port: u16,
}

/// Request to add a robot
#[derive(Debug, Deserialize)]
pub struct AddRobotRequest {
    pub id: String,
    pub name: String,
    pub model: String,
    pub max_linear_velocity: f32,
    pub max_angular_velocity: f32,
}

/// Request to control a robot
#[derive(Debug, Deserialize)]
pub struct ControlRobotRequest {
    pub robot_id: String,
    pub linear_velocity: f32,
    pub angular_velocity: f32,
}

/// Request to add an obstacle
#[derive(Debug, Deserialize)]
pub struct AddObstacleRequest {
    pub id: String,
    pub shape: String,
    pub position: [f32; 3],
    pub rotation: [f32; 3],
    pub scale: [f32; 3],
}

/// Response with simulation state
#[derive(Debug, Serialize)]
pub struct StateResponse {
    pub success: bool,
    pub data: Option<String>,
    pub error: Option<String>,
}

/// Generic API response
#[derive(Debug, Serialize)]
pub struct ApiResponse {
    pub success: bool,
    pub message: String,
}

impl ApiServer {
    /// Create a new API server
    pub fn new(simulation: Arc<Mutex<SimulationEngine>>, port: u16) -> Self {
        Self {
            simulation,
            port,
        }
    }
    
    /// Start the API server
    pub async fn start(&self) -> Result<()> {
        let simulation = self.simulation.clone();
        
        // GET /api/state - Get simulation state
        let get_state = warp::path!("api" / "state")
            .and(warp::get())
            .and(with_simulation(simulation.clone()))
            .and_then(handle_get_state);
        
        // POST /api/start - Start simulation
        let start_sim = warp::path!("api" / "start")
            .and(warp::post())
            .and(with_simulation(simulation.clone()))
            .and_then(handle_start);
        
        // POST /api/pause - Pause simulation
        let pause_sim = warp::path!("api" / "pause")
            .and(warp::post())
            .and(with_simulation(simulation.clone()))
            .and_then(handle_pause);
        
        // POST /api/stop - Stop simulation
        let stop_sim = warp::path!("api" / "stop")
            .and(warp::post())
            .and(with_simulation(simulation.clone()))
            .and_then(handle_stop);
        
        // POST /api/reset - Reset simulation
        let reset_sim = warp::path!("api" / "reset")
            .and(warp::post())
            .and(with_simulation(simulation.clone()))
            .and_then(handle_reset);
        
        // POST /api/robot - Add a robot
        let add_robot = warp::path!("api" / "robot")
            .and(warp::post())
            .and(warp::body::json())
            .and(with_simulation(simulation.clone()))
            .and_then(handle_add_robot);
        
        // POST /api/robot/control - Control a robot
        let control_robot = warp::path!("api" / "robot" / "control")
            .and(warp::post())
            .and(warp::body::json())
            .and(with_simulation(simulation.clone()))
            .and_then(handle_control_robot);
        
        // POST /api/obstacle - Add an obstacle
        let add_obstacle = warp::path!("api" / "obstacle")
            .and(warp::post())
            .and(warp::body::json())
            .and(with_simulation(simulation.clone()))
            .and_then(handle_add_obstacle);
        
        // DELETE /api/obstacle/:id - Remove an obstacle
        let remove_obstacle = warp::path!("api" / "obstacle" / String)
            .and(warp::delete())
            .and(with_simulation(simulation.clone()))
            .and_then(handle_remove_obstacle);
        
        // Combine all routes
        let routes = get_state
            .or(start_sim)
            .or(pause_sim)
            .or(stop_sim)
            .or(reset_sim)
            .or(add_robot)
            .or(control_robot)
            .or(add_obstacle)
            .or(remove_obstacle)
            .with(warp::cors().allow_any_origin());
        
        println!("🚀 NAVΛ Simulation API starting on port {}", self.port);
        warp::serve(routes).run(([127, 0, 0, 1], self.port)).await;
        
        Ok(())
    }
}

// Helper function to pass simulation to handlers
fn with_simulation(
    sim: Arc<Mutex<SimulationEngine>>,
) -> impl Filter<Extract = (Arc<Mutex<SimulationEngine>>,), Error = std::convert::Infallible> + Clone {
    warp::any().map(move || sim.clone())
}

// Handler for getting simulation state
async fn handle_get_state(
    simulation: Arc<Mutex<SimulationEngine>>,
) -> Result<impl Reply, warp::Rejection> {
    let state_json = {
        let sim = simulation.lock().unwrap();
        sim.get_state()
    };
    
    match state_json {
        Ok(data) => Ok(warp::reply::json(&StateResponse {
            success: true,
            data: Some(data),
            error: None,
        })),
        Err(e) => Ok(warp::reply::json(&StateResponse {
            success: false,
            data: None,
            error: Some(e.to_string()),
        })),
    }
}

// Handler for starting simulation
async fn handle_start(
    simulation: Arc<Mutex<SimulationEngine>>,
) -> Result<impl Reply, warp::Rejection> {
    let mut sim = simulation.lock().unwrap();
    sim.resume();
    
    Ok(warp::reply::json(&ApiResponse {
        success: true,
        message: "Simulation started".to_string(),
    }))
}

// Handler for pausing simulation
async fn handle_pause(
    simulation: Arc<Mutex<SimulationEngine>>,
) -> Result<impl Reply, warp::Rejection> {
    let mut sim = simulation.lock().unwrap();
    sim.pause();
    
    Ok(warp::reply::json(&ApiResponse {
        success: true,
        message: "Simulation paused".to_string(),
    }))
}

// Handler for stopping simulation
async fn handle_stop(
    simulation: Arc<Mutex<SimulationEngine>>,
) -> Result<impl Reply, warp::Rejection> {
    let mut sim = simulation.lock().unwrap();
    sim.stop();
    
    Ok(warp::reply::json(&ApiResponse {
        success: true,
        message: "Simulation stopped".to_string(),
    }))
}

// Handler for resetting simulation
async fn handle_reset(
    simulation: Arc<Mutex<SimulationEngine>>,
) -> Result<impl Reply, warp::Rejection> {
    let mut sim = simulation.lock().unwrap();
    sim.reset();
    
    Ok(warp::reply::json(&ApiResponse {
        success: true,
        message: "Simulation reset".to_string(),
    }))
}

// Handler for adding a robot
async fn handle_add_robot(
    req: AddRobotRequest,
    simulation: Arc<Mutex<SimulationEngine>>,
) -> Result<impl Reply, warp::Rejection> {
    let model = match req.model.as_str() {
        "differential_drive" => RobotModel::DifferentialDrive,
        "ackermann" => RobotModel::Ackermann,
        "legged" => RobotModel::Legged,
        "aerial" => RobotModel::Aerial,
        "marine" => RobotModel::Marine,
        _ => RobotModel::DifferentialDrive,
    };
    
    let config = RobotConfig {
        model,
        max_linear_velocity: req.max_linear_velocity,
        max_angular_velocity: req.max_angular_velocity,
        battery_capacity: 100.0,
    };
    
    let robot = Robot::new(req.id.clone(), req.name, config);
    
    let mut sim = simulation.lock().unwrap();
    sim.add_robot(robot);
    
    Ok(warp::reply::json(&ApiResponse {
        success: true,
        message: format!("Robot {} added", req.id),
    }))
}

// Handler for controlling a robot
async fn handle_control_robot(
    req: ControlRobotRequest,
    simulation: Arc<Mutex<SimulationEngine>>,
) -> Result<impl Reply, warp::Rejection> {
    let mut sim = simulation.lock().unwrap();
    
    if let Some(robot) = sim.get_robot_mut(&req.robot_id) {
        // Apply control commands
        let commands = vec![req.linear_velocity, req.angular_velocity];
        let _ = robot.apply_control(commands);
        
        Ok(warp::reply::json(&ApiResponse {
            success: true,
            message: format!("Control applied to robot {}", req.robot_id),
        }))
    } else {
        Ok(warp::reply::json(&ApiResponse {
            success: false,
            message: format!("Robot {} not found", req.robot_id),
        }))
    }
}

// Handler for adding an obstacle
async fn handle_add_obstacle(
    req: AddObstacleRequest,
    simulation: Arc<Mutex<SimulationEngine>>,
) -> Result<impl Reply, warp::Rejection> {
    let shape = match req.shape.to_lowercase().as_str() {
        "box" => ObstacleShape::Box,
        "sphere" => ObstacleShape::Sphere,
        "cylinder" => ObstacleShape::Cylinder,
        "cone" => ObstacleShape::Cone,
        _ => ObstacleShape::Box,
    };
    
    let obstacle = Obstacle {
        id: req.id.clone(),
        shape,
        position: req.position,
        rotation: req.rotation,
        scale: req.scale,
    };
    
    let mut sim = simulation.lock().unwrap();
    sim.add_obstacle(obstacle);
    
    Ok(warp::reply::json(&ApiResponse {
        success: true,
        message: format!("Obstacle {} added", req.id),
    }))
}

// Handler for removing an obstacle
async fn handle_remove_obstacle(
    id: String,
    simulation: Arc<Mutex<SimulationEngine>>,
) -> Result<impl Reply, warp::Rejection> {
    let mut sim = simulation.lock().unwrap();
    sim.remove_obstacle(&id);
    
    Ok(warp::reply::json(&ApiResponse {
        success: true,
        message: format!("Obstacle {} removed", id),
    }))
}

