//! Core simulation engine for the NAVΛ platform
//!
//! This module provides the main simulation loop, time management,
//! and coordination between different simulation components.

use anyhow::Result;
use std::time::Duration;
use std::sync::{Arc, Mutex};
use std::collections::HashMap;
use serde::{Serialize, Deserialize};

use crate::physics::PhysicsEngine;
use crate::robotics::Robot;
use crate::rendering::RenderingEngine;

/// Main simulation engine
pub struct SimulationEngine {
    /// Current simulation time
    pub time: f64,
    
    /// Time step for the simulation
    pub time_step: f64,
    
    /// Whether the simulation is currently running
    pub is_running: bool,
    
    /// Configuration for the simulation
    pub config: SimulationConfig,
    
    /// Physics engine
    pub physics: Arc<Mutex<PhysicsEngine>>,
    
    /// Rendering engine
    pub rendering: Arc<Mutex<RenderingEngine>>,
    
    /// Robots in the simulation
    pub robots: HashMap<String, Robot>,
    
    /// World models
    pub world: World,
}

/// World representation in the simulation
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct World {
    /// World name
    pub name: String,
    
    /// Gravity vector
    pub gravity: [f32; 3],
    
    /// Ground plane configuration
    pub ground_plane: GroundPlane,
    
    /// Obstacles in the world
    pub obstacles: Vec<Obstacle>,
    
    /// Lighting configuration
    pub lighting: LightingConfig,
}

/// Ground plane configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GroundPlane {
    /// Enable ground plane
    pub enabled: bool,
    
    /// Height of the ground plane
    pub height: f32,
    
    /// Size of the ground plane
    pub size: f32,
    
    /// Ground material
    pub material: String,
}

/// Obstacle in the world
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Obstacle {
    /// Obstacle ID
    pub id: String,
    
    /// Obstacle type
    pub shape: ObstacleShape,
    
    /// Position [x, y, z]
    pub position: [f32; 3],
    
    /// Rotation [roll, pitch, yaw]
    pub rotation: [f32; 3],
    
    /// Scale [x, y, z]
    pub scale: [f32; 3],
}

/// Types of obstacle shapes
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum ObstacleShape {
    Box,
    Sphere,
    Cylinder,
    Cone,
    Mesh { path: String },
}

/// Lighting configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LightingConfig {
    /// Ambient light intensity
    pub ambient: f32,
    
    /// Directional lights
    pub directional_lights: Vec<DirectionalLight>,
}

/// Directional light
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DirectionalLight {
    /// Direction vector
    pub direction: [f32; 3],
    
    /// Light intensity
    pub intensity: f32,
    
    /// Light color [r, g, b]
    pub color: [f32; 3],
}

/// Configuration for the simulation engine
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SimulationConfig {
    /// Maximum simulation time (in seconds)
    pub max_time: f64,
    
    /// Time step for the simulation (in seconds)
    pub time_step: f64,
    
    /// Enable real-time synchronization
    pub real_time: bool,
    
    /// Target FPS for real-time mode
    pub target_fps: f64,
}

impl Default for SimulationConfig {
    fn default() -> Self {
        Self {
            max_time: 60.0,
            time_step: 0.01,
            real_time: false,
            target_fps: 60.0,
        }
    }
}

impl SimulationEngine {
    /// Create a new simulation engine with default configuration
    pub fn new() -> Self {
        Self {
            time: 0.0,
            time_step: 0.01,
            is_running: false,
            config: SimulationConfig::default(),
            physics: Arc::new(Mutex::new(PhysicsEngine::new())),
            rendering: Arc::new(Mutex::new(RenderingEngine::new())),
            robots: HashMap::new(),
            world: World::default(),
        }
    }
    
    /// Load configuration from a file
    pub fn load_config(&mut self, path: &str) -> Result<()> {
        use std::fs;
        let contents = fs::read_to_string(path)?;
        let config: SimulationConfig = serde_json::from_str(&contents)?;
        self.config = config;
        Ok(())
    }
    
    /// Add a robot to the simulation
    pub fn add_robot(&mut self, robot: Robot) {
        self.robots.insert(robot.id.clone(), robot);
    }
    
    /// Remove a robot from the simulation
    pub fn remove_robot(&mut self, id: &str) -> Option<Robot> {
        self.robots.remove(id)
    }
    
    /// Get a robot by ID
    pub fn get_robot(&self, id: &str) -> Option<&Robot> {
        self.robots.get(id)
    }
    
    /// Get a mutable reference to a robot
    pub fn get_robot_mut(&mut self, id: &str) -> Option<&mut Robot> {
        self.robots.get_mut(id)
    }
    
    /// Add an obstacle to the world
    pub fn add_obstacle(&mut self, obstacle: Obstacle) {
        self.world.obstacles.push(obstacle);
    }
    
    /// Remove an obstacle from the world
    pub fn remove_obstacle(&mut self, id: &str) {
        self.world.obstacles.retain(|o| o.id != id);
    }
    
    /// Get simulation state as JSON
    pub fn get_state(&self) -> Result<String> {
        #[derive(Serialize)]
        struct SimulationState {
            time: f64,
            is_running: bool,
            robots: Vec<RobotState>,
            obstacles: Vec<Obstacle>,
        }
        
        #[derive(Serialize)]
        struct RobotState {
            id: String,
            name: String,
            position: [f32; 3],
        }
        
        let robot_states: Vec<RobotState> = self.robots.values()
            .map(|r| RobotState {
                id: r.id.clone(),
                name: r.name.clone(),
                position: [r.pose.translation.x, r.pose.translation.y, r.pose.translation.z],
            })
            .collect();
        
        let state = SimulationState {
            time: self.time,
            is_running: self.is_running,
            robots: robot_states,
            obstacles: self.world.obstacles.clone(),
        };
        
        Ok(serde_json::to_string(&state)?)
    }
    
    /// Run the simulation
    pub async fn run(&mut self) -> Result<()> {
        self.is_running = true;
        
        while self.time < self.config.max_time && self.is_running {
            self.step()?;
            
            // Handle real-time synchronization
            if self.config.real_time {
                tokio::time::sleep(Duration::from_secs_f64(self.time_step)).await;
            }
        }
        
        self.is_running = false;
        Ok(())
    }
    
    /// Execute a single simulation step
    pub fn step(&mut self) -> Result<()> {
        // Step physics simulation
        {
            let mut physics = self.physics.lock().unwrap();
            physics.step();
        }
        
        // Update all robots
        for robot in self.robots.values_mut() {
            robot.update_sensors()?;
        }
        
        // Render the scene
        {
            let mut rendering = self.rendering.lock().unwrap();
            rendering.render_frame()?;
        }
        
        self.time += self.time_step;
        Ok(())
    }
    
    /// Pause the simulation
    pub fn pause(&mut self) {
        self.is_running = false;
    }
    
    /// Resume the simulation
    pub fn resume(&mut self) {
        self.is_running = true;
    }
    
    /// Stop the simulation
    pub fn stop(&mut self) {
        self.is_running = false;
        self.time = 0.0;
    }
    
    /// Reset the simulation
    pub fn reset(&mut self) {
        self.time = 0.0;
        self.is_running = false;
        self.robots.clear();
        self.world.obstacles.clear();
    }
}

impl Default for SimulationEngine {
    fn default() -> Self {
        Self::new()
    }
}

impl Default for World {
    fn default() -> Self {
        Self {
            name: "default_world".to_string(),
            gravity: [0.0, -9.81, 0.0],
            ground_plane: GroundPlane::default(),
            obstacles: Vec::new(),
            lighting: LightingConfig::default(),
        }
    }
}

impl Default for GroundPlane {
    fn default() -> Self {
        Self {
            enabled: true,
            height: 0.0,
            size: 100.0,
            material: "concrete".to_string(),
        }
    }
}

impl Default for LightingConfig {
    fn default() -> Self {
        Self {
            ambient: 0.4,
            directional_lights: vec![
                DirectionalLight {
                    direction: [-0.5, -1.0, -0.3],
                    intensity: 0.8,
                    color: [1.0, 1.0, 1.0],
                }
            ],
        }
    }
}