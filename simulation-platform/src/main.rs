//! NAVΛ Simulation Platform
//!
//! A high-performance, memory-safe robotics simulation platform built with Rust.
//! This platform leverages the Van Laarhoven Navigation Calculus for advanced
//! robotics path planning and control.

mod core;
mod physics;
mod rendering;
mod robotics;
mod ros2;
mod api;

use anyhow::Result;
use clap::Parser;
use tracing::{info, error};
use std::sync::{Arc, Mutex};

/// Command line arguments for the simulation platform
#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    /// Path to the simulation configuration file
    #[arg(short, long)]
    config: Option<String>,

    /// Enable verbose logging
    #[arg(short, long, action)]
    verbose: bool,

    /// Run in headless mode (no GUI)
    #[arg(long, action)]
    headless: bool,

    /// Enable ROS2 integration
    #[arg(long, action)]
    ros2: bool,
    
    /// API server port
    #[arg(long, default_value = "3030")]
    port: u16,
    
    /// Run as API server only (no simulation loop)
    #[arg(long, action)]
    api_only: bool,
}

#[tokio::main]
async fn main() -> Result<()> {
    let args = Args::parse();

    // Initialize logging
    if args.verbose {
        tracing_subscriber::fmt()
            .with_max_level(tracing::Level::DEBUG)
            .init();
    } else {
        tracing_subscriber::fmt()
            .with_max_level(tracing::Level::INFO)
            .init();
    }

    info!("🚀 Starting NAVΛ Simulation Platform");
    
    // Initialize the simulation core
    let mut simulation = core::SimulationEngine::new();
    
    // Load configuration if provided
    if let Some(config_path) = &args.config {
        info!("📋 Loading configuration from: {}", config_path);
        simulation.load_config(config_path)?;
    }
    
    // Initialize ROS2 integration if requested
    if args.ros2 {
        info!("🤖 Initializing ROS2 integration");
        ros2::init_ros2()?;
    }
    
    // Wrap simulation in Arc<Mutex> for thread-safe sharing
    let simulation = Arc::new(Mutex::new(simulation));
    
    // Start API server
    info!("🌐 Starting API server on port {}", args.port);
    let api_server = api::ApiServer::new(simulation.clone(), args.port);
    
    if args.api_only {
        // Run only the API server
        info!("📡 Running in API-only mode");
        api_server.start().await?;
    } else {
        // Run both API server and simulation in parallel
        info!("🎮 Running simulation with API server");
        
        let sim_clone = simulation.clone();
        let simulation_task = tokio::spawn(async move {
            loop {
                // Check if simulation is running
                let is_running = {
                    let sim = sim_clone.lock().unwrap();
                    sim.is_running
                };
                
                // Step simulation if running
                if is_running {
                    let result = {
                        let mut sim = sim_clone.lock().unwrap();
                        sim.step()
                    };
                    
                    if let Err(e) = result {
                        error!("Simulation step error: {}", e);
                    }
                }
                
                // Sleep for timestep duration
                tokio::time::sleep(std::time::Duration::from_millis(10)).await;
            }
        });
        
        let api_task = tokio::spawn(async move {
            if let Err(e) = api_server.start().await {
                error!("API server error: {}", e);
            }
        });
        
        // Wait for both tasks
        tokio::select! {
            _ = simulation_task => {
                info!("Simulation task completed");
            }
            _ = api_task => {
                info!("API task completed");
            }
        }
    }
    
    info!("✅ NAVΛ Simulation Platform shut down successfully");
    Ok(())
}