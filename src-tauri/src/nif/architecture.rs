//! NIF Architecture: 7-Layer Framework
//! Implements the layered architecture from Input (z=8) to Trust (z=-4)

use std::collections::HashMap;
use std::sync::Arc;
use tokio::sync::Mutex;
use serde::{Deserialize, Serialize};

/// NIF Layer enumeration representing the conceptual z-coordinate
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum NIFLayer {
    /// Layer 1: Input (z=8) - The Sensory Foundation
    Input = 8,
    /// Layer 2: Encoding (z=6) - Translating the World into Features
    Encoding = 6,
    /// Layer 3: Reasoning (z=4) - The Core of Vision-Language Understanding
    Reasoning = 4,
    /// Layer 4: NIF Core (z=2) - High-Integrity Factor-Graph Fusion
    NIFCore = 2,
    /// Layer 5: Planning (z=0) - From Intention to Action
    Planning = 0,
    /// Layer 6: Evaluation (z=-2) - Rigorous Metrics for Performance and Integrity
    Evaluation = -2,
    /// Layer 7: Trust (z=-4) - Governance as a First-Class Citizen
    Trust = -4,
}

/// Core NIF Architecture implementing the 7-layer framework
pub struct NIFArchitecture {
    layers: HashMap<NIFLayer, Arc<Mutex<dyn NIFLayerTrait>>>,
    data_flow: Vec<NIFLayer>,
}

// Sensor data structures

/// IMU sensor data
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct IMUData {
    pub acceleration: [f64; 3],
    pub angular_velocity: [f64; 3],
    pub timestamp: f64,
}

/// GNSS sensor data
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GNSSData {
    pub pseudorange: Vec<f64>,
    pub satellite_positions: Vec<[f64; 3]>,
    pub timestamp: f64,
}

/// Unified sensor data structure
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SensorData {
    pub imu: Option<IMUData>,
    pub gnss: Option<GNSSData>,
    pub uwb_wifi: Option<RadioData>,
    pub lidar: Option<LidarData>,
    pub timestamp: f64,
}

/// Radio ranging data (UWB/Wi-Fi)
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RadioData {
    pub ranges: Vec<f64>,
    pub anchor_positions: Vec<[f64; 3]>,
}

/// LiDAR sensor data
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LidarData {
    pub point_cloud: Vec<[f64; 3]>,
    pub timestamp: f64,
}

#[async_trait::async_trait]
pub trait NIFLayerTrait: Send + Sync {
    /// Process data flowing through this layer
    async fn process(&mut self, input: NIFData) -> Result<NIFData, NIFError>;

    /// Get layer metadata and status
    fn metadata(&self) -> LayerMetadata;

    /// Validate layer integrity
    fn validate(&self) -> Result<(), NIFError>;
}

/// Unified data structure flowing through NIF layers
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NIFData {
    pub timestamp: f64,
    pub sensor_data: SensorData,
    pub feature_vectors: Vec<Vec<f64>>,
    pub semantic_constraints: Vec<SemanticConstraint>,
    pub pose_estimate: Option<Pose3D>,
    pub integrity_metrics: IntegrityMetrics,
    pub trust_score: f64,
}

 // Duplicate sensor data struct definitions removed — consolidated in a single shared module to avoid duplication.

/// 3D Pose representation
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Pose3D {
    pub position: [f64; 3],
    pub orientation: [f64; 4], // quaternion
    pub velocity: [f64; 3],
    pub timestamp: f64,
}

/// Semantic constraints for navigation
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SemanticConstraint {
    pub constraint_type: ConstraintType,
    pub parameters: HashMap<String, f64>,
    pub priority: f32,
}

/// Types of semantic constraints
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum ConstraintType {
    LaneBoundary,
    SpeedLimit,
    ForbiddenZone,
    PreferredPath,
    ObstacleAvoidance,
}

/// Integrity monitoring metrics
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct IntegrityMetrics {
    pub raim_rejects: usize,
    pub gnss_outliers: usize,
    pub covariance_trace: f64,
    pub chi_squared_test: f64,
    pub mia_auc: f64,
}

/// Topological map representation
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TopologicalMap {
    pub nodes: Vec<MapNode>,
    pub edges: Vec<MapEdge>,
}

/// Map node (waypoint)
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MapNode {
    pub id: String,
    pub position: [f64; 3],
    pub node_type: NodeType,
}

/// Map edge (connection between waypoints)
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MapEdge {
    pub from_id: String,
    pub to_id: String,
    pub cost: f64,
    pub constraints: Vec<String>,
}

/// Node types in topological map
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum NodeType {
    Waypoint,
    Landmark,
    DecisionPoint,
    Goal,
}

/// 2D Polygon for forbidden zones
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Polygon {
    pub vertices: Vec<[f64; 2]>,
}

/// Layer metadata and status
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LayerMetadata {
    pub layer: NIFLayer,
    pub name: String,
    pub status: LayerStatus,
    pub processing_time_ms: f64,
    pub data_throughput: f64,
}

/// Layer operational status
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum LayerStatus {
    Initializing,
    Active,
    Degraded,
    Failed,
}

/// NIF processing errors
#[derive(Debug, Clone)]
pub enum NIFError {
    LayerError { message: String },
    SensorUnavailable { sensor: String },
    IntegrityViolation { violation: String },
    OptimizationFailed,

    PrivacyViolation,

    AcceptanceGateFailed { gate: String },
}

impl NIFArchitecture {
    /// Create new NIF architecture instance
    pub fn new() -> Self {
        let mut layers = HashMap::new();
        let data_flow = vec![
            NIFLayer::Input,
            NIFLayer::Encoding,
            NIFLayer::Reasoning,
            NIFLayer::NIFCore,
            NIFLayer::Planning,
            NIFLayer::Evaluation,
            NIFLayer::Trust,
        ];

        // Initialize layers (implementations will be added)
        // layers.insert(NIFLayer::Input, Arc::new(Mutex::new(InputLayer::new())));
        // layers.insert(NIFLayer::Encoding, Arc::new(Mutex::new(EncodingLayer::new())));
        // ... etc

        NIFArchitecture { layers, data_flow }
    }

    /// Process data through the entire NIF pipeline
    pub async fn process_pipeline(&mut self, input: NIFData) -> Result<NIFData, NIFError> {
        let mut current_data = input;

        for layer_type in &self.data_flow {
            if let Some(layer) = self.layers.get(layer_type) {
                let mut layer_guard = layer.lock().await;
                current_data = layer_guard.process(current_data).await?;
            }
        }

        Ok(current_data)
    }

    /// Get status of all layers
    pub async fn get_system_status(&self) -> HashMap<NIFLayer, LayerMetadata> {
        let mut status = HashMap::new();

        for (layer_type, layer) in &self.layers {
            let layer_guard = layer.lock().await;
            status.insert(*layer_type, layer_guard.metadata());
        }

        status
    }

    /// Validate entire system integrity
    pub async fn validate_system(&self) -> Result<(), NIFError> {
        for layer in self.layers.values() {
            let layer_guard = layer.lock().await;
            layer_guard.validate()?;
        }
        Ok(())
    }
}

impl Default for NIFArchitecture {
    fn default() -> Self {
        Self::new()
    }
}