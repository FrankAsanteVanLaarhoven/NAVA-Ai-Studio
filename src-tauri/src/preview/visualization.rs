use serde::{Serialize, Deserialize};

/// Real-time Visualization Data Generator
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VisualizationData {
    pub paths: Vec<PathVisualization>,
    pub energy_field: Vec<EnergyPoint>,
    pub frame_time: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PathVisualization {
    pub points: Vec<[f64; 3]>,
    pub colors: Vec<[f32; 4]>,
    pub thickness: f32,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EnergyPoint {
    pub position: [f64; 3],
    pub value: f64,
    pub gradient: [f64; 3],
}

pub struct VisualizationGenerator;

impl VisualizationGenerator {
    pub fn generate(paths: &[[f64; 3]]) -> VisualizationData {
        let path_viz = PathVisualization {
            points: paths.to_vec(),
            colors: vec![[0.0, 1.0, 0.0, 1.0]; paths.len()],
            thickness: 2.0,
        };
        
        VisualizationData {
            paths: vec![path_viz],
            energy_field: Vec::new(),
            frame_time: 16.67, // 60 FPS
        }
    }
}

