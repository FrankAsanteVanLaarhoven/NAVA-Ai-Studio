//! Path representation and operations

use crate::manifold::Manifold;
use serde::{Deserialize, Serialize};

/// A point on a path
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PathPoint {
    /// Coordinates
    pub coordinates: Vec<f64>,
    /// Energy at this point
    pub energy: f64,
}

/// A navigation path
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Path {
    /// Waypoints along the path
    pub waypoints: Vec<PathPoint>,
    /// Total energy
    pub total_energy: f64,
    /// Optimization method used
    pub optimization_method: String,
}

impl Path {
    /// Create a path from waypoints
    pub fn from_waypoints(waypoints: Vec<Vec<f64>>, _manifold: &Manifold) -> Self {
        let path_points: Vec<PathPoint> = waypoints
            .iter()
            .enumerate()
            .map(|(i, coords)| PathPoint {
                coordinates: coords.clone(),
                energy: i as f64 * 0.1, // Simplified energy calculation
            })
            .collect();

        let total_energy = path_points.iter().map(|p| p.energy).sum();

        Self {
            waypoints: path_points,
            total_energy,
            optimization_method: "vnc".to_string(), // Van Laarhoven Navigation Calculus
        }
    }

    /// Get number of waypoints
    pub fn len(&self) -> usize {
        self.waypoints.len()
    }

    /// Check if path is empty
    pub fn is_empty(&self) -> bool {
        self.waypoints.is_empty()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::manifold::Manifold;

    #[test]
    fn test_path_creation() {
        let manifold = Manifold::euclidean(3);
        let waypoints = vec![
            vec![0.0, 0.0, 0.0],
            vec![2.5, 2.5, 2.5],
            vec![5.0, 5.0, 5.0],
        ];
        let path = Path::from_waypoints(waypoints, &manifold);
        assert_eq!(path.len(), 3);
    }
}

