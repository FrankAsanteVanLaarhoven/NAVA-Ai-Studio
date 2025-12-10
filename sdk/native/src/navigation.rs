//! Navigation Field implementation
//!
//! Core navigation calculus functionality

use crate::{manifold::Manifold, path::Path, constraints::NavigationConstraints, Result, NavSDKError};
use serde::{Deserialize, Serialize};

/// Navigation Field - Core of NAVΛ SDK
///
/// Represents a navigation field on a manifold for path planning
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NavigationField {
    manifold: Option<Manifold>,
}

impl NavigationField {
    /// Create a new navigation field
    pub fn new() -> Self {
        Self { manifold: None }
    }

    /// Set the manifold for this navigation field
    pub fn set_manifold(&mut self, manifold: Manifold) {
        self.manifold = Some(manifold);
    }

    /// Get the current manifold
    pub fn get_manifold(&self) -> Option<&Manifold> {
        self.manifold.as_ref()
    }

    /// Find an optimal path from start to goal
    ///
    /// # Arguments
    ///
    /// * `start` - Starting point coordinates
    /// * `goal` - Goal point coordinates
    /// * `constraints` - Navigation constraints
    ///
    /// # Returns
    ///
    /// Optimal path or error
    pub fn find_optimal_path(
        &self,
        start: &[f64],
        goal: &[f64],
        constraints: &NavigationConstraints,
    ) -> Result<Path> {
        let manifold = self.manifold.as_ref()
            .ok_or_else(|| NavSDKError::General("Manifold not set".to_string()))?;

        // Validate dimensions
        if start.len() != goal.len() {
            return Err(NavSDKError::General(
                "Start and goal must have same dimension".to_string(),
            ));
        }

        if start.len() != manifold.dimension() {
            return Err(NavSDKError::InvalidManifold(format!(
                "Point dimension {} does not match manifold dimension {}",
                start.len(),
                manifold.dimension()
            )));
        }

        // Validate constraints
        constraints.validate()?;

        // Compute optimal path using navigation calculus
        let path = self.compute_path(start, goal, manifold, constraints)?;

        Ok(path)
    }

    /// Compute path using navigation calculus
    fn compute_path(
        &self,
        start: &[f64],
        goal: &[f64],
        manifold: &Manifold,
        constraints: &NavigationConstraints,
    ) -> Result<Path> {
        // Simplified path computation
        // In production, this would use advanced navigation calculus algorithms
        
        let mut waypoints = vec![start.to_vec()];
        
        // Add intermediate waypoints based on constraints
        let num_waypoints = (constraints.max_velocity * 10.0) as usize;
        for i in 1..num_waypoints {
            let t = i as f64 / num_waypoints as f64;
            let mut waypoint = vec![0.0; start.len()];
            for j in 0..start.len() {
                waypoint[j] = start[j] + (goal[j] - start[j]) * t;
            }
            waypoints.push(waypoint);
        }
        
        waypoints.push(goal.to_vec());

        Ok(Path::from_waypoints(waypoints, manifold))
    }
}

impl Default for NavigationField {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::manifold::ManifoldType;

    #[test]
    fn test_navigation_field_creation() {
        let nav = NavigationField::new();
        assert!(nav.get_manifold().is_none());
    }

    #[test]
    fn test_set_manifold() {
        let mut nav = NavigationField::new();
        let manifold = Manifold::new(ManifoldType::Euclidean, 3);
        nav.set_manifold(manifold);
        assert!(nav.get_manifold().is_some());
    }

    #[test]
    fn test_find_optimal_path() {
        let mut nav = NavigationField::new();
        nav.set_manifold(Manifold::new(ManifoldType::Euclidean, 3));
        
        let start = vec![0.0, 0.0, 0.0];
        let goal = vec![5.0, 5.0, 5.0];
        let constraints = NavigationConstraints::default();
        
        let path = nav.find_optimal_path(&start, &goal, &constraints);
        assert!(path.is_ok());
    }
}

