//! Manifold types and operations

use serde::{Deserialize, Serialize};

/// Manifold type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ManifoldType {
    /// Euclidean space
    Euclidean,
    /// Riemannian manifold
    Riemannian,
    /// Lorentzian manifold
    Lorentzian,
}

/// Manifold representation
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Manifold {
    manifold_type: ManifoldType,
    dimension: usize,
    metric: Option<String>,
}

impl Manifold {
    /// Create a new manifold
    pub fn new(manifold_type: ManifoldType, dimension: usize) -> Self {
        Self {
            manifold_type,
            dimension,
            metric: None,
        }
    }

    /// Create a Euclidean manifold
    pub fn euclidean(dimension: usize) -> Self {
        Self::new(ManifoldType::Euclidean, dimension)
    }

    /// Create a Riemannian manifold
    pub fn riemannian(dimension: usize, metric: String) -> Self {
        Self {
            manifold_type: ManifoldType::Riemannian,
            dimension,
            metric: Some(metric),
        }
    }

    /// Get manifold type
    pub fn manifold_type(&self) -> ManifoldType {
        self.manifold_type
    }

    /// Get dimension
    pub fn dimension(&self) -> usize {
        self.dimension
    }

    /// Get metric
    pub fn metric(&self) -> Option<&str> {
        self.metric.as_deref()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_euclidean_manifold() {
        let m = Manifold::euclidean(3);
        assert_eq!(m.dimension(), 3);
        assert_eq!(m.manifold_type(), ManifoldType::Euclidean);
    }
}

