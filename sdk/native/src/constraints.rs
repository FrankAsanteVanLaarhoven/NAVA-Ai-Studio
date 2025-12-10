//! Navigation constraints

use crate::{Result, NavSDKError};
use serde::{Deserialize, Serialize};

/// Navigation constraints
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NavigationConstraints {
    /// Maximum velocity
    pub max_velocity: f64,
    /// Avoid obstacles flag
    pub avoid_obstacles: bool,
    /// Additional custom constraints
    pub custom: Option<serde_json::Value>,
}

impl NavigationConstraints {
    /// Create default constraints
    pub fn default() -> Self {
        Self {
            max_velocity: 1.0,
            avoid_obstacles: true,
            custom: None,
        }
    }

    /// Validate constraints
    pub fn validate(&self) -> Result<()> {
        if self.max_velocity <= 0.0 {
            return Err(NavSDKError::InvalidConstraints(
                "max_velocity must be positive".to_string(),
            ));
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_default_constraints() {
        let constraints = NavigationConstraints::default();
        assert!(constraints.validate().is_ok());
    }

    #[test]
    fn test_invalid_constraints() {
        let mut constraints = NavigationConstraints::default();
        constraints.max_velocity = -1.0;
        assert!(constraints.validate().is_err());
    }
}

