//! NAVΛ SDK - Ultimate Cross-Platform SDK
//!
//! Expert-led SDK for Navigation Calculus programming
//! Supports Linux, macOS, Windows, and Web platforms

#![deny(missing_docs)]
#![warn(clippy::all)]

use serde::{Deserialize, Serialize};
use std::error::Error;
use std::fmt;

/// Main SDK module
pub mod navigation;
pub mod manifold;
pub mod path;
pub mod constraints;

pub use navigation::NavigationField;
pub use manifold::Manifold;
pub use path::{Path, PathPoint};
pub use constraints::NavigationConstraints;

/// SDK version
pub const SDK_VERSION: &str = env!("CARGO_PKG_VERSION");

/// NAVΛ SDK Error types
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum NavSDKError {
    /// Invalid manifold configuration
    InvalidManifold(String),
    /// Path planning failed
    PathPlanningFailed(String),
    /// Invalid constraints
    InvalidConstraints(String),
    /// General error
    General(String),
}

impl fmt::Display for NavSDKError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            NavSDKError::InvalidManifold(msg) => write!(f, "Invalid manifold: {}", msg),
            NavSDKError::PathPlanningFailed(msg) => write!(f, "Path planning failed: {}", msg),
            NavSDKError::InvalidConstraints(msg) => write!(f, "Invalid constraints: {}", msg),
            NavSDKError::General(msg) => write!(f, "Error: {}", msg),
        }
    }
}

impl Error for NavSDKError {}

/// SDK Result type
pub type Result<T> = std::result::Result<T, NavSDKError>;

/// Initialize the SDK
///
/// Call this before using any SDK functionality
pub fn init() {
    env_logger::init();
    log::info!("NAVΛ SDK v{} initialized", SDK_VERSION);
}

/// Get SDK version
pub fn version() -> &'static str {
    SDK_VERSION
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_version() {
        assert!(!version().is_empty());
    }

    #[test]
    fn test_init() {
        init();
    }
}

