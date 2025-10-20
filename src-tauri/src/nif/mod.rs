//! Navigation Intelligence Framework (NIF)
//! A comprehensive framework for autonomous navigation systems

pub mod architecture;
pub mod sensor_fusion;
pub mod factor_graph;
pub mod integrity_monitoring;
pub mod cna;
pub mod securevla;
pub mod acceptance_gates;

 // Re-export main types
pub use architecture::*;
pub use sensor_fusion::*;
pub use integrity_monitoring::*;
pub use securevla::*;
pub use acceptance_gates::*;
pub use cna::*;