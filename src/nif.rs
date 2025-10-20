#![allow(clippy::all)]
//! NIF module re-exports

// Include the nif module from src-tauri
#[path = "../src-tauri/src/nif/mod.rs"]
pub mod nif_module;

pub use nif_module::*;