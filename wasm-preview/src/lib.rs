use wasm_bindgen::prelude::*;
use serde::{Serialize, Deserialize};

/// WebAssembly Live Preview Engine
/// 
/// Executes NAVΛ code in real-time with near-native performance

#[wasm_bindgen]
pub struct NavigationPath {
    waypoints: Vec<f64>,
    energy: f64,
}

#[wasm_bindgen]
impl NavigationPath {
    #[wasm_bindgen(constructor)]
    pub fn new() -> Self {
        Self {
            waypoints: Vec::new(),
            energy: 0.0,
        }
    }
    
    pub fn add_waypoint(&mut self, x: f64, y: f64, z: f64) {
        self.waypoints.extend_from_slice(&[x, y, z]);
    }
    
    pub fn get_energy(&self) -> f64 {
        self.energy
    }
    
    pub fn waypoint_count(&self) -> usize {
        self.waypoints.len() / 3
    }
    
    pub fn get_waypoint(&self, index: usize) -> Vec<f64> {
        let start = index * 3;
        if start + 3 <= self.waypoints.len() {
            self.waypoints[start..start + 3].to_vec()
        } else {
            vec![]
        }
    }
}

/// Navigate from start to goal using VNC optimization
#[wasm_bindgen]
pub fn navigate_to_vnc(
    start_x: f64, start_y: f64, start_z: f64,
    goal_x: f64, goal_y: f64, goal_z: f64
) -> NavigationPath {
    let mut path = NavigationPath::new();
    
    // VNC optimization algorithm (simplified for preview)
    path.add_waypoint(start_x, start_y, start_z);
    
    // Generate intermediate waypoints with VNC optimization
    let steps = 5;
    for i in 1..steps {
        let t = i as f64 / steps as f64;
        
        // Apply VNC energy minimization
        let vnc_factor = (1.0 - (2.0 * t - 1.0).powi(2)).sqrt();
        
        let x = start_x + (goal_x - start_x) * t;
        let y = start_y + (goal_y - start_y) * t;
        let z = start_z + (goal_z - start_z) * t * vnc_factor;
        
        path.add_waypoint(x, y, z);
    }
    
    path.add_waypoint(goal_x, goal_y, goal_z);
    
    // Calculate path energy
    path.energy = calculate_path_energy(&path);
    
    path
}

/// Calculate energy of a navigation path
fn calculate_path_energy(path: &NavigationPath) -> f64 {
    let mut energy = 0.0;
    
    for i in 0..path.waypoint_count() - 1 {
        let p1 = path.get_waypoint(i);
        let p2 = path.get_waypoint(i + 1);
        
        if !p1.is_empty() && !p2.is_empty() {
            let dx = p2[0] - p1[0];
            let dy = p2[1] - p1[1];
            let dz = p2[2] - p1[2];
            
            // Euclidean distance as energy metric
            energy += (dx * dx + dy * dy + dz * dz).sqrt();
        }
    }
    
    energy
}

/// Find optimal path through energy landscape
#[wasm_bindgen]
pub fn find_optimal_path_vnc(
    start_x: f64, start_y: f64, start_z: f64,
    goal_x: f64, goal_y: f64, goal_z: f64,
    landscape_energy: f64
) -> NavigationPath {
    let mut path = navigate_to_vnc(start_x, start_y, start_z, goal_x, goal_y, goal_z);
    
    // Apply landscape energy influence
    path.energy *= (1.0 + landscape_energy);
    
    path
}

/// Initialize the WASM module
#[wasm_bindgen(start)]
pub fn init() {
    // Set up panic hook for better error messages
    #[cfg(feature = "console_error_panic_hook")]
    console_error_panic_hook::set_once();
    
    log("NAVΛ WASM Preview Engine initialized");
}

/// Log to browser console
#[wasm_bindgen]
pub fn log(message: &str) {
    web_sys::console::log_1(&JsValue::from_str(message));
}

/// Get version information
#[wasm_bindgen]
pub fn version() -> String {
    "NAVΛ WASM Preview Engine v1.0.0".to_string()
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_navigation_path() {
        let mut path = NavigationPath::new();
        path.add_waypoint(0.0, 0.0, 0.0);
        path.add_waypoint(1.0, 1.0, 1.0);
        
        assert_eq!(path.waypoint_count(), 2);
        assert_eq!(path.get_waypoint(0), vec![0.0, 0.0, 0.0]);
    }
    
    #[test]
    fn test_navigate_to_vnc() {
        let path = navigate_to_vnc(0.0, 0.0, 0.0, 10.0, 10.0, 0.0);
        assert!(path.waypoint_count() > 2);
        assert!(path.get_energy() > 0.0);
    }
}

