/// Navigation Execution Runtime for WebAssembly

pub struct NavigationRuntime {
    paths: Vec<NavigationPath>,
}

#[derive(Debug, Clone)]
pub struct NavigationPath {
    pub id: usize,
    pub waypoints: Vec<[f64; 3]>,
    pub energy: f64,
}

impl NavigationRuntime {
    pub fn new() -> Self {
        Self {
            paths: Vec::new(),
        }
    }
    
    pub fn add_path(&mut self, waypoints: Vec<[f64; 3]>) -> usize {
        let id = self.paths.len();
        let energy = self.calculate_path_energy(&waypoints);
        
        self.paths.push(NavigationPath {
            id,
            waypoints,
            energy,
        });
        
        id
    }
    
    fn calculate_path_energy(&self, waypoints: &[[f64; 3]]) -> f64 {
        let mut energy = 0.0;
        
        for i in 0..waypoints.len() - 1 {
            let dx = waypoints[i + 1][0] - waypoints[i][0];
            let dy = waypoints[i + 1][1] - waypoints[i][1];
            let dz = waypoints[i + 1][2] - waypoints[i][2];
            
            energy += (dx * dx + dy * dy + dz * dz).sqrt();
        }
        
        energy
    }
    
    pub fn get_path(&self, id: usize) -> Option<&NavigationPath> {
        self.paths.get(id)
    }
    
    pub fn clear_paths(&mut self) {
        self.paths.clear();
    }
}

impl Default for NavigationRuntime {
    fn default() -> Self {
        Self::new()
    }
}

