use std::collections::HashMap;

/// Advanced Breakpoint Manager for VNC Debugging
pub struct BreakpointManager {
    breakpoints: HashMap<usize, Breakpoint>,
    next_id: usize,
}

#[derive(Debug, Clone)]
pub struct Breakpoint {
    pub id: usize,
    pub line: usize,
    pub column: usize,
    pub condition: Option<String>,
    pub hit_count: usize,
    pub enabled: bool,
    pub breakpoint_type: BreakpointType,
}

#[derive(Debug, Clone, PartialEq)]
pub enum BreakpointType {
    Line,
    Conditional,
    EnergyThreshold(f64),
    NavigationPoint([f64; 3]),
}

impl BreakpointManager {
    pub fn new() -> Self {
        Self {
            breakpoints: HashMap::new(),
            next_id: 1,
        }
    }
    
    pub fn add_breakpoint(&mut self, line: usize, column: usize, bp_type: BreakpointType) -> usize {
        let id = self.next_id;
        self.next_id += 1;
        
        let breakpoint = Breakpoint {
            id,
            line,
            column,
            condition: None,
            hit_count: 0,
            enabled: true,
            breakpoint_type: bp_type,
        };
        
        self.breakpoints.insert(id, breakpoint);
        id
    }
    
    pub fn remove_breakpoint(&mut self, id: usize) -> bool {
        self.breakpoints.remove(&id).is_some()
    }
    
    pub fn toggle_breakpoint(&mut self, id: usize) -> bool {
        if let Some(bp) = self.breakpoints.get_mut(&id) {
            bp.enabled = !bp.enabled;
            true
        } else {
            false
        }
    }
    
    pub fn set_condition(&mut self, id: usize, condition: String) -> bool {
        if let Some(bp) = self.breakpoints.get_mut(&id) {
            bp.condition = Some(condition);
            true
        } else {
            false
        }
    }
    
    pub fn should_break(&mut self, line: usize, energy: f64, position: [f64; 3]) -> Option<usize> {
        for (id, bp) in self.breakpoints.iter_mut() {
            if !bp.enabled {
                continue;
            }
            
            let should_break = match &bp.breakpoint_type {
                BreakpointType::Line => bp.line == line,
                BreakpointType::Conditional => {
                    // Would evaluate condition here
                    bp.line == line
                }
                BreakpointType::EnergyThreshold(threshold) => energy > *threshold,
                BreakpointType::NavigationPoint(target) => {
                    let distance = ((position[0] - target[0]).powi(2) +
                                   (position[1] - target[1]).powi(2) +
                                   (position[2] - target[2]).powi(2)).sqrt();
                    distance < 0.1
                }
            };
            
            if should_break {
                bp.hit_count += 1;
                return Some(*id);
            }
        }
        
        None
    }
    
    pub fn get_breakpoints(&self) -> Vec<&Breakpoint> {
        self.breakpoints.values().collect()
    }
    
    pub fn clear_all(&mut self) {
        self.breakpoints.clear();
    }
}

impl Default for BreakpointManager {
    fn default() -> Self {
        Self::new()
    }
}

