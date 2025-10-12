/// Core Preview Engine for WebAssembly Execution

pub struct PreviewEngine {
    state: EngineState,
}

#[derive(Debug, Clone)]
pub struct EngineState {
    pub running: bool,
    pub frame_count: u64,
    pub memory_used: usize,
}

impl PreviewEngine {
    pub fn new() -> Self {
        Self {
            state: EngineState {
                running: false,
                frame_count: 0,
                memory_used: 0,
            },
        }
    }
    
    pub fn start(&mut self) {
        self.state.running = true;
    }
    
    pub fn stop(&mut self) {
        self.state.running = false;
    }
    
    pub fn update(&mut self) -> Result<(), String> {
        if self.state.running {
            self.state.frame_count += 1;
        }
        Ok(())
    }
    
    pub fn get_state(&self) -> &EngineState {
        &self.state
    }
}

impl Default for PreviewEngine {
    fn default() -> Self {
        Self::new()
    }
}

