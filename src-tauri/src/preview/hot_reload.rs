use std::time::Instant;

/// Hot Code Reloading System
pub struct HotReloader {
    last_reload: Instant,
    reload_count: usize,
}

impl HotReloader {
    pub fn new() -> Self {
        Self {
            last_reload: Instant::now(),
            reload_count: 0,
        }
    }
    
    pub fn should_reload(&self) -> bool {
        self.last_reload.elapsed().as_millis() > 100
    }
    
    pub fn reload(&mut self, code: &str) -> Result<(), String> {
        if !self.should_reload() {
            return Ok(());
        }
        
        self.last_reload = Instant::now();
        self.reload_count += 1;
        
        tracing::info!("Hot reload #{}: {} bytes", self.reload_count, code.len());
        
        Ok(())
    }
    
    pub fn get_reload_count(&self) -> usize {
        self.reload_count
    }
}

impl Default for HotReloader {
    fn default() -> Self {
        Self::new()
    }
}

