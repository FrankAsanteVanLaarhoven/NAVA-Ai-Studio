use super::api::*;
use std::collections::HashMap;

/// Plugin Lifecycle Manager
pub struct PluginLifecycleManager {
    plugins: HashMap<String, PluginState>,
}

#[derive(Debug)]
pub struct PluginState {
    pub name: String,
    pub active: bool,
    pub load_time: std::time::Instant,
}

impl PluginLifecycleManager {
    pub fn new() -> Self {
        Self {
            plugins: HashMap::new(),
        }
    }
    
    pub fn load_plugin(&mut self, name: String) -> Result<(), String> {
        let state = PluginState {
            name: name.clone(),
            active: false,
            load_time: std::time::Instant::now(),
        };
        
        self.plugins.insert(name, state);
        Ok(())
    }
    
    pub fn activate_plugin(&mut self, name: &str) -> Result<(), String> {
        if let Some(state) = self.plugins.get_mut(name) {
            state.active = true;
            tracing::info!("Plugin activated: {}", name);
            Ok(())
        } else {
            Err(format!("Plugin not found: {}", name))
        }
    }
    
    pub fn deactivate_plugin(&mut self, name: &str) -> Result<(), String> {
        if let Some(state) = self.plugins.get_mut(name) {
            state.active = false;
            tracing::info!("Plugin deactivated: {}", name);
            Ok(())
        } else {
            Err(format!("Plugin not found: {}", name))
        }
    }
    
    pub fn unload_plugin(&mut self, name: &str) -> Result<(), String> {
        self.plugins.remove(name);
        tracing::info!("Plugin unloaded: {}", name);
        Ok(())
    }
    
    pub fn list_plugins(&self) -> Vec<&PluginState> {
        self.plugins.values().collect()
    }
}

impl Default for PluginLifecycleManager {
    fn default() -> Self {
        Self::new()
    }
}

