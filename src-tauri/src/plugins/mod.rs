use std::collections::HashMap;

/// Plugin Architecture and Extension System
/// 
/// Allows extending NAVΛ Studio with custom functionality.
pub struct PluginManager {
    plugins: HashMap<String, Box<dyn Plugin>>,
}

pub trait Plugin: Send + Sync {
    fn name(&self) -> &str;
    fn initialize(&mut self) -> Result<(), String>;
    fn execute(&self, command: &str, args: Vec<String>) -> Result<String, String>;
}

impl PluginManager {
    pub fn new() -> Self {
        Self {
            plugins: HashMap::new(),
        }
    }
    
    pub fn register_plugin(&mut self, plugin: Box<dyn Plugin>) {
        let name = plugin.name().to_string();
        self.plugins.insert(name, plugin);
    }
    
    pub fn get_plugin(&self, name: &str) -> Option<&Box<dyn Plugin>> {
        self.plugins.get(name)
    }
    
    pub fn list_plugins(&self) -> Vec<String> {
        self.plugins.keys().cloned().collect()
    }
}

impl Default for PluginManager {
    fn default() -> Self {
        Self::new()
    }
}

