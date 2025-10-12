use std::collections::HashMap;
use std::path::PathBuf;

/// Plugin Discovery and Registry
pub struct PluginRegistry {
    registered_plugins: HashMap<String, PluginMetadata>,
    plugin_paths: Vec<PathBuf>,
}

#[derive(Debug, Clone)]
pub struct PluginMetadata {
    pub name: String,
    pub version: String,
    pub author: String,
    pub description: String,
    pub entry_point: PathBuf,
}

impl PluginRegistry {
    pub fn new() -> Self {
        Self {
            registered_plugins: HashMap::new(),
            plugin_paths: Vec::new(),
        }
    }
    
    pub fn add_plugin_path(&mut self, path: PathBuf) {
        self.plugin_paths.push(path);
    }
    
    pub fn scan_plugins(&mut self) -> Result<Vec<PluginMetadata>, String> {
        let mut discovered = Vec::new();
        
        for path in &self.plugin_paths {
            // In production: scan directory for plugin manifests
            tracing::info!("Scanning for plugins in: {:?}", path);
        }
        
        Ok(discovered)
    }
    
    pub fn register_plugin(&mut self, metadata: PluginMetadata) {
        tracing::info!("Registering plugin: {}", metadata.name);
        self.registered_plugins.insert(metadata.name.clone(), metadata);
    }
    
    pub fn get_plugin(&self, name: &str) -> Option<&PluginMetadata> {
        self.registered_plugins.get(name)
    }
    
    pub fn list_plugins(&self) -> Vec<&PluginMetadata> {
        self.registered_plugins.values().collect()
    }
}

impl Default for PluginRegistry {
    fn default() -> Self {
        Self::new()
    }
}

