/// Plugin API Definitions
/// 
/// Public API for plugin development

pub trait PluginContext {
    fn register_command(&mut self, command: &str, handler: Box<dyn Fn(Vec<String>) -> Result<String, String>>);
    fn register_language_feature(&mut self, feature: LanguageFeature);
    fn get_workspace_root(&self) -> String;
}

#[derive(Debug, Clone)]
pub enum LanguageFeature {
    Completion(CompletionProvider),
    Hover(HoverProvider),
    Diagnostic(DiagnosticProvider),
}

#[derive(Debug, Clone)]
pub struct CompletionProvider {
    pub name: String,
    pub trigger_chars: Vec<String>,
}

#[derive(Debug, Clone)]
pub struct HoverProvider {
    pub name: String,
}

#[derive(Debug, Clone)]
pub struct DiagnosticProvider {
    pub name: String,
}

pub trait PluginLifecycle {
    fn on_activate(&mut self, context: &mut dyn PluginContext) -> Result<(), String>;
    fn on_deactivate(&mut self) -> Result<(), String>;
    fn on_config_change(&mut self, config: &str) -> Result<(), String>;
}

