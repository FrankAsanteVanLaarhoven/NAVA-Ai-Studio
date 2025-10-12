use std::collections::HashMap;
use super::{NavLambdaParser, VncSemanticAnalyzer};

/// NAVΛ Language Server - The Heart of NAVΛ Studio
/// 
/// This server provides revolutionary language features for Van Laarhoven Navigation Calculus,
/// including native ⋋ symbol support, mathematical verification, and multi-target compilation.
pub struct NavLambdaLanguageServer {
    pub parser: NavLambdaParser,
    pub semantic_analyzer: VncSemanticAnalyzer,
    symbols: HashMap<String, SymbolInfo>,
}

#[derive(Debug, Clone)]
pub struct SymbolInfo {
    pub name: String,
    pub kind: SymbolKind,
    pub documentation: String,
    pub vnc_type: VncType,
}

#[derive(Debug, Clone, PartialEq)]
pub enum SymbolKind {
    NavigationFunction,
    NavigationOperator,
    VncSymbol,
    MasterOperator,
    EvolutionOperator,
    Variable,
    Type,
}

#[derive(Debug, Clone, PartialEq)]
pub enum VncType {
    NavigationPath,
    EnergyLandscape,
    OptimizationOperator,
    QuantumState,
    ConsciousnessField,
    TensorProduct,
    Unknown,
}

impl NavLambdaLanguageServer {
    pub fn new() -> Self {
        let mut symbols = HashMap::new();
        
        // Register all VNC symbols
        symbols.insert("⋋".to_string(), SymbolInfo {
            name: "⋋".to_string(),
            kind: SymbolKind::VncSymbol,
            documentation: "Lambda Navigation - Core VNC navigation operator".to_string(),
            vnc_type: VncType::NavigationPath,
        });
        
        symbols.insert("⊗⋋".to_string(), SymbolInfo {
            name: "⊗⋋".to_string(),
            kind: SymbolKind::NavigationOperator,
            documentation: "Navigation Tensor Product - Combines navigation spaces".to_string(),
            vnc_type: VncType::TensorProduct,
        });
        
        symbols.insert("⊕⋋".to_string(), SymbolInfo {
            name: "⊕⋋".to_string(),
            kind: SymbolKind::NavigationOperator,
            documentation: "Navigation Sum - Parallel navigation paths".to_string(),
            vnc_type: VncType::NavigationPath,
        });
        
        symbols.insert("∪⋋".to_string(), SymbolInfo {
            name: "∪⋋".to_string(),
            kind: SymbolKind::NavigationOperator,
            documentation: "Navigation Union - Merge navigation spaces".to_string(),
            vnc_type: VncType::NavigationPath,
        });
        
        symbols.insert("∩⋋".to_string(), SymbolInfo {
            name: "∩⋋".to_string(),
            kind: SymbolKind::NavigationOperator,
            documentation: "Navigation Intersection - Common navigation paths".to_string(),
            vnc_type: VncType::NavigationPath,
        });
        
        symbols.insert("𝒩ℐ".to_string(), SymbolInfo {
            name: "𝒩ℐ".to_string(),
            kind: SymbolKind::MasterOperator,
            documentation: "Master Navigation Operator - Ultimate VNC optimization".to_string(),
            vnc_type: VncType::OptimizationOperator,
        });
        
        symbols.insert("ℰ".to_string(), SymbolInfo {
            name: "ℰ".to_string(),
            kind: SymbolKind::EvolutionOperator,
            documentation: "Evolution Operator - Time evolution of navigation states".to_string(),
            vnc_type: VncType::QuantumState,
        });
        
        // Navigation functions
        symbols.insert("navigate_to⋋".to_string(), SymbolInfo {
            name: "navigate_to⋋".to_string(),
            kind: SymbolKind::NavigationFunction,
            documentation: "Navigate to a target point using VNC optimization".to_string(),
            vnc_type: VncType::NavigationPath,
        });
        
        symbols.insert("find_optimal_path⋋".to_string(), SymbolInfo {
            name: "find_optimal_path⋋".to_string(),
            kind: SymbolKind::NavigationFunction,
            documentation: "Find the globally optimal navigation path".to_string(),
            vnc_type: VncType::NavigationPath,
        });
        
        symbols.insert("energy_landscape⋋".to_string(), SymbolInfo {
            name: "energy_landscape⋋".to_string(),
            kind: SymbolKind::NavigationFunction,
            documentation: "Compute the energy landscape for navigation".to_string(),
            vnc_type: VncType::EnergyLandscape,
        });
        
        Self {
            parser: NavLambdaParser::with_vnc_symbols(),
            semantic_analyzer: VncSemanticAnalyzer::new(),
            symbols,
        }
    }
    
    /// Get code completions at the given position
    pub fn get_completions(&self, code: &str, position: usize) -> Vec<String> {
        let mut completions = Vec::new();
        
        // Get context at cursor position
        let context = self.get_context_at_position(code, position);
        
        // Add VNC symbol completions
        if context.expecting_symbol {
            completions.extend(vec![
                "⋋".to_string(),
                "⊗⋋".to_string(),
                "⊕⋋".to_string(),
                "∪⋋".to_string(),
                "∩⋋".to_string(),
                "𝒩ℐ".to_string(),
                "ℰ".to_string(),
            ]);
        }
        
        // Add function completions
        if context.expecting_function {
            completions.extend(vec![
                "navigate_to⋋".to_string(),
                "find_optimal_path⋋".to_string(),
                "energy_landscape⋋".to_string(),
            ]);
        }
        
        // Add directional navigation completions
        if context.expecting_direction {
            completions.extend(vec![
                "↑⋋".to_string(),
                "↓⋋".to_string(),
                "→⋋".to_string(),
                "←⋋".to_string(),
            ]);
        }
        
        completions
    }
    
    /// Get hover information at the given position
    pub fn get_hover_info(&self, code: &str, position: usize) -> String {
        let symbol = self.get_symbol_at_position(code, position);
        
        if let Some(info) = self.symbols.get(&symbol) {
            format!(
                "**{}** ({})\n\n{}\n\nType: {:?}",
                info.name, 
                format!("{:?}", info.kind),
                info.documentation,
                info.vnc_type
            )
        } else {
            "Unknown symbol".to_string()
        }
    }
    
    fn get_context_at_position(&self, code: &str, position: usize) -> CompletionContext {
        // Simple heuristic - in production this would use the AST
        let before = &code[..position.min(code.len())];
        
        CompletionContext {
            expecting_symbol: before.ends_with("⋋") || before.ends_with(" "),
            expecting_function: before.ends_with(".") || before.contains("let "),
            expecting_direction: before.contains("direction") || before.contains("move"),
        }
    }
    
    fn get_symbol_at_position(&self, code: &str, position: usize) -> String {
        // Simple word extraction - in production this would use the AST
        let start = code[..position].rfind(|c: char| c.is_whitespace()).unwrap_or(0);
        let end = code[position..].find(|c: char| c.is_whitespace()).map(|i| position + i).unwrap_or(code.len());
        
        code[start..end].trim().to_string()
    }
}

#[derive(Debug)]
struct CompletionContext {
    expecting_symbol: bool,
    expecting_function: bool,
    expecting_direction: bool,
}

impl Default for NavLambdaLanguageServer {
    fn default() -> Self {
        Self::new()
    }
}

