use std::collections::HashMap;
use std::fmt;

/// NAVΛ Parser with Native ⋋ Symbol Support
/// 
/// This parser understands all Van Laarhoven Navigation Calculus symbols
/// and generates an AST optimized for VNC mathematical operations.
pub struct NavLambdaParser {
    pub vnc_symbols: HashMap<String, VncSymbol>,
    pub navigation_operators: Vec<NavigationOperator>,
}

#[derive(Debug, Clone, PartialEq)]
pub enum VncSymbol {
    LambdaNav,           // ⋋
    NavTensor,           // ⊗⋋
    NavSum,              // ⊕⋋
    NavUnion,            // ∪⋋
    NavIntersection,     // ∩⋋
    NavNorth,            // ↑⋋
    NavSouth,            // ↓⋋
    NavEast,             // →⋋
    NavWest,             // ←⋋
    MasterOperator,      // 𝒩ℐ
    EvolutionOperator,   // ℰ
}

#[derive(Debug, Clone)]
pub struct NavigationOperator {
    pub symbol: String,
    pub arity: usize,
    pub precedence: u8,
    pub associativity: Associativity,
}

#[derive(Debug, Clone, PartialEq)]
pub enum Associativity {
    Left,
    Right,
    None,
}

impl NavigationOperator {
    pub fn all() -> Vec<Self> {
        vec![
            NavigationOperator {
                symbol: "⊗⋋".to_string(),
                arity: 2,
                precedence: 10,
                associativity: Associativity::Left,
            },
            NavigationOperator {
                symbol: "⊕⋋".to_string(),
                arity: 2,
                precedence: 9,
                associativity: Associativity::Left,
            },
            NavigationOperator {
                symbol: "∪⋋".to_string(),
                arity: 2,
                precedence: 8,
                associativity: Associativity::Left,
            },
            NavigationOperator {
                symbol: "∩⋋".to_string(),
                arity: 2,
                precedence: 8,
                associativity: Associativity::Left,
            },
        ]
    }
}

#[derive(Debug, Clone)]
pub struct NavLambdaAst {
    pub statements: Vec<NavLambdaStatement>,
}

#[derive(Debug, Clone)]
pub enum NavLambdaStatement {
    NavigationDefinition {
        name: String,
        parameters: Vec<String>,
        body: Box<NavLambdaExpression>,
    },
    NavigationOptimization {
        objective: Box<NavLambdaExpression>,
        constraints: Vec<NavLambdaExpression>,
    },
    VncMasterEquation {
        operator: String,
        operands: Vec<NavLambdaExpression>,
    },
    NavigationVisualization {
        path: Box<NavLambdaExpression>,
        options: VisualizationOptions,
    },
    Assignment {
        name: String,
        value: Box<NavLambdaExpression>,
    },
}

#[derive(Debug, Clone)]
pub enum NavLambdaExpression {
    Symbol(String),
    Number(f64),
    String(String),
    NavigationPath {
        start: Box<NavLambdaExpression>,
        goal: Box<NavLambdaExpression>,
        waypoints: Vec<NavLambdaExpression>,
    },
    BinaryOp {
        operator: String,
        left: Box<NavLambdaExpression>,
        right: Box<NavLambdaExpression>,
    },
    FunctionCall {
        name: String,
        arguments: Vec<NavLambdaExpression>,
    },
    Lambda {
        parameters: Vec<String>,
        body: Box<NavLambdaExpression>,
    },
}

#[derive(Debug, Clone)]
pub struct VisualizationOptions {
    pub show_energy: bool,
    pub show_equations: bool,
    pub animation_speed: f32,
}

impl Default for VisualizationOptions {
    fn default() -> Self {
        Self {
            show_energy: true,
            show_equations: true,
            animation_speed: 1.0,
        }
    }
}

impl NavLambdaParser {
    pub fn with_vnc_symbols() -> Self {
        let mut vnc_symbols = HashMap::new();
        
        vnc_symbols.insert("⋋".to_string(), VncSymbol::LambdaNav);
        vnc_symbols.insert("⊗⋋".to_string(), VncSymbol::NavTensor);
        vnc_symbols.insert("⊕⋋".to_string(), VncSymbol::NavSum);
        vnc_symbols.insert("∪⋋".to_string(), VncSymbol::NavUnion);
        vnc_symbols.insert("∩⋋".to_string(), VncSymbol::NavIntersection);
        vnc_symbols.insert("↑⋋".to_string(), VncSymbol::NavNorth);
        vnc_symbols.insert("↓⋋".to_string(), VncSymbol::NavSouth);
        vnc_symbols.insert("→⋋".to_string(), VncSymbol::NavEast);
        vnc_symbols.insert("←⋋".to_string(), VncSymbol::NavWest);
        vnc_symbols.insert("𝒩ℐ".to_string(), VncSymbol::MasterOperator);
        vnc_symbols.insert("ℰ".to_string(), VncSymbol::EvolutionOperator);
        
        Self {
            vnc_symbols,
            navigation_operators: NavigationOperator::all(),
        }
    }
    
    /// Parse NAVΛ code into an AST
    pub fn parse(&self, code: &str) -> Result<NavLambdaAst, ParseError> {
        let mut statements = Vec::new();
        
        // Simple line-based parsing for MVP
        for line in code.lines() {
            let line = line.trim();
            if line.is_empty() || line.starts_with("//") {
                continue;
            }
            
            if let Some(statement) = self.parse_statement(line)? {
                statements.push(statement);
            }
        }
        
        Ok(NavLambdaAst { statements })
    }
    
    fn parse_statement(&self, line: &str) -> Result<Option<NavLambdaStatement>, ParseError> {
        // Check for assignment
        if let Some((name, value)) = line.split_once('=') {
            let name = name.trim().to_string();
            let expr = self.parse_expression(value.trim())?;
            return Ok(Some(NavLambdaStatement::Assignment {
                name,
                value: Box::new(expr),
            }));
        }
        
        // Check for navigation function
        if line.contains("navigate_to⋋") || line.contains("find_optimal_path⋋") {
            let expr = self.parse_expression(line)?;
            return Ok(Some(NavLambdaStatement::NavigationOptimization {
                objective: Box::new(expr),
                constraints: vec![],
            }));
        }
        
        // Check for VNC master operator
        if line.contains("𝒩ℐ") {
            return Ok(Some(NavLambdaStatement::VncMasterEquation {
                operator: "𝒩ℐ".to_string(),
                operands: vec![],
            }));
        }
        
        Ok(None)
    }
    
    fn parse_expression(&self, expr: &str) -> Result<NavLambdaExpression, ParseError> {
        let expr = expr.trim();
        
        // Try to parse as number
        if let Ok(num) = expr.parse::<f64>() {
            return Ok(NavLambdaExpression::Number(num));
        }
        
        // Try to parse as string
        if expr.starts_with('"') && expr.ends_with('"') {
            return Ok(NavLambdaExpression::String(
                expr[1..expr.len() - 1].to_string()
            ));
        }
        
        // Check for function call
        if let Some(paren_pos) = expr.find('(') {
            let name = expr[..paren_pos].trim().to_string();
            let args_str = &expr[paren_pos + 1..expr.len() - 1];
            let arguments = if args_str.is_empty() {
                vec![]
            } else {
                args_str
                    .split(',')
                    .map(|arg| self.parse_expression(arg.trim()))
                    .collect::<Result<Vec<_>, _>>()?
            };
            
            return Ok(NavLambdaExpression::FunctionCall { name, arguments });
        }
        
        // Check for binary operators
        for op in &self.navigation_operators {
            if let Some(pos) = expr.find(&op.symbol) {
                let left = self.parse_expression(&expr[..pos])?;
                let right = self.parse_expression(&expr[pos + op.symbol.len()..])?;
                return Ok(NavLambdaExpression::BinaryOp {
                    operator: op.symbol.clone(),
                    left: Box::new(left),
                    right: Box::new(right),
                });
            }
        }
        
        // Default: symbol/identifier
        Ok(NavLambdaExpression::Symbol(expr.to_string()))
    }
}

#[derive(Debug, Clone)]
pub struct ParseError {
    pub message: String,
}

impl fmt::Display for ParseError {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "Parse error: {}", self.message)
    }
}

impl std::error::Error for ParseError {}

