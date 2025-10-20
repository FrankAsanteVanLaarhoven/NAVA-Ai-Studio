//! NAVΛ Compiler
//! Compiles NAVΛ source code to various target languages

use std::collections::HashMap;
use regex::Regex;
use crate::navlambda::types::*;

/// NAVΛ Compiler
pub struct NavLambdaCompiler {
    symbol_table: HashMap<String, Symbol>,
    target_language: TargetLanguage,
}

/// Symbol table entry
#[derive(Debug, Clone)]
pub struct Symbol {
    pub name: String,
    pub symbol_type: SymbolType,
    pub value: Option<String>,
}

/// Symbol types
#[derive(Debug, Clone)]
pub enum SymbolType {
    Variable,
    Function,
    Class,
    Constant,
}

/// Target languages for compilation
#[derive(Debug, Clone)]
pub enum TargetLanguage {
    Rust,
    Python,
    JavaScript,
    Cpp,
    WebAssembly,
}

/// Compilation result
#[derive(Debug)]
pub struct CompilationResult {
    pub code: String,
    pub errors: Vec<String>,
    pub warnings: Vec<String>,
}

impl NavLambdaCompiler {
    /// Create new compiler
    pub fn new(target: TargetLanguage) -> Self {
        NavLambdaCompiler {
            symbol_table: HashMap::new(),
            target_language: target,
        }
    }

    /// Compile NAVΛ source code
    pub fn compile(&mut self, source: &str) -> Result<CompilationResult, String> {
        let mut result = CompilationResult {
            code: String::new(),
            errors: Vec::new(),
            warnings: Vec::new(),
        };

        // Parse the source code
        let ast = self.parse(source)?;

        // Generate target code
        result.code = self.generate_code(&ast)?;

        Ok(result)
    }

    /// Parse NAVΛ source code into AST
    fn parse(&mut self, source: &str) -> Result<ASTNode, String> {
        let lines: Vec<&str> = source.lines().collect();
        let mut ast = ASTNode::Program(Vec::new());

        for (line_num, line) in lines.iter().enumerate() {
            let trimmed = line.trim();
            if trimmed.is_empty() || trimmed.starts_with("//") {
                continue;
            }

            match self.parse_line(trimmed, line_num) {
                Ok(node) => {
                    if let ASTNode::Program(ref mut nodes) = ast {
                        nodes.push(node);
                    }
                }
                Err(e) => return Err(format!("Line {}: {}", line_num + 1, e)),
            }
        }

        Ok(ast)
    }

    /// Parse a single line
    fn parse_line(&mut self, line: &str, line_num: usize) -> Result<ASTNode, String> {
        // Variable declaration: name⋋ ← value
        if line.contains("←") {
            return self.parse_assignment(line);
        }

        // Function call: function⋋(args)
        if line.contains("⋋(") && line.ends_with(")") {
            return self.parse_function_call(line);
        }

        // Print statement: print⋋("message")
        if line.starts_with("print⋋(") {
            return self.parse_print_statement(line);
        }

        // Navigation operation: ⋋(operation)
        if line.starts_with("⋋(") {
            return self.parse_navigation_operation(line);
        }

        // Program declaration
        if line.starts_with("program ") {
            return self.parse_program_declaration(line);
        }

        // Function declaration
        if line.starts_with("function ") {
            return self.parse_function_declaration(line);
        }

        Err(format!("Unknown syntax: {}", line))
    }

    /// Parse variable assignment
    fn parse_assignment(&mut self, line: &str) -> Result<ASTNode, String> {
        let parts: Vec<&str> = line.split("←").collect();
        if parts.len() != 2 {
            return Err("Invalid assignment syntax".to_string());
        }

        let var_name = parts[0].trim().trim_end_matches("⋋");
        let value_expr = parts[1].trim();

        // Add to symbol table
        self.symbol_table.insert(var_name.to_string(), Symbol {
            name: var_name.to_string(),
            symbol_type: SymbolType::Variable,
            value: Some(value_expr.to_string()),
        });

        Ok(ASTNode::Assignment {
            variable: var_name.to_string(),
            value: Box::new(self.parse_expression(value_expr)?),
        })
    }

    /// Parse function call
    fn parse_function_call(&mut self, line: &str) -> Result<ASTNode, String> {
        let re = Regex::new(r"(\w+)⋋\((.*)\)").unwrap();
        if let Some(caps) = re.captures(line) {
            let func_name = caps.get(1).unwrap().as_str();
            let args_str = caps.get(2).unwrap().as_str();

            let args = if args_str.trim().is_empty() {
                Vec::new()
            } else {
                args_str.split(",")
                    .map(|arg| self.parse_expression(arg.trim()))
                    .collect::<Result<Vec<_>, _>>()?
            };

            Ok(ASTNode::FunctionCall {
                name: func_name.to_string(),
                arguments: args,
            })
        } else {
            Err("Invalid function call syntax".to_string())
        }
    }

    /// Parse print statement
    fn parse_print_statement(&mut self, line: &str) -> Result<ASTNode, String> {
        let re = Regex::new(r"print⋋\((.*)\)").unwrap();
        if let Some(caps) = re.captures(line) {
            let content = caps.get(1).unwrap().as_str();
            Ok(ASTNode::Print {
                expression: Box::new(self.parse_expression(content)?),
            })
        } else {
            Err("Invalid print syntax".to_string())
        }
    }

    /// Parse navigation operation
    fn parse_navigation_operation(&mut self, line: &str) -> Result<ASTNode, String> {
        let content = line.trim_start_matches("⋋(").trim_end_matches(")");
        Ok(ASTNode::NavigationOp {
            operation: content.to_string(),
        })
    }

    /// Parse program declaration
    fn parse_program_declaration(&mut self, line: &str) -> Result<ASTNode, String> {
        let name = line.trim_start_matches("program ").trim();
        Ok(ASTNode::ProgramDecl {
            name: name.to_string(),
        })
    }

    /// Parse function declaration
    fn parse_function_declaration(&mut self, line: &str) -> Result<ASTNode, String> {
        let re = Regex::new(r"function (\w+)⋋\((.*)\)\s*->\s*(.+)").unwrap();
        if let Some(caps) = re.captures(line) {
            let name = caps.get(1).unwrap().as_str();
            let params_str = caps.get(2).unwrap().as_str();
            let return_type = caps.get(3).unwrap().as_str();

            let params = if params_str.trim().is_empty() {
                Vec::new()
            } else {
                params_str.split(",")
                    .map(|param| {
                        let parts: Vec<&str> = param.trim().split(":").collect();
                        if parts.len() == 2 {
                            Ok((parts[0].to_string(), parts[1].to_string()))
                        } else {
                            Err("Invalid parameter syntax".to_string())
                        }
                    })
                    .collect::<Result<Vec<_>, _>>()?
            };

            Ok(ASTNode::FunctionDecl {
                name: name.to_string(),
                parameters: params,
                return_type: return_type.to_string(),
                body: Vec::new(), // Would be parsed from subsequent lines
            })
        } else {
            Err("Invalid function declaration syntax".to_string())
        }
    }

    /// Parse expression
    fn parse_expression(&self, expr: &str) -> Result<ASTNode, String> {
        let expr = expr.trim();

        // String literal
        if expr.starts_with("\"") && expr.ends_with("\"") {
            return Ok(ASTNode::StringLiteral(expr[1..expr.len()-1].to_string()));
        }

        // Number literal
        if let Ok(num) = expr.parse::<f64>() {
            return Ok(ASTNode::NumberLiteral(num));
        }

        // Vector3D constructor
        if expr.starts_with("Vector3D⋋(") && expr.ends_with(")") {
            let args_str = &expr[10..expr.len()-1];
            let args: Vec<f64> = args_str.split(",")
                .map(|arg| arg.trim().parse::<f64>())
                .collect::<Result<Vec<_>, _>>()
                .map_err(|_| "Invalid Vector3D arguments".to_string())?;

            if args.len() == 3 {
                return Ok(ASTNode::Vector3DConstructor(args[0], args[1], args[2]));
            }
        }

        // Variable reference
        if self.symbol_table.contains_key(expr.trim_end_matches("⋋")) {
            return Ok(ASTNode::Variable(expr.trim_end_matches("⋋").to_string()));
        }

        // Function call
        if expr.contains("⋋(") && expr.ends_with(")") {
            return self.parse_function_call(expr);
        }

        Err(format!("Unknown expression: {}", expr))
    }

    /// Generate target code from AST
    fn generate_code(&self, ast: &ASTNode) -> Result<String, String> {
        match &self.target_language {
            TargetLanguage::Rust => self.generate_rust_code(ast),
            TargetLanguage::Python => self.generate_python_code(ast),
            TargetLanguage::JavaScript => self.generate_javascript_code(ast),
            TargetLanguage::Cpp => self.generate_cpp_code(ast),
            TargetLanguage::WebAssembly => self.generate_wasm_code(ast),
        }
    }

    /// Generate Rust code
    fn generate_rust_code(&self, ast: &ASTNode) -> Result<String, String> {
        let mut code = String::from("// Generated NAVΛ code\n\n");

        if let ASTNode::Program(nodes) = ast {
            for node in nodes {
                code.push_str(&self.generate_rust_node(node));
                code.push_str("\n");
            }
        }

        Ok(code)
    }

    /// Generate code for a single AST node (Rust)
    fn generate_rust_node(&self, node: &ASTNode) -> String {
        match node {
            ASTNode::Assignment { variable, value } => {
                format!("let {} = {};", variable, self.generate_rust_expr(value))
            }
            ASTNode::FunctionCall { name, arguments } => {
                let args: Vec<String> = arguments.iter()
                    .map(|arg| self.generate_rust_expr(arg))
                    .collect();
                format!("{}({})", name, args.join(", "))
            }
            ASTNode::Print { expression } => {
                format!("println!(\"{{}}\", {});", self.generate_rust_expr(expression))
            }
            ASTNode::NavigationOp { operation } => {
                format!("// Navigation operation: {}", operation)
            }
            _ => "// Unsupported node".to_string(),
        }
    }

    /// Generate Rust expression
    fn generate_rust_expr(&self, node: &ASTNode) -> String {
        match node {
            ASTNode::StringLiteral(s) => format!("\"{}\"", s),
            ASTNode::NumberLiteral(n) => format!("{}", n),
            ASTNode::Variable(v) => v.clone(),
            ASTNode::Vector3DConstructor(x, y, z) => {
                format!("Vector3D::new({}, {}, {})", x, y, z)
            }
            _ => "/* expr */".to_string(),
        }
    }

    /// Generate Python code
    fn generate_python_code(&self, ast: &ASTNode) -> Result<String, String> {
        let mut code = String::from("# Generated NAVΛ code\n\n");

        if let ASTNode::Program(nodes) = ast {
            for node in nodes {
                code.push_str(&self.generate_python_node(node));
                code.push_str("\n");
            }
        }

        Ok(code)
    }

    /// Generate Python node
    fn generate_python_node(&self, node: &ASTNode) -> String {
        match node {
            ASTNode::Assignment { variable, value } => {
                format!("{} = {}", variable, self.generate_python_expr(value))
            }
            ASTNode::Print { expression } => {
                format!("print({})", self.generate_python_expr(expression))
            }
            _ => "# Unsupported node".to_string(),
        }
    }

    /// Generate Python expression
    fn generate_python_expr(&self, node: &ASTNode) -> String {
        match node {
            ASTNode::StringLiteral(s) => format!("\"{}\"", s),
            ASTNode::NumberLiteral(n) => format!("{}", n),
            ASTNode::Variable(v) => v.clone(),
            ASTNode::Vector3DConstructor(x, y, z) => {
                format!("Vector3D({}, {}, {})", x, y, z)
            }
            _ => "/* expr */".to_string(),
        }
    }

    /// Generate JavaScript code
    fn generate_javascript_code(&self, ast: &ASTNode) -> Result<String, String> {
        let mut code = String::from("// Generated NAVΛ code\n\n");

        if let ASTNode::Program(nodes) = ast {
            for node in nodes {
                code.push_str(&self.generate_js_node(node));
                code.push_str("\n");
            }
        }

        Ok(code)
    }

    /// Generate JavaScript node
    fn generate_js_node(&self, node: &ASTNode) -> String {
        match node {
            ASTNode::Assignment { variable, value } => {
                format!("let {} = {};", variable, self.generate_js_expr(value))
            }
            ASTNode::Print { expression } => {
                format!("console.log({});", self.generate_js_expr(expression))
            }
            _ => "// Unsupported node".to_string(),
        }
    }

    /// Generate JavaScript expression
    fn generate_js_expr(&self, node: &ASTNode) -> String {
        match node {
            ASTNode::StringLiteral(s) => format!("\"{}\"", s),
            ASTNode::NumberLiteral(n) => format!("{}", n),
            ASTNode::Variable(v) => v.clone(),
            ASTNode::Vector3DConstructor(x, y, z) => {
                format!("new Vector3D({}, {}, {})", x, y, z)
            }
            _ => "/* expr */".to_string(),
        }
    }

    /// Generate C++ code
    fn generate_cpp_code(&self, ast: &ASTNode) -> Result<String, String> {
        let mut code = String::from("// Generated NAVΛ code\n\n");

        if let ASTNode::Program(nodes) = ast {
            for node in nodes {
                code.push_str(&self.generate_cpp_node(node));
                code.push_str("\n");
            }
        }

        Ok(code)
    }

    /// Generate C++ node
    fn generate_cpp_node(&self, node: &ASTNode) -> String {
        match node {
            ASTNode::Assignment { variable, value } => {
                format!("auto {} = {};", variable, self.generate_cpp_expr(value))
            }
            _ => "// Unsupported node".to_string(),
        }
    }

    /// Generate C++ expression
    fn generate_cpp_expr(&self, node: &ASTNode) -> String {
        match node {
            ASTNode::StringLiteral(s) => format!("\"{}\"", s),
            ASTNode::NumberLiteral(n) => format!("{}", n),
            ASTNode::Variable(v) => v.clone(),
            ASTNode::Vector3DConstructor(x, y, z) => {
                format!("Vector3D({}, {}, {})", x, y, z)
            }
            _ => "/* expr */".to_string(),
        }
    }

    /// Generate WebAssembly code (placeholder)
    fn generate_wasm_code(&self, _ast: &ASTNode) -> Result<String, String> {
        Ok("// WebAssembly generation not implemented".to_string())
    }
}

/// AST Node types
#[derive(Debug)]
pub enum ASTNode {
    Program(Vec<ASTNode>),
    ProgramDecl { name: String },
    FunctionDecl { name: String, parameters: Vec<(String, String)>, return_type: String, body: Vec<ASTNode> },
    Assignment { variable: String, value: Box<ASTNode> },
    FunctionCall { name: String, arguments: Vec<ASTNode> },
    Print { expression: Box<ASTNode> },
    NavigationOp { operation: String },
    StringLiteral(String),
    NumberLiteral(f64),
    Variable(String),
    Vector3DConstructor(f64, f64, f64),
}