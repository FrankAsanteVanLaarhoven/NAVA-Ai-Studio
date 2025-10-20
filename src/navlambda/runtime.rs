//! NAVΛ Runtime Interpreter
//! Executes NAVΛ code directly

use std::collections::HashMap;
use crate::navlambda::types::*;
use crate::navlambda::compiler::ASTNode;

/// NAVΛ Runtime Interpreter
pub struct NavLambdaRuntime {
    variables: HashMap<String, RuntimeValue>,
    functions: HashMap<String, FunctionDefinition>,
    call_stack: Vec<CallFrame>,
}

/// Runtime value types
#[derive(Debug, Clone)]
pub enum RuntimeValue {
    Number(f64),
    String(String),
    Vector3D(Vector3D),
    Point7D(Point7D),
    NavigationPath(NavigationPath),
    Boolean(bool),
    Null,
}

/// Function definition
#[derive(Debug, Clone)]
pub struct FunctionDefinition {
    pub name: String,
    pub parameters: Vec<String>,
    pub body: Vec<ASTNode>,
}

/// Call frame for function calls
#[derive(Debug, Clone)]
pub struct CallFrame {
    pub function_name: String,
    pub local_variables: HashMap<String, RuntimeValue>,
    pub return_address: usize,
}

impl NavLambdaRuntime {
    /// Create new runtime
    pub fn new() -> Self {
        let mut runtime = NavLambdaRuntime {
            variables: HashMap::new(),
            functions: HashMap::new(),
            call_stack: Vec::new(),
        };

        // Initialize built-in functions
        runtime.initialize_builtins();

        runtime
    }

    /// Initialize built-in functions
    fn initialize_builtins(&mut self) {
        // Navigation functions
        self.functions.insert("navigate⋋".to_string(), FunctionDefinition {
            name: "navigate⋋".to_string(),
            parameters: vec!["start".to_string(), "goal".to_string()],
            body: Vec::new(), // Built-in implementation
        });

        self.functions.insert("distance⋋".to_string(), FunctionDefinition {
            name: "distance⋋".to_string(),
            parameters: vec!["a".to_string(), "b".to_string()],
            body: Vec::new(),
        });

        self.functions.insert("normalize⋋".to_string(), FunctionDefinition {
            name: "normalize⋋".to_string(),
            parameters: vec!["vector".to_string()],
            body: Vec::new(),
        });
    }

    /// Execute NAVΛ program
    pub fn execute(&mut self, ast: &ASTNode) -> Result<RuntimeValue, String> {
        match ast {
            ASTNode::Program(nodes) => {
                let mut result = RuntimeValue::Null;
                for node in nodes {
                    result = self.execute_node(node)?;
                }
                Ok(result)
            }
            _ => self.execute_node(ast),
        }
    }

    /// Execute a single AST node
    fn execute_node(&mut self, node: &ASTNode) -> Result<RuntimeValue, String> {
        match node {
            ASTNode::Assignment { variable, value } => {
                let val = self.evaluate_expression(value)?;
                self.variables.insert(variable.clone(), val);
                Ok(RuntimeValue::Null)
            }
            ASTNode::FunctionCall { name, arguments } => {
                self.execute_function_call(name, arguments)
            }
            ASTNode::Print { expression } => {
                let value = self.evaluate_expression(expression)?;
                println!("{}", self.format_value(&value));
                Ok(RuntimeValue::Null)
            }
            ASTNode::NavigationOp { operation } => {
                self.execute_navigation_operation(operation)
            }
            ASTNode::FunctionDecl { name, parameters, body, .. } => {
                let params: Vec<String> = parameters.iter().map(|(name, _)| name.clone()).collect();
                self.functions.insert(name.clone(), FunctionDefinition {
                    name: name.clone(),
                    parameters: params,
                    body: body.clone(),
                });
                Ok(RuntimeValue::Null)
            }
            _ => Ok(RuntimeValue::Null),
        }
    }

    /// Evaluate expression
    fn evaluate_expression(&mut self, expr: &ASTNode) -> Result<RuntimeValue, String> {
        match expr {
            ASTNode::NumberLiteral(n) => Ok(RuntimeValue::Number(*n)),
            ASTNode::StringLiteral(s) => Ok(RuntimeValue::String(s.clone())),
            ASTNode::Variable(name) => {
                self.variables.get(name)
                    .cloned()
                    .ok_or_else(|| format!("Undefined variable: {}", name))
            }
            ASTNode::Vector3DConstructor(x, y, z) => {
                Ok(RuntimeValue::Vector3D(Vector3D::new(*x, *y, *z)))
            }
            ASTNode::FunctionCall { name, arguments } => {
                self.execute_function_call(name, arguments)
            }
            _ => Err(format!("Cannot evaluate expression: {:?}", expr)),
        }
    }

    /// Execute function call
    fn execute_function_call(&mut self, name: &str, arguments: &[ASTNode]) -> Result<RuntimeValue, String> {
        // Evaluate arguments
        let args: Vec<RuntimeValue> = arguments.iter()
            .map(|arg| self.evaluate_expression(arg))
            .collect::<Result<Vec<_>, _>>()?;

        // Handle built-in functions
        match name {
            "navigate⋋" => self.builtin_navigate(&args),
            "distance⋋" => self.builtin_distance(&args),
            "normalize⋋" => self.builtin_normalize(&args),
            "add⋋" => self.builtin_add(&args),
            "multiply⋋" => self.builtin_multiply(&args),
            _ => {
                // User-defined function - clone the function definition first
                if let Some(func_def) = self.functions.get(name).cloned() {
                    self.execute_user_function(&func_def, &args)
                } else {
                    Err(format!("Unknown function: {}", name))
                }
            }
        }
    }

    /// Execute navigation operation
    fn execute_navigation_operation(&mut self, operation: &str) -> Result<RuntimeValue, String> {
        // Parse navigation operation
        // This is a simplified implementation
        match operation {
            "forward" => {
                // Move forward in navigation space
                Ok(RuntimeValue::String("Navigated forward".to_string()))
            }
            "turn_left" => {
                Ok(RuntimeValue::String("Turned left".to_string()))
            }
            "turn_right" => {
                Ok(RuntimeValue::String("Turned right".to_string()))
            }
            _ => Err(format!("Unknown navigation operation: {}", operation)),
        }
    }

    /// Execute user-defined function
    fn execute_user_function(&mut self, func_def: &FunctionDefinition, args: &[RuntimeValue]) -> Result<RuntimeValue, String> {
        if args.len() != func_def.parameters.len() {
            return Err(format!("Function {} expects {} arguments, got {}",
                func_def.name, func_def.parameters.len(), args.len()));
        }

        // Create new call frame
        let mut local_vars = HashMap::new();
        for (param, arg) in func_def.parameters.iter().zip(args.iter()) {
            local_vars.insert(param.clone(), arg.clone());
        }

        let frame = CallFrame {
            function_name: func_def.name.clone(),
            local_variables: local_vars,
            return_address: 0, // Not used in this simple implementation
        };

        self.call_stack.push(frame);

        // Execute function body
        let mut result = RuntimeValue::Null;
        for node in &func_def.body {
            result = self.execute_node(node)?;
        }

        // Pop call frame
        self.call_stack.pop();

        Ok(result)
    }

    /// Built-in navigate function
    fn builtin_navigate(&self, args: &[RuntimeValue]) -> Result<RuntimeValue, String> {
        if args.len() != 2 {
            return Err("navigate⋋ expects 2 arguments".to_string());
        }

        // Simplified navigation - just return a path
        let path = NavigationPath::new();
        Ok(RuntimeValue::NavigationPath(path))
    }

    /// Built-in distance function
    fn builtin_distance(&self, args: &[RuntimeValue]) -> Result<RuntimeValue, String> {
        if args.len() != 2 {
            return Err("distance⋋ expects 2 arguments".to_string());
        }

        match (&args[0], &args[1]) {
            (RuntimeValue::Vector3D(a), RuntimeValue::Vector3D(b)) => {
                let diff = *a - *b;
                Ok(RuntimeValue::Number(diff.magnitude()))
            }
            (RuntimeValue::Point7D(a), RuntimeValue::Point7D(b)) => {
                Ok(RuntimeValue::Number(a.distance(b)))
            }
            _ => Err("distance⋋ requires two vectors or points".to_string()),
        }
    }

    /// Built-in normalize function
    fn builtin_normalize(&self, args: &[RuntimeValue]) -> Result<RuntimeValue, String> {
        if args.len() != 1 {
            return Err("normalize⋋ expects 1 argument".to_string());
        }

        match &args[0] {
            RuntimeValue::Vector3D(v) => {
                Ok(RuntimeValue::Vector3D(v.normalize()))
            }
            _ => Err("normalize⋋ requires a vector".to_string()),
        }
    }

    /// Built-in add function
    fn builtin_add(&self, args: &[RuntimeValue]) -> Result<RuntimeValue, String> {
        if args.len() != 2 {
            return Err("add⋋ expects 2 arguments".to_string());
        }

        match (&args[0], &args[1]) {
            (RuntimeValue::Number(a), RuntimeValue::Number(b)) => {
                Ok(RuntimeValue::Number(a + b))
            }
            (RuntimeValue::Vector3D(a), RuntimeValue::Vector3D(b)) => {
                Ok(RuntimeValue::Vector3D(a.nav_add(b)))
            }
            _ => Err("add⋋ requires two numbers or two vectors".to_string()),
        }
    }

    /// Built-in multiply function
    fn builtin_multiply(&self, args: &[RuntimeValue]) -> Result<RuntimeValue, String> {
        if args.len() != 2 {
            return Err("multiply⋋ expects 2 arguments".to_string());
        }

        match (&args[0], &args[1]) {
            (RuntimeValue::Number(a), RuntimeValue::Number(b)) => {
                Ok(RuntimeValue::Number(a * b))
            }
            (RuntimeValue::Vector3D(v), RuntimeValue::Number(s)) => {
                Ok(RuntimeValue::Vector3D(v.nav_mul(*s)))
            }
            _ => Err("multiply⋋ requires number * number or vector * scalar".to_string()),
        }
    }

    /// Format runtime value for display
    fn format_value(&self, value: &RuntimeValue) -> String {
        match value {
            RuntimeValue::Number(n) => format!("{}", n),
            RuntimeValue::String(s) => s.clone(),
            RuntimeValue::Vector3D(v) => format!("Vector3D({}, {}, {})", v.x, v.y, v.z),
            RuntimeValue::Point7D(p) => format!("Point7D({}, {}, {}, {}, {}, {}, {})",
                p.x, p.y, p.z, p.t, p.goal, p.intention, p.consciousness),
            RuntimeValue::NavigationPath(p) => format!("NavigationPath({} waypoints, cost: {})",
                p.waypoints.len(), p.energy_cost),
            RuntimeValue::Boolean(b) => format!("{}", b),
            RuntimeValue::Null => "null".to_string(),
        }
    }

    /// Get variable value
    pub fn get_variable(&self, name: &str) -> Option<&RuntimeValue> {
        self.variables.get(name)
    }

    /// Set variable value
    pub fn set_variable(&mut self, name: String, value: RuntimeValue) {
        self.variables.insert(name, value);
    }
}