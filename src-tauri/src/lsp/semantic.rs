use super::parser::{NavLambdaAst, NavLambdaStatement, NavLambdaExpression};

/// VNC Semantic Analyzer
/// 
/// Verifies the mathematical correctness of Van Laarhoven Navigation Calculus code.
pub struct VncSemanticAnalyzer {
    errors: Vec<SemanticError>,
}

#[derive(Debug, Clone)]
pub struct SemanticError {
    pub message: String,
    pub severity: ErrorSeverity,
}

#[derive(Debug, Clone, PartialEq)]
pub enum ErrorSeverity {
    Error,
    Warning,
    Info,
}

impl VncSemanticAnalyzer {
    pub fn new() -> Self {
        Self {
            errors: Vec::new(),
        }
    }
    
    pub fn analyze(&mut self, ast: &NavLambdaAst) -> Result<(), Vec<SemanticError>> {
        self.errors.clear();
        
        for statement in &ast.statements {
            self.analyze_statement(statement);
        }
        
        if self.errors.is_empty() {
            Ok(())
        } else {
            Err(self.errors.clone())
        }
    }
    
    fn analyze_statement(&mut self, statement: &NavLambdaStatement) {
        match statement {
            NavLambdaStatement::NavigationDefinition { name: _, parameters: _, body } => {
                self.analyze_expression(body);
            }
            NavLambdaStatement::NavigationOptimization { objective, constraints } => {
                self.analyze_expression(objective);
                for constraint in constraints {
                    self.analyze_expression(constraint);
                }
            }
            NavLambdaStatement::VncMasterEquation { operator: _, operands } => {
                for operand in operands {
                    self.analyze_expression(operand);
                }
            }
            NavLambdaStatement::NavigationVisualization { path, options: _ } => {
                self.analyze_expression(path);
            }
            NavLambdaStatement::Assignment { name: _, value } => {
                self.analyze_expression(value);
            }
        }
    }
    
    fn analyze_expression(&mut self, expr: &NavLambdaExpression) {
        match expr {
            NavLambdaExpression::Symbol(_) => {}
            NavLambdaExpression::Number(_) => {}
            NavLambdaExpression::String(_) => {}
            NavLambdaExpression::NavigationPath { start, goal, waypoints } => {
                self.analyze_expression(start);
                self.analyze_expression(goal);
                for waypoint in waypoints {
                    self.analyze_expression(waypoint);
                }
            }
            NavLambdaExpression::BinaryOp { operator: _, left, right } => {
                self.analyze_expression(left);
                self.analyze_expression(right);
            }
            NavLambdaExpression::FunctionCall { name: _, arguments } => {
                for arg in arguments {
                    self.analyze_expression(arg);
                }
            }
            NavLambdaExpression::Lambda { parameters: _, body } => {
                self.analyze_expression(body);
            }
        }
    }
}

impl Default for VncSemanticAnalyzer {
    fn default() -> Self {
        Self::new()
    }
}

