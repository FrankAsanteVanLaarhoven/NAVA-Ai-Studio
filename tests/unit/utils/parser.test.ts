import { describe, test, expect, beforeEach, afterEach, jest } from '@jest/globals';
import { VNCParser, parseVNC } from '../../../src/utils/parser';
import { VNCNode } from '../../../src/types/parser';

describe('VNCParser', () => {
  let parser: VNCParser;

  beforeEach(() => {
    parser = new VNCParser();
  });

  afterEach(() => {
    jest.clearAllMocks();
  });

  describe('Basic Parsing', () => {
    test('should parse simple lambda navigation calculus expression', () => {
      const code = '⋋(x, y) = navigate(x, y, energy_function)';
      const result = parser.parse(code);
      
      expect(result.success).toBe(true);
      expect(result.ast).toBeDefined();
      expect(result.ast.type).toBe('navigation_function');
      expect(result.ast.operator).toBe('⋋');
      expect(result.ast.parameters).toEqual(['x', 'y']);
    });

    test('should parse VNC expression with energy function', () => {
      const code = `
⋋(start, goal) = navigate(start, goal, energy_function)
  where energy_function = λ(point) → point.distance + point.obstacle_penalty
      `;
      const result = parser.parse(code);
      
      expect(result.success).toBe(true);
      expect(result.ast).toBeDefined();
      expect(result.ast.energyFunction).toBeDefined();
      expect(result.ast.energyFunction.type).toBe('lambda_expression');
    });

    test('should parse complex VNC program with multiple functions', () => {
      const code = `
⋋(start, end) = navigate(start, end, energy_function)
  where energy_function = λ(p) → p.distance + p.obstacle_penalty

path = ⋋([0, 0], [10, 10])
optimized_path = optimize(path, gradient_descent)

visualize(optimized_path, 3d_renderer)
      `;
      const result = parser.parse(code);
      
      expect(result.success).toBe(true);
      expect(result.ast).toBeDefined();
      expect(result.ast.functions).toHaveLength(4);
    });

    test('should handle empty input gracefully', () => {
      const result = parser.parse('');
      
      expect(result.success).toBe(false);
      expect(result.errors).toContain('Empty input');
    });

    test('should handle whitespace-only input', () => {
      const result = parser.parse('   \n\t  \n   ');
      
      expect(result.success).toBe(false);
      expect(result.errors).toContain('Empty input');
    });
  });

  describe('Lambda Expression Parsing', () => {
    test('should parse simple lambda expression', () => {
      const code = 'λ(x) → x + 1';
      const result = parser.parse(code);
      
      expect(result.success).toBe(true);
      expect(result.ast.type).toBe('lambda_expression');
      expect(result.ast.parameter).toBe('x');
      expect(result.ast.body).toBe('x + 1');
    });

    test('should parse lambda with multiple parameters', () => {
      const code = 'λ(x, y, z) → x + y + z';
      const result = parser.parse(code);
      
      expect(result.success).toBe(true);
      expect(result.ast.parameters).toEqual(['x', 'y', 'z']);
    });

    test('should parse complex lambda with function calls', () => {
      const code = 'λ(point) → distance(point, center) + obstacle_penalty(point)';
      const result = parser.parse(code);
      
      expect(result.success).toBe(true);
      expect(result.ast.parameter).toBe('point');
      expect(result.ast.body).toContain('distance');
      expect(result.ast.body).toContain('obstacle_penalty');
    });

    test('should handle nested lambda expressions', () => {
      const code = 'λ(x) → λ(y) → x + y';
      const result = parser.parse(code);
      
      expect(result.success).toBe(true);
      expect(result.ast.type).toBe('lambda_expression');
      expect(result.ast.body).toContain('λ(y)');
    });
  });

  describe('Navigation Function Parsing', () => {
    test('should parse navigation function with start and end points', () => {
      const code = '⋋([0, 0], [10, 10]) = navigate([0, 0], [10, 10], energy)';
      const result = parser.parse(code);
      
      expect(result.success).toBe(true);
      expect(result.ast.type).toBe('navigation_function');
      expect(result.ast.startPoint).toEqual([0, 0]);
      expect(result.ast.endPoint).toEqual([10, 10]);
    });

    test('should parse navigation function with 3D coordinates', () => {
      const code = '⋋([0, 0, 0], [10, 10, 5]) = navigate([0, 0, 0], [10, 10, 5], energy)';
      const result = parser.parse(code);
      
      expect(result.success).toBe(true);
      expect(result.ast.startPoint).toEqual([0, 0, 0]);
      expect(result.ast.endPoint).toEqual([10, 10, 5]);
    });

    test('should parse navigation function with variable parameters', () => {
      const code = '⋋(start, goal) = navigate(start, goal, energy_function)';
      const result = parser.parse(code);
      
      expect(result.success).toBe(true);
      expect(result.ast.parameters).toEqual(['start', 'goal']);
    });

    test('should parse navigation function with obstacles', () => {
      const code = `
⋋(start, end, obstacles) = navigate(start, end, energy_function)
  where energy_function = λ(p) → ∑(distance(p, obs) for obs in obstacles)
      `;
      const result = parser.parse(code);
      
      expect(result.success).toBe(true);
      expect(result.ast.obstacles).toBeDefined();
      expect(result.ast.energyFunction).toBeDefined();
    });
  });

  describe('Energy Function Parsing', () => {
    test('should parse simple energy function', () => {
      const code = 'energy_function = λ(point) → point.distance';
      const result = parser.parse(code);
      
      expect(result.success).toBe(true);
      expect(result.ast.type).toBe('energy_function');
      expect(result.ast.lambdaExpression).toBeDefined();
      expect(result.ast.lambdaExpression.parameter).toBe('point');
    });

    test('should parse energy function with multiple terms', () => {
      const code = 'energy_function = λ(p) → p.distance + p.obstacle_penalty + p.gradient_cost';
      const result = parser.parse(code);
      
      expect(result.success).toBe(true);
      expect(result.ast.lambdaExpression.body).toContain('distance');
      expect(result.ast.lambdaExpression.body).toContain('obstacle_penalty');
      expect(result.ast.lambdaExpression.body).toContain('gradient_cost');
    });

    test('should parse energy function with conditional logic', () => {
      const code = 'energy_function = λ(p) → p.distance < 2 ? 100 : p.distance';
      const result = parser.parse(code);
      
      expect(result.success).toBe(true);
      expect(result.ast.lambdaExpression.body).toContain('?');
      expect(result.ast.lambdaExpression.body).toContain(':');
    });

    test('should parse energy function with mathematical operations', () => {
      const code = 'energy_function = λ(p) → sqrt(p.x² + p.y²) * weight + bias';
      const result = parser.parse(code);
      
      expect(result.success).toBe(true);
      expect(result.ast.lambdaExpression.body).toContain('sqrt');
      expect(result.ast.lambdaExpression.body).toContain('²');
    });
  });

  describe('Function Call Parsing', () => {
    test('should parse simple function calls', () => {
      const code = 'optimize(path, gradient_descent)';
      const result = parser.parse(code);
      
      expect(result.success).toBe(true);
      expect(result.ast.type).toBe('function_call');
      expect(result.ast.functionName).toBe('optimize');
      expect(result.ast.arguments).toEqual(['path', 'gradient_descent']);
    });

    test('should parse nested function calls', () => {
      const code = 'visualize(optimize(path, gradient_descent), renderer)';
      const result = parser.parse(code);
      
      expect(result.success).toBe(true);
      expect(result.ast.functionName).toBe('visualize');
      expect(result.ast.arguments).toHaveLength(2);
      expect(result.ast.arguments[0].type).toBe('function_call');
    });

    test('should parse function calls with keyword arguments', () => {
      const code = 'render(path, color="red", width=2.0, animated=true)';
      const result = parser.parse(code);
      
      expect(result.success).toBe(true);
      expect(result.ast.functionName).toBe('render');
      expect(result.ast.keywordArguments).toBeDefined();
      expect(result.ast.keywordArguments.color).toBe('red');
      expect(result.ast.keywordArguments.width).toBe(2.0);
      expect(result.ast.keywordArguments.animated).toBe(true);
    });
  });

  describe('Variable Assignment Parsing', () => {
    test('should parse simple variable assignment', () => {
      const code = 'path = ⋋([0, 0], [10, 10])';
      const result = parser.parse(code);
      
      expect(result.success).toBe(true);
      expect(result.ast.type).toBe('assignment');
      expect(result.ast.variable).toBe('path');
      expect(result.ast.value).toBeDefined();
    });

    test('should parse multiple variable assignments', () => {
      const code = `
path = ⋋([0, 0], [10, 10])
optimized_path = optimize(path, gradient_descent)
visualization = render(optimized_path)
      `;
      const result = parser.parse(code);
      
      expect(result.success).toBe(true);
      expect(result.ast.assignments).toHaveLength(3);
      expect(result.ast.assignments[0].variable).toBe('path');
      expect(result.ast.assignments[1].variable).toBe('optimized_path');
      expect(result.ast.assignments[2].variable).toBe('visualization');
    });
  });

  describe('Error Handling', () => {
    test('should detect syntax errors in VNC expressions', () => {
      const code = '⋋(x, y) = navigate(x, y'; // Missing closing parenthesis
      const result = parser.parse(code);
      
      expect(result.success).toBe(false);
      expect(result.errors).toContain('Syntax error');
      expect(result.errors).toContain('Missing closing parenthesis');
    });

    test('should detect invalid lambda expressions', () => {
      const code = 'λ(x)'; // Missing arrow and body
      const result = parser.parse(code);
      
      expect(result.success).toBe(false);
      expect(result.errors).toContain('Invalid lambda expression');
    });

    test('should detect invalid function calls', () => {
      const code = 'optimize('; // Incomplete function call
      const result = parser.parse(code);
      
      expect(result.success).toBe(false);
      expect(result.errors).toContain('Incomplete function call');
    });

    test('should provide helpful error messages', () => {
      const code = '⋋[x, y] = navigate(x, y)'; // Wrong bracket type
      const result = parser.parse(code);
      
      expect(result.success).toBe(false);
      expect(result.errors[0]).toContain('Expected parentheses');
      expect(result.errors[0]).toContain('line 1');
    });

    test('should handle multiple errors in single parse', () => {
      const code = `
⋋(x, y) = navigate(x, y  // Missing closing parenthesis
λ(x)  // Missing arrow and body
optimize(  // Incomplete function call
      `;
      const result = parser.parse(code);
      
      expect(result.success).toBe(false);
      expect(result.errors.length).toBeGreaterThan(1);
    });
  });

  describe('Advanced Features', () => {
    test('should parse list comprehensions', () => {
      const code = 'λ(obstacles) → ∑(distance(p, obs) for obs in obstacles)';
      const result = parser.parse(code);
      
      expect(result.success).toBe(true);
      expect(result.ast.type).toBe('lambda_expression');
      expect(result.ast.body).toContain('for obs in obstacles');
    });

    test('should parse mathematical expressions with operators', () => {
      const code = 'energy = λ(p) → sqrt(p.x² + p.y²) * weight + bias';
      const result = parser.parse(code);
      
      expect(result.success).toBe(true);
      expect(result.ast.body).toContain('sqrt');
      expect(result.ast.body).toContain('²');
      expect(result.ast.body).toContain('*');
      expect(result.ast.body).toContain('+');
    });

    test('should parse conditional expressions', () => {
      const code = 'cost = λ(p) → p.distance < 2 ? 100 : p.distance * 0.5';
      const result = parser.parse(code);
      
      expect(result.success).toBe(true);
      expect(result.ast.body).toContain('?');
      expect(result.ast.body).toContain(':');
    });

    test('should parse function composition', () => {
      const code = 'composed = λ(x) → f(g(h(x)))';
      const result = parser.parse(code);
      
      expect(result.success).toBe(true);
      expect(result.ast.body).toContain('f(g(h(x)))');
    });
  });

  describe('Performance Tests', () => {
    test('should parse large VNC programs efficiently', () => {
      const largeProgram = `
⋋(path1) = navigate(path1, energy1)
⋋(path2) = navigate(path2, energy2)
⋋(path3) = navigate(path3, energy3)
${Array.from({ length: 100 }, (_, i) => `⋋(path${i}) = navigate(path${i}, energy${i})`).join('\n')}
      `;
      
      const startTime = performance.now();
      const result = parser.parse(largeProgram);
      const endTime = performance.now();
      
      expect(result.success).toBe(true);
      expect(endTime - startTime).toBeLessThan(100); // Should parse in less than 100ms
    });

    test('should handle deeply nested expressions efficiently', () => {
      const nestedCode = `
⋋(x) = f(g(h(i(j(k(l(m(n(o(p(q(r(s(t(u(v(w(x)))))))))))))))))
      `;
      
      const startTime = performance.now();
      const result = parser.parse(nestedCode);
      const endTime = performance.now();
      
      expect(result.success).toBe(true);
      expect(endTime - startTime).toBeLessThan(50); // Should parse in less than 50ms
    });
  });

  describe('Edge Cases', () => {
    test('should handle unicode characters in identifiers', () => {
      const code = 'λ(α, β, γ) → α + β + γ';
      const result = parser.parse(code);
      
      expect(result.success).toBe(true);
      expect(result.ast.parameters).toEqual(['α', 'β', 'γ']);
    });

    test('should handle very long identifiers', () => {
      const longName = 'very_long_navigation_function_name_with_many_words';
      const code = `λ(${longName}) → ${longName}.distance`;
      const result = parser.parse(code);
      
      expect(result.success).toBe(true);
      expect(result.ast.parameter).toBe(longName);
    });

    test('should handle mixed line endings', () => {
      const code = '⋋(x, y) = navigate(x, y)\r\nλ(p) → p.distance\noptimize(path)';
      const result = parser.parse(code);
      
      expect(result.success).toBe(true);
      expect(result.ast.functions).toHaveLength(3);
    });

    test('should handle empty lines and comments', () => {
      const code = `
# This is a comment
⋋(x, y) = navigate(x, y)

# Another comment
λ(p) → p.distance

optimize(path)
      `;
      const result = parser.parse(code);
      
      expect(result.success).toBe(true);
      expect(result.ast.functions).toHaveLength(3);
    });
  });

  describe('Semantic Analysis', () => {
    test('should detect undefined variables', () => {
      const code = '⋋(x, y) = navigate(x, y, undefined_function)';
      const result = parser.parse(code);
      
      expect(result.success).toBe(true);
      expect(result.warnings).toContain('Undefined function: undefined_function');
    });

    test('should detect type mismatches', () => {
      const code = '⋋("string", 123) = navigate("string", 123, energy)';
      const result = parser.parse(code);
      
      expect(result.success).toBe(true);
      expect(result.warnings).toContain('Type mismatch: expected array, got string');
    });

    test('should detect unused variables', () => {
      const code = 'λ(x, y, z) → x + y'; // z is unused
      const result = parser.parse(code);
      
      expect(result.success).toBe(true);
      expect(result.warnings).toContain('Unused parameter: z');
    });
  });
});