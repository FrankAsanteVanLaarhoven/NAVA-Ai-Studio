import { describe, test, expect, beforeEach, afterEach, vi } from 'vitest';
import { compiler } from '../../src/api/compiler';

describe('Multi-Target Compilation E2E Tests', () => {
  let page: any;

  test.beforeEach(async ({ page: testPage }) => {
    page = testPage;
    await page.goto('http://localhost:3000');
    await page.waitForLoadState('networkidle');
  });

  test.describe('VNC to C++ Compilation', () => {
    test('should compile simple navigation function to C++', async () => {
      // Create VNC code
      await page.waitForSelector('.monaco-editor');
      await page.click('.monaco-editor');
      await page.keyboard.type('⋋(x, y) = navigate(x, y, energy)');
      
      // Select C++ compilation
      await page.click('[data-testid="compile-button"]');
      await page.click('text=C++');
      
      // Wait for compilation
      await page.waitForSelector('[data-testid="cpp-output"]');
      
      // Verify C++ output
      const cppOutput = await page.locator('[data-testid="cpp-output"]').textContent();
      expect(cppOutput).toContain('#include <iostream>');
      expect(cppOutput).toContain('#include <vector>');
      expect(cppOutput).toContain('#include <cmath>');
      expect(cppOutput).toContain('class NavigationFunction');
      expect(cppOutput).toContain('double navigate');
      expect(cppOutput).toContain('double energy');
      expect(cppOutput).toContain('std::vector<double>');
    });

    test('should compile complex VNC with energy function to C++', async () => {
      // Create complex VNC code
      await page.waitForSelector('.monaco-editor');
      await page.click('.monaco-editor');
      await page.keyboard.type(`
⋋(start, end) = navigate(start, end, energy_function)
  where energy_function = λ(p) → p.distance + p.obstacle_penalty

path = ⋋([0, 0], [10, 10])
optimized_path = optimize(path, gradient_descent)
      `);
      
      // Select C++ compilation
      await page.click('[data-testid="compile-button"]');
      await page.click('text=C++');
      
      // Wait for compilation
      await page.waitForSelector('[data-testid="cpp-output"]');
      
      // Verify C++ output
      const cppOutput = await page.locator('[data-testid="cpp-output"]').textContent();
      expect(cppOutput).toContain('std::function<double(const std::vector<double>&)>');
      expect(cppOutput).toContain('auto energy_function = [](const std::vector<double>& p)');
      expect(cppOutput).toContain('lambda_expression');
      expect(cppOutput).toContain('gradient_descent');
    });

    test('should compile VNC with 3D coordinates to C++', async () => {
      // Create VNC code with 3D coordinates
      await page.waitForSelector('.monaco-editor');
      await page.click('.monaco-editor');
      await page.keyboard.type('⋋([0, 0, 0], [10, 10, 5]) = navigate([0, 0, 0], [10, 10, 5], energy)');
      
      // Select C++ compilation
      await page.click('[data-testid="compile-button"]');
      await page.click('text=C++');
      
      // Wait for compilation
      await page.waitForSelector('[data-testid="cpp-output"]');
      
      // Verify C++ output
      const cppOutput = await page.locator('[data-testid="cpp-output"]').textContent();
      expect(cppOutput).toContain('std::array<double, 3>');
      expect(cppOutput).toContain('navigate(const std::array<double, 3>& start');
      expect(cppOutput).toContain('const std::array<double, 3>& end');
    });

    test('should handle C++ compilation errors gracefully', async () => {
      // Create VNC code with syntax error
      await page.waitForSelector('.monaco-editor');
      await page.click('.monaco-editor');
      await page.keyboard.type('⋋(x, y) = invalid_function(x, y'); // Missing closing parenthesis
      
      // Select C++ compilation
      await page.click('[data-testid="compile-button"]');
      await page.click('text=C++');
      
      // Wait for error
      await page.waitForSelector('[data-testid="compilation-error"]');
      
      // Verify error message
      const errorMessage = await page.locator('[data-testid="compilation-error"]').textContent();
      expect(errorMessage).toContain('Compilation failed');
      expect(errorMessage).toContain('Syntax error');
    });
  });

  test.describe('VNC to Python Compilation', () => {
    test('should compile simple navigation function to Python', async () => {
      // Create VNC code
      await page.waitForSelector('.monaco-editor');
      await page.click('.monaco-editor');
      await page.keyboard.type('⋋(x, y) = navigate(x, y, energy)');
      
      // Select Python compilation
      await page.click('[data-testid="compile-button"]');
      await page.click('text=Python');
      
      // Wait for compilation
      await page.waitForSelector('[data-testid="python-output"]');
      
      // Verify Python output
      const pythonOutput = await page.locator('[data-testid="python-output"]').textContent();
      expect(pythonOutput).toContain('def navigate(x, y, energy):');
      expect(pythonOutput).toContain('def energy_function(');
      expect(pythonOutput).toContain('import numpy as np');
      expect(pythonOutput).toContain('from typing import');
    });

    test('should compile VNC with lambda expressions to Python', async () => {
      // Create VNC code with lambda
      await page.waitForSelector('.monaco-editor');
      await page.click('.monaco-editor');
      await page.keyboard.type(`
⋋(start, end) = navigate(start, end, energy_function)
  where energy_function = λ(p) → p.distance + p.obstacle_penalty
      `);
      
      // Select Python compilation
      await page.click('[data-testid="compile-button"]');
      await page.click('text=Python');
      
      // Wait for compilation
      await page.waitForSelector('[data-testid="python-output"]');
      
      // Verify Python output
      const pythonOutput = await page.locator('[data-testid="python-output"]').textContent();
      expect(pythonOutput).toContain('lambda p:');
      expect(pythonOutput).toContain('p.distance + p.obstacle_penalty');
      expect(pythonOutput).toContain('energy_function = lambda p:');
    });

    test('should compile VNC with mathematical functions to Python', async () => {
      // Create VNC code with math functions
      await page.waitForSelector('.monaco-editor');
      await page.click('.monaco-editor');
      await page.keyboard.type('⋋(x, y) = navigate(x, y, λ(p) → sqrt(p.x² + p.y²))');
      
      // Select Python compilation
      await page.click('[data-testid="compile-button"]');
      await page.click('text=Python');
      
      // Wait for compilation
      await page.waitForSelector('[data-testid="python-output"]');

      // Verify Python output
      const pythonOutput = await page.locator('[data-testid="python-output"]').textContent();
      expect(pythonOutput).toContain('import math');
      expect(pythonOutput).toContain('math.sqrt');
      expect(pythonOutput).toContain('** 2');  // Python exponentiation
    });
  });

  test.describe('VNC to WebAssembly Compilation', () => {
    test('should compile navigation function to WebAssembly', async () => {
      // Create VNC code
      await page.waitForSelector('.monaco-editor');
      await page.click('.monaco-editor');
      await page.keyboard.type('⋋(x, y) = navigate(x, y, energy)');
      
      // Select WebAssembly compilation
      await page.click('[data-testid="compile-button"]');
      await page.click('text=WebAssembly');
      
      // Wait for compilation
      await page.waitForSelector('[data-testid="wasm-output"]');
      
      // Verify WebAssembly output
      const wasmOutput = await page.locator('[data-testid="wasm-output"]').textContent();
      expect(wasmOutput).toContain('(module');
      expect(wasmOutput).toContain('(func');
      expect(wasmOutput).toContain('(export "navigate"');
      expect(wasmOutput).toContain('(param');
      expect(wasmOutput).toContain('(result');
    });

    test('should compile VNC with multiple functions to WebAssembly', async () => {
      // Create VNC code with multiple functions
      await page.waitForSelector('.monaco-editor');
      await page.click('.monaco-editor');
      await page.keyboard.type(`
⋋(x, y) = navigate(x, y, energy_function)
  where energy_function = λ(p) → p.distance + p.obstacle_penalty

path = ⋋([0, 0], [10, 10])
optimized = optimize(path, gradient_descent)
      `);
      
      // Select WebAssembly compilation
      await page.click('[data-testid="compile-button"]');
      await page.click('text=WebAssembly');
      
      // Wait for compilation
      await page.waitForSelector('[data-testid="wasm-output"]');
      
      // Verify WebAssembly output
      const wasmOutput = await page.locator('[data-testid="wasm-output"]').textContent();
      expect(wasmOutput).toContain('(func $navigate');
      expect(wasmOutput).toContain('(func $energy_function');
      expect(wasmOutput).toContain('(func $optimize');
      expect(wasmOutput).toContain('(func $gradient_descent');
    });
  });

  test.describe('VNC to GLSL Shader Compilation', () => {
    test('should compile navigation function to GLSL shader', async () => {
      // Create VNC code
      await page.waitForSelector('.monaco-editor');
      await page.click('.monaco-editor');
      await page.keyboard.type('⋋(position, target) = navigate(position, target, energy_field)');
      
      // Select GLSL compilation
      await page.click('[data-testid="compile-button"]');
      await page.click('text=GLSL Shader');
      
      // Wait for compilation
      await page.waitForSelector('[data-testid="glsl-output"]');
      
      // Verify GLSL output
      const glslOutput = await page.locator('[data-testid="glsl-output"]').textContent();
      expect(glslOutput).toContain('#version 300 es');
      expect(glslOutput).toContain('uniform vec3');
      expect(glslOutput).toContain('void main()');
      expect(glslOutput).toContain('navigate(vec3 position, vec3 target)');
    });

    test('should compile VNC with energy visualization to GLSL', async () => {
      // Create VNC code for energy visualization
      await page.waitForSelector('.monaco-editor');
      await page.click('.monaco-editor');
      await page.keyboard.type(`
⋋(position, target) = navigate(position, target, energy_field)
  where energy_field = λ(p) → length(p - target) * obstacle_penalty(p)

visualize(energy_field, color_map)
      `);
      
      // Select GLSL compilation
      await page.click('[data-testid="compile-button"]');
      await page.click('text=GLSL Shader');
      
      // Wait for compilation
      await page.waitForSelector('[data-testid="glsl-output"]');
      
      // Verify GLSL output
      const glslOutput = await page.locator('[data-testid="glsl-output"]').textContent();
      expect(glslOutput).toContain('uniform sampler2D color_map');
      expect(glslOutput).toContain('vec4 energy_color = texture(color_map, energy_value)');
      expect(glslOutput).toContain('gl_FragColor = energy_color');
    });
  });

  test.describe('VNC to Rust Compilation', () => {
    test('should compile navigation function to Rust', async () => {
      // Create VNC code
      await page.waitForSelector('.monaco-editor');
      await page.click('.monaco-editor');
      await page.keyboard.type('⋋(x, y) = navigate(x, y, energy)');
      
      // Select Rust compilation
      await page.click('[data-testid="compile-button"]');
      await page.click('text=Rust');
      
      // Wait for compilation
      await page.waitForSelector('[data-testid="rust-output"]');
      
      // Verify Rust output
      const rustOutput = await page.locator('[data-testid="rust-output"]').textContent();
      expect(rustOutput).toContain('fn navigate(x: f64, y: f64, energy: f64)');
      expect(rustOutput).toContain('pub fn');
      expect(rustOutput).toContain('use std::');
    });

    test('should compile VNC with complex types to Rust', async () => {
      // Create VNC code with complex types
      await page.waitForSelector('.monaco-editor');
      await page.click('.monaco-editor');
      await page.keyboard.type('⋋(start: [f64; 3], end: [f64; 3]) = navigate(start, end, energy)');
      
      // Select Rust compilation
      await page.click('[data-testid="compile-button"]');
      await page.click('text=Rust');
      
      // Wait for compilation
      await page.waitForSelector('[data-testid="rust-output"]');
      
      // Verify Rust output
      const rustOutput = await page.locator('[data-testid="rust-output"]').textContent();
      expect(rustOutput).toContain('[f64; 3]');
      expect(rustOutput).toContain('fn navigate(start: [f64; 3], end: [f64; 3], energy: f64)');
    });
  });

  test.describe('VNC to Java Compilation', () => {
    test('should compile navigation function to Java', async () => {
      // Create VNC code
      await page.waitForSelector('.monaco-editor');
      await page.click('.monaco-editor');
      await page.keyboard.type('⋋(x, y) = navigate(x, y, energy)');
      
      // Select Java compilation
      await page.click('[data-testid="compile-button"]');
      await page.click('text=Java');
      
      // Wait for compilation
      await page.waitForSelector('[data-testid="java-output"]');
      
      // Verify Java output
      const javaOutput = await page.locator('[data-testid="java-output"]').textContent();
      expect(javaOutput).toContain('public class NavigationFunction');
      expect(javaOutput).toContain('public static double navigate(double x, double y, double energy)');
      expect(javaOutput).toContain('import java.util');
    });

    test('should compile VNC with classes to Java', async () => {
      // Create VNC code that maps to Java classes
      await page.waitForSelector('.monaco-editor');
      await page.click('.monaco-editor');
      await page.keyboard.type(`
⋋(start: Point, end: Point) = navigate(start, end, energy)
  where Point = (x: f64, y: f64)
      `);
      
      // Select Java compilation
      await page.click('[data-testid="compile-button"]');
      await page.click('text=Java');
      
      // Wait for compilation
      await page.waitForSelector('[data-testid="java-output"]');
      
      // Verify Java output
      const javaOutput = await page.locator('[data-testid="java-output"]').textContent();
      expect(javaOutput).toContain('class Point');
      expect(javaOutput).toContain('public double x;');
      expect(javaOutput).toContain('public double y;');
      expect(javaOutput).toContain('public Point(double x, double y)');
    });
  });

  test.describe('VNC to JavaScript Compilation', () => {
    test('should compile navigation function to JavaScript', async () => {
      // Create VNC code
      await page.waitForSelector('.monaco-editor');
      await page.click('.monaco-editor');
      await page.keyboard.type('⋋(x, y) = navigate(x, y, energy)');
      
      // Select JavaScript compilation
      await page.click('[data-testid="compile-button"]');
      await page.click('text=JavaScript');
      
      // Wait for compilation
      await page.waitForSelector('[data-testid="javascript-output"]');
      
      // Verify JavaScript output
      const jsOutput = await page.locator('[data-testid="javascript-output"]').textContent();
      expect(jsOutput).toContain('function navigate(x, y, energy)');
      expect(jsOutput).toContain('export function navigate');
      expect(jsOutput).toContain('module.exports');
    });

    test('should compile VNC with arrow functions to JavaScript', async () => {
      // Create VNC code with lambda
      await page.waitForSelector('.monaco-editor');
      await page.click('.monaco-editor');
      await page.keyboard.type(`
⋋(start, end) = navigate(start, end, energy_function)
  where energy_function = λ(p) → p.distance + p.obstacle_penalty
      `);
      
      // Select JavaScript compilation
      await page.click('[data-testid="compile-button"]');
      await page.click('text=JavaScript');
      
      // Wait for compilation
      await page.waitForSelector('[data-testid="javascript-output"]');
      
      // Verify JavaScript output
      const jsOutput = await page.locator('[data-testid="javascript-output"]').textContent();
      expect(jsOutput).toContain('const energy_function = (p) =>');
      expect(jsOutput).toContain('p.distance + p.obstacle_penalty');
      expect(jsOutput).toContain('arrow function');
    });
  });

  test.describe('Multi-Target Compilation Performance', () => {
    test('should compile to multiple targets simultaneously', async () => {
      // Create VNC code
      await page.waitForSelector('.monaco-editor');
      await page.click('.monaco-editor');
      await page.keyboard.type('⋋(x, y) = navigate(x, y, energy)');
      
      // Select multiple compilation targets
      await page.click('[data-testid="compile-button"]');
      await page.click('[data-testid="multi-target-checkbox"]');
      await page.click('text=Compile All');
      
      // Wait for all compilations
      await page.waitForSelector('[data-testid="multi-compilation-results"]');
      
      // Verify all targets compiled successfully
      const results = await page.locator('[data-testid="compilation-success"]').count();
      expect(results).toBeGreaterThan(5); // Multiple targets
      
      // Verify specific outputs
      await expect(page.locator('[data-testid="cpp-success"]')).toBeVisible();
      await expect(page.locator('[data-testid="python-success"]')).toBeVisible();
      await expect(page.locator('[data-testid="wasm-success"]')).toBeVisible();
    });

    test('should handle large VNC programs efficiently', async () => {
      // Create large VNC program
      await page.waitForSelector('.monaco-editor');
      await page.click('.monaco-editor');
      
      // Generate large VNC code
      for (let i = 0; i < 50; i++) {
        await page.keyboard.type(`⋋(path${i}) = navigate(path${i}, energy${i})\n`);
      }
      
      const startTime = Date.now();
      
      // Compile to Python
      await page.click('[data-testid="compile-button"]');
      await page.click('text=Python');
      
      // Wait for compilation
      await page.waitForSelector('[data-testid="python-output"]');
      
      const compileTime = Date.now() - startTime;
      
      // Should compile large programs in reasonable time
      expect(compileTime).toBeLessThan(5000); // Less than 5 seconds
      
      // Verify output contains all functions
      const pythonOutput = await page.locator('[data-testid="python-output"]').textContent();
      expect(pythonOutput).toContain('def navigate_path0');
      expect(pythonOutput).toContain('def navigate_path49');
    });
  });

  test.describe('Compilation Error Handling', () => {
    test('should provide helpful error messages for syntax errors', async () => {
      // Create VNC code with syntax error
      await page.waitForSelector('.monaco-editor');
      await page.click('.monaco-editor');
      await page.keyboard.type('⋋(x, y) = navigate(x, y'); // Missing closing parenthesis
      
      // Try to compile
      await page.click('[data-testid="compile-button"]');
      await page.click('text=C++');
      
      // Wait for error
      await page.waitForSelector('[data-testid="compilation-error"]');
      
      // Verify helpful error message
      const errorMessage = await page.locator('[data-testid="compilation-error"]').textContent();
      expect(errorMessage).toContain('Syntax error');
      expect(errorMessage).toContain('Missing closing parenthesis');
      expect(errorMessage).toContain('Line 1');
    });

    test('should provide error suggestions', async () => {
      // Create VNC code with common error
      await page.waitForSelector('.monaco-editor');
      await page.click('.monaco-editor');
      await page.keyboard.type('λ(x)'); // Missing arrow and body
      
      // Try to compile
      await page.click('[data-testid="compile-button"]');
      await page.click('text=Python');
      
      // Wait for error with suggestions
      await page.waitForSelector('[data-testid="compilation-error-with-suggestions"]');
      
      // Verify suggestions
      const suggestions = await page.locator('[data-testid="error-suggestions"]').textContent();
      expect(suggestions).toContain('Did you mean:');
      expect(suggestions).toContain('λ(x) → x + 1');
    });

    test('should handle unsupported language features gracefully', async () => {
      // Create VNC code with unsupported feature
      await page.waitForSelector('.monaco-editor');
      await page.click('.monaco-editor');
      await page.keyboard.type('⋋(x, y) = navigate(x, y, unsupported_feature(x, y, z, w, ...))');
      
      // Try to compile to WebAssembly
      await page.click('[data-testid="compile-button"]');
      await page.click('text=WebAssembly');
      
      // Wait for error
      await page.waitForSelector('[data-testid="unsupported-feature-error"]');
      
      // Verify error message
      const errorMessage = await page.locator('[data-testid="unsupported-feature-error"]').textContent();
      expect(errorMessage).toContain('Unsupported feature');
      expect(errorMessage).toContain('Variadic functions not supported in WebAssembly');
    });
  });
});