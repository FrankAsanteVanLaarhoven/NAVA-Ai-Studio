import { describe, test, expect, beforeEach, afterEach, vi } from 'vitest';
// import { executeROSCommand } from '../../src/services/ros-terminal-service';
import { compiler } from '../../src/api/compiler';
import { compiler } from '../../src/api/compiler';

test.describe('NAVΛ Studio E2E Tests', () => {
  let page: Page;
  let browser: Browser;
  let context: BrowserContext;

  test.beforeAll(async ({ browser: testBrowser }) => {
    browser = testBrowser;
    context = await browser.newContext({
      viewport: { width: 1920, height: 1080 },
      deviceScaleFactor: 1,
    });
    page = await context.newPage();
  });

  test.afterAll(async () => {
    await context.close();
    await browser.close();
  });

  test.beforeEach(async () => {
    await page.goto('http://localhost:3000');
    await page.waitForLoadState('networkidle');
  });

  test.describe('User Journey: First Time User', () => {
    test('should complete onboarding flow successfully', async () => {
      // Navigate to onboarding
      await page.click('text=Get Started');
      await page.waitForURL('**/onboarding');
      
      // Welcome screen
      await expect(page.locator('h1')).toContainText('Welcome to NAVΛ Studio');
      await page.click('text=Next');
      
      // Features overview
      await expect(page.locator('h2')).toContainText('Key Features');
      await page.click('text=Next');
      
      // ROS integration
      await expect(page.locator('h2')).toContainText('ROS Integration');
      await page.click('text=Next');
      
      // VNC language introduction
      await expect(page.locator('h2')).toContainText('Van Laarhoven Navigation Calculus');
      await page.click('text=Start Coding');
      
      // Should land on main editor
      await expect(page.locator('.monaco-editor')).toBeVisible();
      await expect(page.locator('text=Welcome to NAVΛ Studio')).toBeVisible();
    });

    test('should access help and documentation', async () => {
      // Click help button
      await page.click('[data-testid="help-button"]');
      
      // Should open help menu
      await expect(page.locator('[data-testid="help-menu"]')).toBeVisible();
      
      // Click documentation
      await page.click('text=Documentation');
      await page.waitForURL('**/docs');
      
      // Should show documentation
      await expect(page.locator('h1')).toContainText('NAVΛ Studio Documentation');
      await expect(page.locator('text=Getting Started')).toBeVisible();
    });

    test('should access ROS learning center', async () => {
      // Click ROS learning center button
      await page.click('[data-testid="ros-learning-button"]');
      
      // Should open ROS learning center
      await expect(page.locator('[data-testid="ros-learning-center"]')).toBeVisible();
      await expect(page.locator('h1')).toContainText('ROS Learning Center');
      
      // Should show available courses
      await expect(page.locator('text=ROS2 Basics')).toBeVisible();
      await expect(page.locator('text=Advanced Navigation')).toBeVisible();
      await expect(page.locator('text=Gazebo Simulation')).toBeVisible();
    });
  });

  test.describe('User Journey: Code Development', () => {
    test('should create and edit VNC code', async () => {
      // Wait for editor to load
      await page.waitForSelector('.monaco-editor');
      
      // Click new file button
      await page.click('[data-testid="new-file-button"]');
      
      // Type VNC code
      const editor = page.locator('.monaco-editor');
      await editor.click();
      await page.keyboard.type('⋋(x, y) = navigate(x, y, energy_function)');
      
      // Verify code is in editor
      const editorContent = await editor.textContent();
      expect(editorContent).toContain('⋋(x, y) = navigate(x, y, energy_function)');
    });

    test('should compile VNC code to multiple targets', async () => {
      // Create VNC code
      await page.waitForSelector('.monaco-editor');
      await page.click('.monaco-editor');
      await page.keyboard.type('⋋(x, y) = navigate(x, y, energy)');
      
      // Open compile menu
      await page.click('[data-testid="compile-button"]');
      
      // Select compilation target
      await page.click('text=C++');
      
      // Wait for compilation
      await page.waitForSelector('[data-testid="compilation-result"]');
      
      // Verify compilation result
      const result = await page.locator('[data-testid="compilation-result"]').textContent();
      expect(result).toContain('Compilation successful');
      expect(result).toContain('#include');
      expect(result).toContain('class NavigationFunction');
    });

    test('should execute VNC code in terminal', async () => {
      // Create VNC code
      await page.waitForSelector('.monaco-editor');
      await page.click('.monaco-editor');
      await page.keyboard.type('path = ⋋([0, 0], [10, 10])');
      
      // Open terminal
      await page.click('[data-testid="terminal-tab"]');
      
      // Execute code
      await page.click('[data-testid="execute-button"]');
      
      // Wait for execution
      await page.waitForSelector('[data-testid="execution-output"]');
      
      // Verify output
      const output = await page.locator('[data-testid="execution-output"]').textContent();
      expect(output).toContain('Navigation function executed');
      expect(output).toContain('Path calculated');
    });

    test('should visualize navigation results', async () => {
      // Create VNC code with visualization
      await page.waitForSelector('.monaco-editor');
      await page.click('.monaco-editor');
      await page.keyboard.type('visualize(⋋([0, 0], [10, 10]), 3d_renderer)');
      
      // Execute and visualize
      await page.click('[data-testid="visualize-button"]');
      
      // Wait for 3D visualization
      await page.waitForSelector('[data-testid="3d-visualization"]');
      
      // Verify visualization is displayed
      const canvas = page.locator('canvas');
      await expect(canvas).toBeVisible();
      
      // Verify visualization controls
      await expect(page.locator('[data-testid="3d-controls"]')).toBeVisible();
    });
  });

  test.describe('User Journey: ROS Integration', () => {
    test('should execute ROS commands in terminal', async () => {
      // Open terminal
      await page.click('[data-testid="terminal-tab"]');
      
      // Type ROS command
      await page.click('[data-testid="terminal-input"]');
      await page.keyboard.type('ros2 topic list');
      await page.keyboard.press('Enter');
      
      // Wait for output
      await page.waitForSelector('[data-testid="terminal-output"]');
      
      // Verify ROS output
      const output = await page.locator('[data-testid="terminal-output"]').textContent();
      expect(output).toContain('/parameter_events');
      expect(output).toContain('/rosout');
    });

    test('should use ROS command completion', async () => {
      // Open terminal
      await page.click('[data-testid="terminal-tab"]');
      
      // Type partial ROS command
      await page.click('[data-testid="terminal-input"]');
      await page.keyboard.type('ros2 to');
      
      // Trigger completion
      await page.keyboard.press('Tab');
      
      // Verify completion
      const input = await page.locator('[data-testid="terminal-input"]').inputValue();
      expect(input).toContain('ros2 topic');
    });

    test('should access ROS learning tutorials', async () => {
      // Open ROS learning center
      await page.click('[data-testid="ros-learning-button"]');
      
      // Select ROS2 basics course
      await page.click('text=ROS2 Basics');
      
      // Should open tutorial
      await expect(page.locator('[data-testid="ros-tutorial"]')).toBeVisible();
      await expect(page.locator('h1')).toContainText('ROS2 Basics');
      
      // Should show interactive exercises
      await expect(page.locator('[data-testid="interactive-exercise"]')).toBeVisible();
      
      // Complete first exercise
      await page.click('[data-testid="run-exercise-button"]');
      await expect(page.locator('[data-testid="exercise-result"]')).toContainText('Success');
    });

    test('should run ROS launch files', async () => {
      // Open terminal
      await page.click('[data-testid="terminal-tab"]');
      
      // Create launch file
      await page.keyboard.type('touch my_robot.launch.py');
      await page.keyboard.press('Enter');
      
      // Run launch file
      await page.keyboard.type('ros2 launch my_robot.launch.py');
      await page.keyboard.press('Enter');
      
      // Wait for launch output
      await page.waitForSelector('[data-testid="launch-output"]');
      
      // Verify launch started
      const output = await page.locator('[data-testid="launch-output"]').textContent();
      expect(output).toContain('Launching nodes');
      expect(output).toContain('Process started');
    });
  });

  test.describe('User Journey: Multi-Target Compilation', () => {
    test('should compile to C++ target', async () => {
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
      expect(cppOutput).toContain('class NavigationFunction');
      expect(cppOutput).toContain('double energy_function');
    });

    test('should compile to Python target', async () => {
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
      expect(pythonOutput).toContain('def navigate');
      expect(pythonOutput).toContain('def energy_function');
      expect(pythonOutput).toContain('lambda p:');
    });

    test('should compile to WebAssembly target', async () => {
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
      expect(wasmOutput).toContain('(export');
    });

    test('should compile to GLSL shader target', async () => {
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
      expect(glslOutput).toContain('void main()');
      expect(glslOutput).toContain('uniform vec3');
    });
  });

  test.describe('User Journey: Error Handling', () => {
    test('should handle syntax errors gracefully', async () => {
      // Create VNC code with syntax error
      await page.waitForSelector('.monaco-editor');
      await page.click('.monaco-editor');
      await page.keyboard.type('⋋(x, y) = navigate(x, y'); // Missing closing parenthesis
      
      // Try to compile
      await page.click('[data-testid="compile-button"]');
      await page.click('text=C++');
      
      // Should show error message
      await page.waitForSelector('[data-testid="error-message"]');
      const errorMessage = await page.locator('[data-testid="error-message"]').textContent();
      expect(errorMessage).toContain('Syntax error');
      expect(errorMessage).toContain('Missing closing parenthesis');
    });

    test('should handle compilation errors gracefully', async () => {
      // Create VNC code that will fail compilation
      await page.waitForSelector('.monaco-editor');
      await page.click('.monaco-editor');
      await page.keyboard.type('⋋(x, y) = invalid_function(x, y)'); // Invalid function
      
      // Try to compile
      await page.click('[data-testid="compile-button"]');
      await page.click('text=C++');
      
      // Should show compilation error
      await page.waitForSelector('[data-testid="compilation-error"]');
      const errorMessage = await page.locator('[data-testid="compilation-error"]').textContent();
      expect(errorMessage).toContain('Compilation failed');
      expect(errorMessage).toContain('Unknown function: invalid_function');
    });

    test('should handle terminal command errors gracefully', async () => {
      // Open terminal
      await page.click('[data-testid="terminal-tab"]');
      
      // Type invalid command
      await page.click('[data-testid="terminal-input"]');
      await page.keyboard.type('invalid-command-that-does-not-exist');
      await page.keyboard.press('Enter');
      
      // Should show error
      await page.waitForSelector('[data-testid="terminal-error"]');
      const errorMessage = await page.locator('[data-testid="terminal-error"]').textContent();
      expect(errorMessage).toContain('command not found');
    });
  });

  test.describe('User Journey: Performance and Responsiveness', () => {
    test('should load application quickly', async () => {
      const startTime = Date.now();
      
      // Navigate to application
      await page.goto('http://localhost:3000');
      
      // Wait for initial load
      await page.waitForLoadState('networkidle');
      
      const loadTime = Date.now() - startTime;
      
      // Should load in less than 3 seconds
      expect(loadTime).toBeLessThan(3000);
      
      // Should show main interface
      await expect(page.locator('.monaco-editor')).toBeVisible();
    });

    test('should compile code quickly', async () => {
      // Create VNC code
      await page.waitForSelector('.monaco-editor');
      await page.click('.monaco-editor');
      await page.keyboard.type('⋋(x, y) = navigate(x, y, energy)');
      
      const startTime = Date.now();
      
      // Compile code
      await page.click('[data-testid="compile-button"]');
      await page.click('text=C++');
      
      // Wait for compilation
      await page.waitForSelector('[data-testid="compilation-result"]');
      
      const compileTime = Date.now() - startTime;
      
      // Should compile in less than 2 seconds
      expect(compileTime).toBeLessThan(2000);
    });

    test('should handle large files efficiently', async () => {
      // Create large VNC file
      await page.waitForSelector('.monaco-editor');
      await page.click('.monaco-editor');
      
      // Type large amount of code
      for (let i = 0; i < 100; i++) {
        await page.keyboard.type(`⋋(path${i}) = navigate(path${i}, energy${i})\n`);
      }
      
      const startTime = Date.now();
      
      // Try to compile
      await page.click('[data-testid="compile-button"]');
      await page.click('text=C++');
      
      // Wait for compilation
      await page.waitForSelector('[data-testid="compilation-result"]');
      
      const compileTime = Date.now() - startTime;
      
      // Should handle large files in reasonable time
      expect(compileTime).toBeLessThan(5000);
    });
  });

  test.describe('User Journey: Accessibility', () => {
    test('should be keyboard navigable', async () => {
      // Tab through main interface elements
      await page.keyboard.press('Tab');
      await expect(page.locator(':focus')).toHaveAttribute('data-testid', 'menu-button');
      
      await page.keyboard.press('Tab');
      await expect(page.locator(':focus')).toHaveAttribute('data-testid', 'new-file-button');
      
      await page.keyboard.press('Tab');
      await expect(page.locator(':focus')).toHaveAttribute('data-testid', 'compile-button');
    });

    test('should have proper ARIA labels', async () => {
      // Check main editor
      await expect(page.locator('.monaco-editor')).toHaveAttribute('role', 'textbox');
      await expect(page.locator('.monaco-editor')).toHaveAttribute('aria-label', 'Code editor');
      
      // Check terminal
      await page.click('[data-testid="terminal-tab"]');
      await expect(page.locator('[data-testid="terminal-input"]')).toHaveAttribute('aria-label', 'Terminal input');
    });

    test('should support screen readers', async () => {
      // Check semantic HTML structure
      await expect(page.locator('main')).toBeVisible();
      await expect(page.locator('nav')).toBeVisible();
      await expect(page.locator('header')).toBeVisible();
      
      // Check proper heading structure
      const h1Count = await page.locator('h1').count();
      expect(h1Count).toBe(1);
    });
  });
});