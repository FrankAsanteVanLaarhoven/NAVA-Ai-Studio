import { test, expect, Page } from '@playwright/test';

test.describe('Performance and Load Testing', () => {
  let page: Page;

  test.beforeEach(async ({ page: testPage }) => {
    page = testPage;
    await page.goto('http://localhost:3000');
    await page.waitForLoadState('networkidle');
  });

  test.describe('Application Load Performance', () => {
    test('should load application within performance budget', async () => {
      // Measure initial load time
      const startTime = Date.now();
      
      // Navigate to application
      await page.goto('http://localhost:3000');
      
      // Wait for main interface to be ready
      await page.waitForSelector('.monaco-editor', { timeout: 3000 });
      await page.waitForSelector('[data-testid="terminal-tab"]', { timeout: 3000 });
      
      const loadTime = Date.now() - startTime;
      
      // Should load within 3 seconds
      expect(loadTime).toBeLessThan(3000);
      
      // Verify all critical components are loaded
      await expect(page.locator('.monaco-editor')).toBeVisible();
      await expect(page.locator('[data-testid="compile-button"]')).toBeVisible();
      await expect(page.locator('[data-testid="terminal-tab"]')).toBeVisible();
    });

    test('should handle multiple concurrent users', async ({ browser }) => {
      // Create multiple browser contexts to simulate concurrent users
      const contexts = [];
      const pages = [];
      
      for (let i = 0; i < 5; i++) {
        const context = await browser.newContext();
        const page = await context.newPage();
        await page.goto('http://localhost:3000');
        contexts.push(context);
        pages.push(page);
      }
      
      // Wait for all pages to load
      await Promise.all(pages.map(page => 
        page.waitForSelector('.monaco-editor', { timeout: 5000 })
      ));
      
      // Verify all users can access the application
      for (const page of pages) {
        await expect(page.locator('.monaco-editor')).toBeVisible();
        await expect(page.locator('[data-testid="compile-button"]')).toBeVisible();
      }
      
      // Clean up
      for (const context of contexts) {
        await context.close();
      }
    });

    test('should maintain responsiveness under load', async () => {
      // Perform multiple rapid interactions
      const interactions = [];
      
      for (let i = 0; i < 10; i++) {
        interactions.push(
          page.click('.monaco-editor'),
          page.keyboard.type(`⋋(x${i}, y${i}) = navigate(x${i}, y${i}, energy${i})\n`),
          page.click('[data-testid="terminal-tab"]'),
          page.click('[data-testid="compile-button"]')
        );
      }
      
      const startTime = Date.now();
      await Promise.all(interactions);
      const interactionTime = Date.now() - startTime;
      
      // Should complete all interactions within reasonable time
      expect(interactionTime).toBeLessThan(5000);
      
      // Verify UI is still responsive
      await expect(page.locator('.monaco-editor')).toBeVisible();
      await expect(page.locator('[data-testid="compile-button"]')).toBeVisible();
    });
  });

  test.describe('Editor Performance', () => {
    test('should handle large files efficiently', async () => {
      // Create large VNC file
      await page.waitForSelector('.monaco-editor');
      await page.click('.monaco-editor');
      
      // Generate large content
      const largeContent = Array.from({ length: 1000 }, (_, i) => 
        `⋋(path${i}, waypoint${i}) = navigate(path${i}, waypoint${i}, energy${i})`
      ).join('\n');
      
      const startTime = Date.now();
      await page.keyboard.type(largeContent);
      const typingTime = Date.now() - startTime;
      
      // Should handle large content efficiently
      expect(typingTime).toBeLessThan(5000); // Less than 5 seconds for 1000 lines
      
      // Verify content is properly displayed
      const editorContent = await page.locator('.monaco-editor').textContent();
      expect(editorContent).toContain('⋋(path999, waypoint999)');
    });

    test('should provide responsive syntax highlighting', async () => {
      // Create VNC code
      await page.waitForSelector('.monaco-editor');
      await page.click('.monaco-editor');
      
      const startTime = Date.now();
      await page.keyboard.type('⋋(x, y) = navigate(x, y, λ(p) → p.distance + p.obstacle_penalty)');
      const highlightingTime = Date.now() - startTime;
      
      // Should highlight syntax quickly
      expect(highlightingTime).toBeLessThan(1000);
      
      // Verify syntax highlighting is applied
      const highlightedElements = await page.locator('.monaco-editor .keyword').count();
      expect(highlightedElements).toBeGreaterThan(0);
    });

    test('should provide responsive code completion', async () => {
      // Create VNC code
      await page.waitForSelector('.monaco-editor');
      await page.click('.monaco-editor');
      
      // Type partial function name
      await page.keyboard.type('nav');
      
      // Trigger completion
      const startTime = Date.now();
      await page.keyboard.press('Control+Space');
      
      // Wait for completion suggestions
      await page.waitForSelector('.monaco-editor .suggest-widget', { timeout: 1000 });
      const completionTime = Date.now() - startTime;
      
      // Should provide completions quickly
      expect(completionTime).toBeLessThan(1000);
      
      // Verify completion suggestions
      const suggestions = await page.locator('.monaco-editor .suggest-widget .monaco-list-row').count();
      expect(suggestions).toBeGreaterThan(0);
    });
  });

  test.describe('Compilation Performance', () => {
    test('should compile simple VNC code quickly', async () => {
      // Create simple VNC code
      await page.waitForSelector('.monaco-editor');
      await page.click('.monaco-editor');
      await page.keyboard.type('⋋(x, y) = navigate(x, y, energy)');
      
      const startTime = Date.now();
      
      // Compile to C++
      await page.click('[data-testid="compile-button"]');
      await page.click('text=C++');
      
      // Wait for compilation
      await page.waitForSelector('[data-testid="cpp-output"]');
      
      const compileTime = Date.now() - startTime;
      
      // Should compile in less than 1 second
      expect(compileTime).toBeLessThan(1000);
      
      // Verify compilation result
      await expect(page.locator('[data-testid="cpp-output"]')).toContainText('class NavigationFunction');
    });

    test('should handle complex compilation efficiently', async () => {
      // Create complex VNC code
      await page.waitForSelector('.monaco-editor');
      await page.click('.monaco-editor');
      await page.keyboard.type(`
⋋(start, end) = navigate(start, end, energy_function)
  where energy_function = λ(p) → p.distance + p.obstacle_penalty + p.gradient_cost

path = ⋋([0, 0], [10, 10])
optimized_path = optimize(path, gradient_descent)
visualize(optimized_path, 3d_renderer)
      `);
      
      const startTime = Date.now();
      
      // Compile to multiple targets
      await page.click('[data-testid="compile-button"]');
      await page.click('text=Python');
      
      // Wait for compilation
      await page.waitForSelector('[data-testid="python-output"]');
      
      const compileTime = Date.now() - startTime;
      
      // Should handle complex code efficiently
      expect(compileTime).toBeLessThan(2000);
      
      // Verify compilation result
      await expect(page.locator('[data-testid="python-output"]')).toContainText('lambda p:');
    });

    test('should compile multiple targets in parallel efficiently', async () => {
      // Create VNC code
      await page.waitForSelector('.monaco-editor');
      await page.click('.monaco-editor');
      await page.keyboard.type('⋋(x, y) = navigate(x, y, energy)');
      
      const startTime = Date.now();
      
      // Compile to multiple targets simultaneously
      await page.click('[data-testid="compile-button"]');
      await page.click('[data-testid="multi-target-checkbox"]');
      await page.click('text=Compile All');
      
      // Wait for all compilations
      await page.waitForSelector('[data-testid="multi-compilation-results"]');
      
      const compileTime = Date.now() - startTime;
      
      // Should compile multiple targets efficiently
      expect(compileTime).toBeLessThan(3000);
      
      // Verify all targets compiled
      await expect(page.locator('[data-testid="cpp-success"]')).toBeVisible();
      await expect(page.locator('[data-testid="python-success"]')).toBeVisible();
      await expect(page.locator('[data-testid="wasm-success"]')).toBeVisible();
    });
  });

  test.describe('Terminal Performance', () => {
    test('should execute commands quickly', async () => {
      // Open terminal
      await page.click('[data-testid="terminal-tab"]');
      
      const startTime = Date.now();
      
      // Execute simple command
      await page.type('[data-testid="terminal-input"]', 'echo "Hello NAVΛ Studio"');
      await page.keyboard.press('Enter');
      
      // Wait for output
      await page.waitForSelector('[data-testid="terminal-output"]');
      
      const executionTime = Date.now() - startTime;
      
      // Should execute quickly
      expect(executionTime).toBeLessThan(1000);
      
      // Verify output
      await expect(page.locator('[data-testid="terminal-output"]')).toContainText('Hello NAVΛ Studio');
    });

    test('should handle multiple rapid commands efficiently', async () => {
      // Open terminal
      await page.click('[data-testid="terminal-tab"]');
      
      const commands = [
        'echo "Command 1"',
        'echo "Command 2"',
        'echo "Command 3"',
        'ls',
        'pwd',
        'date'
      ];
      
      const startTime = Date.now();
      
      // Execute commands rapidly
      for (const command of commands) {
        await page.type('[data-testid="terminal-input"]', command);
        await page.keyboard.press('Enter');
      }
      
      // Wait for all outputs
      await page.waitForTimeout(1000);
      
      const executionTime = Date.now() - startTime;
      
      // Should handle rapid commands efficiently
      expect(executionTime).toBeLessThan(3000);
      
      // Verify all outputs
      const terminalOutput = await page.locator('[data-testid="terminal-output"]').textContent();
      expect(terminalOutput).toContain('Command 1');
      expect(terminalOutput).toContain('Command 2');
      expect(terminalOutput).toContain('Command 3');
    });

    test('should handle large command output efficiently', async () => {
      // Open terminal
      await page.click('[data-testid="terminal-tab"]');
      
      // Execute command with large output
      await page.type('[data-testid="terminal-input"]', 'seq 1 1000');
      await page.keyboard.press('Enter');
      
      const startTime = Date.now();
      
      // Wait for large output
      await page.waitForFunction(
        () => document.querySelector('[data-testid="terminal-output"]')?.textContent?.includes('1000'),
        { timeout: 5000 }
      );
      
      const executionTime = Date.now() - startTime;
      
      // Should handle large output efficiently
      expect(executionTime).toBeLessThan(3000);
      
      // Verify large output
      const terminalOutput = await page.locator('[data-testid="terminal-output"]').textContent();
      expect(terminalOutput).not.toBeNull();
      expect(terminalOutput).toContain('1000');
      if (terminalOutput) {
        expect(terminalOutput.length).toBeGreaterThan(1000);
      }
    });
  });

  test.describe('Memory Usage', () => {
    test('should not leak memory during extended usage', async () => {
      // Get initial memory usage
      const initialMetrics = await page.evaluate(() => {
        if ('memory' in performance) {
          return (performance as any).memory.usedJSHeapSize;
        }
        return 0;
      });
      
      // Perform extensive operations
      for (let i = 0; i < 20; i++) {
        // Create content
        await page.click('.monaco-editor');
        await page.keyboard.type(`⋋(path${i}) = navigate(path${i}, waypoint${i}, energy${i})\n`);
        
        // Compile
        await page.click('[data-testid="compile-button"]');
        await page.click('text=Python');
        await page.waitForSelector('[data-testid="python-output"]');
        
        // Execute in terminal
        await page.click('[data-testid="terminal-tab"]');
        await page.type('[data-testid="terminal-input"]', `echo "Test ${i}"`);
        await page.keyboard.press('Enter');
        
        // Clear for next iteration
        await page.click('.monaco-editor');
        await page.keyboard.press('Control+A');
        await page.keyboard.press('Delete');
      }
      
      // Force garbage collection if available
      await page.evaluate(() => {
        if ('gc' in window) {
          (window as any).gc();
        }
      });
      
      // Get final memory usage
      const finalMetrics = await page.evaluate(() => {
        if ('memory' in performance) {
          return (performance as any).memory.usedJSHeapSize;
        }
        return 0;
      });
      
      // Memory usage should not increase significantly
      const memoryIncrease = finalMetrics - initialMetrics;
      expect(memoryIncrease).toBeLessThan(50 * 1024 * 1024); // Less than 50MB increase
    });
  });

  test.describe('Network Performance', () => {
    test('should handle network latency gracefully', async () => {
      // Simulate network latency
      await page.route('**/*', route => {
        setTimeout(() => route.continue(), 500); // 500ms delay
      });
      
      const startTime = Date.now();
      
      // Try to compile
      await page.click('[data-testid="compile-button"]');
      await page.click('text=C++');
      
      // Wait for compilation
      await page.waitForSelector('[data-testid="cpp-output"]');
      
      const compileTime = Date.now() - startTime;
      
      // Should handle network latency gracefully
      expect(compileTime).toBeLessThan(5000);
      
      // Verify compilation still works
      await expect(page.locator('[data-testid="cpp-output"]')).toBeVisible();
    });

    test('should handle network failures gracefully', async () => {
      // Simulate network failure
      await page.route('**/*', route => {
        if (route.request().url().includes('api')) {
          route.abort('failed');
        } else {
          route.continue();
        }
      });
      
      // Try to use API-dependent features
      await page.click('[data-testid="compile-button"]');
      await page.click('text=C++');
      
      // Should show error message
      await page.waitForSelector('[data-testid="network-error"]');
      
      // Verify error message
      await expect(page.locator('[data-testid="network-error"]')).toContainText('Network error');
    });
  });

  test.describe('Resource Usage Monitoring', () => {
    test('should monitor CPU usage during compilation', async () => {
      // Get CPU usage before compilation
      const cpuBefore = await page.evaluate(() => {
        if ('cpu' in performance) {
          return (performance as any).cpu;
        }
        return { used: 0, total: 1 };
      });
      
      // Perform compilation
      await page.click('[data-testid="compile-button"]');
      await page.click('text=C++');
      await page.waitForSelector('[data-testid="cpp-output"]');
      
      // Get CPU usage after compilation
      const cpuAfter = await page.evaluate(() => {
        if ('cpu' in performance) {
          return (performance as any).cpu;
        }
        return { used: 0, total: 1 };
      });
      
      // CPU usage should be reasonable
      const cpuUsage = (cpuAfter.used - cpuBefore.used) / (cpuAfter.total - cpuBefore.total);
      expect(cpuUsage).toBeLessThan(0.8); // Less than 80% CPU usage
    });

    test('should provide performance metrics', async () => {
      // Enable performance monitoring
      await page.click('[data-testid="performance-settings"]');
      await page.click('[data-testid="enable-performance-monitoring"]');
      
      // Perform operations
      await page.click('.monaco-editor');
      await page.keyboard.type('⋋(x, y) = navigate(x, y, energy)');
      await page.click('[data-testid="compile-button"]');
      await page.click('text=Python');
      
      // Check performance metrics
      await page.waitForSelector('[data-testid="performance-metrics"]');
      
      // Verify metrics are displayed
      const metrics = await page.locator('[data-testid="performance-metrics"]').textContent();
      expect(metrics).toContain('Compilation time:');
      expect(metrics).toContain('Memory usage:');
      expect(metrics).toContain('CPU usage:');
    });
  });
});