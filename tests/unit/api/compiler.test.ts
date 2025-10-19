import { describe, test, expect, beforeEach, afterEach, jest } from '@jest/globals';
import { VNCCompiler, compiler } from '../../src/api/compiler';
import { CompilerOptions, CompilationResult } from '../../src/types/compiler';

describe('CompilerAPI', () => {
  beforeEach(() => {
    // Setup for tests
  });
        type: 'navigation_function',
        operator: '⋋',
        parameters: ['start', 'end'],
        body: 'navigate(start, end, energy_function)'
      },
      functions: [
        {
          name: 'main_navigation',
          type: 'navigation_function',
          parameters: ['start', 'end'],
          body: '⋋(start, end) = navigate(start, end, energy_function)'
        }
      ],
      metadata: {
        version: '1.0.0',
        author: 'NAVΛ Studio',
        description: 'Test navigation program'
      }
    };
  });

  afterEach(() => {
    jest.clearAllMocks();
  });

  describe('compile', () => {
    test('should compile VNC to C++ successfully', async () => {
      const vncCode = `
⋋(start, end) = navigate(start, end, energy_function)
  where energy_function = λ(p) → p.distance + p.obstacle_penalty
      `;
      
      const result = await CompilerAPI.compile(vncCode, 'cpp');
      
      expect(result.success).toBe(true);
      expect(result.output).toBeDefined();
      expect(result.output).toContain('#include');
      expect(result.output).toContain('class NavigationFunction');
      expect(result.output).toContain('double energy_function');
      expect(result.errors).toHaveLength(0);
      expect(result.target).toBe('cpp');
    });

    test('should compile VNC to Python successfully', async () => {
      const vncCode = `
⋋(start, end) = navigate(start, end, energy_function)
  where energy_function = λ(p) → p.distance + p.obstacle_penalty
      `;
      
      const result = await CompilerAPI.compile(vncCode, 'python');
      
      expect(result.success).toBe(true);
      expect(result.output).toBeDefined();
      expect(result.output).toContain('def navigate');
      expect(result.output).toContain('def energy_function');
      expect(result.output).toContain('lambda p:');
      expect(result.target).toBe('python');
    });

    test('should compile VNC to WebAssembly successfully', async () => {
      const vncCode = '⋋(x, y) = navigate(x, y, energy)';
      
      const result = await CompilerAPI.compile(vncCode, 'wasm');
      
      expect(result.success).toBe(true);
      expect(result.output).toBeDefined();
      expect(result.output).toContain('(module');
      expect(result.output).toContain('(func');
      expect(result.output).toContain('(export');
      expect(result.target).toBe('wasm');
    });

    test('should compile VNC to GLSL shader successfully', async () => {
      const vncCode = '⋋(position, target) = navigate(position, target, energy_field)';
      
      const result = await CompilerAPI.compile(vncCode, 'glsl');
      
      expect(result.success).toBe(true);
      expect(result.output).toBeDefined();
      expect(result.output).toContain('#version');
      expect(result.output).toContain('void main()');
      expect(result.output).toContain('uniform vec3');
      expect(result.target).toBe('glsl');
    });

    test('should handle compilation errors gracefully', async () => {
      const invalidVNC = '⋋(x, y'; // Incomplete expression
      
      const result = await CompilerAPI.compile(invalidVNC, 'cpp');
      
      expect(result.success).toBe(false);
      expect(result.errors).toHaveLength(1);
      expect(result.errors[0]).toContain('Syntax error');
      expect(result.output).toBe('');
    });

    test('should handle unsupported compilation targets', async () => {
      const vncCode = '⋋(x, y) = navigate(x, y, energy)';
      
      const result = await CompilerAPI.compile(vncCode, 'unsupported' as any);
      
      expect(result.success).toBe(false);
      expect(result.errors).toContain('Unsupported compilation target: unsupported');
    });
  });

  describe('getSupportedTargets', () => {
    test('should return list of supported compilation targets', () => {
      const targets = CompilerAPI.getSupportedTargets();
      
      expect(targets).toContain('cpp');
      expect(targets).toContain('python');
      expect(targets).toContain('wasm');
      expect(targets).toContain('glsl');
      expect(targets).toContain('rust');
      expect(targets).toContain('java');
      expect(targets).toContain('javascript');
    });

    test('should return targets in consistent order', () => {
      const targets1 = CompilerAPI.getSupportedTargets();
      const targets2 = CompilerAPI.getSupportedTargets();
      
      expect(targets1).toEqual(targets2);
    });
  });

  describe('setOptimizationLevel', () => {
    test('should set optimization level for compilation', () => {
      CompilerAPI.setOptimizationLevel('O3');
      
      expect(CompilerAPI.getCurrentOptimizationLevel()).toBe('O3');
    });

    test('should apply optimization level to compilation', async () => {
      CompilerAPI.setOptimizationLevel('O3');
      const vncCode = '⋋(x, y) = navigate(x, y, energy)';
      
      const result = await CompilerAPI.compile(vncCode, 'cpp');
      
      expect(result.output).toContain('-O3');
      expect(result.output).toContain('inline');
      expect(result.output).toContain('const');
    });

    test('should handle different optimization levels', () => {
      const levels: OptimizationLevel[] = ['O0', 'O1', 'O2', 'O3', 'Os', 'Ofast'];
      
      levels.forEach(level => {
        CompilerAPI.setOptimizationLevel(level);
        expect(CompilerAPI.getCurrentOptimizationLevel()).toBe(level);
      });
    });

    test('should validate optimization level', () => {
      expect(() => {
        CompilerAPI.setOptimizationLevel('invalid' as any);
      }).toThrow('Invalid optimization level: invalid');
    });
  });

  describe('registerCustomTarget', () => {
    test('should register custom compilation target', () => {
      const customTarget = {
        name: 'custom_target',
        compile: jest.fn().mockReturnValue('custom output'),
        fileExtension: '.custom'
      };
      
      CompilerAPI.registerCustomTarget(customTarget);
      
      const targets = CompilerAPI.getSupportedTargets();
      expect(targets).toContain('custom_target');
    });

    test('should use custom target for compilation', async () => {
      const customTarget = {
        name: 'test_target',
        compile: jest.fn().mockResolvedValue({
          success: true,
          output: 'custom compiled output',
          errors: []
        }),
        fileExtension: '.test'
      };
      
      CompilerAPI.registerCustomTarget(customTarget);
      const vncCode = '⋋(x, y) = navigate(x, y, energy)';
      
      const result = await CompilerAPI.compile(vncCode, 'test_target');
      
      expect(customTarget.compile).toHaveBeenCalledWith(vncCode, expect.any(Object));
      expect(result.success).toBe(true);
      expect(result.output).toBe('custom compiled output');
    });

    test('should validate custom target', () => {
      const invalidTarget = {
        name: '',
        compile: jest.fn(),
        fileExtension: '.test'
      };
      
      expect(() => {
        CompilerAPI.registerCustomTarget(invalidTarget as any);
      }).toThrow('Custom target must have a valid name');
    });
  });

  describe('Code Generation Quality', () => {
    test('should generate readable Python code', async () => {
      const vncCode = `
⋋(start, end) = navigate(start, end, energy_function)
  where energy_function = λ(p) → p.distance + p.obstacle_penalty
      `;
      
      const result = await CompilerAPI.compile(vncCode, 'python');
      
      expect(result.output).toContain('def');
      expect(result.output).toContain(':');
      expect(result.output).toContain('import');
      expect(result.output).toMatch(/def navigate/);
      expect(result.output).toMatch(/def energy_function/);
    });

    test('should generate optimized C++ code', async () => {
      CompilerAPI.setOptimizationLevel('O3');
      const vncCode = '⋋(x, y) = navigate(x, y, energy)';
      
      const result = await CompilerAPI.compile(vncCode, 'cpp');
      
      expect(result.output).toContain('inline');
      expect(result.output).toContain('const');
      expect(result.output).toContain('constexpr');
      expect(result.output).toContain('-O3');
    });

    test('should generate correct function signatures', async () => {
      const vncCode = '⋋(start: [number, number], end: [number, number]) = navigate(start, end, energy)';
      
      const cppResult = await CompilerAPI.compile(vncCode, 'cpp');
      expect(cppResult.output).toContain('std::vector<double>');
      
      const pythonResult = await CompilerAPI.compile(vncCode, 'python');
      expect(pythonResult.output).toContain('def navigate(start: List[float], end: List[float], energy):');
    });
  });

  describe('Error Handling and Validation', () => {
    test('should validate VNC syntax before compilation', async () => {
      const invalidVNC = '⋋(x, y) = navigate(x, y'; // Missing closing parenthesis
      
      const result = await CompilerAPI.compile(invalidVNC, 'cpp');
      
      expect(result.success).toBe(false);
      expect(result.errors).toContain('Syntax error in VNC code');
    });

    test('should handle compilation timeout', async () => {
      const vncCode = '⋋(x, y) = navigate(x, y, energy)';
      
      // Mock compilation to simulate timeout
      jest.spyOn(CompilerAPI as any, 'performCompilation').mockImplementation(() => {
        return new Promise((_, reject) => {
          setTimeout(() => reject(new Error('Compilation timeout')), 100);
        });
      });
      
      const result = await CompilerAPI.compile(vncCode, 'cpp');
      
      expect(result.success).toBe(false);
      expect(result.errors).toContain('Compilation timeout');
    });

    test('should handle memory allocation failures', async () => {
      const largeVNC = '⋋(x, y) = navigate(x, y, energy)\n'.repeat(10000);
      
      const result = await CompilerAPI.compile(largeVNC, 'cpp');
      
      expect(result.success).toBe(false);
      expect(result.errors).toContain('Memory allocation failed');
    });
  });

  describe('Performance Tests', () => {
    test('should compile simple VNC code quickly', async () => {
      const vncCode = '⋋(x, y) = navigate(x, y, energy)';
      
      const startTime = performance.now();
      const result = await CompilerAPI.compile(vncCode, 'cpp');
      const endTime = performance.now();
      
      expect(result.success).toBe(true);
      expect(endTime - startTime).toBeLessThan(100); // Should compile in less than 100ms
    });

    test('should handle large VNC programs efficiently', async () => {
      const largeVNC = `
${Array.from({ length: 1000 }, (_, i) => `⋋(path${i}) = navigate(path${i}, energy${i})`).join('\n')}
      `;
      
      const startTime = performance.now();
      const result = await CompilerAPI.compile(largeVNC, 'python');
      const endTime = performance.now();
      
      expect(result.success).toBe(true);
      expect(result.output).toContain('def navigate');
      expect(endTime - startTime).toBeLessThan(1000); // Should compile in less than 1 second
    });

    test('should optimize compilation with caching', async () => {
      const vncCode = '⋋(x, y) = navigate(x, y, energy)';
      
      // First compilation
      const result1 = await CompilerAPI.compile(vncCode, 'cpp');
      expect(result1.success).toBe(true);
      
      // Second compilation (should be cached)
      const result2 = await CompilerAPI.compile(vncCode, 'cpp');
      expect(result2.success).toBe(true);
      expect(result2.cached).toBe(true);
    });
  });

  describe('Multi-Target Compilation', () => {
    test('should compile to multiple targets simultaneously', async () => {
      const vncCode = '⋋(x, y) = navigate(x, y, energy)';
      const targets: CompilationTarget[] = ['cpp', 'python', 'wasm'];
      
      const results = await Promise.all(
        targets.map(target => CompilerAPI.compile(vncCode, target))
      );
      
      results.forEach((result, index) => {
        expect(result.success).toBe(true);
        expect(result.target).toBe(targets[index]);
      });
    });

    test('should maintain consistency across compilation targets', async () => {
      const vncCode = `
⋋(start, end) = navigate(start, end, energy_function)
  where energy_function = λ(p) → p.distance + p.obstacle_penalty
      `;
      
      const cppResult = await CompilerAPI.compile(vncCode, 'cpp');
      const pythonResult = await CompilerAPI.compile(vncCode, 'python');
      
      expect(cppResult.success).toBe(true);
      expect(pythonResult.success).toBe(true);
      
      // Both should contain the core navigation logic
      expect(cppResult.output).toContain('navigate');
      expect(pythonResult.output).toContain('navigate');
      expect(cppResult.output).toContain('energy_function');
      expect(pythonResult.output).toContain('energy_function');
    });
  });

  describe('Cloud Compilation Features', () => {
    test('should support cloud-based compilation', async () => {
      const vncCode = '⋋(x, y) = navigate(x, y, energy)';
      
      const result = await CompilerAPI.compile(vncCode, 'cpp', { cloud: true });
      
      expect(result.success).toBe(true);
      expect(result.cloudCompiled).toBe(true);
      expect(result.output).toBeDefined();
    });

    test('should handle cloud compilation failures', async () => {
      const vncCode = '⋋(x, y) = navigate(x, y, energy)';
      
      // Mock cloud compilation failure
      jest.spyOn(CompilerAPI as any, 'cloudCompile').mockRejectedValue(
        new Error('Cloud service unavailable')
      );
      
      const result = await CompilerAPI.compile(vncCode, 'cpp', { cloud: true });
      
      expect(result.success).toBe(false);
      expect(result.errors).toContain('Cloud service unavailable');
    });
  });

  describe('Debugging and Profiling', () => {
    test('should generate debug information', async () => {
      const vncCode = '⋋(x, y) = navigate(x, y, energy)';
      
      const result = await CompilerAPI.compile(vncCode, 'cpp', { debug: true });
      
      expect(result.success).toBe(true);
      expect(result.debugInfo).toBeDefined();
      expect(result.debugInfo).toContain('Generated intermediate representation');
      expect(result.debugInfo).toContain('Optimization passes applied');
    });

    test('should profile compilation performance', async () => {
      const vncCode = '⋋(x, y) = navigate(x, y, energy)';
      
      const result = await CompilerAPI.compile(vncCode, 'cpp', { profile: true });
      
      expect(result.success).toBe(true);
      expect(result.performanceMetrics).toBeDefined();
      expect(result.performanceMetrics.parseTime).toBeGreaterThan(0);
      expect(result.performanceMetrics.compileTime).toBeGreaterThan(0);
      expect(result.performanceMetrics.totalTime).toBeGreaterThan(0);
    });
  });
});