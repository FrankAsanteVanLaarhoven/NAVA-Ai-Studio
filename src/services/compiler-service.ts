import { invoke } from '@tauri-apps/api/tauri';
import { CompilationTarget, CompilationResult } from '../types';

/**
 * Multi-Target Compiler Service
 */
export class CompilerService {
  async compileToTarget(code: string, target: CompilationTarget): Promise<CompilationResult> {
    try {
      const compiledCode = await invoke<string>('compile_to_target', {
        code,
        target,
      });

      return {
        target,
        code: compiledCode,
        dependencies: [],
        buildInstructions: '',
        success: true,
      };
    } catch (error) {
      return {
        target,
        code: '',
        dependencies: [],
        buildInstructions: '',
        success: false,
        error: String(error),
      };
    }
  }

  async compileToAllTargets(code: string): Promise<CompilationResult[]> {
    const targets: CompilationTarget[] = ['cpp', 'python', 'wasm', 'glsl'];
    const results = await Promise.all(
      targets.map((target) => this.compileToTarget(code, target))
    );
    return results;
  }
}

export const compilerService = new CompilerService();

