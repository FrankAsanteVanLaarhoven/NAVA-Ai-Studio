/**
 * Compiler Types
 * 
 * Type definitions for the VNC compiler
 */

export type CompilationTarget = 'wasm' | 'cpp' | 'python' | 'glsl';

export interface CompilerOptions {
  target: CompilationTarget;
  optimizationLevel: number;
  debug: boolean;
}

export interface CompilationResult {
  success: boolean;
  output: string;
  errors: string[];
  sourceMap?: string;
}

export interface CompilerError {
  message: string;
  code: string;
  location?: {
    file: string;
    line: number;
    column: number;
  };
}

export interface CompilerWarning {
  message: string;
  code: string;
  location?: {
    file: string;
    line: number;
    column: number;
  };
}