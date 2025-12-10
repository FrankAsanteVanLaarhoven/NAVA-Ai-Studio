/**
 * Univarm Starter App Entry Point
 */

export { UnivarmStarterApp as App } from './App';
export { manifest } from './manifest';
export { default } from './App';

// Re-export utilities for external use
export { solveOptimalPath, getAvailableInterpreters, type InterpreterType, type SolverOptions, type SolverResult } from './engine/nava-engine';
export { emitPath, type Target } from './codegen/emit';

