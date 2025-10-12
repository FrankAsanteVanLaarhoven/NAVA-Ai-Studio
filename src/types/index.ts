// NAVΛ Studio Type Definitions

export interface NavigationPath {
  waypoints: [number, number, number][];
  energy: number;
  optimizationMethod: 'vnc' | 'classical';
}

export interface EnergyLandscape {
  points: EnergyPoint[];
  dimensions: [number, number, number];
}

export interface EnergyPoint {
  position: [number, number, number];
  energy: number;
}

export interface OptimizationStep {
  iteration: number;
  energy: number;
  gradientNorm: number;
}

export interface NavigationVisualization {
  paths: NavigationPath[];
  energyLandscape: EnergyPoint[];
  optimizationSteps: OptimizationStep[];
}

export interface VncSymbol {
  symbol: string;
  name: string;
  documentation: string;
  insertText: string;
}

export interface CompilationResult {
  target: CompilationTarget;
  code: string;
  dependencies: string[];
  buildInstructions: string;
  success: boolean;
  error?: string;
}

export type CompilationTarget = 'cpp' | 'python' | 'wasm' | 'glsl' | 'helm' | 'docker';

export interface EditorTheme {
  name: string;
  colors: Record<string, string>;
  tokenColors: TokenColor[];
}

export interface TokenColor {
  token: string;
  foreground: string;
  fontStyle?: string;
}

export interface ProjectFile {
  path: string;
  content: string;
  language: string;
}

