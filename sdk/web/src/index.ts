/**
 * NAVΛ SDK for Web Platform
 * 
 * TypeScript/JavaScript bindings for NAVΛ SDK
 * Uses WebAssembly for high-performance computation
 */

import init, { NavigationField as WasmNavigationField } from '@nava/sdk-wasm';

// Initialize WebAssembly module
let wasmInitialized = false;

/**
 * Initialize the NAVΛ SDK WebAssembly module
 * Must be called before using any SDK functionality
 */
export async function initSDK(): Promise<void> {
  if (!wasmInitialized) {
    await init();
    wasmInitialized = true;
  }
}

/**
 * Manifold types
 */
export enum ManifoldType {
  Euclidean = 'euclidean',
  Riemannian = 'riemannian',
  Lorentzian = 'lorentzian',
}

/**
 * Manifold configuration
 */
export class Manifold {
  constructor(
    public type: ManifoldType,
    public dimension: number,
    public metric?: string
  ) {}

  /**
   * Create a Euclidean manifold
   */
  static euclidean(dimension: number): Manifold {
    return new Manifold(ManifoldType.Euclidean, dimension);
  }

  /**
   * Create a Riemannian manifold
   */
  static riemannian(dimension: number, metric: string): Manifold {
    return new Manifold(ManifoldType.Riemannian, dimension, metric);
  }
}

/**
 * Navigation constraints
 */
export interface NavigationConstraints {
  maxVelocity: number;
  avoidObstacles: boolean;
  custom?: Record<string, any>;
}

/**
 * Default navigation constraints
 */
export const NavigationConstraints = {
  default(): NavigationConstraints {
    return {
      maxVelocity: 1.0,
      avoidObstacles: true,
    };
  },
};

/**
 * Path point
 */
export interface PathPoint {
  coordinates: number[];
  energy: number;
}

/**
 * Navigation path
 */
export interface Path {
  waypoints: PathPoint[];
  totalEnergy: number;
  optimizationMethod: string;
}

/**
 * Navigation Field - Main SDK class
 */
export class NavigationField {
  private wasmField: WasmNavigationField | null = null;

  constructor() {
    // Lazy initialization - will be initialized when first used
  }

  /**
   * Initialize the WebAssembly module if not already initialized
   */
  private async ensureInitialized(): Promise<void> {
    if (!wasmInitialized) {
      await initSDK();
    }
    if (!this.wasmField) {
      this.wasmField = new WasmNavigationField();
    }
  }

  /**
   * Set the manifold for this navigation field
   */
  async setManifold(manifold: Manifold): Promise<void> {
    await this.ensureInitialized();
    // Implementation would call WASM
    // For now, store in memory
  }

  /**
   * Find an optimal path from start to goal
   */
  async findOptimalPath(
    start: number[],
    goal: number[],
    constraints: NavigationConstraints = NavigationConstraints.default()
  ): Promise<Path> {
    await this.ensureInitialized();

    // Validate inputs
    if (start.length !== goal.length) {
      throw new Error('Start and goal must have same dimension');
    }

    // Call WebAssembly implementation
    // For now, return a simplified path
    const numWaypoints = Math.max(10, Math.ceil(constraints.maxVelocity * 10));
    const waypoints: PathPoint[] = [];

    for (let i = 0; i <= numWaypoints; i++) {
      const t = i / numWaypoints;
      const coordinates = start.map((s, idx) => s + (goal[idx] - s) * t);
      waypoints.push({
        coordinates,
        energy: i * 0.1,
      });
    }

    return {
      waypoints,
      totalEnergy: waypoints.reduce((sum, p) => sum + p.energy, 0),
      optimizationMethod: 'vnc', // Van Laarhoven Navigation Calculus
    };
  }
}

/**
 * Get SDK version
 */
export function getVersion(): string {
  return '0.1.0';
}

// Auto-initialize on import (optional - can be removed for manual init)
if (typeof window !== 'undefined') {
  // Browser environment - can auto-init
  // initSDK().catch(console.error);
}

