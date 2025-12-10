/**
 * NAVA Runtime Service
 * 
 * Executes NAVA/VΛNC code and returns results for preview and visualization.
 * This is the backend service that powers live preview in the IDE.
 */

export interface NAVARuntimeResult {
  success: boolean;
  path?: PathPoint[];
  obstacles?: Obstacle[];
  costGrid?: CostGrid;
  metadata?: {
    manifold: string;
    pathLength: number;
    executionTime: number;
    energy?: number;
    jerkMax?: number;
    stability?: number;
  };
  error?: string;
  visualization?: VisualizationData;
}

export interface PathPoint {
  x: number;
  y: number;
  z?: number;
  t?: number; // Time
  cost?: number;
  energy?: number;
}

export interface Obstacle {
  type: 'circle' | 'rectangle' | 'polygon' | 'custom';
  center?: { x: number; y: number; z?: number };
  radius?: number;
  bounds?: { min: { x: number; y: number }; max: { x: number; y: number } };
  vertices?: Array<{ x: number; y: number }>;
  metadata?: Record<string, any>;
}

export interface CostGrid {
  width: number;
  height: number;
  resolution: number;
  values: number[][]; // 2D grid of cost values
  bounds: {
    min: { x: number; y: number };
    max: { x: number; y: number };
  };
}

export interface VisualizationData {
  path: PathPoint[];
  obstacles: Obstacle[];
  costGrid?: CostGrid;
  vectorField?: VectorField;
  reachableSet?: PathPoint[];
  timingProfile?: TimingProfile;
}

export interface VectorField {
  width: number;
  height: number;
  vectors: Array<{
    x: number;
    y: number;
    vx: number; // velocity x
    vy: number; // velocity y
  }>;
}

export interface TimingProfile {
  segments: Array<{
    start: number;
    end: number;
    tier: 'tier-0' | 'tier-1' | 'tier-2';
    dmr: number;
    aj: number;
    ttp: number;
    contractStatus: 'pass' | 'fail' | 'warning';
  }>;
}

class NAVARuntimeService {
  /**
   * Run NAVA code and return results
   */
  async runNAVA(code: string, options?: {
    preview?: boolean;
    samples?: number;
    interpreter?: 'auto' | 'tauri-backend' | 'python' | 'wasm' | 'browser-sim';
  }): Promise<NAVARuntimeResult> {
    try {
      // Parse NAVA code to extract key information
      const parsed = this.parseNAVACode(code);
      
      // Use the existing NAVA engine for path computation
      if (parsed.hasPathComputation) {
        const { solveOptimalPath } = await import('../apps/univarm-starter/engine/nava-engine');
        
        const result = await solveOptimalPath({
          start: parsed.start || { x: 0, y: 0, z: 0 },
          goal: parsed.goal || { x: 10, y: 10, z: 0 },
          samples: options?.samples || 64,
          interpreter: options?.interpreter || 'auto',
          energyLandscape: parsed.hasEnergyLandscape,
        });
        
        // Convert to runtime result format
        const path: PathPoint[] = result.points.map((p, i) => ({
          x: p.x,
          y: p.y,
          z: p.z,
          t: i * 0.1, // Approximate time
          cost: result.cost / result.points.length,
          energy: result.energy ? result.energy / result.points.length : undefined,
        }));
        
        // Extract obstacles if present
        const obstacles: Obstacle[] = parsed.obstacles || [];
        
        // Generate cost grid if preview mode
        let costGrid: CostGrid | undefined;
        if (options?.preview) {
          costGrid = this.generateCostGrid(path, obstacles, parsed.bounds);
        }
        
        // Generate visualization data
        const visualization: VisualizationData = {
          path,
          obstacles,
          costGrid,
        };
        
        return {
          success: true,
          path,
          obstacles,
          costGrid,
          metadata: {
            manifold: parsed.manifold || 'R3',
            pathLength: this.calculatePathLength(path),
            executionTime: result.metadata?.executionTime || 0,
            energy: result.energy,
            jerkMax: this.calculateJerkMax(path),
          },
          visualization,
        };
      }
      
      // For non-path code, return basic success
      return {
        success: true,
        path: [],
        obstacles: [],
        metadata: {
          manifold: parsed.manifold || 'R3',
          pathLength: 0,
          executionTime: 0,
        },
      };
    } catch (error: any) {
      return {
        success: false,
        error: error.message || 'Failed to execute NAVA code',
        path: [],
        obstacles: [],
      };
    }
  }

  /**
   * Analyze NAVA code and return invariants, costs, stability metrics
   */
  async analyzeNAVA(code: string): Promise<{
    invariants: string[];
    costs: { type: string; value: number }[];
    stability: number;
    warnings: string[];
  }> {
    const parsed = this.parseNAVACode(code);
    const invariants: string[] = [];
    const costs: { type: string; value: number }[] = [];
    const warnings: string[] = [];
    
    // Check for common invariants
    if (parsed.hasPathComputation) {
      invariants.push('Path exists between start and goal');
      if (parsed.obstacles && parsed.obstacles.length > 0) {
        invariants.push('Obstacle avoidance is active');
      }
    }
    
    if (parsed.hasEnergyLandscape) {
      invariants.push('Energy landscape is defined');
      costs.push({ type: 'energy', value: 0 }); // Would be computed from actual run
    }
    
    // Check for potential issues
    if (!parsed.start || !parsed.goal) {
      warnings.push('Start or goal not defined');
    }
    
    if (parsed.obstacles && parsed.obstacles.length === 0 && code.includes('obstacle')) {
      warnings.push('Obstacle mentioned but not defined');
    }
    
    return {
      invariants,
      costs,
      stability: 0.85, // Would be computed from actual analysis
      warnings,
    };
  }

  /**
   * Parse NAVA code to extract key information
   */
  private parseNAVACode(code: string): {
    hasPathComputation: boolean;
    start?: { x: number; y: number; z: number };
    goal?: { x: number; y: number; z: number };
    obstacles?: Obstacle[];
    manifold?: string;
    hasEnergyLandscape: boolean;
    bounds?: { min: { x: number; y: number }; max: { x: number; y: number } };
  } {
    const lower = code.toLowerCase();
    const hasPathComputation = lower.includes('find_optimal_path') || 
                               lower.includes('navigate_to') ||
                               lower.includes('compute_optimal_path');
    
    // Extract start point
    const startMatch = code.match(/start\s*⋋\s*=\s*Vector3D\s*⋋\s*\(([^)]+)\)/i) ||
                      code.match(/start\s*=\s*\[([^\]]+)\]/i);
    let start: { x: number; y: number; z: number } | undefined;
    if (startMatch) {
      const coords = startMatch[1].split(',').map(s => parseFloat(s.trim()));
      start = { x: coords[0] || 0, y: coords[1] || 0, z: coords[2] || 0 };
    }
    
    // Extract goal point
    const goalMatch = code.match(/goal\s*⋋\s*=\s*Vector3D\s*⋋\s*\(([^)]+)\)/i) ||
                     code.match(/goal\s*=\s*\[([^\]]+)\]/i);
    let goal: { x: number; y: number; z: number } | undefined;
    if (goalMatch) {
      const coords = goalMatch[1].split(',').map(s => parseFloat(s.trim()));
      goal = { x: coords[0] || 0, y: coords[1] || 0, z: coords[2] || 0 };
    }
    
    // Extract obstacles
    const obstacles: Obstacle[] = [];
    const obstacleMatches = code.matchAll(/obstacle[^=]*=\s*(?:disc|circle)\s*\([^)]+\)/gi);
    for (const match of obstacleMatches) {
      const centerMatch = match[0].match(/center[:\s]*\[([^\]]+)\]/i);
      const radiusMatch = match[0].match(/radius[:\s]*([\d.]+)/i);
      
      if (centerMatch && radiusMatch) {
        const coords = centerMatch[1].split(',').map(s => parseFloat(s.trim()));
        obstacles.push({
          type: 'circle',
          center: { x: coords[0] || 0, y: coords[1] || 0 },
          radius: parseFloat(radiusMatch[1]),
        });
      }
    }
    
    // Detect manifold
    let manifold = 'R3';
    if (code.includes('euclidean_plane') || code.includes('R2')) {
      manifold = 'R2';
    } else if (code.includes('sphere') || code.includes('S2')) {
      manifold = 'S2';
    } else if (code.includes('torus') || code.includes('T2')) {
      manifold = 'T2';
    }
    
    // Check for energy landscape
    const hasEnergyLandscape = lower.includes('energy_landscape') || 
                              lower.includes('energy_field');
    
    // Calculate bounds from start/goal/obstacles
    let bounds: { min: { x: number; y: number }; max: { x: number; y: number } } | undefined;
    if (start || goal || obstacles.length > 0) {
      const allPoints = [
        start ? [start.x, start.y] : [],
        goal ? [goal.x, goal.y] : [],
        ...obstacles.map(o => o.center ? [o.center.x, o.center.y] : []),
      ].filter(p => p.length > 0) as number[][];
      
      if (allPoints.length > 0) {
        const xs = allPoints.map(p => p[0]);
        const ys = allPoints.map(p => p[1]);
        const padding = 2;
        bounds = {
          min: { x: Math.min(...xs) - padding, y: Math.min(...ys) - padding },
          max: { x: Math.max(...xs) + padding, y: Math.max(...ys) + padding },
        };
      }
    }
    
    return {
      hasPathComputation,
      start,
      goal,
      obstacles,
      manifold,
      hasEnergyLandscape,
      bounds,
    };
  }

  /**
   * Generate cost grid for visualization
   */
  private generateCostGrid(
    path: PathPoint[],
    obstacles: Obstacle[],
    bounds?: { min: { x: number; y: number }; max: { x: number; y: number } }
  ): CostGrid {
    const resolution = 50;
    const defaultBounds = bounds || {
      min: { x: -5, y: -5 },
      max: { x: 15, y: 15 },
    };
    
    const width = resolution;
    const height = resolution;
    const values: number[][] = [];
    
    const stepX = (defaultBounds.max.x - defaultBounds.min.x) / width;
    const stepY = (defaultBounds.max.y - defaultBounds.min.y) / height;
    
    for (let i = 0; i < height; i++) {
      const row: number[] = [];
      for (let j = 0; j < width; j++) {
        const x = defaultBounds.min.x + j * stepX;
        const y = defaultBounds.min.y + i * stepY;
        
        // Calculate cost: distance from path + obstacle penalty
        let cost = 0;
        
        // Distance to nearest path point
        if (path.length > 0) {
          const minDist = Math.min(...path.map(p => 
            Math.sqrt((p.x - x) ** 2 + (p.y - y) ** 2)
          ));
          cost += minDist * 0.1;
        }
        
        // Obstacle penalty
        for (const obstacle of obstacles) {
          if (obstacle.type === 'circle' && obstacle.center && obstacle.radius) {
            const dist = Math.sqrt(
              (x - obstacle.center.x) ** 2 + (y - obstacle.center.y) ** 2
            );
            if (dist < obstacle.radius) {
              cost += 100; // High penalty inside obstacle
            } else if (dist < obstacle.radius * 1.5) {
              cost += 10; // Medium penalty near obstacle
            }
          }
        }
        
        row.push(cost);
      }
      values.push(row);
    }
    
    return {
      width,
      height,
      resolution,
      values,
      bounds: defaultBounds,
    };
  }

  /**
   * Calculate path length
   */
  private calculatePathLength(path: PathPoint[]): number {
    if (path.length < 2) return 0;
    
    let length = 0;
    for (let i = 1; i < path.length; i++) {
      const dx = path[i].x - path[i - 1].x;
      const dy = path[i].y - path[i - 1].y;
      const dz = (path[i].z || 0) - (path[i - 1].z || 0);
      length += Math.sqrt(dx * dx + dy * dy + dz * dz);
    }
    return length;
  }

  /**
   * Calculate maximum jerk
   */
  private calculateJerkMax(path: PathPoint[]): number {
    if (path.length < 3) return 0;
    
    let maxJerk = 0;
    for (let i = 2; i < path.length; i++) {
      const dt = (path[i].t || i) - (path[i - 1].t || i - 1);
      if (dt > 0) {
        const acc1 = {
          x: (path[i].x - path[i - 1].x) / dt,
          y: (path[i].y - path[i - 1].y) / dt,
        };
        const acc2 = {
          x: (path[i - 1].x - path[i - 2].x) / dt,
          y: (path[i - 1].y - path[i - 2].y) / dt,
        };
        const jerk = Math.sqrt(
          ((acc1.x - acc2.x) / dt) ** 2 + ((acc1.y - acc2.y) / dt) ** 2
        );
        maxJerk = Math.max(maxJerk, jerk);
      }
    }
    return maxJerk;
  }
}

export const navaRuntimeService = new NAVARuntimeService();

