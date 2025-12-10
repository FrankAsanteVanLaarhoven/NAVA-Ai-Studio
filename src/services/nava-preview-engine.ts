/**
 * NAVA Preview Engine
 * 
 * Converts NAVA runtime results into visualization data for live preview.
 * Handles scenarios, timing profiles, and interactive previews.
 */

import type { NAVARuntimeResult, PathPoint, Obstacle, CostGrid, TimingProfile } from './nava-runtime-service';

export interface PreviewData {
  path: PathPoint[];
  obstacles: Obstacle[];
  costGrid?: CostGrid;
  vectorField?: {
    width: number;
    height: number;
    vectors: Array<{ x: number; y: number; vx: number; vy: number }>;
  };
  reachableSet?: PathPoint[];
  timingProfile?: TimingProfile;
  metadata: {
    manifold: string;
    pathLength: number;
    executionTime: number;
    energy?: number;
    jerkMax?: number;
  };
}

export interface Scenario {
  id: string;
  name: string;
  manifold: string;
  start: { x: number; y: number; z?: number };
  goal: { x: number; y: number; z?: number };
  obstacles: Obstacle[];
  costModel: string;
  navaCode: string;
}

class NAVAPreviewEngine {
  /**
   * Convert runtime result to preview data
   */
  toPreviewData(result: NAVARuntimeResult): PreviewData {
    return {
      path: result.path || [],
      obstacles: result.obstacles || [],
      costGrid: result.costGrid,
      metadata: result.metadata || {
        manifold: 'R3',
        pathLength: 0,
        executionTime: 0,
      },
      timingProfile: result.visualization?.timingProfile,
    };
  }

  /**
   * Generate scenario variants
   */
  generateScenarios(baseCode: string, numVariants: number = 3): Scenario[] {
    const scenarios: Scenario[] = [];
    
    // Parse base code to extract parameters
    const startMatch = baseCode.match(/start\s*⋋\s*=\s*Vector3D\s*⋋\s*\(([^)]+)\)/i);
    const goalMatch = baseCode.match(/goal\s*⋋\s*=\s*Vector3D\s*⋋\s*\(([^)]+)\)/i);
    
    const baseStart = startMatch ? startMatch[1].split(',').map(s => parseFloat(s.trim())) : [0, 0, 0];
    const baseGoal = goalMatch ? goalMatch[1].split(',').map(s => parseFloat(s.trim())) : [10, 10, 0];
    
    for (let i = 0; i < numVariants; i++) {
      // Vary start/goal positions
      const start = {
        x: baseStart[0] + (Math.random() - 0.5) * 5,
        y: baseStart[1] + (Math.random() - 0.5) * 5,
        z: baseStart[2] || 0,
      };
      
      const goal = {
        x: baseGoal[0] + (Math.random() - 0.5) * 5,
        y: baseGoal[1] + (Math.random() - 0.5) * 5,
        z: baseGoal[2] || 0,
      };
      
      // Generate variant code
      const variantCode = baseCode
        .replace(/start\s*⋋\s*=\s*Vector3D\s*⋋\s*\([^)]+\)/i, 
                 `start⋋ = Vector3D⋋(${start.x}, ${start.y}, ${start.z})`)
        .replace(/goal\s*⋋\s*=\s*Vector3D\s*⋋\s*\([^)]+\)/i,
                 `goal⋋ = Vector3D⋋(${goal.x}, ${goal.y}, ${goal.z})`);
      
      scenarios.push({
        id: `scenario-${i + 1}`,
        name: `Scenario ${i + 1}`,
        manifold: 'R3',
        start,
        goal,
        obstacles: [],
        costModel: 'geodesic',
        navaCode: variantCode,
      });
    }
    
    return scenarios;
  }

  /**
   * Generate timing profile for DAAT/PDL/RT-shields visualization
   */
  generateTimingProfile(
    path: PathPoint[],
    options?: {
      frequency?: number; // Hz
      dmrThreshold?: number; // Deadline miss rate
      ajThreshold?: number; // Arrival jitter (ms)
      ttpThreshold?: number; // Time to process (ms)
    }
  ): TimingProfile {
    const frequency = options?.frequency || 50;
    const dt = 1.0 / frequency;
    const segments: TimingProfile['segments'] = [];
    
    // Divide path into segments based on timing constraints
    const segmentLength = Math.ceil(path.length / 10);
    
    for (let i = 0; i < path.length; i += segmentLength) {
      const segment = path.slice(i, Math.min(i + segmentLength, path.length));
      const startTime = segment[0]?.t || i * dt;
      const endTime = segment[segment.length - 1]?.t || (i + segment.length) * dt;
      
      // Calculate timing metrics for this segment
      const segmentDuration = endTime - startTime;
      const expectedDuration = segment.length * dt;
      const dmr = segmentDuration > expectedDuration ? 
        (segmentDuration - expectedDuration) / expectedDuration : 0;
      
      // Determine tier based on timing pressure
      let tier: 'tier-0' | 'tier-1' | 'tier-2';
      if (dmr > 0.1) {
        tier = 'tier-0'; // Emergency
      } else if (dmr > 0.05) {
        tier = 'tier-1'; // Chunked VLA
      } else {
        tier = 'tier-2'; // Offline planner
      }
      
      // Calculate arrival jitter (simplified)
      const aj = Math.abs(segmentDuration - expectedDuration) * 1000; // ms
      
      // Time to process (simplified)
      const ttp = segmentDuration * 1000; // ms
      
      // Contract status
      const dmrThreshold = options?.dmrThreshold || 0.005; // 0.5%
      const ajThreshold = options?.ajThreshold || 10; // 10ms
      const ttpThreshold = options?.ttpThreshold || 10; // 10ms
      
      let contractStatus: 'pass' | 'fail' | 'warning';
      if (dmr <= dmrThreshold && aj <= ajThreshold && ttp <= ttpThreshold) {
        contractStatus = 'pass';
      } else if (dmr > dmrThreshold * 2 || aj > ajThreshold * 2 || ttp > ttpThreshold * 2) {
        contractStatus = 'fail';
      } else {
        contractStatus = 'warning';
      }
      
      segments.push({
        start: startTime,
        end: endTime,
        tier,
        dmr,
        aj,
        ttp,
        contractStatus,
      });
    }
    
    return { segments };
  }

  /**
   * Generate vector field for visualization
   */
  generateVectorField(
    path: PathPoint[],
    obstacles: Obstacle[],
    bounds: { min: { x: number; y: number }; max: { x: number; y: number } },
    resolution: number = 20
  ): {
    width: number;
    height: number;
    vectors: Array<{ x: number; y: number; vx: number; vy: number }>;
  } {
    const width = resolution;
    const height = resolution;
    const vectors: Array<{ x: number; y: number; vx: number; vy: number }> = [];
    
    const stepX = (bounds.max.x - bounds.min.x) / width;
    const stepY = (bounds.max.y - bounds.min.y) / height;
    
    // Find goal (last point of path)
    const goal = path.length > 0 ? path[path.length - 1] : null;
    
    for (let i = 0; i < height; i++) {
      for (let j = 0; j < width; j++) {
        const x = bounds.min.x + j * stepX;
        const y = bounds.min.y + i * stepY;
        
        // Calculate attractive field toward goal
        let vx = 0;
        let vy = 0;
        
        if (goal) {
          const dx = goal.x - x;
          const dy = goal.y - y;
          const dist = Math.sqrt(dx * dx + dy * dy);
          if (dist > 0) {
            vx = dx / dist * 0.5; // Normalized, scaled
            vy = dy / dist * 0.5;
          }
        }
        
        // Add repulsive field from obstacles
        for (const obstacle of obstacles) {
          if (obstacle.type === 'circle' && obstacle.center && obstacle.radius) {
            const dx = x - obstacle.center.x;
            const dy = y - obstacle.center.y;
            const dist = Math.sqrt(dx * dx + dy * dy);
            
            if (dist < obstacle.radius * 2 && dist > 0) {
              const repulsion = obstacle.radius / dist;
              vx -= (dx / dist) * repulsion * 2;
              vy -= (dy / dist) * repulsion * 2;
            }
          }
        }
        
        // Normalize vector
        const magnitude = Math.sqrt(vx * vx + vy * vy);
        if (magnitude > 0) {
          vx = vx / magnitude * 0.3; // Scale for visualization
          vy = vy / magnitude * 0.3;
        }
        
        vectors.push({ x, y, vx, vy });
      }
    }
    
    return { width, height, vectors };
  }
}

export const navaPreviewEngine = new NAVAPreviewEngine();

