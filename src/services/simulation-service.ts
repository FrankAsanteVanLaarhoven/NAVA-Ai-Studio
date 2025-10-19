/**
 * Simulation Service
 * 
 * Communicates with the Rust-based NAVΛ Simulation Platform backend
 * via REST API for real-time robotics simulation
 */

export interface SimulationState {
  time: number;
  is_running: boolean;
  robots: RobotState[];
  obstacles: Obstacle[];
}

export interface RobotState {
  id: string;
  name: string;
  position: [number, number, number];
}

export interface Obstacle {
  id: string;
  shape: 'box' | 'sphere' | 'cylinder' | 'cone';
  position: [number, number, number];
  rotation: [number, number, number];
  scale: [number, number, number];
}

export interface AddRobotRequest {
  id: string;
  name: string;
  model: 'differential_drive' | 'ackermann' | 'legged' | 'aerial' | 'marine';
  max_linear_velocity: number;
  max_angular_velocity: number;
}

export interface ControlRobotRequest {
  robot_id: string;
  linear_velocity: number;
  angular_velocity: number;
}

export interface AddObstacleRequest {
  id: string;
  shape: 'box' | 'sphere' | 'cylinder' | 'cone';
  position: [number, number, number];
  rotation: [number, number, number];
  scale: [number, number, number];
}

class SimulationService {
  private baseUrl: string;
  private pollingInterval: NodeJS.Timeout | null = null;
  private stateCallbacks: ((state: SimulationState) => void)[] = [];

  constructor(baseUrl: string = 'http://localhost:3030') {
    this.baseUrl = baseUrl;
  }

  /**
   * Get current simulation state
   */
  async getState(): Promise<SimulationState> {
    const response = await fetch(`${this.baseUrl}/api/state`);
    const result = await response.json();
    
    if (result.success && result.data) {
      return JSON.parse(result.data);
    }
    throw new Error(result.error || 'Failed to get simulation state');
  }

  /**
   * Start the simulation
   */
  async start(): Promise<void> {
    const response = await fetch(`${this.baseUrl}/api/start`, {
      method: 'POST',
    });
    const result = await response.json();
    
    if (!result.success) {
      throw new Error(result.message || 'Failed to start simulation');
    }
  }

  /**
   * Pause the simulation
   */
  async pause(): Promise<void> {
    const response = await fetch(`${this.baseUrl}/api/pause`, {
      method: 'POST',
    });
    const result = await response.json();
    
    if (!result.success) {
      throw new Error(result.message || 'Failed to pause simulation');
    }
  }

  /**
   * Stop the simulation
   */
  async stop(): Promise<void> {
    const response = await fetch(`${this.baseUrl}/api/stop`, {
      method: 'POST',
    });
    const result = await response.json();
    
    if (!result.success) {
      throw new Error(result.message || 'Failed to stop simulation');
    }
  }

  /**
   * Reset the simulation
   */
  async reset(): Promise<void> {
    const response = await fetch(`${this.baseUrl}/api/reset`, {
      method: 'POST',
    });
    const result = await response.json();
    
    if (!result.success) {
      throw new Error(result.message || 'Failed to reset simulation');
    }
  }

  /**
   * Add a robot to the simulation
   */
  async addRobot(robot: AddRobotRequest): Promise<void> {
    const response = await fetch(`${this.baseUrl}/api/robot`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify(robot),
    });
    const result = await response.json();
    
    if (!result.success) {
      throw new Error(result.message || 'Failed to add robot');
    }
  }

  /**
   * Control a robot
   */
  async controlRobot(control: ControlRobotRequest): Promise<void> {
    const response = await fetch(`${this.baseUrl}/api/robot/control`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify(control),
    });
    const result = await response.json();
    
    if (!result.success) {
      throw new Error(result.message || 'Failed to control robot');
    }
  }

  /**
   * Add an obstacle to the world
   */
  async addObstacle(obstacle: AddObstacleRequest): Promise<void> {
    const response = await fetch(`${this.baseUrl}/api/obstacle`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify(obstacle),
    });
    const result = await response.json();
    
    if (!result.success) {
      throw new Error(result.message || 'Failed to add obstacle');
    }
  }

  /**
   * Remove an obstacle from the world
   */
  async removeObstacle(id: string): Promise<void> {
    const response = await fetch(`${this.baseUrl}/api/obstacle/${id}`, {
      method: 'DELETE',
    });
    const result = await response.json();
    
    if (!result.success) {
      throw new Error(result.message || 'Failed to remove obstacle');
    }
  }

  /**
   * Start polling for simulation state updates
   */
  startPolling(interval: number = 100): void {
    if (this.pollingInterval) {
      this.stopPolling();
    }

    this.pollingInterval = setInterval(async () => {
      try {
        const state = await this.getState();
        this.notifyStateCallbacks(state);
      } catch (error) {
        console.error('Failed to poll simulation state:', error);
      }
    }, interval);
  }

  /**
   * Stop polling for simulation state updates
   */
  stopPolling(): void {
    if (this.pollingInterval) {
      clearInterval(this.pollingInterval);
      this.pollingInterval = null;
    }
  }

  /**
   * Subscribe to simulation state updates
   */
  onStateUpdate(callback: (state: SimulationState) => void): () => void {
    this.stateCallbacks.push(callback);
    
    // Return unsubscribe function
    return () => {
      const index = this.stateCallbacks.indexOf(callback);
      if (index > -1) {
        this.stateCallbacks.splice(index, 1);
      }
    };
  }

  /**
   * Notify all state callbacks
   */
  private notifyStateCallbacks(state: SimulationState): void {
    this.stateCallbacks.forEach(callback => callback(state));
  }
}

// Export singleton instance
export const simulationService = new SimulationService();
export default simulationService;

