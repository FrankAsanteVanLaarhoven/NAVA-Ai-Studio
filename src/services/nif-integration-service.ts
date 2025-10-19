/**
 * NAVΛ-NIF Integration Service
 * 
 * Connects NAVΛ SIM with NIF (Navigation Integrity Framework)
 * Provides real-time pose graph optimization and integrity monitoring
 * 
 * Patent-worthy features:
 * - Real-time optimization during simulation
 * - Live integrity bounds
 * - Automatic anomaly detection
 * - Multi-sensor fusion
 */

export interface NIFStatus {
  poses: number;
  observations: number;
  integrity: number;  // 0-1 score
  optimizing: boolean;
  has_solution: boolean;
}

export interface OptimizationResult {
  status: 'success' | 'error' | 'insufficient_data';
  num_poses?: number;
  num_observations?: number;
  integrity_score?: number;
  optimized_trajectory?: number[][];
  metrics?: {
    rmse: number;
    max_error: number;
    cep95: number;
  };
  message?: string;
}

export interface NIFTrajectory {
  trajectory: number[][];
  integrity: number;
}

class NIFIntegrationService {
  private baseUrl: string;
  private statusPollingInterval: NodeJS.Timeout | null = null;
  private statusCallbacks: ((status: NIFStatus) => void)[] = [];

  constructor(baseUrl: string = 'http://localhost:5051') {
    this.baseUrl = baseUrl;
  }

  /**
   * Get current NIF solver status
   */
  async getStatus(): Promise<NIFStatus> {
    const response = await fetch(`${this.baseUrl}/api/nif/status`);
    if (!response.ok) {
      throw new Error('Failed to get NIF status');
    }
    return await response.json();
  }

  /**
   * Add simulation frame to NIF
   */
  async addFrame(frame: {
    timestamp: number;
    position: [number, number, number];
    orientation: [number, number, number, number];
    lidar?: any;
    camera?: any;
  }): Promise<void> {
    const response = await fetch(`${this.baseUrl}/api/nif/add_frame`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(frame),
    });

    if (!response.ok) {
      throw new Error('Failed to add frame to NIF');
    }
  }

  /**
   * Trigger pose graph optimization
   */
  async optimize(): Promise<OptimizationResult> {
    const response = await fetch(`${this.baseUrl}/api/nif/optimize`, {
      method: 'POST',
    });

    if (!response.ok) {
      throw new Error('Failed to optimize');
    }

    return await response.json();
  }

  /**
   * Start real-time integration
   */
  async start(): Promise<void> {
    const response = await fetch(`${this.baseUrl}/api/nif/start`, {
      method: 'POST',
    });

    if (!response.ok) {
      throw new Error('Failed to start integration');
    }

    console.log('🚀 NIF real-time integration started');
  }

  /**
   * Stop real-time integration
   */
  async stop(): Promise<void> {
    const response = await fetch(`${this.baseUrl}/api/nif/stop`, {
      method: 'POST',
    });

    if (!response.ok) {
      throw new Error('Failed to stop integration');
    }

    console.log('⏹️ NIF integration stopped');
  }

  /**
   * Reset NIF solver
   */
  async reset(): Promise<void> {
    const response = await fetch(`${this.baseUrl}/api/nif/reset`, {
      method: 'POST',
    });

    if (!response.ok) {
      throw new Error('Failed to reset NIF');
    }

    console.log('🔄 NIF solver reset');
  }

  /**
   * Get optimized trajectory
   */
  async getTrajectory(): Promise<NIFTrajectory> {
    const response = await fetch(`${this.baseUrl}/api/nif/trajectory`);
    
    if (!response.ok) {
      throw new Error('Failed to get trajectory');
    }

    return await response.json();
  }

  /**
   * Start polling for status updates
   */
  startStatusPolling(interval: number = 500): void {
    if (this.statusPollingInterval) {
      this.stopStatusPolling();
    }

    this.statusPollingInterval = setInterval(async () => {
      try {
        const status = await this.getStatus();
        this.notifyStatusCallbacks(status);
      } catch (error) {
        console.error('Failed to poll NIF status:', error);
      }
    }, interval);
  }

  /**
   * Stop polling for status updates
   */
  stopStatusPolling(): void {
    if (this.statusPollingInterval) {
      clearInterval(this.statusPollingInterval);
      this.statusPollingInterval = null;
    }
  }

  /**
   * Subscribe to status updates
   */
  onStatusUpdate(callback: (status: NIFStatus) => void): () => void {
    this.statusCallbacks.push(callback);
    
    return () => {
      const index = this.statusCallbacks.indexOf(callback);
      if (index > -1) {
        this.statusCallbacks.splice(index, 1);
      }
    };
  }

  private notifyStatusCallbacks(status: NIFStatus): void {
    this.statusCallbacks.forEach(callback => callback(status));
  }

  /**
   * Check if NIF service is available
   */
  async checkHealth(): Promise<boolean> {
    try {
      const response = await fetch(`${this.baseUrl}/health`);
      return response.ok;
    } catch (error) {
      return false;
    }
  }
}

// Export singleton instance
export const nifIntegrationService = new NIFIntegrationService();
export default nifIntegrationService;

