/**
 * NAVΛ Unified Dataset Ingestion Service
 * 
 * World-class data ingestion from leading robotics datasets:
 * - Waymo Open Dataset (autonomous driving)
 * - KITTI Dataset (autonomous driving benchmark)
 * - Open X-Embodiment RT-X Models (robotic manipulation)
 * 
 * Patent-worthy features:
 * - Real-time streaming architecture
 * - Multi-modal data fusion
 * - Automatic format conversion
 * - Live training pipeline integration
 */

export interface WaymoFrame {
  timestamp: number;
  pose: {
    position: [number, number, number];
    orientation: [number, number, number, number]; // quaternion
  };
  lidar: {
    points: Float32Array;
    intensities: Float32Array;
  };
  camera: {
    images: ImageData[];
    labels: WaymoLabel[];
  };
  velocity: [number, number, number];
}

export interface WaymoLabel {
  id: string;
  type: 'vehicle' | 'pedestrian' | 'cyclist' | 'sign';
  bbox: {
    center: [number, number, number];
    size: [number, number, number];
    heading: number;
  };
}

export interface KITTIFrame {
  timestamp: number;
  pose: {
    position: [number, number, number];
    rotation: number[][]; // 3x3 rotation matrix
  };
  lidar: Float32Array;
  camera: {
    left: ImageData;
    right: ImageData;
  };
  calibration: KITTICalibration;
}

export interface KITTICalibration {
  P0: number[][]; // 3x4 projection matrix
  P1: number[][];
  P2: number[][];
  P3: number[][];
  Tr_velo_to_cam: number[][]; // 3x4 transform
}

export interface RTXFrame {
  timestamp: number;
  observation: {
    image: ImageData;
    depth: Float32Array;
    state: Float32Array; // robot state vector
  };
  action: Float32Array; // action vector
  language_instruction: string;
  task_id: string;
}

export interface UnifiedDataFrame {
  timestamp: number;
  source: 'waymo' | 'kitti' | 'rtx';
  pose: {
    position: [number, number, number];
    orientation: [number, number, number, number];
  };
  sensor_data: {
    lidar?: Float32Array;
    camera?: ImageData[];
    depth?: Float32Array;
  };
  labels?: any[];
  metadata: Record<string, any>;
}

class DatasetIngestionService {
  private baseUrl: string;
  private streamingInterval: NodeJS.Timeout | null = null;
  private frameCallbacks: ((frame: UnifiedDataFrame) => void)[] = [];
  private currentDataset: 'waymo' | 'kitti' | 'rtx' | null = null;
  private currentFrame: number = 0;

  constructor(baseUrl: string = 'http://localhost:5050') {
    this.baseUrl = baseUrl;
  }

  /**
   * Connect to dataset source and start streaming
   */
  async connectDataset(
    dataset: 'waymo' | 'kitti' | 'rtx',
    sequenceId?: string
  ): Promise<void> {
    const response = await fetch(`${this.baseUrl}/api/dataset/connect`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ dataset, sequence_id: sequenceId }),
    });

    if (!response.ok) {
      throw new Error(`Failed to connect to ${dataset} dataset`);
    }

    this.currentDataset = dataset;
    this.currentFrame = 0;
    console.log(`✅ Connected to ${dataset} dataset`);
  }

  /**
   * Start real-time streaming from dataset
   */
  async startStreaming(fps: number = 10): Promise<void> {
    if (!this.currentDataset) {
      throw new Error('No dataset connected. Call connectDataset() first.');
    }

    this.stopStreaming(); // Stop any existing stream

    const interval = 1000 / fps;
    this.streamingInterval = setInterval(async () => {
      try {
        const frame = await this.fetchNextFrame();
        if (frame) {
          this.notifyFrameCallbacks(frame);
        }
      } catch (error) {
        console.error('Streaming error:', error);
      }
    }, interval);

    console.log(`🎬 Started streaming at ${fps} FPS`);
  }

  /**
   * Stop streaming
   */
  stopStreaming(): void {
    if (this.streamingInterval) {
      clearInterval(this.streamingInterval);
      this.streamingInterval = null;
      console.log('⏹️ Stopped streaming');
    }
  }

  /**
   * Fetch next frame from dataset
   */
  private async fetchNextFrame(): Promise<UnifiedDataFrame | null> {
    const response = await fetch(
      `${this.baseUrl}/api/dataset/frame?dataset=${this.currentDataset}&index=${this.currentFrame}`
    );

    if (!response.ok) {
      if (response.status === 404) {
        // End of dataset
        this.stopStreaming();
        console.log('📁 Reached end of dataset');
        return null;
      }
      throw new Error('Failed to fetch frame');
    }

    const data = await response.json();
    this.currentFrame++;
    
    return this.convertToUnifiedFormat(data);
  }

  /**
   * Convert dataset-specific format to unified format
   */
  private convertToUnifiedFormat(data: any): UnifiedDataFrame {
    const source = this.currentDataset!;

    switch (source) {
      case 'waymo':
        return this.convertWaymoToUnified(data);
      case 'kitti':
        return this.convertKITTIToUnified(data);
      case 'rtx':
        return this.convertRTXToUnified(data);
      default:
        throw new Error(`Unknown dataset: ${source}`);
    }
  }

  private convertWaymoToUnified(waymo: WaymoFrame): UnifiedDataFrame {
    return {
      timestamp: waymo.timestamp,
      source: 'waymo',
      pose: waymo.pose,
      sensor_data: {
        lidar: waymo.lidar.points,
        camera: waymo.camera.images,
      },
      labels: waymo.camera.labels,
      metadata: {
        velocity: waymo.velocity,
        intensities: waymo.lidar.intensities,
      },
    };
  }

  private convertKITTIToUnified(kitti: KITTIFrame): UnifiedDataFrame {
    // Convert rotation matrix to quaternion
    const quat = this.rotationMatrixToQuaternion(kitti.pose.rotation);

    return {
      timestamp: kitti.timestamp,
      source: 'kitti',
      pose: {
        position: kitti.pose.position,
        orientation: quat,
      },
      sensor_data: {
        lidar: kitti.lidar,
        camera: [kitti.camera.left, kitti.camera.right],
      },
      labels: [],
      metadata: {
        calibration: kitti.calibration,
      },
    };
  }

  private convertRTXToUnified(rtx: RTXFrame): UnifiedDataFrame {
    return {
      timestamp: rtx.timestamp,
      source: 'rtx',
      pose: {
        position: [0, 0, 0], // RTX is manipulation, not navigation
        orientation: [0, 0, 0, 1],
      },
      sensor_data: {
        camera: [rtx.observation.image],
        depth: rtx.observation.depth,
      },
      labels: [],
      metadata: {
        state: rtx.observation.state,
        action: rtx.action,
        instruction: rtx.language_instruction,
        task_id: rtx.task_id,
      },
    };
  }

  /**
   * Helper: Convert rotation matrix to quaternion
   */
  private rotationMatrixToQuaternion(R: number[][]): [number, number, number, number] {
    const trace = R[0][0] + R[1][1] + R[2][2];
    let w, x, y, z;

    if (trace > 0) {
      const s = 0.5 / Math.sqrt(trace + 1.0);
      w = 0.25 / s;
      x = (R[2][1] - R[1][2]) * s;
      y = (R[0][2] - R[2][0]) * s;
      z = (R[1][0] - R[0][1]) * s;
    } else {
      if (R[0][0] > R[1][1] && R[0][0] > R[2][2]) {
        const s = 2.0 * Math.sqrt(1.0 + R[0][0] - R[1][1] - R[2][2]);
        w = (R[2][1] - R[1][2]) / s;
        x = 0.25 * s;
        y = (R[0][1] + R[1][0]) / s;
        z = (R[0][2] + R[2][0]) / s;
      } else if (R[1][1] > R[2][2]) {
        const s = 2.0 * Math.sqrt(1.0 + R[1][1] - R[0][0] - R[2][2]);
        w = (R[0][2] - R[2][0]) / s;
        x = (R[0][1] + R[1][0]) / s;
        y = 0.25 * s;
        z = (R[1][2] + R[2][1]) / s;
      } else {
        const s = 2.0 * Math.sqrt(1.0 + R[2][2] - R[0][0] - R[1][1]);
        w = (R[1][0] - R[0][1]) / s;
        x = (R[0][2] + R[2][0]) / s;
        y = (R[1][2] + R[2][1]) / s;
        z = 0.25 * s;
      }
    }

    return [x, y, z, w];
  }

  /**
   * Subscribe to frame updates
   */
  onFrame(callback: (frame: UnifiedDataFrame) => void): () => void {
    this.frameCallbacks.push(callback);
    return () => {
      const index = this.frameCallbacks.indexOf(callback);
      if (index > -1) {
        this.frameCallbacks.splice(index, 1);
      }
    };
  }

  private notifyFrameCallbacks(frame: UnifiedDataFrame): void {
    this.frameCallbacks.forEach(callback => callback(frame));
  }

  /**
   * Get available datasets
   */
  async listAvailableDatasets(): Promise<{
    waymo: string[];
    kitti: string[];
    rtx: string[];
  }> {
    const response = await fetch(`${this.baseUrl}/api/dataset/list`);
    if (!response.ok) {
      throw new Error('Failed to list datasets');
    }
    return await response.json();
  }

  /**
   * Get dataset statistics
   */
  async getDatasetStats(dataset: 'waymo' | 'kitti' | 'rtx'): Promise<{
    total_frames: number;
    duration: number;
    fps: number;
    sensors: string[];
  }> {
    const response = await fetch(`${this.baseUrl}/api/dataset/stats?dataset=${dataset}`);
    if (!response.ok) {
      throw new Error('Failed to get dataset stats');
    }
    return await response.json();
  }
}

// Export singleton instance
export const datasetIngestionService = new DatasetIngestionService();
export default datasetIngestionService;

