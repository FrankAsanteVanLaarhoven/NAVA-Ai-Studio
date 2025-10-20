/**
 * Gazebo Simulation Service
 * Handles communication with Gazebo simulator backend
 * Provides real-time 3D simulation capabilities
 */

export interface GazeboWorld {
  id: string;
  name: string;
  description: string;
  gravity: [number, number, number];
  physics: {
    engine: 'ode' | 'bullet' | 'simbody' | 'dart';
    maxStepSize: number;
    realTimeFactor: number;
  };
  models: GazeboModel[];
}

export interface GazeboModel {
  id: string;
  name: string;
  type: 'robot' | 'object' | 'light' | 'sensor';
  position: [number, number, number];
  rotation: [number, number, number];
  scale: [number, number, number];
  mesh?: string;
  color?: string;
  physics?: {
    mass: number;
    friction: number;
    restitution: number;
  };
  sensors?: GazeboSensor[];
  joints?: GazeboJoint[];
}

export interface GazeboSensor {
  id: string;
  name: string;
  type: 'camera' | 'lidar' | 'imu' | 'gps' | 'depth' | 'ultrasonic';
  position: [number, number, number];
  rotation: [number, number, number];
  config: any;
  data?: any;
}

export interface GazeboJoint {
  id: string;
  name: string;
  type: 'revolute' | 'prismatic' | 'fixed' | 'ball';
  parent: string;
  child: string;
  axis: [number, number, number];
  limits: {
    lower: number;
    upper: number;
    effort: number;
    velocity: number;
  };
}

export interface GazeboRobot {
  id: string;
  name: string;
  type: 'differential_drive' | 'quadcopter' | 'manipulator' | 'humanoid' | 'custom';
  urdf?: string;
  sdf?: string;
  position: [number, number, number];
  velocity: [number, number, number];
  sensors: GazeboSensor[];
  actuators: GazeboActuator[];
}

export interface GazeboActuator {
  id: string;
  name: string;
  type: 'motor' | 'servo' | 'thruster';
  jointName: string;
  command: number;
  feedback: number;
}

export interface SimulationState {
  time: number;
  realTime: number;
  paused: boolean;
  models: GazeboModel[];
  contacts: Array<{
    modelA: string;
    modelB: string;
    force: number;
  }>;
}

class GazeboService {
  private ws: WebSocket | null = null;
  private connected: boolean = false;
  private worlds: Map<string, GazeboWorld> = new Map();
  private currentWorld: GazeboWorld | null = null;
  private listeners: Map<string, Set<(data: any) => void>> = new Map();
  private simulationState: SimulationState | null = null;
  private useLocalSimulation: boolean = true; // For demo mode

  constructor() {
    this.initializeDefaultWorlds();
  }

  /**
   * Initialize default simulation worlds
   */
  private initializeDefaultWorlds(): void {
    // Empty world
    const emptyWorld: GazeboWorld = {
      id: 'empty_world',
      name: 'Empty World',
      description: 'A simple empty world with ground plane',
      gravity: [0, 0, -9.81],
      physics: {
        engine: 'ode',
        maxStepSize: 0.001,
        realTimeFactor: 1.0,
      },
      models: [
        {
          id: 'ground_plane',
          name: 'Ground Plane',
          type: 'object',
          position: [0, 0, 0],
          rotation: [0, 0, 0],
          scale: [100, 100, 0.1],
          color: '#808080',
          physics: {
            mass: 0, // Static object
            friction: 0.8,
            restitution: 0.1,
          },
        },
        {
          id: 'sun',
          name: 'Sun',
          type: 'light',
          position: [0, 0, 10],
          rotation: [0, 0, 0],
          scale: [1, 1, 1],
        },
      ],
    };

    // Warehouse world
    const warehouseWorld: GazeboWorld = {
      id: 'warehouse',
      name: 'Warehouse Environment',
      description: 'Indoor warehouse with shelves and obstacles',
      gravity: [0, 0, -9.81],
      physics: {
        engine: 'ode',
        maxStepSize: 0.001,
        realTimeFactor: 1.0,
      },
      models: [
        {
          id: 'ground_plane',
          name: 'Ground Plane',
          type: 'object',
          position: [0, 0, 0],
          rotation: [0, 0, 0],
          scale: [50, 50, 0.1],
          color: '#A0A0A0',
        },
      ],
    };

    // Outdoor world
    const outdoorWorld: GazeboWorld = {
      id: 'outdoor',
      name: 'Outdoor Terrain',
      description: 'Outdoor environment with terrain and obstacles',
      gravity: [0, 0, -9.81],
      physics: {
        engine: 'ode',
        maxStepSize: 0.001,
        realTimeFactor: 1.0,
      },
      models: [
        {
          id: 'terrain',
          name: 'Terrain',
          type: 'object',
          position: [0, 0, 0],
          rotation: [0, 0, 0],
          scale: [100, 100, 1],
          color: '#6B8E23',
        },
      ],
    };

    this.worlds.set('empty_world', emptyWorld);
    this.worlds.set('warehouse', warehouseWorld);
    this.worlds.set('outdoor', outdoorWorld);
    this.currentWorld = emptyWorld;
  }

  /**
   * Connect to Gazebo backend
   */
  async connect(url: string = 'ws://localhost:9090'): Promise<void> {
    if (this.useLocalSimulation) {
      // Use local simulation mode (no backend needed)
      this.connected = true;
      this.emit('connected', { mode: 'local' });
      return;
    }

    return new Promise((resolve, reject) => {
      try {
        this.ws = new WebSocket(url);

        this.ws.onopen = () => {
          this.connected = true;
          console.log('✅ Connected to Gazebo backend');
          this.emit('connected', { url });
          resolve();
        };

        this.ws.onmessage = (event) => {
          const data = JSON.parse(event.data);
          this.handleMessage(data);
        };

        this.ws.onerror = (error) => {
          console.error('❌ Gazebo connection error:', error);
          this.emit('error', error);
          reject(error);
        };

        this.ws.onclose = () => {
          this.connected = false;
          this.emit('disconnected', {});
        };
      } catch (error) {
        reject(error);
      }
    });
  }

  /**
   * Disconnect from Gazebo backend
   */
  disconnect(): void {
    if (this.ws) {
      this.ws.close();
      this.ws = null;
    }
    this.connected = false;
  }

  /**
   * Check if connected
   */
  isConnected(): boolean {
    return this.connected;
  }

  /**
   * Get available worlds
   */
  getWorlds(): GazeboWorld[] {
    return Array.from(this.worlds.values());
  }

  /**
   * Load a world
   */
  async loadWorld(worldId: string): Promise<GazeboWorld> {
    const world = this.worlds.get(worldId);
    if (!world) {
      throw new Error(`World not found: ${worldId}`);
    }

    this.currentWorld = world;
    this.emit('world_loaded', world);
    
    // Initialize simulation state
    this.simulationState = {
      time: 0,
      realTime: 0,
      paused: true,
      models: [...world.models],
      contacts: [],
    };

    return world;
  }

  /**
   * Get current world
   */
  getCurrentWorld(): GazeboWorld | null {
    return this.currentWorld;
  }

  /**
   * Spawn a robot in the simulation
   */
  async spawnRobot(robot: GazeboRobot): Promise<void> {
    if (!this.currentWorld) {
      throw new Error('No world loaded');
    }

    const model: GazeboModel = {
      id: robot.id,
      name: robot.name,
      type: 'robot',
      position: robot.position,
      rotation: [0, 0, 0],
      scale: [1, 1, 1],
      sensors: robot.sensors,
    };

    this.currentWorld.models.push(model);
    
    if (this.simulationState) {
      this.simulationState.models.push(model);
    }

    this.emit('robot_spawned', robot);
  }

  /**
   * Spawn a model in the simulation
   */
  async spawnModel(model: GazeboModel): Promise<void> {
    if (!this.currentWorld) {
      throw new Error('No world loaded');
    }

    this.currentWorld.models.push(model);
    
    if (this.simulationState) {
      this.simulationState.models.push(model);
    }

    this.emit('model_spawned', model);
  }

  /**
   * Delete a model from the simulation
   */
  async deleteModel(modelId: string): Promise<void> {
    if (!this.currentWorld) {
      throw new Error('No world loaded');
    }

    this.currentWorld.models = this.currentWorld.models.filter(m => m.id !== modelId);
    
    if (this.simulationState) {
      this.simulationState.models = this.simulationState.models.filter(m => m.id !== modelId);
    }

    this.emit('model_deleted', { modelId });
  }

  /**
   * Update model position
   */
  updateModelPose(modelId: string, position: [number, number, number], rotation: [number, number, number]): void {
    if (!this.currentWorld) return;

    const model = this.currentWorld.models.find(m => m.id === modelId);
    if (model) {
      model.position = position;
      model.rotation = rotation;
      this.emit('model_pose_updated', { modelId, position, rotation });
    }
  }

  /**
   * Play/pause simulation
   */
  play(): void {
    if (this.simulationState) {
      this.simulationState.paused = false;
      this.emit('simulation_started', {});
      this.startSimulationLoop();
    }
  }

  pause(): void {
    if (this.simulationState) {
      this.simulationState.paused = true;
      this.emit('simulation_paused', {});
    }
  }

  reset(): void {
    if (this.currentWorld) {
      this.loadWorld(this.currentWorld.id);
      this.emit('simulation_reset', {});
    }
  }

  /**
   * Simulation loop (for local mode)
   */
  private startSimulationLoop(): void {
    const dt = 0.01; // 10ms timestep

    const update = () => {
      if (!this.simulationState || this.simulationState.paused) return;

      this.simulationState.time += dt;
      this.simulationState.realTime += dt;

      // Simple physics update (for demo)
      this.simulationState.models.forEach(model => {
        if (model.type === 'robot' && model.physics) {
          // Apply gravity
          const gravity = this.currentWorld?.gravity || [0, 0, -9.81];
          if (model.position[2] > 0.1) {
            model.position[2] += gravity[2] * dt * dt * 0.5;
          } else {
            model.position[2] = Math.max(0.1, model.position[2]);
          }
        }
      });

      this.emit('simulation_update', this.simulationState);

      if (!this.simulationState.paused) {
        requestAnimationFrame(update);
      }
    };

    requestAnimationFrame(update);
  }

  /**
   * Get simulation state
   */
  getSimulationState(): SimulationState | null {
    return this.simulationState;
  }

  /**
   * Send command to robot
   */
  sendRobotCommand(robotId: string, command: {
    linear?: [number, number, number];
    angular?: [number, number, number];
  }): void {
    this.emit('robot_command', { robotId, command });
    
    // Update robot position based on command (simple kinematics)
    if (this.simulationState) {
      const model = this.simulationState.models.find(m => m.id === robotId);
      if (model && command.linear) {
        const dt = 0.01;
        model.position[0] += command.linear[0] * dt;
        model.position[1] += command.linear[1] * dt;
        model.position[2] += command.linear[2] * dt;
      }
    }
  }

  /**
   * Get sensor data
   */
  getSensorData(sensorId: string): any {
    // In real implementation, this would query the backend
    // For now, return mock data
    return {
      timestamp: Date.now(),
      data: Math.random(),
    };
  }

  /**
   * Handle messages from backend
   */
  private handleMessage(data: any): void {
    switch (data.type) {
      case 'state_update':
        this.simulationState = data.state;
        this.emit('simulation_update', data.state);
        break;
      case 'sensor_data':
        this.emit('sensor_data', data);
        break;
      default:
        console.warn('Unknown message type:', data.type);
    }
  }

  /**
   * Event system
   */
  on(event: string, callback: (data: any) => void): void {
    if (!this.listeners.has(event)) {
      this.listeners.set(event, new Set());
    }
    this.listeners.get(event)!.add(callback);
  }

  off(event: string, callback: (data: any) => void): void {
    if (this.listeners.has(event)) {
      this.listeners.get(event)!.delete(callback);
    }
  }

  private emit(event: string, data: any): void {
    if (this.listeners.has(event)) {
      this.listeners.get(event)!.forEach(callback => callback(data));
    }
  }

  /**
   * Export world as SDF
   */
  exportWorldSDF(): string {
    if (!this.currentWorld) {
      throw new Error('No world loaded');
    }

    // Generate SDF XML
    let sdf = `<?xml version="1.0"?>
<sdf version="1.7">
  <world name="${this.currentWorld.name}">
    <gravity>${this.currentWorld.gravity.join(' ')}</gravity>
    <physics name="${this.currentWorld.physics.engine}" type="${this.currentWorld.physics.engine}">
      <max_step_size>${this.currentWorld.physics.maxStepSize}</max_step_size>
      <real_time_factor>${this.currentWorld.physics.realTimeFactor}</real_time_factor>
    </physics>
`;

    this.currentWorld.models.forEach(model => {
      sdf += `
    <model name="${model.name}">
      <pose>${[...model.position, ...model.rotation].join(' ')}</pose>
      <static>${model.physics?.mass === 0 ? 'true' : 'false'}</static>
    </model>
`;
    });

    sdf += `  </world>
</sdf>`;

    return sdf;
  }

  /**
   * Import world from SDF
   */
  async importWorldSDF(sdf: string): Promise<GazeboWorld> {
    // Parse SDF and create world
    // This is a simplified version
    const parser = new DOMParser();
    const doc = parser.parseFromString(sdf, 'text/xml');
    
    const worldElement = doc.querySelector('world');
    if (!worldElement) {
      throw new Error('Invalid SDF: no world element');
    }

    const name = worldElement.getAttribute('name') || 'Imported World';
    
    const world: GazeboWorld = {
      id: `imported_${Date.now()}`,
      name,
      description: 'Imported from SDF',
      gravity: [0, 0, -9.81],
      physics: {
        engine: 'ode',
        maxStepSize: 0.001,
        realTimeFactor: 1.0,
      },
      models: [],
    };

    this.worlds.set(world.id, world);
    return world;
  }
}

// Export singleton instance
export const gazeboService = new GazeboService();

