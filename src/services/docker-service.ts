/**
 * Docker Service
 * 
 * Provides integration with Docker for managing MCP server containers.
 * Handles container lifecycle, image management, and Docker API communication.
 */

export interface DockerContainer {
  id: string;
  name: string;
  image: string;
  status: 'running' | 'stopped' | 'paused' | 'restarting' | 'dead' | 'created';
  state: string;
  created: string;
  ports: DockerPort[];
  mounts: DockerMount[];
  environment: Record<string, string>;
  labels: Record<string, string>;
  networkMode: string;
  restartPolicy: string;
}

export interface DockerImage {
  id: string;
  repository: string;
  tag: string;
  size: number;
  created: string;
  labels: Record<string, string>;
  architecture: string;
  os: string;
}

export interface DockerPort {
  privatePort: number;
  publicPort?: number;
  type: 'tcp' | 'udp';
}

export interface DockerMount {
  type: 'bind' | 'volume' | 'tmpfs';
  source: string;
  destination: string;
  mode: string;
  readonly: boolean;
}

export interface DockerNetwork {
  id: string;
  name: string;
  driver: string;
  scope: string;
  ipam: {
    driver: string;
    config: Array<{
      subnet: string;
      gateway: string;
    }>;
  };
}

export interface DockerVolume {
  name: string;
  driver: string;
  mountpoint: string;
  created: string;
  labels: Record<string, string>;
  size: number;
}

export interface DockerStats {
  containerId: string;
  name: string;
  cpu: {
    usage: number;
    systemUsage: number;
    onlineCpus: number;
  };
  memory: {
    usage: number;
    maxUsage: number;
    limit: number;
  };
  network: {
    rxBytes: number;
    txBytes: number;
  };
  blockIO: {
    readBytes: number;
    writeBytes: number;
  };
  timestamp: string;
}

export class DockerService {
  private static instance: DockerService;
  private dockerAvailable = false;
  private dockerVersion: string | null = null;
  private containers: DockerContainer[] = [];
  private images: DockerImage[] = [];
  private networks: DockerNetwork[] = [];
  private volumes: DockerVolume[] = [];
  private statsInterval: NodeJS.Timeout | null = null;
  private stats: Map<string, DockerStats> = new Map();

  private constructor() {
    this.initializeDocker();
  }

  public static getInstance(): DockerService {
    if (!DockerService.instance) {
      DockerService.instance = new DockerService();
    }
    return DockerService.instance;
  }

  private async initializeDocker(): Promise<void> {
    try {
      // Check if Docker is available
      const response = await this.makeRequest('/api/docker/version');
      if (response.ok) {
        const version = await response.json();
        this.dockerVersion = version.Version;
        this.dockerAvailable = true;
        await this.refreshData();
      }
    } catch (error) {
      console.warn('Docker not available:', error);
      this.dockerAvailable = false;
    }
  }

  private async makeRequest(endpoint: string, options: RequestInit = {}): Promise<Response> {
    const defaultOptions: RequestInit = {
      headers: {
        'Content-Type': 'application/json',
        ...options.headers,
      },
    };

    return fetch(endpoint, { ...defaultOptions, ...options });
  }

  /**
   * Check if Docker is available
   */
  public isDockerAvailable(): boolean {
    return this.dockerAvailable;
  }

  /**
   * Get Docker version
   */
  public getDockerVersion(): string | null {
    return this.dockerVersion;
  }

  /**
   * Refresh all Docker data
   */
  public async refreshData(): Promise<void> {
    if (!this.dockerAvailable) return;

    try {
      await Promise.all([
        this.refreshContainers(),
        this.refreshImages(),
        this.refreshNetworks(),
        this.refreshVolumes(),
      ]);
    } catch (error) {
      console.error('Failed to refresh Docker data:', error);
    }
  }

  /**
   * Get all containers
   */
  public async getContainers(all = false): Promise<DockerContainer[]> {
    if (!this.dockerAvailable) return [];

    try {
      const response = await this.makeRequest(`/api/docker/containers?all=${all}`);
      if (response.ok) {
        this.containers = await response.json();
      }
    } catch (error) {
      console.error('Failed to get containers:', error);
    }

    return this.containers;
  }

  /**
   * Get container by ID
   */
  public async getContainer(containerId: string): Promise<DockerContainer | null> {
    if (!this.dockerAvailable) return null;

    try {
      const response = await this.makeRequest(`/api/docker/containers/${containerId}`);
      if (response.ok) {
        return await response.json();
      }
    } catch (error) {
      console.error('Failed to get container:', error);
    }

    return null;
  }

  /**
   * Create a new container
   */
  public async createContainer(config: {
    image: string;
    name?: string;
    ports?: Array<{ host: number; container: number; protocol?: 'tcp' | 'udp' }>;
    volumes?: Array<{ host: string; container: string; readonly?: boolean }>;
    environment?: Record<string, string>;
    labels?: Record<string, string>;
    networkMode?: string;
    restartPolicy?: 'no' | 'always' | 'unless-stopped' | 'on-failure';
  }): Promise<string | null> {
    if (!this.dockerAvailable) return null;

    try {
      const response = await this.makeRequest('/api/docker/containers/create', {
        method: 'POST',
        body: JSON.stringify(config),
      });

      if (response.ok) {
        const result = await response.json();
        await this.refreshContainers();
        return result.id;
      }
    } catch (error) {
      console.error('Failed to create container:', error);
    }

    return null;
  }

  /**
   * Start a container
   */
  public async startContainer(containerId: string): Promise<boolean> {
    if (!this.dockerAvailable) return false;

    try {
      const response = await this.makeRequest(`/api/docker/containers/${containerId}/start`, {
        method: 'POST',
      });

      if (response.ok) {
        await this.refreshContainers();
        return true;
      }
    } catch (error) {
      console.error('Failed to start container:', error);
    }

    return false;
  }

  /**
   * Stop a container
   */
  public async stopContainer(containerId: string, timeout = 10): Promise<boolean> {
    if (!this.dockerAvailable) return false;

    try {
      const response = await this.makeRequest(`/api/docker/containers/${containerId}/stop`, {
        method: 'POST',
        body: JSON.stringify({ timeout }),
      });

      if (response.ok) {
        await this.refreshContainers();
        return true;
      }
    } catch (error) {
      console.error('Failed to stop container:', error);
    }

    return false;
  }

  /**
   * Remove a container
   */
  public async removeContainer(containerId: string, force = false): Promise<boolean> {
    if (!this.dockerAvailable) return false;

    try {
      const response = await this.makeRequest(`/api/docker/containers/${containerId}?force=${force}`, {
        method: 'DELETE',
      });

      if (response.ok) {
        await this.refreshContainers();
        return true;
      }
    } catch (error) {
      console.error('Failed to remove container:', error);
    }

    return false;
  }

  /**
   * Get container logs
   */
  public async getContainerLogs(
    containerId: string,
    options: {
      follow?: boolean;
      tail?: number;
      since?: string;
      until?: string;
      timestamps?: boolean;
    } = {}
  ): Promise<string> {
    if (!this.dockerAvailable) return '';

    try {
      const params = new URLSearchParams();
      Object.entries(options).forEach(([key, value]) => {
        if (value !== undefined) {
          params.append(key, value.toString());
        }
      });

      const response = await this.makeRequest(`/api/docker/containers/${containerId}/logs?${params}`);
      if (response.ok) {
        return await response.text();
      }
    } catch (error) {
      console.error('Failed to get container logs:', error);
    }

    return '';
  }

  /**
   * Execute command in container
   */
  public async execInContainer(
    containerId: string,
    command: string[],
    options: {
      detach?: boolean;
      tty?: boolean;
      user?: string;
      workingDir?: string;
      environment?: Record<string, string>;
    } = {}
  ): Promise<{ exitCode: number; output: string }> {
    if (!this.dockerAvailable) {
      return { exitCode: 1, output: 'Docker not available' };
    }

    try {
      const response = await this.makeRequest(`/api/docker/containers/${containerId}/exec`, {
        method: 'POST',
        body: JSON.stringify({
          command,
          ...options,
        }),
      });

      if (response.ok) {
        return await response.json();
      }
    } catch (error) {
      console.error('Failed to exec in container:', error);
    }

    return { exitCode: 1, output: 'Execution failed' };
  }

  /**
   * Get all images
   */
  public async getImages(): Promise<DockerImage[]> {
    if (!this.dockerAvailable) return [];

    try {
      const response = await this.makeRequest('/api/docker/images');
      if (response.ok) {
        this.images = await response.json();
      }
    } catch (error) {
      console.error('Failed to get images:', error);
    }

    return this.images;
  }

  /**
   * Pull an image
   */
  public async pullImage(image: string, tag = 'latest'): Promise<boolean> {
    if (!this.dockerAvailable) return false;

    try {
      const response = await this.makeRequest('/api/docker/images/pull', {
        method: 'POST',
        body: JSON.stringify({ image, tag }),
      });

      if (response.ok) {
        await this.refreshImages();
        return true;
      }
    } catch (error) {
      console.error('Failed to pull image:', error);
    }

    return false;
  }

  /**
   * Remove an image
   */
  public async removeImage(imageId: string, force = false): Promise<boolean> {
    if (!this.dockerAvailable) return false;

    try {
      const response = await this.makeRequest(`/api/docker/images/${imageId}?force=${force}`, {
        method: 'DELETE',
      });

      if (response.ok) {
        await this.refreshImages();
        return true;
      }
    } catch (error) {
      console.error('Failed to remove image:', error);
    }

    return false;
  }

  /**
   * Get all networks
   */
  public async getNetworks(): Promise<DockerNetwork[]> {
    if (!this.dockerAvailable) return [];

    try {
      const response = await this.makeRequest('/api/docker/networks');
      if (response.ok) {
        this.networks = await response.json();
      }
    } catch (error) {
      console.error('Failed to get networks:', error);
    }

    return this.networks;
  }

  /**
   * Get all volumes
   */
  public async getVolumes(): Promise<DockerVolume[]> {
    if (!this.dockerAvailable) return [];

    try {
      const response = await this.makeRequest('/api/docker/volumes');
      if (response.ok) {
        this.volumes = await response.json();
      }
    } catch (error) {
      console.error('Failed to get volumes:', error);
    }

    return this.volumes;
  }

  /**
   * Get container statistics
   */
  public async getContainerStats(containerId: string): Promise<DockerStats | null> {
    if (!this.dockerAvailable) return null;

    try {
      const response = await this.makeRequest(`/api/docker/containers/${containerId}/stats`);
      if (response.ok) {
        const stats = await response.json();
        this.stats.set(containerId, stats);
        return stats;
      }
    } catch (error) {
      console.error('Failed to get container stats:', error);
    }

    return null;
  }

  /**
   * Start monitoring container statistics
   */
  public startStatsMonitoring(containerIds: string[], interval = 5000): void {
    if (!this.dockerAvailable) return;

    this.stopStatsMonitoring();

    this.statsInterval = setInterval(async () => {
      for (const containerId of containerIds) {
        await this.getContainerStats(containerId);
      }
    }, interval);
  }

  /**
   * Stop monitoring container statistics
   */
  public stopStatsMonitoring(): void {
    if (this.statsInterval) {
      clearInterval(this.statsInterval);
      this.statsInterval = null;
    }
  }

  /**
   * Get cached statistics
   */
  public getCachedStats(): Map<string, DockerStats> {
    return new Map(this.stats);
  }

  /**
   * Get system information
   */
  public async getSystemInfo(): Promise<{
    containers: number;
    containersRunning: number;
    containersPaused: number;
    containersStopped: number;
    images: number;
    driver: string;
    driverStatus: Array<[string, string]>;
    systemStatus: Array<[string, string]>;
    plugins: {
      volume: string[];
      network: string[];
      authorization: string[];
    };
    memoryLimit: boolean;
    swapLimit: boolean;
    kernelMemory: boolean;
    cpuCfsPeriod: boolean;
    cpuCfsQuota: boolean;
    cpuShares: boolean;
    cpuSet: boolean;
    ipv4Forwarding: boolean;
    bridgeNfIptables: boolean;
    bridgeNfIp6tables: boolean;
    debug: boolean;
    nfd: number;
    ngoroutines: number;
    systemTime: string;
    loggingDriver: string;
    cgroupDriver: string;
    cgroupVersion: string;
    neventsListener: number;
    kernelVersion: string;
    operatingSystem: string;
    osType: string;
    architecture: string;
    nCPU: number;
    memTotal: number;
    dockerRootDir: string;
    httpProxy: string;
    httpsProxy: string;
    noProxy: string;
    name: string;
    labels: string[];
    experimentalBuild: boolean;
    serverVersion: string;
    clusterStore: string;
    clusterAdvertise: string;
    runtimes: Record<string, { path: string; runtimeArgs: string[] }>;
    defaultRuntime: string;
    swarm: {
      nodeID: string;
      nodeAddr: string;
      localNodeState: string;
      controlAvailable: boolean;
      error: string;
      remoteManagers: any;
      nodes: number;
      managers: number;
      cluster: any;
    };
    liveRestoreEnabled: boolean;
    isolation: string;
    initBinary: string;
    containerdCommit: {
      id: string;
      expected: string;
    };
    runcCommit: {
      id: string;
      expected: string;
    };
    initCommit: {
      id: string;
      expected: string;
    };
    securityOptions: string[];
    productLicense: string;
    warnings: string[];
  } | null> {
    if (!this.dockerAvailable) return null;

    try {
      const response = await this.makeRequest('/api/docker/info');
      if (response.ok) {
        return await response.json();
      }
    } catch (error) {
      console.error('Failed to get system info:', error);
    }

    return null;
  }

  private async refreshContainers(): Promise<void> {
    await this.getContainers(true);
  }

  private async refreshImages(): Promise<void> {
    await this.getImages();
  }

  private async refreshNetworks(): Promise<void> {
    await this.getNetworks();
  }

  private async refreshVolumes(): Promise<void> {
    await this.getVolumes();
  }

  /**
   * Format bytes to human readable string
   */
  public formatBytes(bytes: number): string {
    if (bytes === 0) return '0 B';
    const k = 1024;
    const sizes = ['B', 'KB', 'MB', 'GB', 'TB'];
    const i = Math.floor(Math.log(bytes) / Math.log(k));
    return parseFloat((bytes / Math.pow(k, i)).toFixed(2)) + ' ' + sizes[i];
  }

  /**
   * Format duration to human readable string
   */
  public formatDuration(seconds: number): string {
    const days = Math.floor(seconds / 86400);
    const hours = Math.floor((seconds % 86400) / 3600);
    const minutes = Math.floor((seconds % 3600) / 60);
    const secs = Math.floor(seconds % 60);

    if (days > 0) {
      return `${days}d ${hours}h ${minutes}m`;
    } else if (hours > 0) {
      return `${hours}h ${minutes}m`;
    } else if (minutes > 0) {
      return `${minutes}m ${secs}s`;
    } else {
      return `${secs}s`;
    }
  }

  /**
   * Get container status color
   */
  public getStatusColor(status: string): string {
    switch (status.toLowerCase()) {
      case 'running':
        return '#22c55e';
      case 'stopped':
        return '#ef4444';
      case 'paused':
        return '#f59e0b';
      case 'restarting':
        return '#3b82f6';
      case 'dead':
        return '#6b7280';
      case 'created':
        return '#8b5cf6';
      default:
        return '#94a3b8';
    }
  }
}

export default DockerService;
