/**
 * Docker MCP Catalog Service
 * 
 * Integrates with Docker Hub MCP registry to discover, manage, and execute
 * MCP (Model Context Protocol) servers as containerized extensions.
 */

export interface MCPServer {
  id: string;
  name: string;
  description: string;
  longDescription: string;
  author: string;
  version: string;
  category: string;
  tags: string[];
  icon: string;
  dockerImage: string;
  dockerTag: string;
  deploymentType: 'local' | 'remote';
  tools: MCPTool[];
  configuration: MCPConfiguration;
  verified: boolean;
  downloads: number;
  rating: number;
  lastUpdated: string;
  repository?: string;
  documentation?: string;
  screenshots: string[];
  requirements: {
    docker: boolean;
    memory?: string;
    cpu?: string;
    storage?: string;
  };
}

export interface MCPTool {
  name: string;
  description: string;
  inputSchema: any;
  outputSchema: any;
  category: string;
}

export interface MCPConfiguration {
  serverType: 'local' | 'remote';
  endpoint?: string;
  authentication?: {
    type: 'none' | 'api-key' | 'oauth' | 'basic';
    config: any;
  };
  environment?: Record<string, string>;
  volumes?: Array<{
    host: string;
    container: string;
    readonly?: boolean;
  }>;
  ports?: Array<{
    host: number;
    container: number;
  }>;
}

export interface MCPConnection {
  id: string;
  serverId: string;
  name: string;
  status: 'disconnected' | 'connecting' | 'connected' | 'error';
  lastConnected?: string;
  error?: string;
  configuration: MCPConfiguration;
}

export class MCPCatalogService {
  private static instance: MCPCatalogService;
  private servers: MCPServer[] = [];
  private connections: MCPConnection[] = [];
  private dockerAvailable = false;

  private constructor() {
    this.initializeDocker();
  }

  public static getInstance(): MCPCatalogService {
    if (!MCPCatalogService.instance) {
      MCPCatalogService.instance = new MCPCatalogService();
    }
    return MCPCatalogService.instance;
  }

  private async initializeDocker(): Promise<void> {
    try {
      // Check if Docker is available
      const response = await fetch('/api/docker/status');
      this.dockerAvailable = response.ok;
    } catch (error) {
      console.warn('Docker not available:', error);
      this.dockerAvailable = false;
    }
  }

  /**
   * Discover MCP servers from Docker Hub registry
   */
  public async discoverServers(query?: string, category?: string): Promise<MCPServer[]> {
    try {
      // In a real implementation, this would query Docker Hub API
      // For now, we'll use mock data that represents real MCP servers
      const mockServers: MCPServer[] = [
        {
          id: 'docker-filesystem',
          name: 'Docker Filesystem',
          description: 'Access and manipulate Docker container filesystems',
          longDescription: 'Provides tools to browse, read, write, and manage files within Docker containers. Perfect for debugging, data extraction, and container management tasks.',
          author: 'Docker Inc.',
          version: '1.0.0',
          category: 'Infrastructure',
          tags: ['docker', 'filesystem', 'containers', 'debugging'],
          icon: '🐳',
          dockerImage: 'mcp/docker-filesystem',
          dockerTag: 'latest',
          deploymentType: 'local',
          verified: true,
          downloads: 15420,
          rating: 4.8,
          lastUpdated: '2024-01-15',
          repository: 'https://github.com/docker/mcp-filesystem',
          documentation: 'https://docs.docker.com/ai/mcp-catalog-and-toolkit/catalog/docker-filesystem',
          screenshots: [],
          requirements: {
            docker: true,
            memory: '512MB',
            cpu: '1 core'
          },
          tools: [
            {
              name: 'list_files',
              description: 'List files and directories in a container',
              inputSchema: {
                type: 'object',
                properties: {
                  containerId: { type: 'string' },
                  path: { type: 'string', default: '/' }
                },
                required: ['containerId']
              },
              outputSchema: {
                type: 'array',
                items: {
                  type: 'object',
                  properties: {
                    name: { type: 'string' },
                    type: { type: 'string', enum: ['file', 'directory'] },
                    size: { type: 'number' },
                    modified: { type: 'string' }
                  }
                }
              },
              category: 'filesystem'
            },
            {
              name: 'read_file',
              description: 'Read contents of a file from a container',
              inputSchema: {
                type: 'object',
                properties: {
                  containerId: { type: 'string' },
                  filePath: { type: 'string' }
                },
                required: ['containerId', 'filePath']
              },
              outputSchema: {
                type: 'object',
                properties: {
                  content: { type: 'string' },
                  encoding: { type: 'string' }
                }
              },
              category: 'filesystem'
            }
          ],
          configuration: {
            serverType: 'local',
            environment: {
              DOCKER_HOST: 'unix:///var/run/docker.sock'
            },
            volumes: [
              {
                host: '/var/run/docker.sock',
                container: '/var/run/docker.sock',
                readonly: true
              }
            ]
          }
        },
        {
          id: 'github-integration',
          name: 'GitHub Integration',
          description: 'Interact with GitHub repositories, issues, and pull requests',
          longDescription: 'Comprehensive GitHub API integration for managing repositories, issues, pull requests, and more. Supports both personal and organization repositories.',
          author: 'GitHub Inc.',
          version: '2.1.0',
          category: 'Development',
          tags: ['github', 'git', 'repositories', 'collaboration'],
          icon: '🐙',
          dockerImage: 'mcp/github-integration',
          dockerTag: 'latest',
          deploymentType: 'remote',
          verified: true,
          downloads: 28450,
          rating: 4.9,
          lastUpdated: '2024-01-20',
          repository: 'https://github.com/github/mcp-integration',
          documentation: 'https://docs.github.com/en/rest',
          screenshots: [],
          requirements: {
            docker: false,
            memory: '256MB'
          },
          tools: [
            {
              name: 'get_repository',
              description: 'Get repository information',
              inputSchema: {
                type: 'object',
                properties: {
                  owner: { type: 'string' },
                  repo: { type: 'string' }
                },
                required: ['owner', 'repo']
              },
              outputSchema: {
                type: 'object',
                properties: {
                  name: { type: 'string' },
                  description: { type: 'string' },
                  stars: { type: 'number' },
                  forks: { type: 'number' }
                }
              },
              category: 'repository'
            },
            {
              name: 'create_issue',
              description: 'Create a new GitHub issue',
              inputSchema: {
                type: 'object',
                properties: {
                  owner: { type: 'string' },
                  repo: { type: 'string' },
                  title: { type: 'string' },
                  body: { type: 'string' },
                  labels: { type: 'array', items: { type: 'string' } }
                },
                required: ['owner', 'repo', 'title']
              },
              outputSchema: {
                type: 'object',
                properties: {
                  number: { type: 'number' },
                  url: { type: 'string' }
                }
              },
              category: 'issues'
            }
          ],
          configuration: {
            serverType: 'remote',
            endpoint: 'https://api.github.com',
            authentication: {
              type: 'api-key',
              config: {
                header: 'Authorization',
                format: 'Bearer {token}'
              }
            }
          }
        },
        {
          id: 'stripe-payments',
          name: 'Stripe Payments',
          description: 'Process payments and manage Stripe accounts',
          longDescription: 'Full Stripe API integration for processing payments, managing customers, subscriptions, and financial operations.',
          author: 'Stripe Inc.',
          version: '1.5.0',
          category: 'Business',
          tags: ['stripe', 'payments', 'ecommerce', 'billing'],
          icon: '💳',
          dockerImage: 'mcp/stripe-payments',
          dockerTag: 'latest',
          deploymentType: 'remote',
          verified: true,
          downloads: 12300,
          rating: 4.7,
          lastUpdated: '2024-01-18',
          repository: 'https://github.com/stripe/mcp-integration',
          documentation: 'https://stripe.com/docs/api',
          screenshots: [],
          requirements: {
            docker: false,
            memory: '128MB'
          },
          tools: [
            {
              name: 'create_payment_intent',
              description: 'Create a payment intent for processing payments',
              inputSchema: {
                type: 'object',
                properties: {
                  amount: { type: 'number' },
                  currency: { type: 'string', default: 'usd' },
                  customer: { type: 'string' }
                },
                required: ['amount']
              },
              outputSchema: {
                type: 'object',
                properties: {
                  id: { type: 'string' },
                  client_secret: { type: 'string' },
                  status: { type: 'string' }
                }
              },
              category: 'payments'
            }
          ],
          configuration: {
            serverType: 'remote',
            endpoint: 'https://api.stripe.com/v1',
            authentication: {
              type: 'api-key',
              config: {
                header: 'Authorization',
                format: 'Bearer {token}'
              }
            }
          }
        },
        {
          id: 'grafana-monitoring',
          name: 'Grafana Monitoring',
          description: 'Monitor systems and create dashboards with Grafana',
          longDescription: 'Comprehensive Grafana integration for creating dashboards, managing alerts, and monitoring system metrics.',
          author: 'Grafana Labs',
          version: '1.3.0',
          category: 'Monitoring',
          tags: ['grafana', 'monitoring', 'dashboards', 'metrics'],
          icon: '📊',
          dockerImage: 'mcp/grafana-monitoring',
          dockerTag: 'latest',
          deploymentType: 'local',
          verified: true,
          downloads: 8750,
          rating: 4.6,
          lastUpdated: '2024-01-12',
          repository: 'https://github.com/grafana/mcp-integration',
          documentation: 'https://grafana.com/docs/grafana/latest/http_api',
          screenshots: [],
          requirements: {
            docker: true,
            memory: '1GB',
            cpu: '2 cores'
          },
          tools: [
            {
              name: 'create_dashboard',
              description: 'Create a new Grafana dashboard',
              inputSchema: {
                type: 'object',
                properties: {
                  title: { type: 'string' },
                  panels: { type: 'array', items: { type: 'object' } }
                },
                required: ['title']
              },
              outputSchema: {
                type: 'object',
                properties: {
                  id: { type: 'number' },
                  url: { type: 'string' }
                }
              },
              category: 'dashboards'
            }
          ],
          configuration: {
            serverType: 'local',
            endpoint: 'http://localhost:3000',
            authentication: {
              type: 'basic',
              config: {
                username: 'admin',
                password: 'admin'
              }
            },
            ports: [
              { host: 3000, container: 3000 }
            ]
          }
        }
      ];

      let filteredServers = mockServers;

      if (query) {
        const searchQuery = query.toLowerCase();
        filteredServers = filteredServers.filter(server =>
          server.name.toLowerCase().includes(searchQuery) ||
          server.description.toLowerCase().includes(searchQuery) ||
          server.tags.some(tag => tag.toLowerCase().includes(searchQuery))
        );
      }

      if (category && category !== 'All') {
        filteredServers = filteredServers.filter(server => server.category === category);
      }

      this.servers = filteredServers;
      return filteredServers;
    } catch (error) {
      console.error('Failed to discover MCP servers:', error);
      throw new Error('Failed to discover MCP servers');
    }
  }

  /**
   * Get a specific MCP server by ID
   */
  public async getServer(serverId: string): Promise<MCPServer | null> {
    const server = this.servers.find(s => s.id === serverId);
    if (server) {
      return server;
    }

    // If not in cache, try to fetch from registry
    try {
      const response = await fetch(`/api/mcp/servers/${serverId}`);
      if (response.ok) {
        const server = await response.json();
        this.servers.push(server);
        return server;
      }
    } catch (error) {
      console.error('Failed to fetch server:', error);
    }

    return null;
  }

  /**
   * Install an MCP server (pull Docker image)
   */
  public async installServer(serverId: string): Promise<void> {
    const server = await this.getServer(serverId);
    if (!server) {
      throw new Error('Server not found');
    }

    if (server.deploymentType === 'local' && !this.dockerAvailable) {
      throw new Error('Docker is required for local MCP servers');
    }

    try {
      if (server.deploymentType === 'local') {
        // Pull Docker image
        const response = await fetch('/api/docker/pull', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({
            image: `${server.dockerImage}:${server.dockerTag}`
          })
        });

        if (!response.ok) {
          throw new Error('Failed to pull Docker image');
        }
      }

      // Store installation status
      localStorage.setItem(`mcp-server-${serverId}`, JSON.stringify({
        installed: true,
        installedAt: new Date().toISOString()
      }));

    } catch (error) {
      console.error('Failed to install MCP server:', error);
      throw error;
    }
  }

  /**
   * Uninstall an MCP server
   */
  public async uninstallServer(serverId: string): Promise<void> {
    const server = await this.getServer(serverId);
    if (!server) {
      throw new Error('Server not found');
    }

    try {
      if (server.deploymentType === 'local') {
        // Remove Docker image
        const response = await fetch('/api/docker/remove', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({
            image: `${server.dockerImage}:${server.dockerTag}`
          })
        });

        if (!response.ok) {
          throw new Error('Failed to remove Docker image');
        }
      }

      // Remove installation status
      localStorage.removeItem(`mcp-server-${serverId}`);

      // Remove any active connections
      this.connections = this.connections.filter(conn => conn.serverId !== serverId);

    } catch (error) {
      console.error('Failed to uninstall MCP server:', error);
      throw error;
    }
  }

  /**
   * Check if a server is installed
   */
  public isServerInstalled(serverId: string): boolean {
    const installation = localStorage.getItem(`mcp-server-${serverId}`);
    return installation ? JSON.parse(installation).installed : false;
  }

  /**
   * Create a connection to an MCP server
   */
  public async createConnection(serverId: string, name: string, config: MCPConfiguration): Promise<MCPConnection> {
    const server = await this.getServer(serverId);
    if (!server) {
      throw new Error('Server not found');
    }

    const connection: MCPConnection = {
      id: `conn-${Date.now()}`,
      serverId,
      name,
      status: 'disconnected',
      configuration: config
    };

    this.connections.push(connection);
    return connection;
  }

  /**
   * Connect to an MCP server
   */
  public async connectToServer(connectionId: string): Promise<void> {
    const connection = this.connections.find(c => c.id === connectionId);
    if (!connection) {
      throw new Error('Connection not found');
    }

    connection.status = 'connecting';

    try {
      if (connection.configuration.serverType === 'local') {
        // Start Docker container
        const response = await fetch('/api/docker/run', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({
            connectionId,
            configuration: connection.configuration
          })
        });

        if (!response.ok) {
          throw new Error('Failed to start container');
        }
      } else {
        // Connect to remote server
        const response = await fetch('/api/mcp/connect', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({
            connectionId,
            configuration: connection.configuration
          })
        });

        if (!response.ok) {
          throw new Error('Failed to connect to remote server');
        }
      }

      connection.status = 'connected';
      connection.lastConnected = new Date().toISOString();
      connection.error = undefined;

    } catch (error) {
      connection.status = 'error';
      connection.error = error instanceof Error ? error.message : 'Unknown error';
      throw error;
    }
  }

  /**
   * Disconnect from an MCP server
   */
  public async disconnectFromServer(connectionId: string): Promise<void> {
    const connection = this.connections.find(c => c.id === connectionId);
    if (!connection) {
      throw new Error('Connection not found');
    }

    try {
      if (connection.configuration.serverType === 'local') {
        // Stop Docker container
        await fetch('/api/docker/stop', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ connectionId })
        });
      } else {
        // Disconnect from remote server
        await fetch('/api/mcp/disconnect', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ connectionId })
        });
      }

      connection.status = 'disconnected';

    } catch (error) {
      console.error('Failed to disconnect:', error);
      throw error;
    }
  }

  /**
   * Execute a tool on an MCP server
   */
  public async executeTool(connectionId: string, toolName: string, parameters: any): Promise<any> {
    const connection = this.connections.find(c => c.id === connectionId);
    if (!connection) {
      throw new Error('Connection not found');
    }

    if (connection.status !== 'connected') {
      throw new Error('Server not connected');
    }

    try {
      const response = await fetch('/api/mcp/execute', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          connectionId,
          toolName,
          parameters
        })
      });

      if (!response.ok) {
        throw new Error('Tool execution failed');
      }

      return await response.json();

    } catch (error) {
      console.error('Failed to execute tool:', error);
      throw error;
    }
  }

  /**
   * Get all connections
   */
  public getConnections(): MCPConnection[] {
    return [...this.connections];
  }

  /**
   * Get available categories
   */
  public getCategories(): string[] {
    const categories = new Set(this.servers.map(s => s.category));
    return ['All', ...Array.from(categories).sort()];
  }

  /**
   * Get Docker availability status
   */
  public isDockerAvailable(): boolean {
    return this.dockerAvailable;
  }
}

export default MCPCatalogService;
