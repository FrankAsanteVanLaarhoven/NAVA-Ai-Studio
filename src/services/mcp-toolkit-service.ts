/**
 * MCP Toolkit Service
 * 
 * Provides integration with MCP (Model Context Protocol) for AI agent connectivity.
 * Manages connections to MCP servers and provides tools for AI agents to use.
 */

import { MCPServer, MCPConnection, MCPTool } from './mcp-catalog-service';

export interface MCPAgent {
  id: string;
  name: string;
  model: string;
  provider: string;
  connections: string[]; // Connection IDs
  active: boolean;
  lastUsed?: string;
}

export interface MCPToolExecution {
  id: string;
  connectionId: string;
  toolName: string;
  parameters: any;
  result?: any;
  error?: string;
  timestamp: string;
  duration?: number;
}

export interface MCPToolkitConfig {
  defaultAgent?: string;
  autoConnect: boolean;
  maxConcurrentConnections: number;
  timeout: number;
  retryAttempts: number;
}

export class MCPToolkitService {
  private static instance: MCPToolkitService;
  private agents: MCPAgent[] = [];
  private executions: MCPToolExecution[] = [];
  private config: MCPToolkitConfig = {
    autoConnect: true,
    maxConcurrentConnections: 10,
    timeout: 30000,
    retryAttempts: 3
  };

  private constructor() {
    this.loadConfig();
    this.loadAgents();
  }

  public static getInstance(): MCPToolkitService {
    if (!MCPToolkitService.instance) {
      MCPToolkitService.instance = new MCPToolkitService();
    }
    return MCPToolkitService.instance;
  }

  private loadConfig(): void {
    const saved = localStorage.getItem('mcp-toolkit-config');
    if (saved) {
      this.config = { ...this.config, ...JSON.parse(saved) };
    }
  }

  private saveConfig(): void {
    localStorage.setItem('mcp-toolkit-config', JSON.stringify(this.config));
  }

  private loadAgents(): void {
    const saved = localStorage.getItem('mcp-agents');
    if (saved) {
      this.agents = JSON.parse(saved);
    } else {
      // Initialize with default agents
      this.agents = [
        {
          id: 'claude-3-5-sonnet',
          name: 'Claude 3.5 Sonnet',
          model: 'claude-3-5-sonnet-20241022',
          provider: 'anthropic',
          connections: [],
          active: true
        },
        {
          id: 'gpt-4-turbo',
          name: 'GPT-4 Turbo',
          model: 'gpt-4-turbo-preview',
          provider: 'openai',
          connections: [],
          active: true
        },
        {
          id: 'gemini-pro',
          name: 'Gemini Pro',
          model: 'gemini-pro',
          provider: 'google',
          connections: [],
          active: true
        }
      ];
      this.saveAgents();
    }
  }

  private saveAgents(): void {
    localStorage.setItem('mcp-agents', JSON.stringify(this.agents));
  }

  /**
   * Get all available agents
   */
  public getAgents(): MCPAgent[] {
    return [...this.agents];
  }

  /**
   * Get active agents
   */
  public getActiveAgents(): MCPAgent[] {
    return this.agents.filter(agent => agent.active);
  }

  /**
   * Add a new agent
   */
  public addAgent(agent: Omit<MCPAgent, 'id'>): MCPAgent {
    const newAgent: MCPAgent = {
      ...agent,
      id: `agent-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`
    };
    this.agents.push(newAgent);
    this.saveAgents();
    return newAgent;
  }

  /**
   * Update an agent
   */
  public updateAgent(agentId: string, updates: Partial<MCPAgent>): boolean {
    const index = this.agents.findIndex(a => a.id === agentId);
    if (index === -1) return false;
    
    this.agents[index] = { ...this.agents[index], ...updates };
    this.saveAgents();
    return true;
  }

  /**
   * Remove an agent
   */
  public removeAgent(agentId: string): boolean {
    const index = this.agents.findIndex(a => a.id === agentId);
    if (index === -1) return false;
    
    this.agents.splice(index, 1);
    this.saveAgents();
    return true;
  }

  /**
   * Connect an agent to an MCP server
   */
  public async connectAgentToServer(agentId: string, connectionId: string): Promise<boolean> {
    const agent = this.agents.find(a => a.id === agentId);
    if (!agent) return false;

    if (!agent.connections.includes(connectionId)) {
      agent.connections.push(connectionId);
      agent.lastUsed = new Date().toISOString();
      this.saveAgents();
    }

    return true;
  }

  /**
   * Disconnect an agent from an MCP server
   */
  public disconnectAgentFromServer(agentId: string, connectionId: string): boolean {
    const agent = this.agents.find(a => a.id === agentId);
    if (!agent) return false;

    const index = agent.connections.indexOf(connectionId);
    if (index !== -1) {
      agent.connections.splice(index, 1);
      this.saveAgents();
    }

    return true;
  }

  /**
   * Get available tools for an agent
   */
  public async getAvailableTools(agentId: string): Promise<MCPTool[]> {
    const agent = this.agents.find(a => a.id === agentId);
    if (!agent) return [];

    const tools: MCPTool[] = [];
    
    // Import the MCP catalog service to get server details
    const { default: MCPCatalogService } = await import('./mcp-catalog-service');
    const catalogService = MCPCatalogService.getInstance();
    const connections = catalogService.getConnections();

    for (const connectionId of agent.connections) {
      const connection = connections.find(c => c.id === connectionId);
      if (connection && connection.status === 'connected') {
        const server = await catalogService.getServer(connection.serverId);
        if (server) {
          tools.push(...server.tools);
        }
      }
    }

    return tools;
  }

  /**
   * Execute a tool for an agent
   */
  public async executeTool(
    agentId: string, 
    toolName: string, 
    parameters: any
  ): Promise<MCPToolExecution> {
    const agent = this.agents.find(a => a.id === agentId);
    if (!agent) {
      throw new Error('Agent not found');
    }

    const execution: MCPToolExecution = {
      id: `exec-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`,
      connectionId: '',
      toolName,
      parameters,
      timestamp: new Date().toISOString()
    };

    try {
      // Find the connection that has this tool
      const { default: MCPCatalogService } = await import('./mcp-catalog-service');
      const catalogService = MCPCatalogService.getInstance();
      const connections = catalogService.getConnections();

      let toolFound = false;
      for (const connectionId of agent.connections) {
        const connection = connections.find(c => c.id === connectionId);
        if (connection && connection.status === 'connected') {
          const server = await catalogService.getServer(connection.serverId);
          if (server && server.tools.some(tool => tool.name === toolName)) {
            execution.connectionId = connectionId;
            toolFound = true;
            break;
          }
        }
      }

      if (!toolFound) {
        throw new Error(`Tool '${toolName}' not found in any connected server`);
      }

      const startTime = Date.now();
      const result = await catalogService.executeTool(execution.connectionId, toolName, parameters);
      execution.duration = Date.now() - startTime;
      execution.result = result;

      // Update agent last used
      agent.lastUsed = new Date().toISOString();
      this.saveAgents();

    } catch (error) {
      execution.error = error instanceof Error ? error.message : 'Unknown error';
    }

    this.executions.push(execution);
    this.saveExecutions();

    return execution;
  }

  /**
   * Get execution history
   */
  public getExecutions(agentId?: string, limit?: number): MCPToolExecution[] {
    let filtered = this.executions;
    
    if (agentId) {
      // Filter by agent's connections
      const agent = this.agents.find(a => a.id === agentId);
      if (agent) {
        filtered = this.executions.filter(exec => 
          agent.connections.includes(exec.connectionId)
        );
      }
    }

    // Sort by timestamp (newest first)
    filtered.sort((a, b) => new Date(b.timestamp).getTime() - new Date(a.timestamp).getTime());

    if (limit) {
      filtered = filtered.slice(0, limit);
    }

    return filtered;
  }

  /**
   * Get toolkit configuration
   */
  public getConfig(): MCPToolkitConfig {
    return { ...this.config };
  }

  /**
   * Update toolkit configuration
   */
  public updateConfig(updates: Partial<MCPToolkitConfig>): void {
    this.config = { ...this.config, ...updates };
    this.saveConfig();
  }

  /**
   * Get agent statistics
   */
  public getAgentStats(agentId: string): {
    totalExecutions: number;
    successfulExecutions: number;
    failedExecutions: number;
    averageDuration: number;
    lastUsed?: string;
  } {
    const agent = this.agents.find(a => a.id === agentId);
    if (!agent) {
      return {
        totalExecutions: 0,
        successfulExecutions: 0,
        failedExecutions: 0,
        averageDuration: 0
      };
    }

    const agentExecutions = this.executions.filter(exec => 
      agent.connections.includes(exec.connectionId)
    );

    const successful = agentExecutions.filter(exec => !exec.error);
    const failed = agentExecutions.filter(exec => exec.error);
    const durations = agentExecutions
      .filter(exec => exec.duration)
      .map(exec => exec.duration!);

    return {
      totalExecutions: agentExecutions.length,
      successfulExecutions: successful.length,
      failedExecutions: failed.length,
      averageDuration: durations.length > 0 
        ? durations.reduce((a, b) => a + b, 0) / durations.length 
        : 0,
      lastUsed: agent.lastUsed
    };
  }

  /**
   * Clear execution history
   */
  public clearExecutions(agentId?: string): void {
    if (agentId) {
      const agent = this.agents.find(a => a.id === agentId);
      if (agent) {
        this.executions = this.executions.filter(exec => 
          !agent.connections.includes(exec.connectionId)
        );
      }
    } else {
      this.executions = [];
    }
    this.saveExecutions();
  }

  private saveExecutions(): void {
    // Keep only last 1000 executions to prevent storage bloat
    if (this.executions.length > 1000) {
      this.executions = this.executions
        .sort((a, b) => new Date(b.timestamp).getTime() - new Date(a.timestamp).getTime())
        .slice(0, 1000);
    }
    localStorage.setItem('mcp-executions', JSON.stringify(this.executions));
  }

  /**
   * Initialize the toolkit (called on app startup)
   */
  public async initialize(): Promise<void> {
    // Load saved executions
    const saved = localStorage.getItem('mcp-executions');
    if (saved) {
      this.executions = JSON.parse(saved);
    }

    // Auto-connect agents if enabled
    if (this.config.autoConnect) {
      await this.autoConnectAgents();
    }
  }

  private async autoConnectAgents(): Promise<void> {
    const { default: MCPCatalogService } = await import('./mcp-catalog-service');
    const catalogService = MCPCatalogService.getInstance();
    const connections = catalogService.getConnections();

    for (const agent of this.agents) {
      if (agent.active) {
        for (const connectionId of agent.connections) {
          const connection = connections.find(c => c.id === connectionId);
          if (connection && connection.status === 'disconnected') {
            try {
              await catalogService.connectToServer(connectionId);
            } catch (error) {
              console.warn(`Failed to auto-connect agent ${agent.name} to ${connectionId}:`, error);
            }
          }
        }
      }
    }
  }

  /**
   * Generate a tool call prompt for an agent
   */
  public generateToolCallPrompt(agentId: string, userMessage: string): string {
    const agent = this.agents.find(a => a.id === agentId);
    if (!agent) return userMessage;

    // This would be used to enhance the AI prompt with available tools
    // Implementation depends on the specific AI provider integration
    return userMessage;
  }

  /**
   * Parse tool call from agent response
   */
  public parseToolCall(response: string): { toolName: string; parameters: any } | null {
    // Parse tool calls from agent responses
    // This is a simplified implementation - real parsing would be more sophisticated
    try {
      const toolCallMatch = response.match(/<tool_call>(.*?)<\/tool_call>/s);
      if (toolCallMatch) {
        const toolCall = JSON.parse(toolCallMatch[1]);
        return {
          toolName: toolCall.tool,
          parameters: toolCall.parameters
        };
      }
    } catch (error) {
      console.warn('Failed to parse tool call:', error);
    }
    return null;
  }
}

export default MCPToolkitService;
