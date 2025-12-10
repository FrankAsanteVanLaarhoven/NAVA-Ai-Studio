/**
 * MCP Connections Service
 * Manages auto-connections to MCP-enabled services (GitHub, Slack, Linear, etc.)
 */

export type MCPServiceType = 'github' | 'slack' | 'linear' | 'jira' | 'notion' | 'figma' | 'google-drive' | 'confluence';

export interface MCPConnection {
  id: string;
  serviceType: MCPServiceType;
  name: string;
  enabled: boolean;
  connected: boolean;
  connectedAt?: string;
  lastUsed?: string;
  config: {
    apiKey?: string;
    accessToken?: string;
    refreshToken?: string;
    organization?: string;
    workspace?: string;
    webhookUrl?: string;
    scopes?: string[];
  };
  metadata: {
    userId?: string;
    userName?: string;
    avatar?: string;
    email?: string;
  };
}

export interface MCPServiceDefinition {
  type: MCPServiceType;
  name: string;
  description: string;
  icon: string;
  authType: 'oauth' | 'api-key' | 'pat' | 'webhook';
  authUrl?: string;
  scopes?: string[];
  apiEndpoint?: string;
  documentationUrl?: string;
}

class MCPConnectionsService {
  private readonly STORAGE_KEY = 'nava_mcp_connections';
  private readonly CONNECTIONS_STORAGE_KEY = 'nava_mcp_connection_tokens';

  // Service definitions
  private readonly serviceDefinitions: MCPServiceDefinition[] = [
    {
      type: 'github',
      name: 'GitHub',
      description: 'Access repositories, issues, pull requests, and more',
      icon: '🐙',
      authType: 'oauth',
      authUrl: 'https://github.com/login/oauth/authorize',
      scopes: ['repo', 'read:org', 'read:user', 'read:project'],
      apiEndpoint: 'https://api.github.com',
      documentationUrl: 'https://docs.github.com/en/rest',
    },
    {
      type: 'slack',
      name: 'Slack',
      description: 'Send messages, manage channels, and interact with your workspace',
      icon: '💬',
      authType: 'oauth',
      authUrl: 'https://slack.com/oauth/v2/authorize',
      scopes: ['chat:write', 'channels:read', 'users:read', 'files:read'],
      apiEndpoint: 'https://slack.com/api',
      documentationUrl: 'https://api.slack.com',
    },
    {
      type: 'linear',
      name: 'Linear',
      description: 'Manage issues, projects, and team workflows',
      icon: '📋',
      authType: 'api-key',
      apiEndpoint: 'https://api.linear.app/graphql',
      documentationUrl: 'https://developers.linear.app/docs',
    },
    {
      type: 'jira',
      name: 'Jira',
      description: 'Track issues, sprints, and project management',
      icon: '🎯',
      authType: 'api-key',
      apiEndpoint: 'https://your-domain.atlassian.net/rest/api/3',
      documentationUrl: 'https://developer.atlassian.com/cloud/jira/platform/rest/v3',
    },
    {
      type: 'notion',
      name: 'Notion',
      description: 'Access and manage your Notion pages and databases',
      icon: '📝',
      authType: 'oauth',
      authUrl: 'https://api.notion.com/v1/oauth/authorize',
      scopes: ['read', 'update', 'insert'],
      apiEndpoint: 'https://api.notion.com/v1',
      documentationUrl: 'https://developers.notion.com',
    },
    {
      type: 'figma',
      name: 'Figma',
      description: 'Access design files, components, and assets',
      icon: '🎨',
      authType: 'pat',
      apiEndpoint: 'https://api.figma.com/v1',
      documentationUrl: 'https://www.figma.com/developers/api',
    },
    {
      type: 'google-drive',
      name: 'Google Drive',
      description: 'Access and manage files in Google Drive',
      icon: '📁',
      authType: 'oauth',
      authUrl: 'https://accounts.google.com/o/oauth2/v2/auth',
      scopes: ['https://www.googleapis.com/auth/drive.readonly'],
      apiEndpoint: 'https://www.googleapis.com/drive/v3',
      documentationUrl: 'https://developers.google.com/drive/api',
    },
    {
      type: 'confluence',
      name: 'Confluence',
      description: 'Access and manage Confluence pages and spaces',
      icon: '📚',
      authType: 'api-key',
      apiEndpoint: 'https://your-domain.atlassian.net/wiki/rest/api',
      documentationUrl: 'https://developer.atlassian.com/cloud/confluence/rest/v2',
    },
  ];

  /**
   * Get all service definitions
   */
  getServiceDefinitions(): MCPServiceDefinition[] {
    return this.serviceDefinitions;
  }

  /**
   * Get service definition by type
   */
  getServiceDefinition(type: MCPServiceType): MCPServiceDefinition | undefined {
    return this.serviceDefinitions.find(s => s.type === type);
  }

  /**
   * Get all connections
   */
  getConnections(): MCPConnection[] {
    const stored = localStorage.getItem(this.STORAGE_KEY);
    if (stored) {
      try {
        return JSON.parse(stored);
      } catch {
        // Fall through to defaults
      }
    }
    return [];
  }

  /**
   * Get connection by ID
   */
  getConnection(id: string): MCPConnection | undefined {
    return this.getConnections().find(c => c.id === id);
  }

  /**
   * Get connections by service type
   */
  getConnectionsByService(type: MCPServiceType): MCPConnection[] {
    return this.getConnections().filter(c => c.serviceType === type);
  }

  /**
   * Create a new connection
   */
  createConnection(serviceType: MCPServiceType, name: string, config: MCPConnection['config']): MCPConnection {
    const connections = this.getConnections();
    const newConnection: MCPConnection = {
      id: `mcp-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`,
      serviceType,
      name,
      enabled: true,
      connected: false,
      config,
      metadata: {},
    };
    connections.push(newConnection);
    this.saveConnections(connections);
    return newConnection;
  }

  /**
   * Update connection
   */
  updateConnection(id: string, updates: Partial<MCPConnection>): MCPConnection | null {
    const connections = this.getConnections();
    const index = connections.findIndex(c => c.id === id);
    if (index === -1) return null;

    connections[index] = { ...connections[index], ...updates };
    this.saveConnections(connections);
    return connections[index];
  }

  /**
   * Delete connection
   */
  deleteConnection(id: string): boolean {
    const connections = this.getConnections();
    const index = connections.findIndex(c => c.id === id);
    if (index === -1) return false;

    // Remove tokens
    this.removeConnectionTokens(id);
    connections.splice(index, 1);
    this.saveConnections(connections);
    return true;
  }

  /**
   * Connect to service (OAuth flow)
   */
  async connectOAuth(serviceType: MCPServiceType, connectionId?: string): Promise<void> {
    const serviceDef = this.getServiceDefinition(serviceType);
    if (!serviceDef || serviceDef.authType !== 'oauth') {
      throw new Error(`Service ${serviceType} does not support OAuth`);
    }

    // Generate OAuth URL
    const state = connectionId || `mcp-${Date.now()}`;
    const redirectUri = `${window.location.origin}/mcp-oauth-callback`;
    const clientId = this.getOAuthClientId(serviceType);
    
    if (!clientId) {
      throw new Error(`OAuth client ID not configured for ${serviceType}`);
    }

    const params = new URLSearchParams({
      client_id: clientId,
      redirect_uri: redirectUri,
      state,
      scope: serviceDef.scopes?.join(' ') || '',
      response_type: 'code',
    });

    const authUrl = `${serviceDef.authUrl}?${params.toString()}`;
    
    // Store state for callback
    localStorage.setItem(`mcp_oauth_state_${state}`, JSON.stringify({ serviceType, connectionId }));
    
    // Open OAuth window
    window.open(authUrl, 'mcp-oauth', 'width=600,height=700');
  }

  /**
   * Handle OAuth callback
   */
  async handleOAuthCallback(code: string, state: string): Promise<MCPConnection> {
    const stateData = localStorage.getItem(`mcp_oauth_state_${state}`);
    if (!stateData) {
      throw new Error('Invalid OAuth state');
    }

    const { serviceType, connectionId } = JSON.parse(stateData);
    localStorage.removeItem(`mcp_oauth_state_${state}`);

    // Exchange code for token (this would typically be done server-side)
    const accessToken = await this.exchangeCodeForToken(serviceType, code);
    
    // Update or create connection
    let connection: MCPConnection;
    if (connectionId) {
      connection = this.updateConnection(connectionId, {
        connected: true,
        connectedAt: new Date().toISOString(),
        config: { accessToken },
      })!;
    } else {
      const serviceDef = this.getServiceDefinition(serviceType)!;
      connection = this.createConnection(serviceType, serviceDef.name, { accessToken });
      this.updateConnection(connection.id, {
        connected: true,
        connectedAt: new Date().toISOString(),
      });
    }

    // Fetch user metadata
    await this.fetchUserMetadata(connection);

    // Store tokens securely
    this.saveConnectionTokens(connection.id, { accessToken });

    return connection;
  }

  /**
   * Connect with API key
   */
  async connectWithApiKey(serviceType: MCPServiceType, apiKey: string, name?: string): Promise<MCPConnection> {
    const serviceDef = this.getServiceDefinition(serviceType);
    if (!serviceDef || serviceDef.authType !== 'api-key' && serviceDef.authType !== 'pat') {
      throw new Error(`Service ${serviceType} does not support API key authentication`);
    }

    const connectionName = name || serviceDef.name;
    const connection = this.createConnection(serviceType, connectionName, { apiKey });
    
    // Test connection
    const isValid = await this.testConnection(connection);
    if (!isValid) {
      this.deleteConnection(connection.id);
      throw new Error('Invalid API key or connection failed');
    }

    // Store tokens securely
    this.saveConnectionTokens(connection.id, { apiKey });

    // Update connection status
    this.updateConnection(connection.id, {
      connected: true,
      connectedAt: new Date().toISOString(),
    });

    // Fetch user metadata
    await this.fetchUserMetadata(connection);

    return connection;
  }

  /**
   * Disconnect service
   */
  disconnectConnection(id: string): void {
    this.updateConnection(id, {
      connected: false,
      config: {},
      metadata: {},
    });
    this.removeConnectionTokens(id);
  }

  /**
   * Test connection
   */
  async testConnection(connection: MCPConnection): Promise<boolean> {
    try {
      const serviceDef = this.getServiceDefinition(connection.serviceType);
      if (!serviceDef) return false;

      // Simple test based on service type
      switch (connection.serviceType) {
        case 'github':
          const githubResponse = await fetch('https://api.github.com/user', {
            headers: {
              'Authorization': `token ${connection.config.accessToken || connection.config.apiKey}`,
            },
          });
          return githubResponse.ok;
        
        case 'slack':
          const slackResponse = await fetch('https://slack.com/api/auth.test', {
            headers: {
              'Authorization': `Bearer ${connection.config.accessToken}`,
            },
          });
          const slackData = await slackResponse.json();
          return slackData.ok === true;
        
        case 'linear':
          const linearResponse = await fetch('https://api.linear.app/graphql', {
            method: 'POST',
            headers: {
              'Authorization': connection.config.apiKey || '',
              'Content-Type': 'application/json',
            },
            body: JSON.stringify({ query: '{ viewer { id } }' }),
          });
          return linearResponse.ok;
        
        default:
          return true; // Assume valid for unknown services
      }
    } catch {
      return false;
    }
  }

  /**
   * Fetch user metadata
   */
  private async fetchUserMetadata(connection: MCPConnection): Promise<void> {
    try {
      switch (connection.serviceType) {
        case 'github':
          const githubResponse = await fetch('https://api.github.com/user', {
            headers: {
              'Authorization': `token ${connection.config.accessToken || connection.config.apiKey}`,
            },
          });
          if (githubResponse.ok) {
            const user = await githubResponse.json();
            this.updateConnection(connection.id, {
              metadata: {
                userId: user.id.toString(),
                userName: user.login,
                avatar: user.avatar_url,
                email: user.email,
              },
            });
          }
          break;
        
        case 'slack':
          const slackResponse = await fetch('https://slack.com/api/auth.test', {
            headers: {
              'Authorization': `Bearer ${connection.config.accessToken}`,
            },
          });
          if (slackResponse.ok) {
            const data = await slackResponse.json();
            this.updateConnection(connection.id, {
              metadata: {
                userId: data.user_id,
                userName: data.user,
              },
            });
          }
          break;
      }
    } catch {
      // Silently fail - metadata is optional
    }
  }

  /**
   * Exchange OAuth code for token
   * Note: In production, this should be done server-side for security
   */
  private async exchangeCodeForToken(serviceType: MCPServiceType, code: string): Promise<string> {
    const clientId = this.getOAuthClientId(serviceType);
    const clientSecret = this.getOAuthClientSecret(serviceType);
    const redirectUri = `${window.location.origin}/mcp-oauth-callback`;
    
    if (!clientId || !clientSecret) {
      throw new Error(`OAuth credentials not configured for ${serviceType}`);
    }

    const serviceDef = this.getServiceDefinition(serviceType);
    if (!serviceDef) {
      throw new Error(`Service definition not found for ${serviceType}`);
    }

    try {
      // For GitHub
      if (serviceType === 'github') {
        const response = await fetch('https://github.com/login/oauth/access_token', {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
            'Accept': 'application/json',
          },
          body: JSON.stringify({
            client_id: clientId,
            client_secret: clientSecret,
            code,
            redirect_uri: redirectUri,
          }),
        });
        const data = await response.json();
        if (data.error) {
          throw new Error(data.error_description || data.error);
        }
        return data.access_token;
      }

      // For Slack
      if (serviceType === 'slack') {
        const response = await fetch('https://slack.com/api/oauth.v2.access', {
          method: 'POST',
          headers: {
            'Content-Type': 'application/x-www-form-urlencoded',
          },
          body: new URLSearchParams({
            client_id: clientId,
            client_secret: clientSecret,
            code,
            redirect_uri: redirectUri,
          }),
        });
        const data = await response.json();
        if (!data.ok) {
          throw new Error(data.error || 'Slack OAuth failed');
        }
        return data.access_token;
      }

      // For other services, you would implement their specific OAuth flow
      // For now, return a placeholder (in production, implement proper OAuth)
      throw new Error(`OAuth token exchange not yet implemented for ${serviceType}`);
    } catch (error: any) {
      console.error(`OAuth token exchange failed for ${serviceType}:`, error);
      throw new Error(`Failed to exchange OAuth code: ${error.message}`);
    }
  }

  /**
   * Get OAuth client ID (from env or config)
   */
  private getOAuthClientId(serviceType: MCPServiceType): string | null {
    const envKey = `VITE_MCP_${serviceType.toUpperCase()}_CLIENT_ID`;
    return import.meta.env[envKey] || localStorage.getItem(`mcp_${serviceType}_client_id`);
  }

  /**
   * Get OAuth client secret (from env or config)
   */
  private getOAuthClientSecret(serviceType: MCPServiceType): string | null {
    const envKey = `VITE_MCP_${serviceType.toUpperCase()}_CLIENT_SECRET`;
    return import.meta.env[envKey] || localStorage.getItem(`mcp_${serviceType}_client_secret`);
  }

  /**
   * Save connections
   */
  private saveConnections(connections: MCPConnection[]): void {
    // Don't save sensitive data in connections
    const sanitized = connections.map(c => ({
      ...c,
      config: {
        ...c.config,
        apiKey: undefined,
        accessToken: undefined,
        refreshToken: undefined,
      },
    }));
    localStorage.setItem(this.STORAGE_KEY, JSON.stringify(sanitized));
  }

  /**
   * Save connection tokens securely
   */
  private saveConnectionTokens(connectionId: string, tokens: { accessToken?: string; apiKey?: string; refreshToken?: string }): void {
    const allTokens = this.getConnectionTokens();
    allTokens[connectionId] = tokens;
    localStorage.setItem(this.CONNECTIONS_STORAGE_KEY, JSON.stringify(allTokens));
  }

  /**
   * Get connection tokens
   */
  private getConnectionTokens(): Record<string, { accessToken?: string; apiKey?: string; refreshToken?: string }> {
    const stored = localStorage.getItem(this.CONNECTIONS_STORAGE_KEY);
    if (stored) {
      try {
        return JSON.parse(stored);
      } catch {
        // Fall through to defaults
      }
    }
    return {};
  }

  /**
   * Remove connection tokens
   */
  private removeConnectionTokens(connectionId: string): void {
    const allTokens = this.getConnectionTokens();
    delete allTokens[connectionId];
    localStorage.setItem(this.CONNECTIONS_STORAGE_KEY, JSON.stringify(allTokens));
  }
}

export const mcpConnectionsService = new MCPConnectionsService();

