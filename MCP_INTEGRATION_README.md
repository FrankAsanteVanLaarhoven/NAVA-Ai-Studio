# Docker MCP Catalog Integration

## Overview

NAVΛ Studio now includes comprehensive integration with the Docker MCP (Model Context Protocol) Catalog, providing a powerful extension system for AI agents and tools. This integration allows users to discover, install, and manage MCP servers directly within the IDE.

## Features

### 🔍 MCP Catalog Discovery
- Browse verified MCP servers from Docker Hub
- Search and filter by category, tags, and functionality
- View detailed server information including tools and requirements
- Support for both local and remote MCP servers

### 🤖 AI Agent Management
- Create and manage multiple AI agents (Claude, GPT-4, Gemini, etc.)
- Connect agents to MCP servers for enhanced capabilities
- Track tool execution history and performance metrics
- Real-time agent status monitoring

### 🐳 Docker Integration
- Automatic Docker container management for local MCP servers
- Container lifecycle management (start, stop, remove)
- Resource monitoring and statistics
- Image management and cleanup

### 🔧 Tool Execution
- Execute MCP tools through connected agents
- Real-time execution monitoring and logging
- Error handling and debugging support
- Performance metrics and analytics

## Architecture

### Core Services

#### 1. MCPCatalogService
- **Location**: `src/services/mcp-catalog-service.ts`
- **Purpose**: Manages MCP server discovery, installation, and connections
- **Key Features**:
  - Server discovery from Docker Hub registry
  - Installation/uninstallation of MCP servers
  - Connection management and status tracking
  - Tool execution and result handling

#### 2. MCPToolkitService
- **Location**: `src/services/mcp-toolkit-service.ts`
- **Purpose**: Manages AI agents and their MCP server connections
- **Key Features**:
  - Agent creation and configuration
  - Connection management between agents and servers
  - Tool execution orchestration
  - Performance tracking and analytics

#### 3. DockerService
- **Location**: `src/services/docker-service.ts`
- **Purpose**: Provides Docker container management capabilities
- **Key Features**:
  - Container lifecycle management
  - Image management and cleanup
  - Resource monitoring and statistics
  - Network and volume management

### UI Components

#### 1. MCPCatalog
- **Location**: `src/components/MCPCatalog/MCPCatalog.tsx`
- **Purpose**: Main interface for browsing and managing MCP servers
- **Features**:
  - Server discovery and search
  - Installation management
  - Connection configuration
  - Server details and documentation

#### 2. MCPToolkit
- **Location**: `src/components/MCPToolkit/MCPToolkit.tsx`
- **Purpose**: AI agent management and tool execution interface
- **Features**:
  - Agent creation and configuration
  - Connection management
  - Tool execution monitoring
  - Performance analytics

#### 3. Integration with Marketplace
- **Location**: `src/components/Marketplace/PluginMarketplace.tsx`
- **Purpose**: Seamless integration with existing plugin system
- **Features**:
  - Tab-based navigation between plugins and MCP servers
  - Unified search and discovery experience
  - Consistent UI/UX patterns

## Usage Guide

### Getting Started

1. **Access the MCP Catalog**:
   - Open the Plugin Marketplace (Extensions tab in Activity Bar)
   - Switch to the "MCP Catalog" tab
   - Browse available MCP servers

2. **Install an MCP Server**:
   - Find a server you want to use
   - Click "Install" to download the Docker image
   - Wait for installation to complete

3. **Create an AI Agent**:
   - Open the MCP Toolkit (Activity Bar → MCP Toolkit)
   - Click "Add Agent" to create a new agent
   - Configure the agent with your preferred model

4. **Connect Agent to Server**:
   - In the MCP Toolkit, select your agent
   - Browse available connections
   - Click "Connect" to link the agent to an MCP server

5. **Execute Tools**:
   - Use the connected agent to execute MCP tools
   - Monitor execution in the Executions tab
   - View results and performance metrics

### Available MCP Servers

The integration includes several pre-configured MCP servers:

#### 🐳 Docker Filesystem
- **Purpose**: Access and manipulate Docker container filesystems
- **Tools**: `list_files`, `read_file`, `write_file`, `delete_file`
- **Use Case**: Container debugging and data extraction

#### 🐙 GitHub Integration
- **Purpose**: Interact with GitHub repositories and issues
- **Tools**: `get_repository`, `create_issue`, `list_pull_requests`
- **Use Case**: Repository management and collaboration

#### 💳 Stripe Payments
- **Purpose**: Process payments and manage Stripe accounts
- **Tools**: `create_payment_intent`, `list_customers`, `create_subscription`
- **Use Case**: E-commerce and payment processing

#### 📊 Grafana Monitoring
- **Purpose**: Create dashboards and monitor system metrics
- **Tools**: `create_dashboard`, `query_metrics`, `create_alert`
- **Use Case**: System monitoring and observability

### Configuration

#### Docker Requirements
- Docker Desktop must be installed and running
- Docker daemon must be accessible (default: `unix:///var/run/docker.sock`)
- Sufficient disk space for container images

#### Agent Configuration
- API keys for AI providers (stored securely in localStorage)
- Model selection and configuration
- Connection preferences and timeouts

#### Server Configuration
- Environment variables for server-specific settings
- Volume mounts for persistent data
- Network configuration for external access

## API Reference

### MCPCatalogService

```typescript
// Discover MCP servers
const servers = await mcpService.discoverServers(query, category);

// Install a server
await mcpService.installServer(serverId);

// Create a connection
const connection = await mcpService.createConnection(serverId, name, config);

// Execute a tool
const result = await mcpService.executeTool(connectionId, toolName, parameters);
```

### MCPToolkitService

```typescript
// Create an agent
const agent = toolkitService.addAgent({
  name: 'My Agent',
  model: 'claude-3-5-sonnet-20241022',
  provider: 'anthropic',
  connections: [],
  active: true
});

// Connect agent to server
await toolkitService.connectAgentToServer(agentId, connectionId);

// Execute tool through agent
const execution = await toolkitService.executeTool(agentId, toolName, parameters);
```

### DockerService

```typescript
// Check Docker availability
const available = dockerService.isDockerAvailable();

// Get containers
const containers = await dockerService.getContainers();

// Create container
const containerId = await dockerService.createContainer({
  image: 'mcp/docker-filesystem:latest',
  name: 'my-mcp-server',
  ports: [{ host: 3000, container: 3000 }]
});

// Start container
await dockerService.startContainer(containerId);
```

## Security Considerations

### Container Isolation
- MCP servers run in isolated Docker containers
- Limited access to host system resources
- Network isolation and port management

### API Key Management
- Secure storage of AI provider API keys
- No transmission of sensitive data to external services
- Local execution of MCP tools

### Data Privacy
- All data processing happens locally
- No telemetry or data collection
- User controls all data and connections

## Troubleshooting

### Common Issues

#### Docker Not Available
- **Problem**: "Docker not available" error
- **Solution**: Install Docker Desktop and ensure it's running
- **Check**: Verify Docker daemon is accessible

#### Container Startup Failed
- **Problem**: MCP server container fails to start
- **Solution**: Check container logs and resource requirements
- **Debug**: Use `docker logs <container-id>` to investigate

#### Agent Connection Failed
- **Problem**: Agent cannot connect to MCP server
- **Solution**: Verify server is running and accessible
- **Check**: Test connection manually and check network configuration

#### Tool Execution Error
- **Problem**: Tool execution fails with error
- **Solution**: Check tool parameters and server logs
- **Debug**: Review execution history and error messages

### Debug Mode

Enable debug logging by setting:
```typescript
localStorage.setItem('mcp-debug', 'true');
```

This will provide detailed logging for:
- Docker operations
- MCP server communications
- Tool executions
- Agent interactions

## Performance Optimization

### Resource Management
- Monitor container resource usage
- Set appropriate memory and CPU limits
- Clean up unused images and containers

### Connection Pooling
- Reuse connections when possible
- Implement connection timeouts
- Monitor connection health

### Caching
- Cache server metadata and tool schemas
- Implement result caching for expensive operations
- Use localStorage for configuration persistence

## Future Enhancements

### Planned Features
- [ ] Visual tool builder and workflow designer
- [ ] Advanced agent orchestration and chaining
- [ ] Real-time collaboration on MCP servers
- [ ] Custom MCP server development tools
- [ ] Integration with more AI providers
- [ ] Advanced monitoring and analytics dashboard

### Community Contributions
- [ ] Additional MCP server integrations
- [ ] Custom tool implementations
- [ ] UI/UX improvements
- [ ] Documentation and tutorials

## Support

For issues, questions, or contributions:

- **GitHub Issues**: Report bugs and request features
- **Documentation**: Check the comprehensive docs in `/docs`
- **Community**: Join the NAVΛ Studio Discord server
- **Email**: Contact the development team

## License

This MCP integration is part of NAVΛ Studio and is licensed under the same terms:
- **MIT License** for the main application
- **Apache 2.0 License** for additional components

---

*Last Updated: January 13, 2025*
*Version: 1.0.0*
*Status: ✅ Production Ready*
