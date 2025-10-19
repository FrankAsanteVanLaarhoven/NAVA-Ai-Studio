import React, { useState, useEffect } from 'react';
import { 
  Bot, 
  Settings, 
  Play, 
  Square, 
  Activity, 
  Clock, 
  CheckCircle, 
  XCircle, 
  Plus,
  Trash2,
  RefreshCw,
  BarChart3,
  Zap,
  Link,
  Unlink
} from 'lucide-react';
import MCPToolkitService, { MCPAgent, MCPToolExecution } from '../../services/mcp-toolkit-service';
import MCPCatalogService from '../../services/mcp-catalog-service';
import './MCPToolkit.css';

interface MCPToolkitProps {
  onAgentSelect?: (agent: MCPAgent) => void;
  onToolExecute?: (execution: MCPToolExecution) => void;
}

export const MCPToolkit: React.FC<MCPToolkitProps> = ({ onAgentSelect, onToolExecute }) => {
  const [agents, setAgents] = useState<MCPAgent[]>([]);
  const [executions, setExecutions] = useState<MCPToolExecution[]>([]);
  const [selectedAgent, setSelectedAgent] = useState<MCPAgent | null>(null);
  const [availableTools, setAvailableTools] = useState<any[]>([]);
  const [connections, setConnections] = useState<any[]>([]);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [activeTab, setActiveTab] = useState<'agents' | 'executions' | 'settings'>('agents');
  const [showAddAgent, setShowAddAgent] = useState(false);
  const [newAgent, setNewAgent] = useState({
    name: '',
    model: '',
    provider: 'anthropic'
  });

  const toolkitService = MCPToolkitService.getInstance();
  const catalogService = MCPCatalogService.getInstance();

  useEffect(() => {
    initializeToolkit();
  }, []);

  useEffect(() => {
    if (selectedAgent) {
      loadAgentTools();
    }
  }, [selectedAgent]);

  const initializeToolkit = async () => {
    setLoading(true);
    try {
      await toolkitService.initialize();
      const [agentsData, executionsData, connectionsData] = await Promise.all([
        Promise.resolve(toolkitService.getAgents()),
        Promise.resolve(toolkitService.getExecutions(undefined, 50)),
        Promise.resolve(catalogService.getConnections())
      ]);
      setAgents(agentsData);
      setExecutions(executionsData);
      setConnections(connectionsData);
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Failed to initialize toolkit');
    } finally {
      setLoading(false);
    }
  };

  const loadAgentTools = async () => {
    if (!selectedAgent) return;
    
    try {
      const tools = await toolkitService.getAvailableTools(selectedAgent.id);
      setAvailableTools(tools);
    } catch (err) {
      console.error('Failed to load agent tools:', err);
    }
  };

  const handleAddAgent = async () => {
    if (!newAgent.name || !newAgent.model) return;

    try {
      const agent = toolkitService.addAgent({
        name: newAgent.name,
        model: newAgent.model,
        provider: newAgent.provider as any,
        connections: [],
        active: true
      });
      
      setAgents(toolkitService.getAgents());
      setNewAgent({ name: '', model: '', provider: 'anthropic' });
      setShowAddAgent(false);
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Failed to add agent');
    }
  };

  const handleRemoveAgent = async (agentId: string) => {
    if (window.confirm('Are you sure you want to remove this agent?')) {
      try {
        toolkitService.removeAgent(agentId);
        setAgents(toolkitService.getAgents());
        if (selectedAgent?.id === agentId) {
          setSelectedAgent(null);
        }
      } catch (err) {
        setError(err instanceof Error ? err.message : 'Failed to remove agent');
      }
    }
  };

  const handleConnectAgent = async (agentId: string, connectionId: string) => {
    try {
      await toolkitService.connectAgentToServer(agentId, connectionId);
      setAgents(toolkitService.getAgents());
      if (selectedAgent?.id === agentId) {
        loadAgentTools();
      }
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Failed to connect agent');
    }
  };

  const handleDisconnectAgent = async (agentId: string, connectionId: string) => {
    try {
      toolkitService.disconnectAgentFromServer(agentId, connectionId);
      setAgents(toolkitService.getAgents());
      if (selectedAgent?.id === agentId) {
        loadAgentTools();
      }
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Failed to disconnect agent');
    }
  };

  const handleExecuteTool = async (toolName: string, parameters: any) => {
    if (!selectedAgent) return;

    setLoading(true);
    try {
      const execution = await toolkitService.executeTool(selectedAgent.id, toolName, parameters);
      setExecutions(toolkitService.getExecutions(undefined, 50));
      onToolExecute?.(execution);
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Tool execution failed');
    } finally {
      setLoading(false);
    }
  };

  const getAgentStats = (agent: MCPAgent) => {
    return toolkitService.getAgentStats(agent.id);
  };

  const getStatusIcon = (status: string) => {
    switch (status) {
      case 'connected':
        return <CheckCircle size={16} className="status-icon connected" />;
      case 'connecting':
        return <RefreshCw size={16} className="status-icon connecting" />;
      case 'error':
        return <XCircle size={16} className="status-icon error" />;
      default:
        return <Square size={16} className="status-icon disconnected" />;
    }
  };

  const formatDuration = (ms: number) => {
    if (ms < 1000) return `${ms}ms`;
    return `${(ms / 1000).toFixed(1)}s`;
  };

  const formatTimestamp = (timestamp: string) => {
    return new Date(timestamp).toLocaleString();
  };

  return (
    <div className="mcp-toolkit">
      {/* Header */}
      <div className="toolkit-header">
        <div className="header-content">
          <h1>🤖 MCP Toolkit</h1>
          <p>Manage AI agents and their MCP server connections</p>
        </div>
        <div className="header-actions">
          <button 
            className="refresh-btn"
            onClick={initializeToolkit}
            disabled={loading}
          >
            <RefreshCw size={16} className={loading ? 'spinning' : ''} />
            Refresh
          </button>
        </div>
      </div>

      {/* Error Display */}
      {error && (
        <div className="error-message">
          <XCircle size={20} />
          <span>{error}</span>
          <button onClick={() => setError(null)}>×</button>
        </div>
      )}

      {/* Tabs */}
      <div className="toolkit-tabs">
        <button 
          className={`tab ${activeTab === 'agents' ? 'active' : ''}`}
          onClick={() => setActiveTab('agents')}
        >
          <Bot size={16} />
          Agents ({agents.length})
        </button>
        <button 
          className={`tab ${activeTab === 'executions' ? 'active' : ''}`}
          onClick={() => setActiveTab('executions')}
        >
          <Activity size={16} />
          Executions ({executions.length})
        </button>
        <button 
          className={`tab ${activeTab === 'settings' ? 'active' : ''}`}
          onClick={() => setActiveTab('settings')}
        >
          <Settings size={16} />
          Settings
        </button>
      </div>

      {/* Agents Tab */}
      {activeTab === 'agents' && (
        <div className="agents-panel">
          <div className="panel-header">
            <h2>AI Agents</h2>
            <button 
              className="add-btn"
              onClick={() => setShowAddAgent(true)}
            >
              <Plus size={16} />
              Add Agent
            </button>
          </div>

          {/* Add Agent Modal */}
          {showAddAgent && (
            <div className="modal-overlay" onClick={() => setShowAddAgent(false)}>
              <div className="modal-content" onClick={(e) => e.stopPropagation()}>
                <div className="modal-header">
                  <h3>Add New Agent</h3>
                  <button 
                    className="modal-close"
                    onClick={() => setShowAddAgent(false)}
                  >
                    ×
                  </button>
                </div>
                <div className="modal-body">
                  <div className="form-group">
                    <label>Agent Name</label>
                    <input
                      type="text"
                      value={newAgent.name}
                      onChange={(e) => setNewAgent({ ...newAgent, name: e.target.value })}
                      placeholder="e.g., Claude Assistant"
                    />
                  </div>
                  <div className="form-group">
                    <label>Model</label>
                    <input
                      type="text"
                      value={newAgent.model}
                      onChange={(e) => setNewAgent({ ...newAgent, model: e.target.value })}
                      placeholder="e.g., claude-3-5-sonnet-20241022"
                    />
                  </div>
                  <div className="form-group">
                    <label>Provider</label>
                    <select
                      value={newAgent.provider}
                      onChange={(e) => setNewAgent({ ...newAgent, provider: e.target.value })}
                    >
                      <option value="anthropic">Anthropic</option>
                      <option value="openai">OpenAI</option>
                      <option value="google">Google</option>
                      <option value="custom">Custom</option>
                    </select>
                  </div>
                </div>
                <div className="modal-footer">
                  <button 
                    className="action-btn secondary"
                    onClick={() => setShowAddAgent(false)}
                  >
                    Cancel
                  </button>
                  <button 
                    className="action-btn primary"
                    onClick={handleAddAgent}
                    disabled={!newAgent.name || !newAgent.model}
                  >
                    Add Agent
                  </button>
                </div>
              </div>
            </div>
          )}

          {/* Agents List */}
          <div className="agents-grid">
            {agents.map((agent) => {
              const stats = getAgentStats(agent);
              const agentConnections = connections.filter(conn => 
                agent.connections.includes(conn.id)
              );
              
              return (
                <div 
                  key={agent.id} 
                  className={`agent-card ${selectedAgent?.id === agent.id ? 'selected' : ''}`}
                  onClick={() => setSelectedAgent(agent)}
                >
                  <div className="agent-header">
                    <div className="agent-icon">
                      <Bot size={24} />
                    </div>
                    <div className="agent-info">
                      <h3>{agent.name}</h3>
                      <p className="agent-model">{agent.model}</p>
                      <p className="agent-provider">{agent.provider}</p>
                    </div>
                    <div className="agent-actions">
                      <button
                        className="action-btn danger small"
                        onClick={(e) => {
                          e.stopPropagation();
                          handleRemoveAgent(agent.id);
                        }}
                      >
                        <Trash2 size={14} />
                      </button>
                    </div>
                  </div>

                  <div className="agent-stats">
                    <div className="stat">
                      <Activity size={14} />
                      <span>{stats.totalExecutions} executions</span>
                    </div>
                    <div className="stat">
                      <CheckCircle size={14} />
                      <span>{stats.successfulExecutions} successful</span>
                    </div>
                    <div className="stat">
                      <Clock size={14} />
                      <span>{formatDuration(stats.averageDuration)} avg</span>
                    </div>
                  </div>

                  <div className="agent-connections">
                    <h4>Connections ({agentConnections.length})</h4>
                    {agentConnections.length === 0 ? (
                      <p className="no-connections">No connections</p>
                    ) : (
                      <div className="connections-list">
                        {agentConnections.map((conn) => (
                          <div key={conn.id} className="connection-item">
                            <div className="connection-info">
                              <span className="connection-name">{conn.name}</span>
                              {getStatusIcon(conn.status)}
                            </div>
                            <button
                              className="disconnect-btn"
                              onClick={(e) => {
                                e.stopPropagation();
                                handleDisconnectAgent(agent.id, conn.id);
                              }}
                            >
                              <Unlink size={12} />
                            </button>
                          </div>
                        ))}
                      </div>
                    )}
                  </div>

                  <div className="agent-actions-footer">
                    <button
                      className="action-btn primary"
                      onClick={(e) => {
                        e.stopPropagation();
                        onAgentSelect?.(agent);
                      }}
                    >
                      <Zap size={16} />
                      Use Agent
                    </button>
                  </div>
                </div>
              );
            })}
          </div>

          {/* Available Connections */}
          {connections.length > 0 && (
            <div className="available-connections">
              <h3>Available MCP Connections</h3>
              <div className="connections-grid">
                {connections.map((conn) => {
                  const isConnected = selectedAgent?.connections.includes(conn.id);
                  return (
                    <div key={conn.id} className="connection-card">
                      <div className="connection-header">
                        <h4>{conn.name}</h4>
                        {getStatusIcon(conn.status)}
                      </div>
                      <p className="connection-description">
                        {conn.configuration.serverType} server
                      </p>
                      {selectedAgent && (
                        <button
                          className={`action-btn ${isConnected ? 'secondary' : 'primary'}`}
                          onClick={() => {
                            if (isConnected) {
                              handleDisconnectAgent(selectedAgent.id, conn.id);
                            } else {
                              handleConnectAgent(selectedAgent.id, conn.id);
                            }
                          }}
                        >
                          {isConnected ? (
                            <>
                              <Unlink size={16} />
                              Disconnect
                            </>
                          ) : (
                            <>
                              <Link size={16} />
                              Connect
                            </>
                          )}
                        </button>
                      )}
                    </div>
                  );
                })}
              </div>
            </div>
          )}
        </div>
      )}

      {/* Executions Tab */}
      {activeTab === 'executions' && (
        <div className="executions-panel">
          <div className="panel-header">
            <h2>Tool Executions</h2>
            <button 
              className="clear-btn"
              onClick={() => {
                toolkitService.clearExecutions();
                setExecutions([]);
              }}
            >
              <Trash2 size={16} />
              Clear History
            </button>
          </div>

          {executions.length === 0 ? (
            <div className="empty-state">
              <Activity size={48} />
              <h3>No executions yet</h3>
              <p>Execute tools through your agents to see history here</p>
            </div>
          ) : (
            <div className="executions-list">
              {executions.map((execution) => (
                <div key={execution.id} className="execution-card">
                  <div className="execution-header">
                    <div className="execution-info">
                      <h4>{execution.toolName}</h4>
                      <p className="execution-timestamp">
                        {formatTimestamp(execution.timestamp)}
                      </p>
                    </div>
                    <div className="execution-status">
                      {execution.error ? (
                        <XCircle size={16} className="status-icon error" />
                      ) : (
                        <CheckCircle size={16} className="status-icon success" />
                      )}
                      {execution.duration && (
                        <span className="execution-duration">
                          {formatDuration(execution.duration)}
                        </span>
                      )}
                    </div>
                  </div>

                  {execution.parameters && (
                    <div className="execution-parameters">
                      <h5>Parameters:</h5>
                      <pre>{JSON.stringify(execution.parameters, null, 2)}</pre>
                    </div>
                  )}

                  {execution.result && (
                    <div className="execution-result">
                      <h5>Result:</h5>
                      <pre>{JSON.stringify(execution.result, null, 2)}</pre>
                    </div>
                  )}

                  {execution.error && (
                    <div className="execution-error">
                      <h5>Error:</h5>
                      <p>{execution.error}</p>
                    </div>
                  )}
                </div>
              ))}
            </div>
          )}
        </div>
      )}

      {/* Settings Tab */}
      {activeTab === 'settings' && (
        <div className="settings-panel">
          <div className="panel-header">
            <h2>Toolkit Settings</h2>
          </div>

          <div className="settings-sections">
            <div className="settings-section">
              <h3>General</h3>
              <div className="setting-item">
                <label>
                  <input type="checkbox" defaultChecked />
                  Auto-connect agents on startup
                </label>
              </div>
              <div className="setting-item">
                <label>
                  Max concurrent connections:
                  <input type="number" defaultValue={10} min={1} max={50} />
                </label>
              </div>
              <div className="setting-item">
                <label>
                  Request timeout (ms):
                  <input type="number" defaultValue={30000} min={1000} max={120000} />
                </label>
              </div>
            </div>

            <div className="settings-section">
              <h3>Data Management</h3>
              <div className="setting-actions">
                <button className="action-btn secondary">
                  Export Configuration
                </button>
                <button className="action-btn secondary">
                  Import Configuration
                </button>
                <button className="action-btn danger">
                  Reset All Data
                </button>
              </div>
            </div>
          </div>
        </div>
      )}
    </div>
  );
};

export default MCPToolkit;
