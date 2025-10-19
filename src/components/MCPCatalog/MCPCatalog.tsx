import React, { useState, useEffect } from 'react';
import { Search, Download, Settings, Star, TrendingUp, Docker, Cloud, Server, Play, Square, RefreshCw, AlertCircle, CheckCircle, ExternalLink } from 'lucide-react';
import MCPCatalogService, { MCPServer, MCPConnection } from '../../services/mcp-catalog-service';
import './MCPCatalog.css';

interface MCPCatalogProps {
  onServerSelect?: (server: MCPServer) => void;
  onConnectionSelect?: (connection: MCPConnection) => void;
}

export const MCPCatalog: React.FC<MCPCatalogProps> = ({ onServerSelect, onConnectionSelect }) => {
  const [servers, setServers] = useState<MCPServer[]>([]);
  const [connections, setConnections] = useState<MCPConnection[]>([]);
  const [searchQuery, setSearchQuery] = useState('');
  const [selectedCategory, setSelectedCategory] = useState('All');
  const [sortBy, setSortBy] = useState<'popular' | 'recent' | 'rating'>('popular');
  const [selectedServer, setSelectedServer] = useState<MCPServer | null>(null);
  const [selectedConnection, setSelectedConnection] = useState<MCPConnection | null>(null);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [activeTab, setActiveTab] = useState<'catalog' | 'connections'>('catalog');
  const [dockerAvailable, setDockerAvailable] = useState(false);

  const mcpService = MCPCatalogService.getInstance();

  useEffect(() => {
    initializeCatalog();
  }, []);

  const initializeCatalog = async () => {
    setLoading(true);
    setError(null);
    try {
      const [serversData, connectionsData] = await Promise.all([
        mcpService.discoverServers(),
        Promise.resolve(mcpService.getConnections())
      ]);
      setServers(serversData);
      setConnections(connectionsData);
      setDockerAvailable(mcpService.isDockerAvailable());
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Failed to load MCP catalog');
    } finally {
      setLoading(false);
    }
  };

  const handleSearch = async () => {
    setLoading(true);
    setError(null);
    try {
      const serversData = await mcpService.discoverServers(searchQuery, selectedCategory);
      setServers(serversData);
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Search failed');
    } finally {
      setLoading(false);
    }
  };

  const handleInstallServer = async (server: MCPServer) => {
    setLoading(true);
    try {
      await mcpService.installServer(server.id);
      // Refresh servers to update installation status
      const serversData = await mcpService.discoverServers(searchQuery, selectedCategory);
      setServers(serversData);
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Installation failed');
    } finally {
      setLoading(false);
    }
  };

  const handleUninstallServer = async (server: MCPServer) => {
    setLoading(true);
    try {
      await mcpService.uninstallServer(server.id);
      // Refresh servers to update installation status
      const serversData = await mcpService.discoverServers(searchQuery, selectedCategory);
      setServers(serversData);
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Uninstallation failed');
    } finally {
      setLoading(false);
    }
  };

  const handleConnectServer = async (connection: MCPConnection) => {
    setLoading(true);
    try {
      await mcpService.connectToServer(connection.id);
      // Refresh connections
      setConnections(mcpService.getConnections());
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Connection failed');
    } finally {
      setLoading(false);
    }
  };

  const handleDisconnectServer = async (connection: MCPConnection) => {
    setLoading(true);
    try {
      await mcpService.disconnectFromServer(connection.id);
      // Refresh connections
      setConnections(mcpService.getConnections());
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Disconnection failed');
    } finally {
      setLoading(false);
    }
  };

  const filteredServers = servers
    .filter((server) => {
      if (selectedCategory !== 'All' && server.category !== selectedCategory) {
        return false;
      }
      if (searchQuery) {
        const query = searchQuery.toLowerCase();
        return (
          server.name.toLowerCase().includes(query) ||
          server.description.toLowerCase().includes(query) ||
          server.tags.some((tag) => tag.includes(query))
        );
      }
      return true;
    })
    .sort((a, b) => {
      switch (sortBy) {
        case 'popular':
          return b.downloads - a.downloads;
        case 'rating':
          return b.rating - a.rating;
        case 'recent':
          return new Date(b.lastUpdated).getTime() - new Date(a.lastUpdated).getTime();
        default:
          return 0;
      }
    });

  const categories = mcpService.getCategories();

  const getStatusIcon = (status: MCPConnection['status']) => {
    switch (status) {
      case 'connected':
        return <CheckCircle size={16} className="status-icon connected" />;
      case 'connecting':
        return <RefreshCw size={16} className="status-icon connecting" />;
      case 'error':
        return <AlertCircle size={16} className="status-icon error" />;
      default:
        return <Square size={16} className="status-icon disconnected" />;
    }
  };

  const getDeploymentIcon = (type: 'local' | 'remote') => {
    return type === 'local' ? <Docker size={16} /> : <Cloud size={16} />;
  };

  return (
    <div className="mcp-catalog">
      {/* Header */}
      <div className="catalog-header">
        <div className="header-content">
          <h1>Docker MCP Catalog</h1>
          <p>Discover and manage Model Context Protocol servers</p>
        </div>
        <div className="header-actions">
          <button 
            className="refresh-btn"
            onClick={initializeCatalog}
            disabled={loading}
          >
            <RefreshCw size={16} className={loading ? 'spinning' : ''} />
            Refresh
          </button>
        </div>
      </div>

      {/* Docker Status */}
      {!dockerAvailable && (
        <div className="docker-warning">
          <AlertCircle size={20} />
          <div>
            <strong>Docker not available</strong>
            <p>Local MCP servers require Docker. Install Docker Desktop to use local servers.</p>
          </div>
        </div>
      )}

      {/* Error Display */}
      {error && (
        <div className="error-message">
          <AlertCircle size={20} />
          <span>{error}</span>
          <button onClick={() => setError(null)}>×</button>
        </div>
      )}

      {/* Tabs */}
      <div className="catalog-tabs">
        <button 
          className={`tab ${activeTab === 'catalog' ? 'active' : ''}`}
          onClick={() => setActiveTab('catalog')}
        >
          <Server size={16} />
          Catalog
        </button>
        <button 
          className={`tab ${activeTab === 'connections' ? 'active' : ''}`}
          onClick={() => setActiveTab('connections')}
        >
          <Settings size={16} />
          Connections ({connections.length})
        </button>
      </div>

      {/* Catalog Tab */}
      {activeTab === 'catalog' && (
        <>
          {/* Search and Filters */}
          <div className="catalog-controls">
            <div className="search-box">
              <Search size={18} className="search-icon" />
              <input
                type="text"
                placeholder="Search MCP servers..."
                value={searchQuery}
                onChange={(e) => setSearchQuery(e.target.value)}
                onKeyPress={(e) => e.key === 'Enter' && handleSearch()}
              />
            </div>
            <div className="filter-controls">
              <select
                value={selectedCategory}
                onChange={(e) => setSelectedCategory(e.target.value)}
                className="category-select"
              >
                {categories.map((category) => (
                  <option key={category} value={category}>
                    {category}
                  </option>
                ))}
              </select>
              <select
                value={sortBy}
                onChange={(e) => setSortBy(e.target.value as any)}
                className="sort-select"
              >
                <option value="popular">Most Popular</option>
                <option value="rating">Highest Rated</option>
                <option value="recent">Recently Updated</option>
              </select>
            </div>
          </div>

          {/* Servers Grid */}
          <div className="servers-grid">
            {loading ? (
              <div className="loading-state">
                <RefreshCw size={32} className="spinning" />
                <p>Loading MCP servers...</p>
              </div>
            ) : filteredServers.length === 0 ? (
              <div className="empty-state">
                <Server size={48} />
                <h3>No servers found</h3>
                <p>Try adjusting your search or filters</p>
              </div>
            ) : (
              filteredServers.map((server) => (
                <div key={server.id} className="server-card">
                  <div className="server-header">
                    <div className="server-icon">{server.icon}</div>
                    <div className="server-info">
                      <h3>{server.name}</h3>
                      <p className="server-author">{server.author}</p>
                    </div>
                    <div className="server-badges">
                      {server.verified && (
                        <span className="badge verified">Verified</span>
                      )}
                      <span className="badge deployment">
                        {getDeploymentIcon(server.deploymentType)}
                        {server.deploymentType}
                      </span>
                    </div>
                  </div>
                  
                  <p className="server-description">{server.description}</p>
                  
                  <div className="server-stats">
                    <span className="stat">
                      <Download size={12} />
                      {server.downloads.toLocaleString()}
                    </span>
                    <span className="stat">
                      <Star size={12} fill="#ffa500" />
                      {server.rating}
                    </span>
                    <span className="stat">
                      {server.category}
                    </span>
                  </div>

                  <div className="server-tags">
                    {server.tags.slice(0, 3).map((tag) => (
                      <span key={tag} className="tag">
                        {tag}
                      </span>
                    ))}
                  </div>

                  <div className="server-actions">
                    {mcpService.isServerInstalled(server.id) ? (
                      <>
                        <button
                          className="action-btn secondary"
                          onClick={() => handleUninstallServer(server)}
                          disabled={loading}
                        >
                          Uninstall
                        </button>
                        <button
                          className="action-btn primary"
                          onClick={() => setSelectedServer(server)}
                        >
                          Configure
                        </button>
                      </>
                    ) : (
                      <button
                        className="action-btn primary"
                        onClick={() => handleInstallServer(server)}
                        disabled={loading || (server.deploymentType === 'local' && !dockerAvailable)}
                      >
                        <Download size={16} />
                        Install
                      </button>
                    )}
                  </div>
                </div>
              ))
            )}
          </div>
        </>
      )}

      {/* Connections Tab */}
      {activeTab === 'connections' && (
        <div className="connections-panel">
          {connections.length === 0 ? (
            <div className="empty-state">
              <Settings size={48} />
              <h3>No connections</h3>
              <p>Install and configure MCP servers to create connections</p>
            </div>
          ) : (
            <div className="connections-list">
              {connections.map((connection) => {
                const server = servers.find(s => s.id === connection.serverId);
                return (
                  <div key={connection.id} className="connection-card">
                    <div className="connection-header">
                      <div className="connection-info">
                        <h4>{connection.name}</h4>
                        <p>{server?.name || 'Unknown Server'}</p>
                      </div>
                      <div className="connection-status">
                        {getStatusIcon(connection.status)}
                        <span className={`status-text ${connection.status}`}>
                          {connection.status}
                        </span>
                      </div>
                    </div>
                    
                    {connection.error && (
                      <div className="connection-error">
                        <AlertCircle size={16} />
                        <span>{connection.error}</span>
                      </div>
                    )}

                    <div className="connection-actions">
                      {connection.status === 'connected' ? (
                        <button
                          className="action-btn secondary"
                          onClick={() => handleDisconnectServer(connection)}
                          disabled={loading}
                        >
                          <Square size={16} />
                          Disconnect
                        </button>
                      ) : (
                        <button
                          className="action-btn primary"
                          onClick={() => handleConnectServer(connection)}
                          disabled={loading || connection.status === 'connecting'}
                        >
                          <Play size={16} />
                          Connect
                        </button>
                      )}
                      <button
                        className="action-btn secondary"
                        onClick={() => setSelectedConnection(connection)}
                      >
                        <Settings size={16} />
                        Configure
                      </button>
                    </div>
                  </div>
                );
              })}
            </div>
          )}
        </div>
      )}

      {/* Server Detail Modal */}
      {selectedServer && (
        <div className="modal-overlay" onClick={() => setSelectedServer(null)}>
          <div className="modal-content" onClick={(e) => e.stopPropagation()}>
            <div className="modal-header">
              <div className="modal-icon">{selectedServer.icon}</div>
              <div className="modal-info">
                <h2>{selectedServer.name}</h2>
                <p className="modal-author">{selectedServer.author}</p>
                <div className="modal-badges">
                  {selectedServer.verified && (
                    <span className="badge verified">Verified</span>
                  )}
                  <span className="badge deployment">
                    {getDeploymentIcon(selectedServer.deploymentType)}
                    {selectedServer.deploymentType}
                  </span>
                </div>
              </div>
              <button 
                className="modal-close"
                onClick={() => setSelectedServer(null)}
              >
                ×
              </button>
            </div>

            <div className="modal-body">
              <div className="modal-section">
                <h3>Description</h3>
                <p>{selectedServer.longDescription}</p>
              </div>

              <div className="modal-section">
                <h3>Tools ({selectedServer.tools.length})</h3>
                <div className="tools-list">
                  {selectedServer.tools.map((tool, index) => (
                    <div key={index} className="tool-item">
                      <h4>{tool.name}</h4>
                      <p>{tool.description}</p>
                      <span className="tool-category">{tool.category}</span>
                    </div>
                  ))}
                </div>
              </div>

              <div className="modal-section">
                <h3>Requirements</h3>
                <div className="requirements">
                  <div className="requirement">
                    <strong>Docker:</strong> {selectedServer.requirements.docker ? 'Required' : 'Not required'}
                  </div>
                  {selectedServer.requirements.memory && (
                    <div className="requirement">
                      <strong>Memory:</strong> {selectedServer.requirements.memory}
                    </div>
                  )}
                  {selectedServer.requirements.cpu && (
                    <div className="requirement">
                      <strong>CPU:</strong> {selectedServer.requirements.cpu}
                    </div>
                  )}
                </div>
              </div>

              {selectedServer.repository && (
                <div className="modal-section">
                  <h3>Links</h3>
                  <div className="modal-links">
                    <a 
                      href={selectedServer.repository} 
                      target="_blank" 
                      rel="noopener noreferrer"
                      className="modal-link"
                    >
                      <ExternalLink size={16} />
                      Repository
                    </a>
                    {selectedServer.documentation && (
                      <a 
                        href={selectedServer.documentation} 
                        target="_blank" 
                        rel="noopener noreferrer"
                        className="modal-link"
                      >
                        <ExternalLink size={16} />
                        Documentation
                      </a>
                    )}
                  </div>
                </div>
              )}
            </div>

            <div className="modal-footer">
              {mcpService.isServerInstalled(selectedServer.id) ? (
                <button
                  className="action-btn secondary"
                  onClick={() => {
                    handleUninstallServer(selectedServer);
                    setSelectedServer(null);
                  }}
                >
                  Uninstall
                </button>
              ) : (
                <button
                  className="action-btn primary"
                  onClick={() => {
                    handleInstallServer(selectedServer);
                    setSelectedServer(null);
                  }}
                  disabled={selectedServer.deploymentType === 'local' && !dockerAvailable}
                >
                  <Download size={16} />
                  Install
                </button>
              )}
            </div>
          </div>
        </div>
      )}
    </div>
  );
};

export default MCPCatalog;
