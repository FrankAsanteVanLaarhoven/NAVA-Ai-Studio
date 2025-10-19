import React, { useState, useEffect } from 'react';
import { Server, Package } from 'lucide-react';
import MCPCatalog from '../MCPCatalog/MCPCatalog';
import './PluginMarketplace.css';

interface Plugin {
  id: string;
  name: string;
  author: string;
  version: string;
  description: string;
  longDescription: string;
  category: string;
  icon: string;
  downloads: number;
  rating: number;
  reviews: number;
  tags: string[];
  installed: boolean;
  featured: boolean;
  price: number; // 0 for free
  screenshots: string[];
  homepage?: string;
  repository?: string;
}

const PLUGIN_CATEGORIES = [
  'All',
  'AI & ML',
  'Visualization',
  'Cloud & DevOps',
  'Languages',
  'Themes',
  'Tools',
  'Education',
];

export const PluginMarketplace: React.FC = () => {
  const [plugins, setPlugins] = useState<Plugin[]>([]);
  const [selectedCategory, setSelectedCategory] = useState('All');
  const [searchQuery, setSearchQuery] = useState('');
  const [sortBy, setSortBy] = useState<'popular' | 'recent' | 'rating'>('popular');
  const [selectedPlugin, setSelectedPlugin] = useState<Plugin | null>(null);
  const [activeTab, setActiveTab] = useState<'plugins' | 'mcp'>('plugins');

  useEffect(() => {
    loadPlugins();
  }, []);

  const loadPlugins = async () => {
    // TODO: Load from actual plugin registry
    const mockPlugins: Plugin[] = [
      {
        id: 'tensorflow-integration',
        name: 'TensorFlow Integration',
        author: 'NAVΛ Team',
        version: '1.2.0',
        description: 'ML-powered navigation field optimization using TensorFlow',
        longDescription: 'This plugin integrates TensorFlow for machine learning-guided navigation optimization. Train neural networks to predict optimal paths and use reinforcement learning for dynamic navigation scenarios.',
        category: 'AI & ML',
        icon: '🧠',
        downloads: 15420,
        rating: 4.8,
        reviews: 342,
        tags: ['machine-learning', 'tensorflow', 'optimization', 'neural-networks'],
        installed: false,
        featured: true,
        price: 0,
        screenshots: [],
        repository: 'https://github.com/navlambda/tensorflow-plugin',
      },
      {
        id: 'gpu-accelerator',
        name: 'CUDA GPU Acceleration',
        author: 'Performance Labs',
        version: '2.0.1',
        description: 'Accelerate navigation computations with NVIDIA CUDA',
        longDescription: 'Unlock massive performance gains by offloading navigation field calculations to your NVIDIA GPU. Supports CUDA 12.0+ with automatic kernel optimization.',
        category: 'Tools',
        icon: '⚡',
        downloads: 8934,
        rating: 4.9,
        reviews: 178,
        tags: ['gpu', 'cuda', 'performance', 'acceleration'],
        installed: true,
        featured: true,
        price: 0,
        screenshots: [],
        homepage: 'https://gpu-accelerator.dev',
      },
      {
        id: '3d-visualizer-pro',
        name: '3D Visualizer Pro',
        author: 'VisualWorks',
        version: '3.1.4',
        description: 'Advanced 3D visualization with ray tracing and particles',
        longDescription: 'Professional-grade visualization with real-time ray tracing, particle systems, and cinematic camera controls. Export high-resolution renders and animations.',
        category: 'Visualization',
        icon: '🎨',
        downloads: 12567,
        rating: 4.7,
        reviews: 289,
        tags: ['visualization', '3d', 'rendering', 'ray-tracing'],
        installed: false,
        featured: true,
        price: 29.99,
        screenshots: [],
      },
      {
        id: 'aws-deployer',
        name: 'AWS Cloud Deployer',
        author: 'CloudOps Inc',
        version: '1.5.2',
        description: 'One-click deployment to AWS Lambda and EC2',
        longDescription: 'Deploy your navigation algorithms to AWS with a single click. Automatic Docker containerization, Lambda function creation, and EC2 fleet management.',
        category: 'Cloud & DevOps',
        icon: '☁️',
        downloads: 6234,
        rating: 4.6,
        reviews: 145,
        tags: ['aws', 'cloud', 'deployment', 'devops'],
        installed: false,
        featured: false,
        price: 0,
        screenshots: [],
      },
      {
        id: 'python-bridge',
        name: 'Python Bridge',
        author: 'NAVΛ Team',
        version: '1.0.8',
        description: 'Seamless Python interop and NumPy integration',
        longDescription: 'Call Python libraries directly from VNC code. Full NumPy, SciPy, and pandas integration with automatic type conversion.',
        category: 'Languages',
        icon: '🐍',
        downloads: 18723,
        rating: 4.9,
        reviews: 423,
        tags: ['python', 'numpy', 'scipy', 'interop'],
        installed: true,
        featured: false,
        price: 0,
        screenshots: [],
      },
      {
        id: 'dark-cosmos-theme',
        name: 'Dark Cosmos Theme',
        author: 'ThemeStudio',
        version: '2.3.0',
        description: 'Beautiful dark theme with cosmic colors',
        longDescription: 'A stunning dark theme inspired by deep space. High contrast, carefully chosen colors, and perfect for long coding sessions.',
        category: 'Themes',
        icon: '🌌',
        downloads: 23456,
        rating: 4.8,
        reviews: 567,
        tags: ['theme', 'dark', 'color-scheme'],
        installed: false,
        featured: false,
        price: 4.99,
        screenshots: [],
      },
      {
        id: 'quantum-simulator',
        name: 'Quantum State Navigator',
        author: 'QuantumLab',
        version: '0.9.5',
        description: 'Navigate quantum state spaces on Hilbert manifolds',
        longDescription: 'Simulate and visualize quantum navigation on Hilbert space manifolds. Includes Bloch sphere visualization, entanglement detection, and quantum circuit generation.',
        category: 'Education',
        icon: '⚛️',
        downloads: 4123,
        rating: 4.9,
        reviews: 89,
        tags: ['quantum', 'physics', 'simulation', 'education'],
        installed: false,
        featured: true,
        price: 0,
        screenshots: [],
      },
      {
        id: 'ros-integration',
        name: 'ROS Robotics Integration',
        author: 'RoboticsSoft',
        version: '1.4.1',
        description: 'Connect to ROS robots and simulate navigation',
        longDescription: 'Integrate with ROS (Robot Operating System) for real robot testing. Simulate navigation in Gazebo and deploy to physical robots.',
        category: 'Tools',
        icon: '🤖',
        downloads: 7856,
        rating: 4.7,
        reviews: 198,
        tags: ['robotics', 'ros', 'simulation', 'hardware'],
        installed: false,
        featured: true,
        price: 0,
        screenshots: [],
      },
    ];

    setPlugins(mockPlugins);
  };

  const filteredPlugins = plugins
    .filter((plugin) => {
      if (selectedCategory !== 'All' && plugin.category !== selectedCategory) {
        return false;
      }
      if (searchQuery) {
        const query = searchQuery.toLowerCase();
        return (
          plugin.name.toLowerCase().includes(query) ||
          plugin.description.toLowerCase().includes(query) ||
          plugin.tags.some((tag) => tag.includes(query))
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
          return 0; // TODO: Add timestamp
        default:
          return 0;
      }
    });

  const featuredPlugins = plugins.filter((p) => p.featured);

  const installPlugin = async (plugin: Plugin) => {
    // TODO: Implement actual plugin installation
    console.log('Installing plugin:', plugin.id);
    setPlugins(
      plugins.map((p) =>
        p.id === plugin.id ? { ...p, installed: true } : p
      )
    );
  };

  const uninstallPlugin = async (plugin: Plugin) => {
    // TODO: Implement actual plugin uninstallation
    console.log('Uninstalling plugin:', plugin.id);
    setPlugins(
      plugins.map((p) =>
        p.id === plugin.id ? { ...p, installed: false } : p
      )
    );
  };

  return (
    <div className="plugin-marketplace">
      <div className="marketplace-header">
        <div className="header-content">
          <h1>🔌 Plugin Marketplace</h1>
          <p>Extend NAVΛ Studio with powerful plugins and MCP servers</p>
        </div>
        <button className="publish-btn">
          📤 Publish Plugin
        </button>
      </div>

      {/* Tab Navigation */}
      <div className="marketplace-tabs">
        <button 
          className={`tab ${activeTab === 'plugins' ? 'active' : ''}`}
          onClick={() => setActiveTab('plugins')}
        >
          <Package size={16} />
          NAVΛ Plugins
        </button>
        <button 
          className={`tab ${activeTab === 'mcp' ? 'active' : ''}`}
          onClick={() => setActiveTab('mcp')}
        >
          <Server size={16} />
          MCP Catalog
        </button>
      </div>

      {/* Plugins Tab Content */}
      {activeTab === 'plugins' && (
        <>
          {/* Search and Filters */}
          <div className="marketplace-controls">
        <div className="search-box">
          <span className="search-icon">🔍</span>
          <input
            type="text"
            placeholder="Search plugins, tags, or authors..."
            value={searchQuery}
            onChange={(e) => setSearchQuery(e.target.value)}
            className="search-input"
          />
        </div>

        <div className="filter-controls">
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

      {/* Categories */}
      <div className="categories">
        {PLUGIN_CATEGORIES.map((category) => (
          <button
            key={category}
            className={`category-btn ${selectedCategory === category ? 'active' : ''}`}
            onClick={() => setSelectedCategory(category)}
          >
            {category}
          </button>
        ))}
      </div>

      {/* Featured Plugins */}
      {!searchQuery && selectedCategory === 'All' && (
        <div className="featured-section">
          <h2>⭐ Featured Plugins</h2>
          <div className="featured-grid">
            {featuredPlugins.map((plugin) => (
              <div key={plugin.id} className="featured-plugin-card">
                <div className="plugin-icon-large">{plugin.icon}</div>
                <div className="featured-plugin-info">
                  <h3>{plugin.name}</h3>
                  <p className="plugin-author">by {plugin.author}</p>
                  <p className="plugin-description">{plugin.description}</p>
                  <div className="plugin-stats">
                    <span>⭐ {plugin.rating}</span>
                    <span>📥 {plugin.downloads.toLocaleString()}</span>
                  </div>
                  {plugin.installed ? (
                    <button className="action-btn secondary" onClick={() => uninstallPlugin(plugin)}>
                      ✓ Installed
                    </button>
                  ) : (
                    <button className="action-btn primary" onClick={() => installPlugin(plugin)}>
                      {plugin.price > 0 ? `Buy $${plugin.price}` : 'Install'}
                    </button>
                  )}
                </div>
              </div>
            ))}
          </div>
        </div>
      )}

      {/* All Plugins */}
      <div className="plugins-section">
        <h2>
          {selectedCategory === 'All' ? 'All Plugins' : selectedCategory}
          <span className="count">({filteredPlugins.length})</span>
        </h2>
        <div className="plugins-grid">
          {filteredPlugins.map((plugin) => (
            <div
              key={plugin.id}
              className="plugin-card"
              onClick={() => setSelectedPlugin(plugin)}
            >
              <div className="plugin-card-header">
                <div className="plugin-icon">{plugin.icon}</div>
                <div className="plugin-header-info">
                  <h3>{plugin.name}</h3>
                  <p className="plugin-author">by {plugin.author}</p>
                </div>
                {plugin.installed && (
                  <span className="installed-badge">✓</span>
                )}
              </div>
              <p className="plugin-description">{plugin.description}</p>
              <div className="plugin-tags">
                {plugin.tags.slice(0, 3).map((tag) => (
                  <span key={tag} className="tag">
                    {tag}
                  </span>
                ))}
              </div>
              <div className="plugin-footer">
                <div className="plugin-stats">
                  <span>⭐ {plugin.rating}</span>
                  <span>📥 {(plugin.downloads / 1000).toFixed(1)}k</span>
                </div>
                <span className="plugin-version">v{plugin.version}</span>
              </div>
            </div>
          ))}
        </div>
      </div>

      {/* Plugin Detail Modal */}
      {selectedPlugin && (
        <div className="plugin-modal-overlay" onClick={() => setSelectedPlugin(null)}>
          <div className="plugin-modal" onClick={(e) => e.stopPropagation()}>
            <button className="modal-close-btn" onClick={() => setSelectedPlugin(null)}>
              ✕
            </button>
            
            <div className="modal-header">
              <div className="modal-icon">{selectedPlugin.icon}</div>
              <div className="modal-header-info">
                <h2>{selectedPlugin.name}</h2>
                <p className="modal-author">by {selectedPlugin.author}</p>
                <div className="modal-stats">
                  <span>⭐ {selectedPlugin.rating} ({selectedPlugin.reviews} reviews)</span>
                  <span>📥 {selectedPlugin.downloads.toLocaleString()} downloads</span>
                  <span>v{selectedPlugin.version}</span>
                </div>
              </div>
              <div className="modal-actions">
                {selectedPlugin.installed ? (
                  <button
                    className="action-btn secondary large"
                    onClick={() => uninstallPlugin(selectedPlugin)}
                  >
                    Uninstall
                  </button>
                ) : (
                  <button
                    className="action-btn primary large"
                    onClick={() => installPlugin(selectedPlugin)}
                  >
                    {selectedPlugin.price > 0 ? `Buy $${selectedPlugin.price}` : 'Install'}
                  </button>
                )}
              </div>
            </div>

            <div className="modal-content">
              <div className="modal-section">
                <h3>Description</h3>
                <p>{selectedPlugin.longDescription}</p>
              </div>

              <div className="modal-section">
                <h3>Tags</h3>
                <div className="plugin-tags">
                  {selectedPlugin.tags.map((tag) => (
                    <span key={tag} className="tag">
                      {tag}
                    </span>
                  ))}
                </div>
              </div>

              {(selectedPlugin.homepage || selectedPlugin.repository) && (
                <div className="modal-section">
                  <h3>Links</h3>
                  <div className="plugin-links">
                    {selectedPlugin.homepage && (
                      <a href={selectedPlugin.homepage} target="_blank" rel="noopener noreferrer">
                        🌐 Homepage
                      </a>
                    )}
                    {selectedPlugin.repository && (
                      <a href={selectedPlugin.repository} target="_blank" rel="noopener noreferrer">
                        📦 Repository
                      </a>
                    )}
                  </div>
                </div>
              )}
            </div>
          </div>
        </div>
      )}
        </>
      )}

      {/* MCP Catalog Tab Content */}
      {activeTab === 'mcp' && (
        <MCPCatalog />
      )}
    </div>
  );
};

