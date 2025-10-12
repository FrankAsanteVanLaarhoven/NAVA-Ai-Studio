import React, { useState } from 'react';
import { Search, Download, Settings, Star, TrendingUp } from 'lucide-react';
import './ActivityPanels.css';

export const ExtensionsPanel: React.FC = () => {
  const [searchQuery, setSearchQuery] = useState('');

  const extensions = [
    { 
      name: 'NAVΛ Syntax Highlighter', 
      description: 'Advanced syntax highlighting for VNC', 
      author: 'NAVΛ Studio',
      downloads: '10.5K',
      rating: 4.9,
      installed: true
    },
    { 
      name: 'VNC IntelliSense', 
      description: 'Smart code completion for Navigation Calculus', 
      author: 'Van Laarhoven',
      downloads: '8.2K',
      rating: 4.8,
      installed: true
    },
    { 
      name: 'Navigation Debugger', 
      description: 'Visual debugging tools for navigation paths', 
      author: 'NAVΛ Team',
      downloads: '5.1K',
      rating: 4.7,
      installed: false
    },
    { 
      name: 'WebAssembly Preview', 
      description: 'Live preview for WASM compilation', 
      author: 'WASM Tools',
      downloads: '12.3K',
      rating: 4.9,
      installed: false
    },
  ];

  return (
    <div className="activity-panel extensions-panel">
      <div className="panel-header">
        <div className="panel-title">
          <Download size={16} />
          <span>Extensions</span>
        </div>
        <div className="panel-actions">
          <button className="icon-btn" title="Settings">
            <Settings size={16} />
          </button>
        </div>
      </div>

      <div className="panel-content">
        {/* Search */}
        <div className="search-box">
          <Search size={16} />
          <input
            type="text"
            placeholder="Search Extensions..."
            value={searchQuery}
            onChange={(e) => setSearchQuery(e.target.value)}
          />
        </div>

        {/* Categories */}
        <div className="extensions-categories">
          <button className="category-btn active">
            <TrendingUp size={14} />
            <span>Popular</span>
          </button>
          <button className="category-btn">
            <Star size={14} />
            <span>Recommended</span>
          </button>
          <button className="category-btn">
            <Download size={14} />
            <span>Installed</span>
          </button>
        </div>

        {/* Extensions List */}
        <div className="extensions-list">
          {extensions.map((ext, index) => (
            <div key={index} className="extension-item">
              <div className="extension-icon">
                <div className="icon-placeholder">{ext.name[0]}</div>
              </div>
              <div className="extension-info">
                <div className="extension-name">{ext.name}</div>
                <div className="extension-description">{ext.description}</div>
                <div className="extension-meta">
                  <span className="extension-author">{ext.author}</span>
                  <span className="extension-downloads">
                    <Download size={12} /> {ext.downloads}
                  </span>
                  <span className="extension-rating">
                    <Star size={12} fill="#ffa500" /> {ext.rating}
                  </span>
                </div>
              </div>
              <div className="extension-actions">
                {ext.installed ? (
                  <button className="extension-btn installed">Installed</button>
                ) : (
                  <button className="extension-btn install">Install</button>
                )}
              </div>
            </div>
          ))}
        </div>
      </div>
    </div>
  );
};

