import React, { useState, useEffect } from 'react';
import { Search, Download, Settings, Star, TrendingUp, Check, X, Loader, Package } from 'lucide-react';
import { extensionService, Extension } from '../../services/extension-service';
import './ActivityPanels.css';

type CategoryFilter = 'all' | 'popular' | 'recommended' | 'installed';

export const ExtensionsPanel: React.FC = () => {
  const [searchQuery, setSearchQuery] = useState('');
  const [extensions, setExtensions] = useState<Extension[]>([]);
  const [filteredExtensions, setFilteredExtensions] = useState<Extension[]>([]);
  const [categoryFilter, setCategoryFilter] = useState<CategoryFilter>('popular');
  const [installingExtensionId, setInstallingExtensionId] = useState<string | null>(null);
  const [message, setMessage] = useState<{ text: string; type: 'success' | 'error' | 'info' } | null>(null);

  useEffect(() => {
    loadExtensions();
  }, []);

  useEffect(() => {
    filterExtensions();
  }, [extensions, searchQuery, categoryFilter]);

  const loadExtensions = () => {
    const allExtensions = extensionService.getAllExtensions();
    setExtensions(allExtensions);
  };

  const filterExtensions = () => {
    let filtered = extensions;

    // Apply search filter
    if (searchQuery.trim()) {
      filtered = extensionService.searchExtensions(searchQuery);
    }

    // Apply category filter
    switch (categoryFilter) {
      case 'popular':
        filtered = filtered.sort((a, b) => {
          const aDownloads = parseFloat(a.downloads.replace('K', '')) * 1000;
          const bDownloads = parseFloat(b.downloads.replace('K', '')) * 1000;
          return bDownloads - aDownloads;
        });
        break;
      case 'recommended':
        filtered = filtered.sort((a, b) => b.rating - a.rating);
        break;
      case 'installed':
        filtered = extensionService.getInstalledExtensions();
        break;
      case 'all':
      default:
        break;
    }

    setFilteredExtensions(filtered);
  };

  const handleInstall = async (extension: Extension) => {
    setInstallingExtensionId(extension.id);
    showMessage(`Installing ${extension.displayName}...`, 'info');

    const result = await extensionService.install(extension.id);
    
    setInstallingExtensionId(null);
    showMessage(result.message, result.success ? 'success' : 'error');
    
    if (result.success) {
      loadExtensions();
    }
  };

  const handleUninstall = async (extension: Extension) => {
    if (!confirm(`Are you sure you want to uninstall ${extension.displayName}?`)) {
      return;
    }

    setInstallingExtensionId(extension.id);
    showMessage(`Uninstalling ${extension.displayName}...`, 'info');

    const result = await extensionService.uninstall(extension.id);
    
    setInstallingExtensionId(null);
    showMessage(result.message, result.success ? 'success' : 'error');
    
    if (result.success) {
      loadExtensions();
    }
  };

  const handleToggle = async (extension: Extension) => {
    setInstallingExtensionId(extension.id);

    const result = await extensionService.toggleExtension(extension.id);
    
    setInstallingExtensionId(null);
    showMessage(result.message, result.success ? 'success' : 'error');
    
    if (result.success) {
      loadExtensions();
    }
  };

  const showMessage = (text: string, type: 'success' | 'error' | 'info') => {
    setMessage({ text, type });
    setTimeout(() => setMessage(null), 3000);
  };

  const getIconInitial = (name: string): string => {
    const words = name.split(' ');
    if (words.length >= 2) {
      return words[0][0] + words[1][0];
    }
    return name[0];
  };

  return (
    <div className="activity-panel extensions-panel">
      <div className="panel-header">
        <div className="panel-title">
          <Package size={16} />
          <span>EXTENSIONS</span>
        </div>
        <div className="panel-actions">
          <button className="icon-btn" title="Reload Extensions" onClick={loadExtensions}>
            <Settings size={16} />
          </button>
        </div>
      </div>

      <div className="panel-content">
        {/* Message Toast */}
        {message && (
          <div className={`extension-message extension-message-${message.type}`}>
            {message.type === 'success' && <Check size={16} />}
            {message.type === 'error' && <X size={16} />}
            {message.type === 'info' && <Loader size={16} className="spinning" />}
            <span>{message.text}</span>
          </div>
        )}

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
          <button 
            className={`category-btn ${categoryFilter === 'popular' ? 'active' : ''}`}
            onClick={() => setCategoryFilter('popular')}
          >
            <TrendingUp size={14} />
            <span>Popular</span>
          </button>
          <button 
            className={`category-btn ${categoryFilter === 'recommended' ? 'active' : ''}`}
            onClick={() => setCategoryFilter('recommended')}
          >
            <Star size={14} />
            <span>Recommended</span>
          </button>
          <button 
            className={`category-btn ${categoryFilter === 'installed' ? 'active' : ''}`}
            onClick={() => setCategoryFilter('installed')}
          >
            <Download size={14} />
            <span>Installed</span>
          </button>
        </div>

        {/* Extensions List */}
        <div className="extensions-list">
          {filteredExtensions.length === 0 ? (
            <div className="no-extensions">
              <Package size={48} opacity={0.3} />
              <p>No extensions found</p>
              {searchQuery && <p className="hint">Try a different search term</p>}
            </div>
          ) : (
            filteredExtensions.map((ext) => (
              <div key={ext.id} className={`extension-item ${!ext.enabled ? 'disabled' : ''}`}>
                <div className="extension-icon">
                  <div className="icon-placeholder" style={{
                    background: ext.installed ? '#00ff00' : '#3b82f6'
                  }}>
                    {getIconInitial(ext.displayName)}
                  </div>
                </div>
                <div className="extension-info">
                  <div className="extension-name">
                    {ext.displayName}
                    {ext.installed && !ext.enabled && (
                      <span className="extension-badge disabled-badge">Disabled</span>
                    )}
                  </div>
                  <div className="extension-description">{ext.description}</div>
                  <div className="extension-meta">
                    <span className="extension-author">{ext.author}</span>
                    <span className="extension-downloads">
                      <Download size={12} /> {ext.downloads}
                    </span>
                    <span className="extension-rating">
                      <Star size={12} fill="#ffa500" /> {ext.rating}
                    </span>
                    <span className="extension-version">v{ext.version}</span>
                  </div>
                </div>
                <div className="extension-actions">
                  {installingExtensionId === ext.id ? (
                    <button className="extension-btn loading" disabled>
                      <Loader size={14} className="spinning" />
                      <span>Loading...</span>
                    </button>
                  ) : ext.installed ? (
                    <div className="installed-actions">
                      <button 
                        className={`extension-btn toggle ${ext.enabled ? 'enabled' : 'disabled'}`}
                        onClick={() => handleToggle(ext)}
                        title={ext.enabled ? 'Disable' : 'Enable'}
                      >
                        {ext.enabled ? <Check size={14} /> : <X size={14} />}
                      </button>
                      <button 
                        className="extension-btn uninstall"
                        onClick={() => handleUninstall(ext)}
                        title="Uninstall"
                      >
                        Uninstall
                      </button>
                    </div>
                  ) : (
                    <button 
                      className="extension-btn install"
                      onClick={() => handleInstall(ext)}
                    >
                      Install
                    </button>
                  )}
                </div>
              </div>
            ))
          )}
        </div>
      </div>
    </div>
  );
};
