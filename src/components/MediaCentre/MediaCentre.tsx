/**
 * NAVA Media Centre
 * 
 * Apple Control Center-like interface for managing all media applications
 * Full CRUD operations: Create, Read, Update, Delete
 * All media apps are active and wired up to their services
 */

import React, { useState, useRef, useEffect, useCallback } from 'react';
import {
  getAllMediaApps,
  createMediaApp,
  updateMediaApp,
  deleteMediaApp,
  renameMediaApp,
  toggleMediaApp,
  getMediaAppsByCategory,
  getMediaCategories,
  type MediaItem,
} from '../../utils/mediaStorage';
import './MediaCentre.css';

interface MediaCentreProps {
  onClose?: () => void;
}

export const MediaCentre: React.FC<MediaCentreProps> = ({ onClose }) => {
  const [apps, setApps] = useState<MediaItem[]>(getAllMediaApps());
  const [selectedApps, setSelectedApps] = useState<Set<string>>(new Set());
  const [selectedCategory, setSelectedCategory] = useState<string | null>(null);
  const [contextMenu, setContextMenu] = useState<{ x: number; y: number; appId: string } | null>(null);
  const [editingApp, setEditingApp] = useState<string | null>(null);
  const [editName, setEditName] = useState('');
  const [showNewAppDialog, setShowNewAppDialog] = useState(false);
  const [newApp, setNewApp] = useState({
    name: '',
    icon: '📱',
    color: '#3b82f6',
    description: '',
    category: '',
    route: '',
  });
  const editInputRef = useRef<HTMLInputElement>(null);
  const contextMenuRef = useRef<HTMLDivElement>(null);

  // Refresh apps from storage
  const refreshApps = useCallback(() => {
    setApps(getAllMediaApps());
  }, []);

  // Handle app click
  const handleAppClick = (app: MediaItem, e: React.MouseEvent) => {
    if (e.shiftKey || e.metaKey || e.ctrlKey) {
      // Multi-select
      const newSelected = new Set(selectedApps);
      if (newSelected.has(app.id)) {
        newSelected.delete(app.id);
      } else {
        newSelected.add(app.id);
      }
      setSelectedApps(newSelected);
    } else {
      // Single click - execute app
      if (app.onClick) {
        app.onClick();
      } else if (app.route) {
        if (app.route.startsWith('http')) {
          window.open(app.route, '_blank');
        } else {
          window.location.href = app.route;
        }
      }
      // Close media centre after action
      if (onClose) {
        setTimeout(() => onClose(), 300);
      }
    }
  };

  // Handle context menu
  const handleContextMenu = (e: React.MouseEvent, appId: string) => {
    e.preventDefault();
    e.stopPropagation();
    setContextMenu({ x: e.clientX, y: e.clientY, appId });
  };

  // Close context menu on outside click
  useEffect(() => {
    const handleClickOutside = (e: MouseEvent) => {
      if (contextMenuRef.current && !contextMenuRef.current.contains(e.target as Node)) {
        setContextMenu(null);
      }
    };
    document.addEventListener('mousedown', handleClickOutside);
    return () => document.removeEventListener('mousedown', handleClickOutside);
  }, []);

  // Start editing app name
  const startEditing = (app: MediaItem) => {
    setEditingApp(app.id);
    setEditName(app.name);
    setTimeout(() => {
      editInputRef.current?.focus();
      editInputRef.current?.select();
    }, 0);
  };

  // Save edited name
  const saveEdit = () => {
    if (editingApp && editName.trim()) {
      renameMediaApp(editingApp, editName.trim());
      refreshApps();
    }
    setEditingApp(null);
    setEditName('');
  };

  // Cancel editing
  const cancelEdit = () => {
    setEditingApp(null);
    setEditName('');
  };

  // Handle delete
  const handleDelete = (appId: string) => {
    if (confirm(`Are you sure you want to delete "${apps.find(a => a.id === appId)?.name}"?`)) {
      deleteMediaApp(appId);
      refreshApps();
      setSelectedApps(new Set());
    }
  };

  // Handle create new app
  const handleCreateApp = () => {
    if (!newApp.name.trim()) return;
    
    createMediaApp({
      name: newApp.name.trim(),
      icon: newApp.icon,
      color: newApp.color,
      description: newApp.description.trim(),
      category: newApp.category.trim() || undefined,
      route: newApp.route.trim() || null,
    });
    
    refreshApps();
    setShowNewAppDialog(false);
    setNewApp({
      name: '',
      icon: '📱',
      color: '#3b82f6',
      description: '',
      category: '',
      route: '',
    });
  };

  // Toggle enabled state
  const handleToggle = (appId: string) => {
    toggleMediaApp(appId);
    refreshApps();
  };

  // Get filtered apps
  const filteredApps = React.useMemo(() => {
    let filtered = selectedCategory 
      ? getMediaAppsByCategory(selectedCategory)
      : apps;
    
    return filtered.sort((a, b) => a.name.localeCompare(b.name));
  }, [apps, selectedCategory]);

  const categories = getMediaCategories();

  return (
    <div className="media-centre-overlay" onClick={onClose}>
      <div className="media-centre" onClick={(e) => e.stopPropagation()}>
        {/* Header */}
        <div className="media-centre-header">
          <div className="media-centre-title">
            <span className="media-centre-icon">🎛️</span>
            <h1>NAVA Media Centre</h1>
          </div>
          <button className="close-btn" onClick={onClose}>✕</button>
        </div>

        {/* Content */}
        <div className="media-centre-content">
          {/* Sidebar with categories */}
          <div className="media-centre-sidebar">
            <div className="sidebar-section">
              <h3>Categories</h3>
              <button
                className={`sidebar-item ${selectedCategory === null ? 'active' : ''}`}
                onClick={() => setSelectedCategory(null)}
              >
                All Media ({apps.length})
              </button>
              {categories.map(cat => {
                const count = getMediaAppsByCategory(cat).length;
                return (
                  <button
                    key={cat}
                    className={`sidebar-item ${selectedCategory === cat ? 'active' : ''}`}
                    onClick={() => setSelectedCategory(cat)}
                  >
                    {cat} ({count})
                  </button>
                );
              })}
            </div>
          </div>

          {/* Main content - Apple Control Center style grid */}
          <div className="media-centre-main">
            {filteredApps.length === 0 ? (
              <div className="media-centre-empty">
                <div className="empty-icon">🎛️</div>
                <h2>No media apps found</h2>
                <p>Create a new media app to get started</p>
                <button
                  className="primary-button"
                  onClick={() => setShowNewAppDialog(true)}
                >
                  + Create Media App
                </button>
              </div>
            ) : (
              <div className="media-centre-grid">
                {filteredApps.map(app => (
                  <div
                    key={app.id}
                    className={`media-item ${selectedApps.has(app.id) ? 'selected' : ''} ${!app.enabled ? 'disabled' : ''}`}
                    onClick={(e) => handleAppClick(app, e)}
                    onContextMenu={(e) => handleContextMenu(e, app.id)}
                    style={{ 
                      borderColor: app.color,
                      background: app.enabled 
                        ? `linear-gradient(135deg, rgba(255, 255, 255, 0.05) 0%, rgba(255, 255, 255, 0.02) 100%)`
                        : 'rgba(255, 255, 255, 0.02)'
                    }}
                  >
                    {editingApp === app.id ? (
                      <input
                        ref={editInputRef}
                        type="text"
                        value={editName}
                        onChange={(e) => setEditName(e.target.value)}
                        onBlur={saveEdit}
                        onKeyDown={(e) => {
                          if (e.key === 'Enter') saveEdit();
                          if (e.key === 'Escape') cancelEdit();
                        }}
                        className="media-edit-input"
                        onClick={(e) => e.stopPropagation()}
                      />
                    ) : (
                      <>
                        <div className="media-icon" style={{ color: app.color }}>
                          {app.icon}
                        </div>
                        <div className="media-name">{app.name}</div>
                        {!app.enabled && <div className="media-disabled-badge">OFF</div>}
                        <div className="media-toggle" onClick={(e) => { e.stopPropagation(); handleToggle(app.id); }}>
                          <div className={`toggle-switch ${app.enabled ? 'on' : 'off'}`}>
                            <div className="toggle-slider"></div>
                          </div>
                        </div>
                      </>
                    )}
                  </div>
                ))}
              </div>
            )}
          </div>
        </div>

        {/* Context Menu */}
        {contextMenu && (
          <div
            ref={contextMenuRef}
            className="media-context-menu"
            style={{ left: contextMenu.x, top: contextMenu.y }}
          >
            <div
              className="context-menu-item"
              onClick={() => {
                const app = apps.find(a => a.id === contextMenu.appId);
                if (app) handleAppClick(app, {} as React.MouseEvent);
                setContextMenu(null);
              }}
            >
              Open
            </div>
            <div
              className="context-menu-item"
              onClick={() => {
                startEditing(apps.find(a => a.id === contextMenu.appId)!);
                setContextMenu(null);
              }}
            >
              Rename
            </div>
            <div className="context-menu-divider" />
            <div
              className="context-menu-item"
              onClick={() => {
                handleToggle(contextMenu.appId);
                setContextMenu(null);
              }}
            >
              {apps.find(a => a.id === contextMenu.appId)?.enabled ? 'Disable' : 'Enable'}
            </div>
            <div className="context-menu-divider" />
            <div
              className="context-menu-item danger"
              onClick={() => {
                handleDelete(contextMenu.appId);
                setContextMenu(null);
              }}
            >
              Delete
            </div>
          </div>
        )}

        {/* New App Dialog */}
        {showNewAppDialog && (
          <div className="media-dialog-overlay" onClick={() => setShowNewAppDialog(false)}>
            <div className="media-dialog" onClick={(e) => e.stopPropagation()}>
              <h2>Create New Media App</h2>
              <div className="dialog-form">
                <label>
                  App Name *
                  <input
                    type="text"
                    value={newApp.name}
                    onChange={(e) => setNewApp({ ...newApp, name: e.target.value })}
                    placeholder="My Media App"
                    autoFocus
                  />
                </label>
                <label>
                  Icon
                  <input
                    type="text"
                    value={newApp.icon}
                    onChange={(e) => setNewApp({ ...newApp, icon: e.target.value })}
                    placeholder="📱"
                  />
                </label>
                <label>
                  Color
                  <input
                    type="color"
                    value={newApp.color}
                    onChange={(e) => setNewApp({ ...newApp, color: e.target.value })}
                  />
                </label>
                <label>
                  Description
                  <input
                    type="text"
                    value={newApp.description}
                    onChange={(e) => setNewApp({ ...newApp, description: e.target.value })}
                    placeholder="App description"
                  />
                </label>
                <label>
                  Category
                  <input
                    type="text"
                    value={newApp.category}
                    onChange={(e) => setNewApp({ ...newApp, category: e.target.value })}
                    placeholder="Recording, Streaming, etc."
                  />
                </label>
                <label>
                  Route/URL
                  <input
                    type="text"
                    value={newApp.route}
                    onChange={(e) => setNewApp({ ...newApp, route: e.target.value })}
                    placeholder="/app.html?activity=..."
                  />
                </label>
              </div>
              <div className="dialog-actions">
                <button onClick={() => setShowNewAppDialog(false)}>Cancel</button>
                <button
                  className="primary"
                  onClick={handleCreateApp}
                  disabled={!newApp.name.trim()}
                >
                  Create
                </button>
              </div>
            </div>
          </div>
        )}
      </div>
    </div>
  );
};

