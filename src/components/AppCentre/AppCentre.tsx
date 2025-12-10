/**
 * NAVA App Centre
 * 
 * Apple Finder-like interface for managing all applications
 * Full CRUD operations: Create, Read, Update, Delete
 * Drag and drop to dock functionality
 * Context menu with "Keep in Dock" / "Remove from Dock"
 */

import React, { useState, useRef, useEffect, useCallback } from 'react';
import {
  getAllApps,
  createApp,
  updateApp,
  deleteApp,
  renameApp,
  addToDock,
  removeFromDock,
  getAppsByCategory,
  getCategories,
  type AppItem,
} from '../../utils/appStorage';
import './AppCentre.css';

interface AppCentreProps {
  onAppClick?: (app: AppItem) => void;
  onClose?: () => void;
}

export const AppCentre: React.FC<AppCentreProps> = ({ onAppClick, onClose }) => {
  const [apps, setApps] = useState<AppItem[]>(getAllApps());
  const [selectedApps, setSelectedApps] = useState<Set<string>>(new Set());
  const [viewMode, setViewMode] = useState<'grid' | 'list'>('grid');
  const [sortBy, setSortBy] = useState<'name' | 'date' | 'category'>('name');
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
    setApps(getAllApps());
  }, []);

  // Handle app click
  const handleAppClick = (app: AppItem, e: React.MouseEvent) => {
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
      // Single click - open app
      if (onAppClick) {
        onAppClick(app);
      } else if (app.onClick) {
        app.onClick();
      } else if (app.id === 'robotis-systemic') {
        // Special handling for ROBOTIS - navigate to workspace first, then trigger overlay
        if (!window.location.href.includes('activity=workspace')) {
          // Navigate to workspace, overlay will open after navigation
          window.location.href = '/app.html?activity=workspace';
          // Dispatch event after a short delay to ensure workspace is loaded
          setTimeout(() => {
            const event = new CustomEvent('nava:open-robotis-overlay');
            window.dispatchEvent(event);
          }, 500);
        } else {
          // Already on workspace, trigger overlay immediately
          const event = new CustomEvent('nava:open-robotis-overlay');
          window.dispatchEvent(event);
        }
      } else if (app.route) {
        if (app.route.startsWith('http')) {
          window.open(app.route, '_blank');
        } else {
          window.location.href = app.route;
        }
      } else {
        // Default: navigate to app.html with app ID
        window.location.href = `/app.html?activity=${app.id}`;
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

  // Handle drag start
  const handleDragStart = (e: React.DragEvent, app: AppItem) => {
    e.dataTransfer.setData('application/json', JSON.stringify({ type: 'app', id: app.id }));
    e.dataTransfer.effectAllowed = 'move';
  };

  // Start editing app name
  const startEditing = (app: AppItem) => {
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
      renameApp(editingApp, editName.trim());
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
      deleteApp(appId);
      refreshApps();
      setSelectedApps(new Set());
    }
  };

  // Handle create new app
  const handleCreateApp = () => {
    if (!newApp.name.trim()) return;
    
    createApp({
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

  // Toggle dock status
  const toggleDock = (appId: string) => {
    const app = apps.find(a => a.id === appId);
    if (!app) return;
    
    if (app.inDock) {
      removeFromDock(appId);
    } else {
      addToDock(appId);
    }
    refreshApps();
  };

  // Get filtered and sorted apps
  const filteredApps = React.useMemo(() => {
    let filtered = selectedCategory 
      ? getAppsByCategory(selectedCategory)
      : apps;
    
    // Sort
    filtered = [...filtered].sort((a, b) => {
      switch (sortBy) {
        case 'name':
          return a.name.localeCompare(b.name);
        case 'date':
          return b.updatedAt - a.updatedAt;
        case 'category':
          return (a.category || '').localeCompare(b.category || '');
        default:
          return 0;
      }
    });
    
    return filtered;
  }, [apps, selectedCategory, sortBy]);

  const categories = getCategories();

  return (
    <div className="app-centre">
      {/* Header */}
      <div className="app-centre-header">
        <div className="app-centre-title">
          <span className="app-centre-icon">📱</span>
          <h1>NAV<span className="lambda-in-text">Λ</span> App Centre</h1>
        </div>
        <div className="app-centre-toolbar">
          <button
            className="toolbar-button"
            onClick={() => setViewMode(viewMode === 'grid' ? 'list' : 'grid')}
            title={viewMode === 'grid' ? 'List View' : 'Grid View'}
          >
            {viewMode === 'grid' ? '☰' : '⊞'}
          </button>
          <select
            className="toolbar-select"
            value={sortBy}
            onChange={(e) => setSortBy(e.target.value as any)}
          >
            <option value="name">Sort by Name</option>
            <option value="date">Sort by Date</option>
            <option value="category">Sort by Category</option>
          </select>
          <button
            className="toolbar-button primary"
            onClick={() => setShowNewAppDialog(true)}
          >
            + New App
          </button>
        </div>
      </div>

      {/* Sidebar with categories */}
      <div className="app-centre-content">
        <div className="app-centre-sidebar">
          <div className="sidebar-section">
            <h3>Categories</h3>
            <button
              className={`sidebar-item ${selectedCategory === null ? 'active' : ''}`}
              onClick={() => setSelectedCategory(null)}
            >
              All Apps ({apps.length})
            </button>
            {categories.map(cat => {
              const count = getAppsByCategory(cat).length;
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

        {/* Main content area */}
        <div className="app-centre-main">
          {filteredApps.length === 0 ? (
            <div className="app-centre-empty">
              <div className="empty-icon">📱</div>
              <h2>No apps found</h2>
              <p>Create a new app to get started</p>
              <button
                className="primary-button"
                onClick={() => setShowNewAppDialog(true)}
              >
                + Create App
              </button>
            </div>
          ) : (
            <div className={`app-centre-view ${viewMode}`}>
              {filteredApps.map(app => (
                <div
                  key={app.id}
                  className={`app-item ${selectedApps.has(app.id) ? 'selected' : ''} ${app.inDock ? 'in-dock' : ''}`}
                  onClick={(e) => handleAppClick(app, e)}
                  onContextMenu={(e) => handleContextMenu(e, app.id)}
                  draggable
                  onDragStart={(e) => handleDragStart(e, app)}
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
                      className="app-edit-input"
                      onClick={(e) => e.stopPropagation()}
                    />
                  ) : (
                    <>
                      <div className="app-icon" style={{ color: app.color }}>
                        {app.icon}
                      </div>
                      <div className="app-name">{app.name}</div>
                      {app.inDock && <div className="app-dock-badge">📌</div>}
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
          className="app-context-menu"
          style={{ left: contextMenu.x, top: contextMenu.y }}
        >
          <div
            className="context-menu-item"
            onClick={() => {
              const app = apps.find(a => a.id === contextMenu.appId);
              if (app && onAppClick) onAppClick(app);
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
              toggleDock(contextMenu.appId);
              setContextMenu(null);
            }}
          >
            {apps.find(a => a.id === contextMenu.appId)?.inDock
              ? 'Remove from Dock'
              : 'Keep in Dock'}
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
        <div className="app-dialog-overlay" onClick={() => setShowNewAppDialog(false)}>
          <div className="app-dialog" onClick={(e) => e.stopPropagation()}>
            <h2>Create New App</h2>
            <div className="dialog-form">
              <label>
                App Name *
                <input
                  type="text"
                  value={newApp.name}
                  onChange={(e) => setNewApp({ ...newApp, name: e.target.value })}
                  placeholder="My App"
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
                  placeholder="Development, Robotics, etc."
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
  );
};

