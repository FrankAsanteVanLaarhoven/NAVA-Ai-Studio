/**
 * NAVA Trash Bin Component
 * 
 * Apple-like trash bin with full CRUD functionalities:
 * - View deleted items
 * - Restore items
 * - Delete permanently
 * - Rename items
 * - Drag and drop support
 * - Empty trash
 */

import React, { useState, useEffect, useRef } from 'react';
import { 
  getTrashItems, 
  restoreFromTrash, 
  deletePermanently, 
  emptyTrash, 
  renameTrashItem,
  TrashItem 
} from '../../utils/trashStorage';
import { createApp, AppItem } from '../../utils/appStorage';
import './TrashBin.css';

interface TrashBinProps {
  onClose: () => void;
}

export const TrashBin: React.FC<TrashBinProps> = ({ onClose }) => {
  const [trashItems, setTrashItems] = useState<TrashItem[]>([]);
  const [selectedItems, setSelectedItems] = useState<Set<string>>(new Set());
  const [contextMenu, setContextMenu] = useState<{ x: number; y: number; item: TrashItem } | null>(null);
  const [renamingItem, setRenamingItem] = useState<string | null>(null);
  const [renameValue, setRenameValue] = useState('');
  const containerRef = useRef<HTMLDivElement>(null);

  useEffect(() => {
    loadTrashItems();
    
    // Listen for trash updates
    const handleTrashUpdate = () => {
      loadTrashItems();
    };
    window.addEventListener('nava:trash-updated', handleTrashUpdate);
    
    return () => {
      window.removeEventListener('nava:trash-updated', handleTrashUpdate);
    };
  }, []);

  useEffect(() => {
    const handleClickOutside = (e: MouseEvent) => {
      if (contextMenu && containerRef.current && !containerRef.current.contains(e.target as Node)) {
        setContextMenu(null);
      }
    };
    document.addEventListener('mousedown', handleClickOutside);
    return () => document.removeEventListener('mousedown', handleClickOutside);
  }, [contextMenu]);

  const loadTrashItems = () => {
    setTrashItems(getTrashItems());
  };

  const handleRestore = (item: TrashItem) => {
    if (item.type === 'app') {
      // Restore app to App Centre
      const restoredApp: Omit<AppItem, 'id' | 'createdAt' | 'updatedAt' | 'inDock'> = {
        name: item.name,
        icon: item.icon,
        color: item.color,
        description: item.description,
        route: item.route,
        category: item.category,
      };
      createApp(restoredApp);
    }
    
    restoreFromTrash(item.id);
    loadTrashItems();
    setSelectedItems(new Set());
    // Notify parent to update trash icon
    window.dispatchEvent(new CustomEvent('nava:trash-updated'));
  };

  const handleDeletePermanently = (item: TrashItem) => {
    if (confirm(`Are you sure you want to permanently delete "${item.name}"? This action cannot be undone.`)) {
      deletePermanently(item.id);
      loadTrashItems();
      setSelectedItems(new Set());
      // Notify parent to update trash icon
      window.dispatchEvent(new CustomEvent('nava:trash-updated'));
    }
  };

  const handleEmptyTrash = () => {
    const count = trashItems.length;
    if (count === 0) return;
    
    if (confirm(`Are you sure you want to permanently delete all ${count} item${count > 1 ? 's' : ''} in Trash? This action cannot be undone.`)) {
      emptyTrash();
      loadTrashItems();
      setSelectedItems(new Set());
      // Notify parent to update trash icon
      window.dispatchEvent(new CustomEvent('nava:trash-updated'));
    }
  };

  const handleRename = (item: TrashItem) => {
    setRenamingItem(item.id);
    setRenameValue(item.name);
  };

  const handleRenameSubmit = (itemId: string) => {
    if (renameValue.trim()) {
      renameTrashItem(itemId, renameValue.trim());
      loadTrashItems();
    }
    setRenamingItem(null);
    setRenameValue('');
  };

  const handleContextMenu = (e: React.MouseEvent, item: TrashItem) => {
    e.preventDefault();
    setContextMenu({ x: e.clientX, y: e.clientY, item });
  };

  const formatDate = (timestamp: number) => {
    const date = new Date(timestamp);
    const now = new Date();
    const diff = now.getTime() - date.getTime();
    const days = Math.floor(diff / (1000 * 60 * 60 * 24));
    
    if (days === 0) return 'Today';
    if (days === 1) return 'Yesterday';
    if (days < 7) return `${days} days ago`;
    return date.toLocaleDateString();
  };

  const handleDragOver = (e: React.DragEvent) => {
    e.preventDefault();
    e.dataTransfer.dropEffect = 'move';
  };

  const handleDrop = (e: React.DragEvent) => {
    e.preventDefault();
    try {
      const data = e.dataTransfer.getData('application/json');
      if (data) {
        const itemData = JSON.parse(data);
        if (itemData.type === 'app' && itemData.id) {
          // Import and add to trash
          import('../../utils/appStorage').then(({ getAllApps, deleteApp }) => {
            const apps = getAllApps();
            const app = apps.find(a => a.id === itemData.id);
            if (app) {
              import('../../utils/trashStorage').then(({ addToTrash }) => {
                addToTrash({
                  originalId: app.id,
                  name: app.name,
                  icon: app.icon,
                  color: app.color,
                  description: app.description,
                  route: app.route,
                  category: app.category,
                  type: 'app',
                  originalData: app,
                });
                deleteApp(app.id);
                loadTrashItems();
                // Notify parent to update trash icon
                window.dispatchEvent(new CustomEvent('nava:trash-updated'));
              });
            }
          });
        }
      }
    } catch (error) {
      console.error('[TrashBin] Error handling drop:', error);
    }
  };

  return (
    <div className="trash-bin-window" ref={containerRef}>
      <div className="trash-bin-header">
        <div className="trash-bin-title">
          <span className="trash-bin-icon">🗑️</span>
          <h2>Trash</h2>
          {trashItems.length > 0 && (
            <span className="trash-count">({trashItems.length})</span>
          )}
        </div>
        <div className="trash-bin-actions">
          {trashItems.length > 0 && (
            <button className="trash-empty-btn" onClick={handleEmptyTrash}>
              Empty Trash
            </button>
          )}
          <button className="trash-close-btn" onClick={onClose}>✕</button>
        </div>
      </div>

      <div 
        className="trash-bin-content"
        onDragOver={handleDragOver}
        onDrop={handleDrop}
      >
        {trashItems.length === 0 ? (
          <div className="trash-empty-state">
            <div className="trash-empty-icon">🗑️</div>
            <p className="trash-empty-text">Trash is empty</p>
            <p className="trash-empty-hint">Drag items here to delete them</p>
          </div>
        ) : (
          <div className="trash-items-grid">
            {trashItems.map((item) => (
              <div
                key={item.id}
                className={`trash-item ${selectedItems.has(item.id) ? 'selected' : ''}`}
                onClick={() => {
                  const newSelected = new Set(selectedItems);
                  if (newSelected.has(item.id)) {
                    newSelected.delete(item.id);
                  } else {
                    newSelected.add(item.id);
                  }
                  setSelectedItems(newSelected);
                }}
                onContextMenu={(e) => handleContextMenu(e, item)}
                draggable
                onDragStart={(e) => {
                  e.dataTransfer.setData('application/json', JSON.stringify({
                    type: 'trash-item',
                    id: item.id,
                    item,
                  }));
                }}
              >
                <div className="trash-item-icon">
                  <span style={{ fontSize: '48px' }}>{item.icon}</span>
                </div>
                <div className="trash-item-info">
                  {renamingItem === item.id ? (
                    <input
                      className="trash-item-rename-input"
                      value={renameValue}
                      onChange={(e) => setRenameValue(e.target.value)}
                      onBlur={() => handleRenameSubmit(item.id)}
                      onKeyDown={(e) => {
                        if (e.key === 'Enter') {
                          handleRenameSubmit(item.id);
                        } else if (e.key === 'Escape') {
                          setRenamingItem(null);
                          setRenameValue('');
                        }
                      }}
                      autoFocus
                      onClick={(e) => e.stopPropagation()}
                    />
                  ) : (
                    <div className="trash-item-name">{item.name}</div>
                  )}
                  <div className="trash-item-date">Deleted {formatDate(item.deletedAt)}</div>
                  {item.type && (
                    <div className="trash-item-type">{item.type.toUpperCase()}</div>
                  )}
                </div>
              </div>
            ))}
          </div>
        )}
      </div>

      {contextMenu && (
        <div
          className="trash-context-menu"
          style={{ left: contextMenu.x, top: contextMenu.y }}
          onClick={() => setContextMenu(null)}
        >
          <div className="context-menu-item" onClick={() => handleRestore(contextMenu.item)}>
            <span>↩️</span> Restore
          </div>
          <div className="context-menu-item" onClick={() => handleRename(contextMenu.item)}>
            <span>✏️</span> Rename
          </div>
          <div className="context-menu-divider" />
          <div className="context-menu-item danger" onClick={() => handleDeletePermanently(contextMenu.item)}>
            <span>🗑️</span> Delete Permanently
          </div>
        </div>
      )}
    </div>
  );
};

