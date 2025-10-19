import React, { useState } from 'react';
import './FileExplorer.css';

interface FileItem {
  name: string;
  type: 'file' | 'folder';
  size?: number;
  modified?: Date;
  children?: FileItem[];
  path: string;
}

interface FileExplorerProps {
  onClose?: () => void;
  onFileSelect?: (file: FileItem) => void;
}

// Mock file system data
const mockFileSystem: FileItem[] = [
  {
    name: 'src',
    type: 'folder',
    path: '/src',
    children: [
      {
        name: 'components',
        type: 'folder',
        path: '/src/components',
        children: [
          { name: 'App.tsx', type: 'file', size: 2048, modified: new Date('2025-01-13'), path: '/src/components/App.tsx' },
          { name: 'Header.tsx', type: 'file', size: 1024, modified: new Date('2025-01-12'), path: '/src/components/Header.tsx' },
          { name: 'Sidebar.tsx', type: 'file', size: 3072, modified: new Date('2025-01-11'), path: '/src/components/Sidebar.tsx' },
        ],
      },
      {
        name: 'services',
        type: 'folder',
        path: '/src/services',
        children: [
          { name: 'api.ts', type: 'file', size: 4096, modified: new Date('2025-01-10'), path: '/src/services/api.ts' },
          { name: 'auth.ts', type: 'file', size: 2560, modified: new Date('2025-01-09'), path: '/src/services/auth.ts' },
        ],
      },
      { name: 'main.tsx', type: 'file', size: 512, modified: new Date('2025-01-13'), path: '/src/main.tsx' },
      { name: 'App.css', type: 'file', size: 1536, modified: new Date('2025-01-12'), path: '/src/App.css' },
    ],
  },
  {
    name: 'public',
    type: 'folder',
    path: '/public',
    children: [
      { name: 'index.html', type: 'file', size: 1024, modified: new Date('2025-01-08'), path: '/public/index.html' },
      { name: 'favicon.ico', type: 'file', size: 256, modified: new Date('2025-01-07'), path: '/public/favicon.ico' },
    ],
  },
  { name: 'package.json', type: 'file', size: 2048, modified: new Date('2025-01-13'), path: '/package.json' },
  { name: 'README.md', type: 'file', size: 4096, modified: new Date('2025-01-12'), path: '/README.md' },
  { name: 'tsconfig.json', type: 'file', size: 1024, modified: new Date('2025-01-11'), path: '/tsconfig.json' },
];

export const FileExplorer: React.FC<FileExplorerProps> = ({ onClose, onFileSelect }) => {
  const [expandedFolders, setExpandedFolders] = useState<Set<string>>(new Set(['/src']));
  const [selectedFile, setSelectedFile] = useState<FileItem | null>(null);
  const [searchTerm, setSearchTerm] = useState('');
  const [viewMode, setViewMode] = useState<'tree' | 'list'>('tree');

  const toggleFolder = (path: string) => {
    const newExpanded = new Set(expandedFolders);
    if (newExpanded.has(path)) {
      newExpanded.delete(path);
    } else {
      newExpanded.add(path);
    }
    setExpandedFolders(newExpanded);
  };

  const handleFileClick = (file: FileItem) => {
    setSelectedFile(file);
    if (onFileSelect) {
      onFileSelect(file);
    }
  };

  const getFileIcon = (file: FileItem) => {
    if (file.type === 'folder') {
      return expandedFolders.has(file.path) ? '📂' : '📁';
    }

    const extension = file.name.split('.').pop()?.toLowerCase();
    switch (extension) {
      case 'tsx':
      case 'ts':
        return '📘';
      case 'js':
      case 'jsx':
        return '📙';
      case 'css':
        return '🎨';
      case 'html':
        return '🌐';
      case 'json':
        return '⚙️';
      case 'md':
        return '📝';
      case 'ico':
        return '🖼️';
      default:
        return '📄';
    }
  };

  const formatFileSize = (bytes: number) => {
    if (bytes < 1024) return `${bytes} B`;
    if (bytes < 1024 * 1024) return `${(bytes / 1024).toFixed(1)} KB`;
    return `${(bytes / (1024 * 1024)).toFixed(1)} MB`;
  };

  const formatDate = (date: Date) => {
    return date.toLocaleDateString('en-US', {
      month: 'short',
      day: 'numeric',
      year: 'numeric',
    });
  };

  const searchFiles = (items: FileItem[], term: string): FileItem[] => {
    const results: FileItem[] = [];

    for (const item of items) {
      if (item.name.toLowerCase().includes(term.toLowerCase())) {
        results.push(item);
      }

      if (item.children) {
        const childResults = searchFiles(item.children, term);
        results.push(...childResults);
      }
    }

    return results;
  };

  const renderTreeItem = (item: FileItem, depth = 0) => {
    const isExpanded = expandedFolders.has(item.path);
    const isSelected = selectedFile?.path === item.path;

    return (
      <div key={item.path}>
        <div
          className={`file-item ${isSelected ? 'selected' : ''}`}
          style={{ paddingLeft: `${depth * 20 + 12}px` }}
          onClick={() => {
            if (item.type === 'folder') {
              toggleFolder(item.path);
            } else {
              handleFileClick(item);
            }
          }}
        >
          <span className="file-icon">{getFileIcon(item)}</span>
          <span className="file-name">{item.name}</span>
          {item.type === 'file' && item.size && (
            <span className="file-size">{formatFileSize(item.size)}</span>
          )}
        </div>

        {item.type === 'folder' && isExpanded && item.children && (
          <div className="folder-children">
            {item.children.map(child => renderTreeItem(child, depth + 1))}
          </div>
        )}
      </div>
    );
  };

  const renderListItem = (item: FileItem) => (
    <div
      key={item.path}
      className={`file-list-item ${selectedFile?.path === item.path ? 'selected' : ''}`}
      onClick={() => handleFileClick(item)}
    >
      <span className="file-icon">{getFileIcon(item)}</span>
      <div className="file-details">
        <span className="file-name">{item.name}</span>
        <span className="file-path">{item.path}</span>
      </div>
      <div className="file-meta">
        {item.size && <span className="file-size">{formatFileSize(item.size)}</span>}
        {item.modified && <span className="file-date">{formatDate(item.modified)}</span>}
      </div>
    </div>
  );

  const filteredFiles = searchTerm ? searchFiles(mockFileSystem, searchTerm) : mockFileSystem;

  return (
    <div className="file-explorer-widget">
      <div className="file-explorer-header">
        <h3>📁 File Explorer</h3>
        <div className="file-explorer-controls">
          <button
            className={`view-mode-btn ${viewMode === 'tree' ? 'active' : ''}`}
            onClick={() => setViewMode('tree')}
            title="Tree View"
          >
            🌳
          </button>
          <button
            className={`view-mode-btn ${viewMode === 'list' ? 'active' : ''}`}
            onClick={() => setViewMode('list')}
            title="List View"
          >
            📋
          </button>
          {onClose && (
            <button className="file-explorer-close" onClick={onClose}>
              ×
            </button>
          )}
        </div>
      </div>

      <div className="file-explorer-search">
        <input
          type="text"
          placeholder="Search files..."
          value={searchTerm}
          onChange={(e) => setSearchTerm(e.target.value)}
          className="search-input"
        />
      </div>

      <div className="file-explorer-content">
        {viewMode === 'tree' ? (
          <div className="file-tree">
            {filteredFiles.map(item => renderTreeItem(item))}
          </div>
        ) : (
          <div className="file-list">
            {searchTerm ? (
              filteredFiles.map(item => renderListItem(item))
            ) : (
              mockFileSystem.flatMap(item => {
                const allFiles = [item];
                const collectFiles = (items: FileItem[]) => {
                  for (const child of items) {
                    allFiles.push(child);
                    if (child.children) {
                      collectFiles(child.children);
                    }
                  }
                };
                if (item.children) {
                  collectFiles(item.children);
                }
                return allFiles;
              }).map(item => renderListItem(item))
            )}
          </div>
        )}
      </div>

      {selectedFile && (
        <div className="file-explorer-footer">
          <div className="selected-file-info">
            <strong>{selectedFile.name}</strong>
            <span>{selectedFile.path}</span>
            {selectedFile.size && <span>{formatFileSize(selectedFile.size)}</span>}
            {selectedFile.modified && <span>{formatDate(selectedFile.modified)}</span>}
          </div>
        </div>
      )}
    </div>
  );
};