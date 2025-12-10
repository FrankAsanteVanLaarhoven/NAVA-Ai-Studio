import React, { useState, useEffect } from 'react';
import { RefreshCw, FilePlus, FolderPlus, Upload, Download, Trash2, MoreVertical, Search, X } from 'lucide-react';
import { fileService, FileEntry } from '../../services/file-service';
import './FileExplorer.css';

interface FileExplorerProps {
  onFileSelect: (path: string, content: string) => void;
  currentFilePath?: string;
  onOpenProject?: () => void;
}

export const FileExplorer: React.FC<FileExplorerProps> = ({
  onFileSelect,
  currentFilePath,
  onOpenProject,
}) => {
  const [files, setFiles] = useState<FileEntry[]>([]);
  const [expandedDirs, setExpandedDirs] = useState<Set<string>>(new Set());
  const [contextMenu, setContextMenu] = useState<{
    x: number;
    y: number;
    path: string;
    isDirectory: boolean;
  } | null>(null);
  const [searchQuery, setSearchQuery] = useState('');
  const [draggedItem, setDraggedItem] = useState<FileEntry | null>(null);

  useEffect(() => {
    loadCurrentProject();
    
    // Listen for refresh events
    const handleRefresh = () => {
      loadCurrentProject();
    };
    
    // Also listen for storage changes (when files are saved in other tabs/windows)
    const handleStorageChange = (e: StorageEvent) => {
      if (e.key?.startsWith('file:') || e.key === 'navlambda-current-project') {
        loadCurrentProject();
      }
    };
    
    window.addEventListener('nava:refresh-explorer', handleRefresh);
    window.addEventListener('storage', handleStorageChange);
    
    // Periodic refresh to catch any missed updates
    const refreshInterval = setInterval(loadCurrentProject, 2000);
    
    return () => {
      window.removeEventListener('nava:refresh-explorer', handleRefresh);
      window.removeEventListener('storage', handleStorageChange);
      clearInterval(refreshInterval);
    };
  }, []);

  const loadCurrentProject = async () => {
    const project = fileService.getCurrentProject();
    if (project) {
      // Ensure all files are properly loaded
      const loadedFiles = await Promise.all(
        project.files.map(async (file) => {
          if (!file.isDirectory) {
            // Try to ensure file content is accessible
            try {
              await fileService.readFile(file.path);
            } catch {
              // File might not be stored yet, that's okay
            }
          }
          return file;
        })
      );
      setFiles(loadedFiles);
    }
  };

  const handleFileClick = async (file: FileEntry) => {
    if (file.isDirectory) {
      toggleDirectory(file.path);
    } else {
      try {
        console.log(`[FileExplorer] Opening file: ${file.path}`);
        let content = '';
        
        // First, try reading from file service
        try {
          content = await fileService.readFile(file.path);
          console.log(`[FileExplorer] ✓ Read file from service: ${file.path} (${content.length} bytes)`);
        } catch (readError) {
          console.warn(`[FileExplorer] File service read failed, trying alternatives...`);
          
          // Check if content is in the file entry (for newly imported files)
          if ((file as any).content !== undefined) {
            content = (file as any).content || '';
            console.log(`[FileExplorer] Found content in file entry: ${content.length} bytes`);
            
            // Store it for future access
            try {
              await fileService.createFile(file.path, content);
              console.log(`[FileExplorer] ✓ Stored file content for future access`);
            } catch (storeError) {
              console.error(`[FileExplorer] Failed to store file:`, storeError);
            }
          } else {
            // Try reading directly from localStorage with various key formats
            const normalizedPath = file.path.startsWith('/') ? file.path.substring(1) : file.path;
            const altKeys = [
              `file:${file.path}`,
              `file:${normalizedPath}`,
              `file:/${file.path}`,
              `file:/${normalizedPath}`,
            ];
            
            for (const key of altKeys) {
              const stored = localStorage.getItem(key);
              if (stored !== null) {
                content = stored;
                console.log(`[FileExplorer] ✓ Found file in localStorage with key: ${key}`);
                break;
              }
            }
            
            if (!content) {
              // Last resort: scan all localStorage keys
              for (let i = 0; i < localStorage.length; i++) {
                const key = localStorage.key(i);
                if (key && key.startsWith('file:')) {
                  const storedPath = key.substring(5);
                  const fileName = fileService.getFileName(storedPath);
                  if (fileName === file.name || storedPath.endsWith(file.path) || storedPath.endsWith(`/${file.name}`)) {
                    content = localStorage.getItem(key) || '';
                    console.log(`[FileExplorer] ✓ Found file by name match: ${key}`);
                    // Update the file path to match the stored key
                    (file as any).path = storedPath;
                    break;
                  }
                }
              }
            }
          }
        }
        
        if (content !== null && content !== undefined) {
          console.log(`[FileExplorer] ✓ File content ready: ${file.path} (${content.length} bytes)`);
          onFileSelect(file.path, content);
        } else {
          console.error(`[FileExplorer] ✗ File content is empty or null: ${file.path}`);
          alert(`Unable to read file: ${file.path}\n\nThe file may not have been properly imported.\n\nPlease try:\n1. Re-importing the file/folder\n2. Refreshing the explorer\n3. Checking the browser console for errors`);
        }
      } catch (error) {
        console.error('[FileExplorer] Error reading file:', error);
        alert(`Error reading file: ${file.path}\n\n${error instanceof Error ? error.message : 'Unknown error'}\n\nPlease check the browser console for details.`);
      }
    }
  };

  const toggleDirectory = (path: string) => {
    const newExpanded = new Set(expandedDirs);
    if (newExpanded.has(path)) {
      newExpanded.delete(path);
    } else {
      newExpanded.add(path);
    }
    setExpandedDirs(newExpanded);
  };

  const handleContextMenu = (
    e: React.MouseEvent,
    file: FileEntry
  ) => {
    e.preventDefault();
    setContextMenu({
      x: e.clientX,
      y: e.clientY,
      path: file.path,
      isDirectory: file.isDirectory,
    });
  };

  const handleNewFile = async () => {
    if (!contextMenu) return;

    const fileName = prompt('Enter file name:');
    if (!fileName) return;

    try {
      const newPath = contextMenu.isDirectory
        ? `${contextMenu.path}/${fileName}`
        : `${fileService.getDirectoryName(contextMenu.path)}/${fileName}`;

      await fileService.createFile(newPath, '');
      await loadCurrentProject();
      setContextMenu(null);
    } catch (error) {
      console.error('Error creating file:', error);
      alert('Failed to create file');
    }
  };

  const handleNewFolder = async () => {
    if (!contextMenu) return;

    const folderName = prompt('Enter folder name:');
    if (!folderName) return;

    try {
      const newPath = contextMenu.isDirectory
        ? `${contextMenu.path}/${folderName}`
        : `${fileService.getDirectoryName(contextMenu.path)}/${folderName}`;

      await fileService.createDirectory(newPath);
      await loadCurrentProject();
      setContextMenu(null);
    } catch (error) {
      console.error('Error creating folder:', error);
      alert('Failed to create folder');
    }
  };

  const handleDelete = async () => {
    if (!contextMenu) return;

    const confirmed = confirm(
      `Are you sure you want to delete ${fileService.getFileName(contextMenu.path)}?`
    );
    if (!confirmed) return;

    try {
      if (contextMenu.isDirectory) {
        await fileService.deleteDirectory(contextMenu.path);
      } else {
        await fileService.deleteFile(contextMenu.path);
      }
      await loadCurrentProject();
      setContextMenu(null);
    } catch (error) {
      console.error('Error deleting:', error);
      alert('Failed to delete');
    }
  };

  const handleRename = async () => {
    if (!contextMenu) return;

    const currentName = fileService.getFileName(contextMenu.path);
    const newName = prompt('Enter new name:', currentName);
    if (!newName || newName === currentName) return;

    try {
      const dirPath = fileService.getDirectoryName(contextMenu.path);
      const newPath = `${dirPath}/${newName}`;

      // Read content, create new file, delete old file
      if (!contextMenu.isDirectory) {
        const content = await fileService.readFile(contextMenu.path);
        await fileService.createFile(newPath, content);
        await fileService.deleteFile(contextMenu.path);
      }

      await loadCurrentProject();
      setContextMenu(null);
    } catch (error) {
      console.error('Error renaming:', error);
      alert('Failed to rename');
    }
  };

  const filterFiles = (fileList: FileEntry[]): FileEntry[] => {
    if (!searchQuery.trim()) return fileList;
    
    const query = searchQuery.toLowerCase();
    const filtered: FileEntry[] = [];
    
    fileList.forEach(file => {
      const matches = file.name.toLowerCase().includes(query);
      let hasMatchingChildren = false;
      
      if (file.isDirectory && file.children) {
        const filteredChildren = filterFiles(file.children);
        if (filteredChildren.length > 0) {
          hasMatchingChildren = true;
          filtered.push({
            ...file,
            children: filteredChildren,
          });
        }
      }
      
      if (matches && !hasMatchingChildren) {
        filtered.push(file);
      }
    });
    
    return filtered;
  };

  const handleDragStart = (e: React.DragEvent, file: FileEntry) => {
    setDraggedItem(file);
    e.dataTransfer.effectAllowed = 'move';
  };

  const handleDragOver = (e: React.DragEvent) => {
    e.preventDefault();
    e.stopPropagation();
    e.dataTransfer.dropEffect = 'move';
  };

  const handleDrop = async (e: React.DragEvent, targetFile?: FileEntry) => {
    e.preventDefault();
    e.stopPropagation();
    
    if (!draggedItem) return;
    
    const targetPath = targetFile?.isDirectory 
      ? `${targetFile.path}/${draggedItem.name}`
      : targetFile 
        ? `${fileService.getDirectoryName(targetFile.path)}/${draggedItem.name}`
        : `/project/${draggedItem.name}`;
    
    try {
      // Move file/folder
      if (draggedItem.isDirectory) {
        // For directories, we'd need to recursively move all files
        // This is a simplified version
        await fileService.createDirectory(targetPath);
      } else {
        const content = await fileService.readFile(draggedItem.path);
        await fileService.createFile(targetPath, content);
        await fileService.deleteFile(draggedItem.path);
      }
      
      await loadCurrentProject();
      setDraggedItem(null);
    } catch (error) {
      console.error('Error moving file:', error);
      alert('Failed to move file');
      setDraggedItem(null);
    }
  };

  const renderFileTree = (fileList: FileEntry[], depth: number = 0) => {
    const filtered = filterFiles(fileList);
    return filtered.map((file) => {
      const isExpanded = expandedDirs.has(file.path);
      const isSelected = currentFilePath === file.path;

      return (
        <div key={file.path} className="file-tree-item">
          <div
            className={`file-item ${isSelected ? 'selected' : ''} ${draggedItem?.path === file.path ? 'dragging' : ''}`}
            style={{ paddingLeft: `${depth * 20 + 8}px` }}
            onClick={() => handleFileClick(file)}
            onContextMenu={(e) => handleContextMenu(e, file)}
            draggable
            onDragStart={(e) => handleDragStart(e, file)}
            onDragOver={handleDragOver}
            onDrop={(e) => handleDrop(e, file)}
          >
            {file.isDirectory && (
              <span className="folder-icon">
                {isExpanded ? '📂' : '📁'}
              </span>
            )}
            {!file.isDirectory && (
              <span className="file-icon">
                {getFileIcon(file.name)}
              </span>
            )}
            <span className="file-name">{file.name}</span>
          </div>
          {file.isDirectory && isExpanded && file.children && (
            <div className="file-tree-children">
              {renderFileTree(file.children, depth + 1)}
            </div>
          )}
        </div>
      );
    });
  };

  const getFileIcon = (fileName: string): string => {
    const ext = fileService.getFileExtension(fileName);
    switch (ext) {
      case 'vnc':
      case 'navλ':
      case 'nav':
        return '⋋';
      case 'js':
      case 'ts':
        return '📜';
      case 'json':
        return '📋';
      case 'md':
        return '📝';
      default:
        return '📄';
    }
  };

  // Handle external file drag and drop
  const handleExternalDragOver = (e: React.DragEvent) => {
    e.preventDefault();
    e.stopPropagation();
  };

  const handleExternalDrop = async (e: React.DragEvent) => {
    e.preventDefault();
    e.stopPropagation();

    const droppedFiles = Array.from(e.dataTransfer.files);
    
    for (const file of droppedFiles) {
      try {
        const content = await file.text();
        const fileName = file.name;
        await fileService.createFile(`/project/${fileName}`, content);
        console.log(`✅ Imported: ${fileName}`);
      } catch (error) {
        console.error(`Failed to import ${file.name}:`, error);
      }
    }
    
    await loadCurrentProject();
    alert(`✅ Imported ${droppedFiles.length} file(s)!`);
  };

  const handleRefresh = async () => {
    await loadCurrentProject();
  };

  const handleNewFileClick = () => {
    const fileName = prompt('Enter file name:');
    if (!fileName) return;
    
    fileService.createFile(`/project/${fileName}`, '').then(() => {
      loadCurrentProject();
    }).catch(error => {
      console.error('Error creating file:', error);
      alert('Failed to create file');
    });
  };

  const handleNewFolderClick = () => {
    const folderName = prompt('Enter folder name:');
    if (!folderName) return;
    
    fileService.createDirectory(`/project/${folderName}`).then(() => {
      loadCurrentProject();
    }).catch(error => {
      console.error('Error creating folder:', error);
      alert('Failed to create folder');
    });
  };

  return (
    <div 
      className="file-explorer"
      onDragOver={handleExternalDragOver}
      onDrop={handleExternalDrop}
    >
      <div className="file-explorer-header">
        <div 
          className="file-explorer-logo-title"
          onClick={() => {
            // Navigate back to desktop/workspace
            const url = new URL(window.location.href);
            url.pathname = '/workspace.html';
            url.search = '';
            window.location.href = url.toString();
          }}
          title="Back to Desktop"
        >
          <span className="file-explorer-logo">λ</span>
          <span className="file-explorer-logo-capital">Λ</span>
          <span className="file-explorer-title">NAVΛ STUDIO IDE</span>
        </div>
        <div className="file-explorer-actions">
          <button
            className="explorer-toolbar-btn"
            onClick={handleRefresh}
            title="Refresh Explorer"
          >
            <RefreshCw size={14} />
          </button>
          <button
            className="explorer-toolbar-btn"
            onClick={handleNewFileClick}
            title="New File (Ctrl+N)"
          >
            <FilePlus size={14} />
          </button>
          <button
            className="explorer-toolbar-btn"
            onClick={handleNewFolderClick}
            title="New Folder (Ctrl+Shift+N)"
          >
            <FolderPlus size={14} />
          </button>
          <button
            className="explorer-toolbar-btn"
            onClick={() => {}}
            title="More Actions"
          >
            <MoreVertical size={14} />
          </button>
        </div>
      </div>
      <div className="file-explorer-search">
        <Search size={14} className="search-icon" />
        <input
          type="text"
          placeholder="Search files..."
          value={searchQuery}
          onChange={(e) => setSearchQuery(e.target.value)}
          className="file-explorer-search-input"
        />
        {searchQuery && (
          <button
            className="file-explorer-search-clear"
            onClick={() => setSearchQuery('')}
            title="Clear search"
          >
            <X size={12} />
          </button>
        )}
      </div>
      <div className="file-tree">
        {files.length > 0 ? (
          renderFileTree(files)
        ) : (
          <div className="empty-state">
            <p>No project open</p>
            <button 
              onClick={() => {
                if (onOpenProject) {
                  onOpenProject();
                } else {
                  // Fallback: try to open project via file service
                  const path = prompt('Enter project path or folder:');
                  if (path) {
                    fileService.openProject(path).then(project => {
                      if (project) {
                        setFiles(project.files);
                        fileService.setCurrentProject(project);
                      } else {
                        alert('Failed to open project. Please check the path.');
                      }
                    }).catch(error => {
                      console.error('Error opening project:', error);
                      alert('Failed to open project');
                    });
                  }
                }
              }}
              className="open-project-btn"
            >
              Open Project
            </button>
          </div>
        )}
      </div>
      {contextMenu && (
        <>
          <div
            className="context-menu-overlay"
            onClick={() => setContextMenu(null)}
          />
          <div
            className="context-menu"
            style={{ left: contextMenu.x, top: contextMenu.y }}
          >
            <div className="context-menu-item" onClick={handleNewFile}>
              New File
            </div>
            <div className="context-menu-item" onClick={handleNewFolder}>
              New Folder
            </div>
            <div className="context-menu-divider" />
            <div className="context-menu-item" onClick={handleRename}>
              Rename
            </div>
            <div className="context-menu-item" onClick={handleDelete}>
              Delete
            </div>
          </div>
        </>
      )}
    </div>
  );
};
