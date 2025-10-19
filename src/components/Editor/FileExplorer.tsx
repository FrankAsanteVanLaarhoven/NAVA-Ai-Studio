import React, { useState, useEffect } from 'react';
import { RefreshCw, FilePlus, FolderPlus, Upload, Download, Trash2, MoreVertical } from 'lucide-react';
import { fileService, FileEntry } from '../../services/file-service';
import './FileExplorer.css';

interface FileExplorerProps {
  onFileSelect: (path: string, content: string) => void;
  currentFilePath?: string;
}

export const FileExplorer: React.FC<FileExplorerProps> = ({
  onFileSelect,
  currentFilePath,
}) => {
  const [files, setFiles] = useState<FileEntry[]>([]);
  const [expandedDirs, setExpandedDirs] = useState<Set<string>>(new Set());
  const [contextMenu, setContextMenu] = useState<{
    x: number;
    y: number;
    path: string;
    isDirectory: boolean;
  } | null>(null);

  useEffect(() => {
    loadCurrentProject();
  }, []);

  const loadCurrentProject = async () => {
    const project = fileService.getCurrentProject();
    if (project) {
      setFiles(project.files);
    }
  };

  const handleFileClick = async (file: FileEntry) => {
    if (file.isDirectory) {
      toggleDirectory(file.path);
    } else {
      try {
        const content = await fileService.readFile(file.path);
        onFileSelect(file.path, content);
      } catch (error) {
        console.error('Error reading file:', error);
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

  const renderFileTree = (fileList: FileEntry[], depth: number = 0) => {
    return fileList.map((file) => {
      const isExpanded = expandedDirs.has(file.path);
      const isSelected = currentFilePath === file.path;

      return (
        <div key={file.path} className="file-tree-item">
          <div
            className={`file-item ${isSelected ? 'selected' : ''}`}
            style={{ paddingLeft: `${depth * 20 + 8}px` }}
            onClick={() => handleFileClick(file)}
            onContextMenu={(e) => handleContextMenu(e, file)}
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

  // Handle drag and drop
  const handleDragOver = (e: React.DragEvent) => {
    e.preventDefault();
    e.stopPropagation();
  };

  const handleDrop = async (e: React.DragEvent) => {
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
      onDragOver={handleDragOver}
      onDrop={handleDrop}
    >
      <div className="file-explorer-header">
        <h3>Explorer</h3>
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
      <div className="file-tree">
        {files.length > 0 ? (
          renderFileTree(files)
        ) : (
          <div className="empty-state">
            <p>No project open</p>
            <button onClick={() => {/* Open project dialog */}}>
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
