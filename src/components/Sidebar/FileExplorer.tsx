import React, { useState } from 'react';
import { ChevronRight, ChevronDown, File, Folder, FolderOpen, RefreshCw, FilePlus, FolderPlus, MoreVertical } from 'lucide-react';
import './FileExplorer.css';

interface FileNode {
  name: string;
  type: 'file' | 'folder';
  path: string;
  children?: FileNode[];
  expanded?: boolean;
}

const PROJECT_STRUCTURE: FileNode = {
  name: 'NAVΛ STUDIO IDE',
  type: 'folder',
  path: '/',
  expanded: true,
  children: [
    {
      name: 'assets',
      type: 'folder',
      path: '/assets',
      expanded: true,
      children: [
        {
          name: 'examples',
          type: 'folder',
          path: '/assets/examples',
          expanded: true,
          children: [
            { name: 'basic-navigation.vnc', type: 'file', path: '/assets/examples/basic-navigation.vnc' },
            { name: 'consciousness-integration.vnc', type: 'file', path: '/assets/examples/consciousness-integration.vnc' },
            { name: 'energy-landscape.vnc', type: 'file', path: '/assets/examples/energy-landscape.vnc' },
            { name: 'multi-goal-navigation.vnc', type: 'file', path: '/assets/examples/multi-goal-navigation.vnc' },
            { name: 'quantum-navigation.vnc', type: 'file', path: '/assets/examples/quantum-navigation.vnc' },
            { name: 'real-world-robotics.vnc', type: 'file', path: '/assets/examples/real-world-robotics.vnc' },
          ],
        },
        { name: 'fonts', type: 'folder', path: '/assets/fonts', children: [] },
        { name: 'icons', type: 'folder', path: '/assets/icons', children: [] },
        { name: 'themes', type: 'folder', path: '/assets/themes', children: [] },
      ],
    },
    {
      name: 'docs',
      type: 'folder',
      path: '/docs',
      expanded: true,
      children: [
        { name: 'architecture.md', type: 'file', path: '/docs/architecture.md' },
        { name: 'compilation-targets.md', type: 'file', path: '/docs/compilation-targets.md' },
        { name: 'deployment-guide.md', type: 'file', path: '/docs/deployment-guide.md' },
        { name: 'plugin-development.md', type: 'file', path: '/docs/plugin-development.md' },
        { name: 'vnc-language-reference.md', type: 'file', path: '/docs/vnc-language-reference.md' },
      ],
    },
    { name: 'src', type: 'folder', path: '/src', children: [] },
    { name: 'src-tauri', type: 'folder', path: '/src-tauri', children: [] },
  ],
};

export const FileExplorer: React.FC = () => {
  const [tree, setTree] = useState(PROJECT_STRUCTURE);
  const [selectedPath, setSelectedPath] = useState<string>('/');
  const [showMoreMenu, setShowMoreMenu] = useState(false);

  const toggleFolder = (path: string) => {
    const updateTree = (node: FileNode): FileNode => {
      if (node.path === path) {
        return { ...node, expanded: !node.expanded };
      }
      if (node.children) {
        return { ...node, children: node.children.map(updateTree) };
      }
      return node;
    };
    setTree(updateTree(tree));
  };

  const addFileToTree = (parentPath: string, fileName: string) => {
    const updateTree = (node: FileNode): FileNode => {
      if (node.path === parentPath && node.type === 'folder') {
        const newFile: FileNode = {
          name: fileName,
          type: 'file',
          path: `${parentPath}/${fileName}`.replace('//', '/'),
        };
        return {
          ...node,
          expanded: true,
          children: [...(node.children || []), newFile].sort((a, b) => {
            if (a.type === b.type) return a.name.localeCompare(b.name);
            return a.type === 'folder' ? -1 : 1;
          }),
        };
      }
      if (node.children) {
        return { ...node, children: node.children.map(updateTree) };
      }
      return node;
    };
    setTree(updateTree(tree));
  };

  const addFolderToTree = (parentPath: string, folderName: string) => {
    const updateTree = (node: FileNode): FileNode => {
      if (node.path === parentPath && node.type === 'folder') {
        const newFolder: FileNode = {
          name: folderName,
          type: 'folder',
          path: `${parentPath}/${folderName}`.replace('//', '/'),
          expanded: false,
          children: [],
        };
        return {
          ...node,
          expanded: true,
          children: [...(node.children || []), newFolder].sort((a, b) => {
            if (a.type === b.type) return a.name.localeCompare(b.name);
            return a.type === 'folder' ? -1 : 1;
          }),
        };
      }
      if (node.children) {
        return { ...node, children: node.children.map(updateTree) };
      }
      return node;
    };
    setTree(updateTree(tree));
  };

  const renderNode = (node: FileNode, depth: number = 0) => {
    const isFolder = node.type === 'folder';
    const isSelected = node.path === selectedPath;

    return (
      <div key={node.path}>
        <div
          className={`file-node ${isSelected ? 'selected' : ''}`}
          style={{ paddingLeft: `${depth * 16 + 8}px` }}
          onClick={() => {
            setSelectedPath(node.path);
            if (isFolder) toggleFolder(node.path);
          }}
        >
          {isFolder ? (
            <>
              {node.expanded ? <ChevronDown size={16} /> : <ChevronRight size={16} />}
              {node.expanded ? <FolderOpen size={16} /> : <Folder size={16} />}
            </>
          ) : (
            <>
              <span style={{ width: 16 }} />
              <File size={16} />
            </>
          )}
          <span className="file-name">{node.name}</span>
        </div>
        {isFolder && node.expanded && node.children && (
          <div className="folder-children">
            {node.children.map((child) => renderNode(child, depth + 1))}
          </div>
        )}
      </div>
    );
  };

  const findNodeByPath = (path: string, node: FileNode = tree): FileNode | null => {
    if (node.path === path) return node;
    if (node.children) {
      for (const child of node.children) {
        const found = findNodeByPath(path, child);
        if (found) return found;
      }
    }
    return null;
  };

  const getParentPath = (path: string): string => {
    const parts = path.split('/').filter(Boolean);
    parts.pop();
    return parts.length > 0 ? '/' + parts.join('/') : '/';
  };

  const handleRefresh = () => {
    setTree({ ...PROJECT_STRUCTURE });
  };

  const handleNewFile = () => {
    const fileName = prompt('Enter file name (e.g., mycode.vnc):');
    if (!fileName || fileName.trim() === '') return;

    const selectedNode = findNodeByPath(selectedPath);
    let targetPath = selectedPath;

    // If selected node is a file, add to its parent folder
    if (selectedNode && selectedNode.type === 'file') {
      targetPath = getParentPath(selectedPath);
    }

    addFileToTree(targetPath, fileName.trim());
  };

  const handleNewFolder = () => {
    const folderName = prompt('Enter folder name:');
    if (!folderName || folderName.trim() === '') return;

    const selectedNode = findNodeByPath(selectedPath);
    let targetPath = selectedPath;

    // If selected node is a file, add to its parent folder
    if (selectedNode && selectedNode.type === 'file') {
      targetPath = getParentPath(selectedPath);
    }

    addFolderToTree(targetPath, folderName.trim());
  };

  const handleMoreActions = () => {
    setShowMoreMenu(!showMoreMenu);
  };

  const handleCollapseAll = () => {
    const collapseTree = (node: FileNode): FileNode => {
      if (node.type === 'folder') {
        return {
          ...node,
          expanded: false,
          children: node.children?.map(collapseTree),
        };
      }
      return node;
    };
    setTree(collapseTree(tree));
    setShowMoreMenu(false);
  };

  const handleExpandAll = () => {
    const expandTree = (node: FileNode): FileNode => {
      if (node.type === 'folder') {
        return {
          ...node,
          expanded: true,
          children: node.children?.map(expandTree),
        };
      }
      return node;
    };
    setTree(expandTree(tree));
    setShowMoreMenu(false);
  };

  return (
    <div className="file-explorer">
      <div className="explorer-header">
        <h3>EXPLORER</h3>
        <div className="explorer-toolbar">
          <button 
            className="explorer-toolbar-btn" 
            onClick={handleRefresh}
            title="Refresh Explorer"
          >
            <RefreshCw size={14} />
          </button>
          <button 
            className="explorer-toolbar-btn" 
            onClick={handleNewFile}
            title="New File (Ctrl+N)"
          >
            <FilePlus size={14} />
          </button>
          <button 
            className="explorer-toolbar-btn" 
            onClick={handleNewFolder}
            title="New Folder (Ctrl+Shift+N)"
          >
            <FolderPlus size={14} />
          </button>
          <div style={{ position: 'relative' }}>
            <button 
              className={`explorer-toolbar-btn ${showMoreMenu ? 'active' : ''}`}
              onClick={handleMoreActions}
              title="More Actions"
            >
              <MoreVertical size={14} />
            </button>
            {showMoreMenu && (
              <div className="more-actions-menu">
                <button onClick={handleExpandAll}>
                  Expand All Folders
                </button>
                <button onClick={handleCollapseAll}>
                  Collapse All Folders
                </button>
                <button onClick={() => { setShowMoreMenu(false); }}>
                  Close
                </button>
              </div>
            )}
          </div>
        </div>
      </div>
      <div className="explorer-tree">{renderNode(tree)}</div>
    </div>
  );
};

