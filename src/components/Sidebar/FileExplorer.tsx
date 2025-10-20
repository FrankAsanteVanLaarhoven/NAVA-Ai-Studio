import React, { useState, useRef } from 'react';
import { ChevronRight, ChevronDown, File, Folder, FolderOpen, RefreshCw, FilePlus, FolderPlus, MoreVertical, Upload, Github, GitBranch, FolderUp } from 'lucide-react';
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
  const [isDragging, setIsDragging] = useState(false);
  const [showGitHubModal, setShowGitHubModal] = useState(false);
  const [githubToken, setGithubToken] = useState(localStorage.getItem('github_token') || '');
  const [repoName, setRepoName] = useState('');
  const [repoDescription, setRepoDescription] = useState('');
  const [isUploading, setIsUploading] = useState(false);
  const fileInputRef = useRef<HTMLInputElement>(null);
  const folderInputRef = useRef<HTMLInputElement>(null);

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

  // Drag and Drop Handlers
  const handleDragOver = (e: React.DragEvent) => {
    e.preventDefault();
    e.stopPropagation();
    setIsDragging(true);
  };

  const handleDragLeave = (e: React.DragEvent) => {
    e.preventDefault();
    e.stopPropagation();
    setIsDragging(false);
  };

  const handleDrop = async (e: React.DragEvent) => {
    e.preventDefault();
    e.stopPropagation();
    setIsDragging(false);

    const items = Array.from(e.dataTransfer.items);
    
    for (const item of items) {
      if (item.kind === 'file') {
        const entry = item.webkitGetAsEntry();
        if (entry) {
          await processEntry(entry, '/');
        }
      }
    }
  };

  const processEntry = async (entry: any, parentPath: string): Promise<void> => {
    if (entry.isFile) {
      entry.file(async (file: File) => {
        const content = await file.text();
        const filePath = `${parentPath}/${file.name}`.replace('//', '/');
        addFileToTree(parentPath, file.name);
        console.log(`Loaded file: ${filePath}`, content);
        // TODO: Store file content in a file system service
      });
    } else if (entry.isDirectory) {
      addFolderToTree(parentPath, entry.name);
      const dirPath = `${parentPath}/${entry.name}`.replace('//', '/');
      
      const reader = entry.createReader();
      reader.readEntries(async (entries: any[]) => {
        for (const childEntry of entries) {
          await processEntry(childEntry, dirPath);
        }
      });
    }
  };

  // File/Folder Upload Handlers
  const handleFileUpload = async (e: React.ChangeEvent<HTMLInputElement>) => {
    const files = e.target.files;
    if (!files) return;

    for (let i = 0; i < files.length; i++) {
      const file = files[i];
      const content = await file.text();
      addFileToTree('/', file.name);
      console.log(`Uploaded file: ${file.name}`, content);
    }

    if (e.target) e.target.value = '';
  };

  const handleFolderUpload = async (e: React.ChangeEvent<HTMLInputElement>) => {
    const files = e.target.files;
    if (!files) return;

    const folderStructure: { [key: string]: File[] } = {};
    
    // Group files by folder
    for (let i = 0; i < files.length; i++) {
      const file = files[i];
      const pathParts = file.webkitRelativePath.split('/');
      const folderName = pathParts[0];
      
      if (!folderStructure[folderName]) {
        folderStructure[folderName] = [];
      }
      folderStructure[folderName].push(file);
    }

    // Process each folder
    for (const [folderName, folderFiles] of Object.entries(folderStructure)) {
      addFolderToTree('/', folderName);
      
      for (const file of folderFiles) {
        const pathParts = file.webkitRelativePath.split('/');
        pathParts.shift(); // Remove root folder name
        const relativePath = pathParts.join('/');
        const parentPath = '/' + folderName + (pathParts.length > 1 ? '/' + pathParts.slice(0, -1).join('/') : '');
        
        addFileToTree(parentPath.replace('//', '/'), file.name);
        console.log(`Loaded: ${file.webkitRelativePath}`);
      }
    }

    if (e.target) e.target.value = '';
  };

  // GitHub Integration
  const handleGitHubUpload = async () => {
    if (!githubToken || !repoName) {
      alert('Please provide GitHub token and repository name');
      return;
    }

    setIsUploading(true);

    try {
      // Create repository
      const createRepoResponse = await fetch('https://api.github.com/user/repos', {
        method: 'POST',
        headers: {
          'Authorization': `token ${githubToken}`,
          'Accept': 'application/vnd.github.v3+json',
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          name: repoName,
          description: repoDescription || 'Project created with NAVΛ Studio IDE',
          private: false,
          auto_init: true,
        }),
      });

      if (!createRepoResponse.ok) {
        const error = await createRepoResponse.json();
        throw new Error(error.message || 'Failed to create repository');
      }

      const repo = await createRepoResponse.json();
      
      // Upload files to repository
      await uploadFilesToGitHub(repo.owner.login, repo.name, tree);

      alert(`✅ Successfully created and uploaded to: ${repo.html_url}`);
      window.open(repo.html_url, '_blank');
      
      setShowGitHubModal(false);
      setRepoName('');
      setRepoDescription('');
    } catch (error: any) {
      console.error('GitHub upload error:', error);
      alert(`❌ Error: ${error.message}`);
    } finally {
      setIsUploading(false);
    }
  };

  const uploadFilesToGitHub = async (owner: string, repo: string, node: FileNode, basePath = ''): Promise<void> => {
    if (node.type === 'file') {
      const path = basePath + '/' + node.name;
      const content = `// File: ${node.name}\n// Created with NAVΛ Studio IDE\n`;
      const encodedContent = btoa(unescape(encodeURIComponent(content)));

      await fetch(`https://api.github.com/repos/${owner}/${repo}/contents${path}`, {
        method: 'PUT',
        headers: {
          'Authorization': `token ${githubToken}`,
          'Accept': 'application/vnd.github.v3+json',
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          message: `Add ${node.name}`,
          content: encodedContent,
        }),
      });
    } else if (node.children) {
      for (const child of node.children) {
        await uploadFilesToGitHub(owner, repo, child, basePath + '/' + node.name);
      }
    }
  };

  const saveGitHubToken = () => {
    localStorage.setItem('github_token', githubToken);
    alert('✅ GitHub token saved securely!');
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
          <button 
            className="explorer-toolbar-btn" 
            onClick={() => folderInputRef.current?.click()}
            title="Open Folder"
          >
            <FolderUp size={14} />
          </button>
          <button 
            className="explorer-toolbar-btn" 
            onClick={() => setShowGitHubModal(true)}
            title="Push to GitHub"
          >
            <Github size={14} />
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
                <button onClick={() => { fileInputRef.current?.click(); setShowMoreMenu(false); }}>
                  <Upload size={14} /> Upload Files
                </button>
                <button onClick={() => { folderInputRef.current?.click(); setShowMoreMenu(false); }}>
                  <FolderUp size={14} /> Upload Folder
                </button>
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

      {/* Hidden File Inputs */}
      <input
        ref={fileInputRef}
        type="file"
        multiple
        onChange={handleFileUpload}
        style={{ display: 'none' }}
      />
      <input
        ref={folderInputRef}
        type="file"
        multiple
        webkitdirectory=""
        directory=""
        onChange={handleFolderUpload}
        style={{ display: 'none' }}
      />

      {/* Drag and Drop Zone */}
      <div 
        className={`explorer-tree ${isDragging ? 'dragging' : ''}`}
        onDragOver={handleDragOver}
        onDragLeave={handleDragLeave}
        onDrop={handleDrop}
      >
        {isDragging && (
          <div className="drop-overlay">
            <div className="drop-message">
              <FolderUp size={48} />
              <h3>Drop your files or folders here</h3>
              <p>To load them into the project</p>
            </div>
          </div>
        )}
        {renderNode(tree)}
      </div>

      {/* GitHub Modal */}
      {showGitHubModal && (
        <div className="github-modal-overlay" onClick={() => setShowGitHubModal(false)}>
          <div className="github-modal" onClick={(e) => e.stopPropagation()}>
            <div className="github-modal-header">
              <h3><Github size={20} /> Push to GitHub</h3>
              <button onClick={() => setShowGitHubModal(false)} className="close-btn">×</button>
            </div>
            <div className="github-modal-content">
              <div className="form-group">
                <label>
                  <GitBranch size={14} /> GitHub Personal Access Token
                </label>
                <input
                  type="password"
                  value={githubToken}
                  onChange={(e) => setGithubToken(e.target.value)}
                  placeholder="ghp_xxxxxxxxxxxx"
                  className="github-input"
                />
                <button onClick={saveGitHubToken} className="save-token-btn">
                  💾 Save Token
                </button>
                <small style={{ display: 'block', marginTop: '8px', color: '#858585' }}>
                  Generate token at: <a href="https://github.com/settings/tokens" target="_blank" rel="noopener noreferrer">github.com/settings/tokens</a>
                </small>
              </div>

              <div className="form-group">
                <label>Repository Name *</label>
                <input
                  type="text"
                  value={repoName}
                  onChange={(e) => setRepoName(e.target.value)}
                  placeholder="my-awesome-project"
                  className="github-input"
                />
              </div>

              <div className="form-group">
                <label>Description (Optional)</label>
                <textarea
                  value={repoDescription}
                  onChange={(e) => setRepoDescription(e.target.value)}
                  placeholder="A brief description of your project..."
                  className="github-textarea"
                  rows={3}
                />
              </div>

              <div className="github-modal-footer">
                <button onClick={() => setShowGitHubModal(false)} className="btn-cancel">
                  Cancel
                </button>
                <button 
                  onClick={handleGitHubUpload} 
                  disabled={isUploading || !githubToken || !repoName}
                  className="btn-push"
                >
                  {isUploading ? (
                    <>⏳ Uploading...</>
                  ) : (
                    <><Github size={16} /> Create & Push</>
                  )}
                </button>
              </div>
            </div>
          </div>
        </div>
      )}
    </div>
  );
};

