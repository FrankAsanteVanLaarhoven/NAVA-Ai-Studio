import React, { useState } from 'react';
import { ChevronRight, ChevronDown, File, Folder, FolderOpen } from 'lucide-react';
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

  const renderNode = (node: FileNode, depth: number = 0) => {
    const isFolder = node.type === 'folder';

    return (
      <div key={node.path}>
        <div
          className="file-node"
          style={{ paddingLeft: `${depth * 16 + 8}px` }}
          onClick={() => isFolder && toggleFolder(node.path)}
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

  return (
    <div className="file-explorer">
      <div className="explorer-header">
        <h3>EXPLORER</h3>
      </div>
      <div className="explorer-tree">{renderNode(tree)}</div>
    </div>
  );
};

