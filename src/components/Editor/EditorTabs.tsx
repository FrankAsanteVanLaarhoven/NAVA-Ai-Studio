import React from 'react';
import { X } from 'lucide-react';
import './EditorTabs.css';

export interface EditorTab {
  path: string;
  name: string;
  isModified?: boolean;
  isActive?: boolean;
}

interface EditorTabsProps {
  tabs: EditorTab[];
  activeTab?: string;
  onTabClick: (path: string) => void;
  onTabClose: (path: string, e: React.MouseEvent) => void;
}

export const EditorTabs: React.FC<EditorTabsProps> = ({
  tabs,
  activeTab,
  onTabClick,
  onTabClose,
}) => {
  return (
    <div className="editor-tabs-container">
      <div className="editor-tabs">
        {tabs.map((tab) => (
          <div
            key={tab.path}
            className={`editor-tab ${activeTab === tab.path ? 'active' : ''} ${tab.isModified ? 'modified' : ''}`}
            onClick={() => onTabClick(tab.path)}
            title={tab.path}
          >
            <span className="editor-tab-icon">{getFileIcon(tab.name)}</span>
            <span className="editor-tab-name">{tab.name}</span>
            {tab.isModified && <span className="editor-tab-dot">●</span>}
            <button
              className="editor-tab-close"
              onClick={(e) => onTabClose(tab.path, e)}
              onMouseDown={(e) => e.stopPropagation()}
              title="Close"
            >
              <X size={12} />
            </button>
          </div>
        ))}
      </div>
    </div>
  );
};

const getFileIcon = (fileName: string): string => {
  const ext = fileName.split('.').pop()?.toLowerCase();
  switch (ext) {
    case 'navλ':
    case 'vnc':
    case 'nav':
      return '⋋';
    case 'ts':
    case 'tsx':
      return '📘';
    case 'js':
    case 'jsx':
      return '📙';
    case 'py':
      return '🐍';
    case 'rs':
      return '🦀';
    case 'css':
      return '🎨';
    case 'html':
      return '🌐';
    case 'json':
      return '⚙️';
    case 'md':
      return '📝';
    default:
      return '📄';
  }
};

