import React from 'react';
import { Play, Code, Eye, Cloud, Settings, LayoutGrid, Folder, Check, Bot } from 'lucide-react';
import './MenuBar.css';

interface MenuBarProps {
  onRun: () => void;
  onCompile: () => void;
  onVisualize: () => void;
  onDeploy: () => void;
  onSettings: () => void;
  onTogglePanel?: () => void;
  showPanel?: boolean;
  onToggleAIPane?: () => void;
  showAIPane?: boolean;
  onNavigateToWorkspace?: () => void;
  currentFile?: string;
  fileCount?: number;
  isSaved?: boolean;
}

export const MenuBar: React.FC<MenuBarProps> = ({
  onRun,
  onCompile,
  onVisualize,
  onDeploy,
  onSettings,
  onTogglePanel,
  showPanel = false,
  onToggleAIPane,
  showAIPane = false,
  onNavigateToWorkspace,
  currentFile = 'No file open',
  fileCount = 12,
  isSaved = true,
}) => {
  const handleLogoClick = () => {
    if (onNavigateToWorkspace) {
      onNavigateToWorkspace();
    } else {
      // Fallback: navigate to workspace.html
      window.location.href = '/workspace.html';
    }
  };
  return (
    <div className="menu-bar-container">
      {/* Top Menu Bar */}
      <div className="menu-bar-top">
        {/* Left: Logo Icons */}
        <div 
          className="menu-bar-logo" 
          onClick={handleLogoClick}
          style={{ cursor: 'pointer' }}
          title="Back to Workspace"
        >
          <span className="menu-logo-icon lambda-icon">λ</span>
          <span className="menu-logo-icon caret-icon">Λ</span>
        </div>

        {/* Middle: Action Buttons */}
        <div className="menu-bar-actions">
          <button 
            className="menu-action-btn menu-action-btn-primary" 
            onClick={onRun}
            title="Run (F5)"
          >
            <Play size={16} />
            <span>Run</span>
          </button>
          <button 
            className="menu-action-btn menu-action-btn-secondary" 
            onClick={onCompile}
            title="Compile"
          >
            <Code size={16} />
            <span>Compile</span>
          </button>
          <button 
            className="menu-action-btn menu-action-btn-secondary" 
            onClick={onVisualize}
            title="Visualize"
          >
            <Eye size={16} />
            <span>Visualize</span>
          </button>
          <button 
            className="menu-action-btn menu-action-btn-secondary" 
            onClick={onDeploy}
            title="Deploy"
          >
            <Cloud size={16} />
            <span>Deploy</span>
          </button>
        </div>

        {/* Right: Utility Icons */}
        <div className="menu-bar-utils">
          {onTogglePanel && (
            <button 
              className={`menu-util-icon ${showPanel ? 'active' : ''}`}
              onClick={onTogglePanel}
              title={`${showPanel ? 'Hide' : 'Show'} Layout`}
            >
              <LayoutGrid size={18} />
            </button>
          )}
          {onToggleAIPane && (
            <button 
              className={`menu-util-icon ${showAIPane ? 'active' : ''}`}
              onClick={onToggleAIPane}
              title={`${showAIPane ? 'Hide' : 'Show'} AI Assistant`}
            >
              <Bot size={18} />
            </button>
          )}
          <button 
            className="menu-util-icon menu-util-icon-notification"
            title="Notifications"
          >
            <Folder size={18} />
            <span className="notification-dot"></span>
          </button>
          <button 
            className="menu-util-icon"
            onClick={onSettings}
            title="Settings"
          >
            <Settings size={18} />
          </button>
        </div>
      </div>

      {/* Status Bar */}
      <div className="menu-bar-status">
        <div className="status-left">
          <Folder size={16} className="status-icon" />
          <span className="status-text">NAV<span style={{ color: '#00ff00', textShadow: '0 0 10px rgba(0, 255, 0, 0.5)' }}>Λ</span> STUDIO IDE</span>
          <Folder size={16} className="status-icon status-file-icon" />
          <span className="status-text">{currentFile}</span>
        </div>
        <div className="status-right">
          <span className="status-text">{fileCount} files</span>
          <button className="status-saved-btn">
            <Check size={14} />
            <span>Saved</span>
          </button>
        </div>
      </div>
    </div>
  );
};

