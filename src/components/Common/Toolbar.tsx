import React from 'react';
import { Play, Code, Eye, Cloud, Settings, Sidebar, Bot, Book, PanelLeft, LayoutGrid } from 'lucide-react';
import './Toolbar.css';

interface ToolbarProps {
  onRun: () => void;
  onCompile: () => void;
  onVisualize: () => void;
  onDeploy: () => void;
  onSettings: () => void;
  onToggleSidebar?: () => void;
  showSidebar?: boolean;
  onToggleAIPane?: () => void;
  showAIPane?: boolean;
  onTogglePanel?: () => void;
  showPanel?: boolean;
}

export const Toolbar: React.FC<ToolbarProps> = ({
  onRun,
  onCompile,
  onVisualize,
  onDeploy,
  onSettings,
  onToggleSidebar,
  showSidebar = true,
  onToggleAIPane,
  showAIPane = false,
  onTogglePanel,
  showPanel = false,
}) => {
  return (
    <div className="toolbar">
      {/* Logo on Left */}
      <div className="toolbar-logo">
        <a href="/workspace.html" className="logo-link" title="Back to Workspace">
          <span className="logo-text">
            NAV<span className="logo-lambda lambda-in-text">Λ</span> Studio
          </span>
        </a>
      </div>

      {/* Action Buttons in Middle */}
      <div className="toolbar-actions">
        <button className="toolbar-btn primary" onClick={onRun} title="Run (F5)">
          <Play size={18} />
          <span>Run</span>
        </button>

        <button className="toolbar-btn secondary" onClick={onCompile} title="Compile">
          <Code size={18} />
          <span>Compile</span>
        </button>

        <button className="toolbar-btn secondary" onClick={onVisualize} title="Visualize">
          <Eye size={18} />
          <span>Visualize</span>
        </button>

        <button className="toolbar-btn secondary" onClick={onDeploy} title="Deploy">
          <Cloud size={18} />
          <span>Deploy</span>
        </button>
      </div>

      {/* Icons on Right */}
      <div className="toolbar-right-icons">
        {onTogglePanel && (
          <button 
            className={`toolbar-icon-btn ${showPanel ? 'active' : ''}`}
            onClick={onTogglePanel} 
            title={`${showPanel ? 'Hide' : 'Show'} Layout`}
          >
            <LayoutGrid size={18} />
          </button>
        )}

        {onToggleAIPane && (
          <button 
            className={`toolbar-icon-btn ${showAIPane ? 'active' : ''}`}
            onClick={onToggleAIPane} 
            title={`${showAIPane ? 'Hide' : 'Show'} AI Assistant`}
          >
            <Bot size={18} />
          </button>
        )}

        <button className="toolbar-icon-btn toolbar-icon-btn-settings" onClick={onSettings} title="Settings">
          <Settings size={18} />
        </button>
      </div>
    </div>
  );
};

