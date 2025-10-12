import React from 'react';
import { Play, Code, Eye, Cloud, Settings, Sidebar, Bot, Book, PanelLeft } from 'lucide-react';
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
      <div className="toolbar-logo">
        <span className="logo-text">NAV<span className="logo-lambda">Λ</span> Studio</span>
      </div>

      <div className="toolbar-actions">
        <button className="toolbar-btn primary" onClick={onRun} title="Run (F5)">
          <Play size={18} />
          <span>Run</span>
        </button>

        <button className="toolbar-btn" onClick={onCompile} title="Compile">
          <Code size={18} />
          <span>Compile</span>
        </button>

        <button className="toolbar-btn" onClick={onVisualize} title="Visualize">
          <Eye size={18} />
          <span>Visualize</span>
        </button>

        <button className="toolbar-btn" onClick={onDeploy} title="Deploy">
          <Cloud size={18} />
          <span>Deploy</span>
        </button>

        <div className="toolbar-spacer" />

        {/* View Toggle Controls */}
        {onToggleSidebar && (
          <button 
            className={`toolbar-btn icon-only ${showSidebar ? 'active' : ''}`} 
            onClick={onToggleSidebar} 
            title="Toggle Sidebar (Ctrl+B)"
          >
            <PanelLeft size={18} />
          </button>
        )}

        {onTogglePanel && (
          <button 
            className={`toolbar-btn icon-only ${showPanel ? 'active' : ''}`} 
            onClick={onTogglePanel} 
            title="Toggle Panel (Ctrl+J)"
          >
            <Book size={18} />
          </button>
        )}

        {onToggleAIPane && (
          <button 
            className={`toolbar-btn icon-only ${showAIPane ? 'active' : ''}`} 
            onClick={onToggleAIPane} 
            title="Toggle AI Assistant (Ctrl+K)"
          >
            <Bot size={18} />
          </button>
        )}

        <button className="toolbar-btn icon-only" onClick={onSettings} title="Settings">
          <Settings size={18} />
        </button>
      </div>
    </div>
  );
};

