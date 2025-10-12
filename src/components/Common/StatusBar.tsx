import React from 'react';
import './StatusBar.css';

interface StatusBarProps {
  line: number;
  column: number;
  language: string;
  status: 'ready' | 'compiling' | 'running' | 'error';
}

export const StatusBar: React.FC<StatusBarProps> = ({ line, column, language, status }) => {
  const getStatusColor = () => {
    switch (status) {
      case 'ready':
        return '#00ff00';
      case 'compiling':
        return '#ffff00';
      case 'running':
        return '#00ccff';
      case 'error':
        return '#ff0000';
      default:
        return '#888';
    }
  };

  return (
    <div className="status-bar">
      <div className="status-section">
        <span className="status-indicator" style={{ background: getStatusColor() }} />
        <span className="status-text">{status.toUpperCase()}</span>
      </div>

      <div className="status-section">
        <span className="status-label">Language:</span>
        <span className="status-value">{language}</span>
      </div>

      <div className="status-section">
        <span className="status-label">Ln {line}, Col {column}</span>
      </div>

      <div className="status-section">
        <span className="vnc-badge">⋋ VNC Enabled</span>
      </div>
    </div>
  );
};

