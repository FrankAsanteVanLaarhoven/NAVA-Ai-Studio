import React from 'react';
import { Play, Bug, StopCircle, RotateCw, Settings } from 'lucide-react';
import './ActivityPanels.css';

export const DebugPanel: React.FC = () => {
  const configurations = [
    { name: 'Launch NAVΛ Studio', type: 'node' },
    { name: 'Debug VNC Code', type: 'vnc' },
    { name: 'Attach to WebAssembly', type: 'wasm' },
  ];

  return (
    <div className="activity-panel debug-panel">
      <div className="panel-header">
        <div className="panel-title">
          <Bug size={16} />
          <span>Run and Debug</span>
        </div>
        <div className="panel-actions">
          <button className="icon-btn" title="Settings">
            <Settings size={16} />
          </button>
        </div>
      </div>

      <div className="panel-content">
        {/* Debug Controls */}
        <div className="debug-controls">
          <button className="debug-btn primary">
            <Play size={18} />
            <span>Run and Debug</span>
          </button>
        </div>

        {/* Configurations */}
        <div className="debug-configs">
          <div className="section-header">
            <span>Configurations</span>
          </div>
          {configurations.map((config, index) => (
            <div key={index} className="config-item">
              <div className="config-icon">
                <Play size={14} />
              </div>
              <div className="config-info">
                <div className="config-name">{config.name}</div>
                <div className="config-type">{config.type}</div>
              </div>
              <button className="icon-btn">
                <Play size={14} />
              </button>
            </div>
          ))}
        </div>

        {/* Quick Actions */}
        <div className="debug-actions">
          <button className="action-btn">
            <Play size={16} />
            <span>Start Debugging (F5)</span>
          </button>
          <button className="action-btn">
            <RotateCw size={16} />
            <span>Run Without Debugging (Ctrl+F5)</span>
          </button>
        </div>

        {/* Debug Info */}
        <div className="debug-info">
          <div className="info-item">
            <span className="info-label">Status:</span>
            <span className="info-value">Ready</span>
          </div>
          <div className="info-item">
            <span className="info-label">Breakpoints:</span>
            <span className="info-value">0</span>
          </div>
        </div>
      </div>
    </div>
  );
};

