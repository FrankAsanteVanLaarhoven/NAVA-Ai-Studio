import React, { useState } from 'react';
import { GitBranch, GitCommit, GitPullRequest, RefreshCw, Plus, MoreHorizontal } from 'lucide-react';
import './ActivityPanels.css';

export const SourceControlPanel: React.FC = () => {
  const [commitMessage, setCommitMessage] = useState('');
  
  const changes = [
    { file: 'src/App.tsx', status: 'modified', changes: '+12 -3' },
    { file: 'src/components/ActivityBar/ActivityBar.tsx', status: 'new', changes: '+156' },
    { file: 'src/components/ActivityBar/ActivityBar.css', status: 'new', changes: '+98' },
    { file: 'src/App.css', status: 'modified', changes: '+24 -8' },
  ];

  return (
    <div className="activity-panel source-control-panel">
      <div className="panel-header">
        <div className="panel-title">
          <GitBranch size={16} />
          <span>Source Control</span>
        </div>
        <div className="panel-actions">
          <button className="icon-btn" title="Refresh">
            <RefreshCw size={16} />
          </button>
          <button className="icon-btn" title="More Actions">
            <MoreHorizontal size={16} />
          </button>
        </div>
      </div>

      <div className="panel-content">
        {/* Commit Section */}
        <div className="commit-section">
          <textarea
            className="commit-input"
            placeholder="Message (Ctrl+Enter to commit)"
            value={commitMessage}
            onChange={(e) => setCommitMessage(e.target.value)}
            rows={3}
          />
          <button className="commit-btn" disabled={!commitMessage}>
            <GitCommit size={16} />
            <span>Commit</span>
          </button>
        </div>

        {/* Branch Info */}
        <div className="branch-info">
          <div className="branch-name">
            <GitBranch size={14} />
            <span>main</span>
          </div>
          <button className="branch-action">
            <GitPullRequest size={14} />
            <span>Sync Changes</span>
          </button>
        </div>

        {/* Changes */}
        <div className="changes-section">
          <div className="section-header">
            <span>Changes ({changes.length})</span>
            <button className="icon-btn" title="Stage All Changes">
              <Plus size={14} />
            </button>
          </div>
          <div className="changes-list">
            {changes.map((change, index) => (
              <div key={index} className="change-item">
                <div className="change-status">
                  <span className={`status-badge ${change.status}`}>
                    {change.status === 'modified' ? 'M' : 'U'}
                  </span>
                  <span className="change-file">{change.file}</span>
                </div>
                <span className="change-count">{change.changes}</span>
              </div>
            ))}
          </div>
        </div>
      </div>
    </div>
  );
};

