import React from 'react';
import { File, FolderOpen, AlertCircle, CheckCircle } from 'lucide-react';
import './ProjectStatusBar.css';

interface ProjectStatusBarProps {
  projectName: string;
  currentFile?: string;
  fileStatus?: 'saved' | 'unsaved' | 'error';
  totalFiles?: number;
  modifiedFiles?: number;
}

export const ProjectStatusBar: React.FC<ProjectStatusBarProps> = ({
  projectName,
  currentFile,
  fileStatus = 'saved',
  totalFiles = 0,
  modifiedFiles = 0,
}) => {
  const getStatusIcon = () => {
    switch (fileStatus) {
      case 'unsaved':
        return <AlertCircle size={14} className="status-icon warning" />;
      case 'error':
        return <AlertCircle size={14} className="status-icon error" />;
      default:
        return <CheckCircle size={14} className="status-icon success" />;
    }
  };

  const getStatusText = () => {
    switch (fileStatus) {
      case 'unsaved':
        return 'Unsaved changes';
      case 'error':
        return 'Error';
      default:
        return 'Saved';
    }
  };

  return (
    <div className="project-status-bar">
      <div className="project-info">
        <FolderOpen size={16} className="project-icon" />
        <span className="project-name">{projectName}</span>
        {currentFile && (
          <>
            <span className="separator">—</span>
            <File size={14} className="file-icon" />
            <span className="file-name">{currentFile}</span>
          </>
        )}
      </div>

      <div className="project-stats">
        {modifiedFiles > 0 && (
          <span className="stat-item modified">
            {modifiedFiles} modified
          </span>
        )}
        <span className="stat-item">
          {totalFiles} files
        </span>
        <div className="file-status">
          {getStatusIcon()}
          <span className="status-text">{getStatusText()}</span>
        </div>
      </div>
    </div>
  );
};

