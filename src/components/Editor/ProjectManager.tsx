import React, { useState, useEffect } from 'react';
import { fileService, Project } from '../../services/file-service';
import './ProjectManager.css';

interface ProjectManagerProps {
  onProjectOpen: (project: Project) => void;
  onClose: () => void;
}

export const ProjectManager: React.FC<ProjectManagerProps> = ({
  onProjectOpen,
  onClose,
}) => {
  const [projectName, setProjectName] = useState('');
  const [recentFiles, setRecentFiles] = useState<string[]>([]);
  const [isCreating, setIsCreating] = useState(false);

  useEffect(() => {
    loadRecentFiles();
  }, []);

  const loadRecentFiles = () => {
    const recent = fileService.getRecentFiles();
    setRecentFiles(recent);
  };

  const handleCreateProject = async () => {
    if (!projectName.trim()) {
      alert('Please enter a project name');
      return;
    }

    setIsCreating(true);
    try {
      // In a real app, you'd use a directory picker
      // For now, we'll use a default location
      const defaultPath = `/projects/${projectName}`;
      const project = await fileService.createProject(projectName, defaultPath);
      onProjectOpen(project);
      onClose();
    } catch (error) {
      console.error('Error creating project:', error);
      alert('Failed to create project');
    } finally {
      setIsCreating(false);
    }
  };

  const handleOpenProject = async () => {
    try {
      // In a real app, you'd use a directory picker dialog
      const path = prompt('Enter project path:');
      if (!path) return;

      const project = await fileService.openProject(path);
      if (project) {
        onProjectOpen(project);
        onClose();
      } else {
        alert('Failed to open project');
      }
    } catch (error) {
      console.error('Error opening project:', error);
      alert('Failed to open project');
    }
  };

  const handleOpenRecentFile = async (filePath: string) => {
    try {
      await fileService.readFile(filePath);
      // Create a temporary project with just this file
      const fileName = fileService.getFileName(filePath);
      const project: Project = {
        name: fileName,
        path: fileService.getDirectoryName(filePath),
        files: [{
          path: filePath,
          name: fileName,
          isDirectory: false,
        }],
        createdAt: new Date(),
        lastModified: new Date(),
      };
      onProjectOpen(project);
      onClose();
    } catch (error) {
      console.error('Error opening recent file:', error);
      alert('Failed to open file');
    }
  };

  const handleKeyDown = (e: React.KeyboardEvent<HTMLInputElement>) => {
    if (e.key === 'Enter') {
      handleCreateProject();
    }
  };

  return (
    <div className="project-manager-overlay">
      <div className="project-manager">
        <div className="project-manager-header">
          <h2>Project Manager</h2>
          <button className="close-btn" onClick={onClose}>
            ✕
          </button>
        </div>

        <div className="project-manager-content">
          {/* Create New Project */}
          <section className="pm-section">
            <h3>Create New Project</h3>
            <div className="form-group">
              <label htmlFor="project-name">Project Name</label>
              <input
                id="project-name"
                type="text"
                value={projectName}
                onChange={(e) => setProjectName(e.target.value)}
                placeholder="my-navlambda-project"
                onKeyDown={handleKeyDown}
              />
            </div>
            <button
              className="primary-btn"
              onClick={handleCreateProject}
              disabled={isCreating || !projectName.trim()}
            >
              {isCreating ? 'Creating...' : 'Create Project'}
            </button>
          </section>

          {/* Open Existing Project */}
          <section className="pm-section">
            <h3>Open Project</h3>
            <button className="secondary-btn" onClick={handleOpenProject}>
              📁 Open Folder
            </button>
          </section>

          {/* Recent Files */}
          {recentFiles.length > 0 && (
            <section className="pm-section">
              <h3>Recent Files</h3>
              <div className="recent-files-list">
                {recentFiles.map((filePath, index) => (
                  <div
                    key={index}
                    className="recent-file-item"
                    onClick={() => handleOpenRecentFile(filePath)}
                  >
                    <span className="file-icon">⋋</span>
                    <div className="file-details">
                      <div className="file-name">
                        {fileService.getFileName(filePath)}
                      </div>
                      <div className="file-path-small" title={filePath}>
                        {filePath}
                      </div>
                    </div>
                  </div>
                ))}
              </div>
            </section>
          )}

          {/* Quick Start */}
          <section className="pm-section">
            <h3>Quick Start</h3>
            <div className="quick-start-grid">
              <div className="quick-start-card">
                <div className="card-icon">⋋</div>
                <h4>New File</h4>
                <p>Start with a blank NAVΛ file</p>
              </div>
              <div className="quick-start-card">
                <div className="card-icon">📚</div>
                <h4>Examples</h4>
                <p>Browse example projects</p>
              </div>
              <div className="quick-start-card">
                <div className="card-icon">📖</div>
                <h4>Documentation</h4>
                <p>Learn Van Laarhoven calculus</p>
              </div>
            </div>
          </section>
        </div>
      </div>
    </div>
  );
};
