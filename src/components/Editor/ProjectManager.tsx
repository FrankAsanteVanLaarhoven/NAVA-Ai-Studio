import React, { useState, useEffect } from 'react';
import { fileService, Project } from '../../services/file-service';
import { projectSourceService, projectSources, type ProjectSource } from '../../services/project-source-service';
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
  const [selectedSource, setSelectedSource] = useState<ProjectSource | null>(null);
  const [isLoading, setIsLoading] = useState(false);
  const [sourceInput, setSourceInput] = useState('');

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

  const handleOpenProject = async (source?: ProjectSource) => {
    try {
      setIsLoading(true);
      let projectData: { path: string; files: any[] } | null = null;

      if (source === 'local' || !source) {
        // Use File System Access API or file input
        projectData = await projectSourceService.openFromLocal();
      } else if (source === 'github') {
        const repoUrl = sourceInput || prompt('Enter GitHub repository URL:');
        if (!repoUrl) {
          setIsLoading(false);
          return;
        }
        projectData = await projectSourceService.cloneFromGitHub(repoUrl);
      } else if (source === 'huggingface') {
        const repoId = sourceInput || prompt('Enter HuggingFace repository ID:');
        if (!repoId) {
          setIsLoading(false);
          return;
        }
        projectData = await projectSourceService.importFromHuggingFace(repoId);
      } else {
        alert(`${source} integration coming soon!`);
        setIsLoading(false);
        return;
      }

      if (!projectData) {
        setIsLoading(false);
        return;
      }

      // Import files into file service
      if (projectData.files && projectData.files.length > 0) {
        await fileService.importFiles(projectData.files);
      }

      // Convert to Project format
      const project: Project = {
        name: projectData.path,
        path: projectData.path,
        files: projectData.files,
        createdAt: new Date(),
        lastModified: new Date(),
      };

      // Save project metadata
      await fileService.saveProject(project);
      fileService.setCurrentProject(project);
      
      // Trigger explorer refresh
      const refreshEvent = new CustomEvent('nava:refresh-explorer');
      window.dispatchEvent(refreshEvent);
      
      onProjectOpen(project);
      onClose();
    } catch (error) {
      console.error('Error opening project:', error);
      alert('Failed to open project');
    } finally {
      setIsLoading(false);
      setSelectedSource(null);
      setSourceInput('');
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
            {!selectedSource ? (
              <div className="source-grid">
                {projectSources.map((source) => (
                  <button
                    key={source.source}
                    className="source-btn"
                    onClick={() => {
                      if (source.requiresAuth) {
                        setSelectedSource(source.source);
                      } else {
                        handleOpenProject(source.source);
                      }
                    }}
                    title={source.description}
                  >
                    <span className="source-icon">{source.icon}</span>
                    <span className="source-name">{source.name}</span>
                  </button>
                ))}
              </div>
            ) : (
              <div className="source-input-container">
                <div className="source-header">
                  <span className="source-icon-large">
                    {projectSources.find(s => s.source === selectedSource)?.icon}
                  </span>
                  <div>
                    <h4>{projectSources.find(s => s.source === selectedSource)?.name}</h4>
                    <p className="source-description">
                      {projectSources.find(s => s.source === selectedSource)?.description}
                    </p>
                  </div>
                </div>
                {selectedSource === 'github' && (
                  <div className="form-group">
                    <label htmlFor="github-url">GitHub Repository URL</label>
                    <input
                      id="github-url"
                      type="text"
                      value={sourceInput}
                      onChange={(e) => setSourceInput(e.target.value)}
                      placeholder="https://github.com/owner/repo"
                      onKeyDown={(e) => {
                        if (e.key === 'Enter' && sourceInput) {
                          handleOpenProject(selectedSource);
                        }
                      }}
                    />
                  </div>
                )}
                {selectedSource === 'huggingface' && (
                  <div className="form-group">
                    <label htmlFor="hf-repo">HuggingFace Repository ID</label>
                    <input
                      id="hf-repo"
                      type="text"
                      value={sourceInput}
                      onChange={(e) => setSourceInput(e.target.value)}
                      placeholder="username/dataset-name"
                      onKeyDown={(e) => {
                        if (e.key === 'Enter' && sourceInput) {
                          handleOpenProject(selectedSource);
                        }
                      }}
                    />
                  </div>
                )}
                <div className="source-actions">
                  <button
                    className="secondary-btn"
                    onClick={() => {
                      setSelectedSource(null);
                      setSourceInput('');
                    }}
                  >
                    Cancel
                  </button>
                  <button
                    className="primary-btn"
                    onClick={() => handleOpenProject(selectedSource)}
                    disabled={isLoading || (selectedSource !== 'local' && !sourceInput.trim())}
                  >
                    {isLoading ? 'Loading...' : 'Open'}
                  </button>
                </div>
              </div>
            )}
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
