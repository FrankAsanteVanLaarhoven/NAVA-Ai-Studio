import React, { useState } from 'react';
import { Cloud, Container, Server, GitBranch, Check } from 'lucide-react';
import './DeploymentPanel.css';

interface DeploymentPanelProps {
  code: string;
}

export const DeploymentPanel: React.FC<DeploymentPanelProps> = ({ code }) => {
  const [selectedTarget, setSelectedTarget] = useState<'docker' | 'kubernetes' | 'cloud'>('docker');
  const [isDeploying, setIsDeploying] = useState(false);
  const [deploymentStatus, setDeploymentStatus] = useState<string[]>([]);

  const handleDeploy = async () => {
    setIsDeploying(true);
    setDeploymentStatus([]);

    const steps = [
      'Building VNC application...',
      'Generating Dockerfile...',
      'Creating container image...',
      'Pushing to registry...',
      'Deploying to cluster...',
      'Deployment complete! ✓',
    ];

    for (const step of steps) {
      await new Promise((resolve) => setTimeout(resolve, 800));
      setDeploymentStatus((prev) => [...prev, step]);
    }

    setIsDeploying(false);
  };

  return (
    <div className="deployment-panel">
      <h2 className="deployment-title">
        <Cloud size={24} />
        Cloud Deployment
      </h2>

      <div className="deployment-targets">
        <button
          className={`target-btn ${selectedTarget === 'docker' ? 'active' : ''}`}
          onClick={() => setSelectedTarget('docker')}
        >
          <Container size={20} />
          Docker
        </button>
        <button
          className={`target-btn ${selectedTarget === 'kubernetes' ? 'active' : ''}`}
          onClick={() => setSelectedTarget('kubernetes')}
        >
          <Server size={20} />
          Kubernetes
        </button>
        <button
          className={`target-btn ${selectedTarget === 'cloud' ? 'active' : ''}`}
          onClick={() => setSelectedTarget('cloud')}
        >
          <Cloud size={20} />
          Cloud
        </button>
      </div>

      <div className="deployment-config">
        <h3>Configuration</h3>
        <div className="config-field">
          <label>Registry:</label>
          <input type="text" defaultValue="docker.io/navlambda" />
        </div>
        <div className="config-field">
          <label>Tag:</label>
          <input type="text" defaultValue="latest" />
        </div>
        <div className="config-field">
          <label>Namespace:</label>
          <input type="text" defaultValue="default" />
        </div>
      </div>

      <button
        className="deploy-btn"
        onClick={handleDeploy}
        disabled={isDeploying}
      >
        {isDeploying ? 'Deploying...' : 'Deploy Application'}
      </button>

      {deploymentStatus.length > 0 && (
        <div className="deployment-log">
          <h3>Deployment Log</h3>
          {deploymentStatus.map((status, index) => (
            <div key={index} className="log-entry">
              <Check size={16} className="log-icon" />
              {status}
            </div>
          ))}
        </div>
      )}
    </div>
  );
};

