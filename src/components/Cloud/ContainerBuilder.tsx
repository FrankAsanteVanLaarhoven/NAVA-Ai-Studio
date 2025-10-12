import React, { useState } from 'react';
import { Container, Play, Download } from 'lucide-react';

interface ContainerConfig {
  baseImage: string;
  ports: number[];
  environment: Record<string, string>;
}

export const ContainerBuilder: React.FC = () => {
  const [config, setConfig] = useState<ContainerConfig>({
    baseImage: 'rust:1.75',
    ports: [8080],
    environment: {},
  });

  const [buildLog, setBuildLog] = useState<string[]>([]);

  const buildContainer = async () => {
    setBuildLog(['Building Docker container...', 'Pulling base image...']);

    await new Promise((resolve) => setTimeout(resolve, 1000));
    setBuildLog((prev) => [...prev, 'Installing dependencies...']);

    await new Promise((resolve) => setTimeout(resolve, 1000));
    setBuildLog((prev) => [...prev, 'Compiling NAVΛ application...']);

    await new Promise((resolve) => setTimeout(resolve, 1000));
    setBuildLog((prev) => [...prev, 'Container build complete! ✓']);
  };

  return (
    <div className="container-builder">
      <h3>
        <Container size={20} /> Container Builder
      </h3>

      <div className="builder-config">
        <div className="config-field">
          <label>Base Image:</label>
          <input
            type="text"
            value={config.baseImage}
            onChange={(e) => setConfig({ ...config, baseImage: e.target.value })}
          />
        </div>

        <div className="config-field">
          <label>Exposed Ports:</label>
          <input
            type="text"
            value={config.ports.join(', ')}
            onChange={(e) =>
              setConfig({
                ...config,
                ports: e.target.value.split(',').map((p) => parseInt(p.trim())),
              })
            }
          />
        </div>
      </div>

      <div className="builder-actions">
        <button className="build-btn" onClick={buildContainer}>
          <Play size={16} /> Build Container
        </button>
        <button className="download-btn">
          <Download size={16} /> Download Dockerfile
        </button>
      </div>

      {buildLog.length > 0 && (
        <div className="build-log">
          <h4>Build Log:</h4>
          {buildLog.map((log, index) => (
            <div key={index} className="log-line">
              {log}
            </div>
          ))}
        </div>
      )}
    </div>
  );
};

