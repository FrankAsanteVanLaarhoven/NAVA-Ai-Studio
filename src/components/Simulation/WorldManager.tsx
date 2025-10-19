/**
 * World Manager Component
 * 
 * Manages robots, obstacles, and environment settings in the simulation
 */

import React, { useState } from 'react';
import './WorldManager.css';
import { simulationService } from '../../services/simulation-service';

export const WorldManager: React.FC = () => {
  const [robotName, setRobotName] = useState('');
  const [robotModel, setRobotModel] = useState<'differential_drive' | 'ackermann' | 'legged' | 'aerial' | 'marine'>('differential_drive');
  const [obstacleName, setObstacleName] = useState('');
  const [obstacleShape, setObstacleShape] = useState<'box' | 'sphere' | 'cylinder' | 'cone'>('box');
  const [obstacleX, setObstacleX] = useState(0);
  const [obstacleY, setObstacleY] = useState(0);
  const [obstacleZ, setObstacleZ] = useState(0);

  const handleAddRobot = async () => {
    if (!robotName.trim()) {
      alert('Please enter a robot name');
      return;
    }

    try {
      await simulationService.addRobot({
        id: `robot_${Date.now()}`,
        name: robotName,
        model: robotModel,
        max_linear_velocity: 2.0,
        max_angular_velocity: 1.5,
      });

      setRobotName('');
      alert(`Robot "${robotName}" added successfully!`);
    } catch (error) {
      console.error('Failed to add robot:', error);
      alert('Failed to add robot');
    }
  };

  const handleAddObstacle = async () => {
    if (!obstacleName.trim()) {
      alert('Please enter an obstacle name');
      return;
    }

    try {
      await simulationService.addObstacle({
        id: obstacleName,
        shape: obstacleShape,
        position: [obstacleX, obstacleY, obstacleZ],
        rotation: [0, 0, 0],
        scale: [1, 1, 1],
      });

      setObstacleName('');
      alert(`Obstacle "${obstacleName}" added successfully!`);
    } catch (error) {
      console.error('Failed to add obstacle:', error);
      alert('Failed to add obstacle');
    }
  };

  return (
    <div className="world-manager">
      <div className="world-header">
        <h2>🌍 World Manager</h2>
        <p>Configure robots and obstacles in your simulation environment</p>
      </div>

      <div className="world-sections">
        {/* Add Robot Section */}
        <div className="world-section">
          <h3>🤖 Add Robot</h3>
          
          <div className="form-group">
            <label>Robot Name</label>
            <input
              type="text"
              placeholder="Enter robot name..."
              value={robotName}
              onChange={(e) => setRobotName(e.target.value)}
              className="form-input"
            />
          </div>

          <div className="form-group">
            <label>Robot Model</label>
            <select
              value={robotModel}
              onChange={(e) => setRobotModel(e.target.value as any)}
              className="form-select"
            >
              <option value="differential_drive">Differential Drive</option>
              <option value="ackermann">Ackermann Steering</option>
              <option value="legged">Legged Robot</option>
              <option value="aerial">Aerial (Drone)</option>
              <option value="marine">Marine Robot</option>
            </select>
          </div>

          <div className="model-info">
            {robotModel === 'differential_drive' && (
              <p>📖 Two-wheeled mobile robot with independent wheel control</p>
            )}
            {robotModel === 'ackermann' && (
              <p>📖 Car-like robot with Ackermann steering geometry</p>
            )}
            {robotModel === 'legged' && (
              <p>📖 Multi-legged robot for complex terrain navigation</p>
            )}
            {robotModel === 'aerial' && (
              <p>📖 Quadcopter or fixed-wing aerial vehicle</p>
            )}
            {robotModel === 'marine' && (
              <p>📖 Underwater or surface marine vehicle</p>
            )}
          </div>

          <button 
            className="add-btn robot-add"
            onClick={handleAddRobot}
          >
            ➕ Add Robot
          </button>
        </div>

        {/* Add Obstacle Section */}
        <div className="world-section">
          <h3>📦 Add Obstacle</h3>
          
          <div className="form-group">
            <label>Obstacle Name</label>
            <input
              type="text"
              placeholder="Enter obstacle name..."
              value={obstacleName}
              onChange={(e) => setObstacleName(e.target.value)}
              className="form-input"
            />
          </div>

          <div className="form-group">
            <label>Shape</label>
            <div className="shape-selector">
              <button
                className={`shape-btn ${obstacleShape === 'box' ? 'active' : ''}`}
                onClick={() => setObstacleShape('box')}
              >
                📦 Box
              </button>
              <button
                className={`shape-btn ${obstacleShape === 'sphere' ? 'active' : ''}`}
                onClick={() => setObstacleShape('sphere')}
              >
                ⚪ Sphere
              </button>
              <button
                className={`shape-btn ${obstacleShape === 'cylinder' ? 'active' : ''}`}
                onClick={() => setObstacleShape('cylinder')}
              >
                🛢️ Cylinder
              </button>
              <button
                className={`shape-btn ${obstacleShape === 'cone' ? 'active' : ''}`}
                onClick={() => setObstacleShape('cone')}
              >
                🔺 Cone
              </button>
            </div>
          </div>

          <div className="form-group">
            <label>Position</label>
            <div className="position-grid">
              <div className="position-input-group">
                <label>X</label>
                <input
                  type="number"
                  step="0.1"
                  value={obstacleX}
                  onChange={(e) => setObstacleX(parseFloat(e.target.value))}
                  className="form-input-small"
                />
              </div>
              <div className="position-input-group">
                <label>Y</label>
                <input
                  type="number"
                  step="0.1"
                  value={obstacleY}
                  onChange={(e) => setObstacleY(parseFloat(e.target.value))}
                  className="form-input-small"
                />
              </div>
              <div className="position-input-group">
                <label>Z</label>
                <input
                  type="number"
                  step="0.1"
                  value={obstacleZ}
                  onChange={(e) => setObstacleZ(parseFloat(e.target.value))}
                  className="form-input-small"
                />
              </div>
            </div>
          </div>

          <button 
            className="add-btn obstacle-add"
            onClick={handleAddObstacle}
          >
            ➕ Add Obstacle
          </button>
        </div>

        {/* Environment Settings */}
        <div className="world-section">
          <h3>⚙️ Environment Settings</h3>
          
          <div className="form-group">
            <label>Gravity</label>
            <select className="form-select">
              <option value="earth">Earth (9.81 m/s²)</option>
              <option value="moon">Moon (1.62 m/s²)</option>
              <option value="mars">Mars (3.71 m/s²)</option>
              <option value="zero">Zero Gravity</option>
              <option value="custom">Custom</option>
            </select>
          </div>

          <div className="form-group">
            <label>Ground Plane</label>
            <div className="checkbox-group">
              <input type="checkbox" id="ground-enabled" defaultChecked />
              <label htmlFor="ground-enabled">Enable Ground Plane</label>
            </div>
          </div>

          <div className="form-group">
            <label>Lighting</label>
            <select className="form-select">
              <option value="sunny">Sunny</option>
              <option value="cloudy">Cloudy</option>
              <option value="night">Night</option>
              <option value="indoor">Indoor</option>
            </select>
          </div>

          <button className="apply-settings-btn">
            ✅ Apply Settings
          </button>
        </div>

        {/* Presets */}
        <div className="world-section">
          <h3>🎨 Load Preset World</h3>
          
          <div className="preset-grid">
            <button className="preset-btn">
              <span className="preset-icon">🏭</span>
              <span className="preset-name">Warehouse</span>
            </button>
            <button className="preset-btn">
              <span className="preset-icon">🏞️</span>
              <span className="preset-name">Outdoor</span>
            </button>
            <button className="preset-btn">
              <span className="preset-icon">🏢</span>
              <span className="preset-name">Office</span>
            </button>
            <button className="preset-btn">
              <span className="preset-icon">🌊</span>
              <span className="preset-name">Underwater</span>
            </button>
          </div>
        </div>
      </div>
    </div>
  );
};

export default WorldManager;

