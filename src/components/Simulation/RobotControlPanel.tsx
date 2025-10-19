/**
 * Robot Control Panel
 * 
 * Control interface for robots in the simulation
 * Supports velocity control, position setting, and path planning
 */

import React, { useState } from 'react';
import './RobotControlPanel.css';
import { simulationService } from '../../services/simulation-service';

interface RobotControlPanelProps {
  robotId: string | null;
  robotName?: string;
}

export const RobotControlPanel: React.FC<RobotControlPanelProps> = ({ robotId, robotName }) => {
  const [linearVelocity, setLinearVelocity] = useState(0);
  const [angularVelocity, setAngularVelocity] = useState(0);
  const [targetX, setTargetX] = useState(0);
  const [targetY, setTargetY] = useState(0);
  const [targetZ, setTargetZ] = useState(0);

  const handleControlUpdate = async () => {
    if (!robotId) return;

    try {
      await simulationService.controlRobot({
        robot_id: robotId,
        linear_velocity: linearVelocity,
        angular_velocity: angularVelocity,
      });
    } catch (error) {
      console.error('Failed to control robot:', error);
    }
  };

  const handleStop = () => {
    setLinearVelocity(0);
    setAngularVelocity(0);
    if (robotId) {
      simulationService.controlRobot({
        robot_id: robotId,
        linear_velocity: 0,
        angular_velocity: 0,
      });
    }
  };

  const handlePresetCommand = (linear: number, angular: number) => {
    setLinearVelocity(linear);
    setAngularVelocity(angular);
    if (robotId) {
      simulationService.controlRobot({
        robot_id: robotId,
        linear_velocity: linear,
        angular_velocity: angular,
      });
    }
  };

  if (!robotId) {
    return (
      <div className="robot-control-panel">
        <div className="no-robot-selected">
          <div className="no-robot-icon">🤖</div>
          <p>No robot selected</p>
          <span>Select a robot from the simulation to control it</span>
        </div>
      </div>
    );
  }

  return (
    <div className="robot-control-panel">
      <div className="control-header">
        <div className="robot-info">
          <span className="robot-icon">🤖</span>
          <div>
            <h3>Robot Control</h3>
            <p className="robot-name">{robotName || robotId}</p>
          </div>
        </div>
      </div>

      <div className="control-sections">
        {/* Velocity Control */}
        <div className="control-section">
          <h4>Velocity Control</h4>
          
          <div className="control-group">
            <label>
              Linear Velocity (m/s)
              <span className="value-display">{linearVelocity.toFixed(2)}</span>
            </label>
            <input
              type="range"
              min="-2"
              max="2"
              step="0.1"
              value={linearVelocity}
              onChange={(e) => setLinearVelocity(parseFloat(e.target.value))}
              className="velocity-slider"
            />
          </div>

          <div className="control-group">
            <label>
              Angular Velocity (rad/s)
              <span className="value-display">{angularVelocity.toFixed(2)}</span>
            </label>
            <input
              type="range"
              min="-1.5"
              max="1.5"
              step="0.1"
              value={angularVelocity}
              onChange={(e) => setAngularVelocity(parseFloat(e.target.value))}
              className="velocity-slider"
            />
          </div>

          <div className="control-buttons">
            <button 
              className="apply-btn"
              onClick={handleControlUpdate}
            >
              Apply Control
            </button>
            <button 
              className="stop-btn"
              onClick={handleStop}
            >
              🛑 Stop
            </button>
          </div>
        </div>

        {/* Quick Commands */}
        <div className="control-section">
          <h4>Quick Commands</h4>
          
          <div className="command-grid">
            <button 
              className="command-btn"
              onClick={() => handlePresetCommand(1, 0)}
            >
              ⬆️ Forward
            </button>
            <button 
              className="command-btn"
              onClick={() => handlePresetCommand(-1, 0)}
            >
              ⬇️ Backward
            </button>
            <button 
              className="command-btn"
              onClick={() => handlePresetCommand(0, 1)}
            >
              ⬅️ Turn Left
            </button>
            <button 
              className="command-btn"
              onClick={() => handlePresetCommand(0, -1)}
            >
              ➡️ Turn Right
            </button>
            <button 
              className="command-btn"
              onClick={() => handlePresetCommand(1, 0.5)}
            >
              ↗️ Forward Left
            </button>
            <button 
              className="command-btn"
              onClick={() => handlePresetCommand(1, -0.5)}
            >
              ↘️ Forward Right
            </button>
          </div>
        </div>

        {/* Position Control */}
        <div className="control-section">
          <h4>Target Position</h4>
          
          <div className="position-inputs">
            <div className="position-input">
              <label>X (m)</label>
              <input
                type="number"
                step="0.1"
                value={targetX}
                onChange={(e) => setTargetX(parseFloat(e.target.value))}
                className="position-field"
              />
            </div>
            <div className="position-input">
              <label>Y (m)</label>
              <input
                type="number"
                step="0.1"
                value={targetY}
                onChange={(e) => setTargetY(parseFloat(e.target.value))}
                className="position-field"
              />
            </div>
            <div className="position-input">
              <label>Z (m)</label>
              <input
                type="number"
                step="0.1"
                value={targetZ}
                onChange={(e) => setTargetZ(parseFloat(e.target.value))}
                className="position-field"
              />
            </div>
          </div>

          <button className="goto-btn">
            🎯 Go to Position
          </button>
        </div>

        {/* Path Planning */}
        <div className="control-section">
          <h4>Path Planning</h4>
          
          <div className="path-options">
            <select className="path-algorithm">
              <option value="astar">A* Algorithm</option>
              <option value="rrt">RRT</option>
              <option value="dijkstra">Dijkstra</option>
              <option value="vnc">VNC Navigator</option>
            </select>
            
            <button className="plan-btn">
              🗺️ Plan Path
            </button>
          </div>

          <div className="path-info">
            <span>No active path</span>
          </div>
        </div>
      </div>
    </div>
  );
};

export default RobotControlPanel;

