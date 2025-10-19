/**
 * NAVΛ SIM Component
 * 
 * Main simulation interface powered by Rust backend
 * Provides 3D visualization, robot control, and world management
 */

import React, { useState, useEffect, useRef } from 'react';
import './GazeboSimulation.css';
import { simulationService, SimulationState } from '../../services/simulation-service';

export const GazeboSimulation: React.FC = () => {
  const [state, setState] = useState<SimulationState | null>(null);
  const [isConnected, setIsConnected] = useState(false);
  const [selectedRobot, setSelectedRobot] = useState<string | null>(null);
  const [viewMode, setViewMode] = useState<'3d' | 'top' | 'side'>('3d');
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const [cameraAngle, setCameraAngle] = useState({ x: 45, y: 45 });

  useEffect(() => {
    // Start polling for simulation state
    simulationService.startPolling(50);

    // Subscribe to state updates
    const unsubscribe = simulationService.onStateUpdate((newState) => {
      setState(newState);
      setIsConnected(true);
    });

    // Test connection
    simulationService.getState()
      .then(() => setIsConnected(true))
      .catch(() => setIsConnected(false));

    return () => {
      simulationService.stopPolling();
      unsubscribe();
    };
  }, []);

  useEffect(() => {
    // Render the 3D scene
    if (canvasRef.current && state) {
      renderScene();
    }
  }, [state, viewMode, cameraAngle]);

  const renderScene = () => {
    const canvas = canvasRef.current;
    if (!canvas || !state) return;

    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    // Clear canvas
    ctx.fillStyle = '#0a1735';
    ctx.fillRect(0, 0, canvas.width, canvas.height);

    // Draw grid
    drawGrid(ctx);

    // Draw obstacles
    state.obstacles.forEach(obstacle => {
      drawObstacle(ctx, obstacle);
    });

    // Draw robots
    state.robots.forEach(robot => {
      drawRobot(ctx, robot, robot.id === selectedRobot);
    });

    // Draw axes
    drawAxes(ctx);
  };

  const drawGrid = (ctx: CanvasRenderingContext2D) => {
    const size = 100;
    const step = 10;
    
    ctx.strokeStyle = 'rgba(59, 130, 246, 0.1)';
    ctx.lineWidth = 1;

    // Convert world coordinates to screen coordinates
    const toScreen = (x: number, y: number): [number, number] => {
      const centerX = ctx.canvas.width / 2;
      const centerY = ctx.canvas.height / 2;
      const scale = 5;
      
      if (viewMode === '3d') {
        // Isometric projection
        const screenX = centerX + (x - y) * scale * Math.cos(Math.PI / 6);
        const screenY = centerY + (x + y) * scale * Math.sin(Math.PI / 6);
        return [screenX, screenY];
      } else if (viewMode === 'top') {
        return [centerX + x * scale, centerY + y * scale];
      } else {
        return [centerX + x * scale, centerY - y * scale];
      }
    };

    // Draw grid lines
    for (let i = -size; i <= size; i += step) {
      ctx.beginPath();
      const [x1, y1] = toScreen(i, -size);
      const [x2, y2] = toScreen(i, size);
      ctx.moveTo(x1, y1);
      ctx.lineTo(x2, y2);
      ctx.stroke();

      ctx.beginPath();
      const [x3, y3] = toScreen(-size, i);
      const [x4, y4] = toScreen(size, i);
      ctx.moveTo(x3, y3);
      ctx.lineTo(x4, y4);
      ctx.stroke();
    }
  };

  const drawAxes = (ctx: CanvasRenderingContext2D) => {
    const centerX = ctx.canvas.width / 2;
    const centerY = ctx.canvas.height / 2;
    const scale = 50;

    // X axis (red)
    ctx.strokeStyle = '#ef4444';
    ctx.lineWidth = 3;
    ctx.beginPath();
    ctx.moveTo(centerX, centerY);
    ctx.lineTo(centerX + scale, centerY);
    ctx.stroke();

    // Y axis (green)
    ctx.strokeStyle = '#22c55e';
    ctx.beginPath();
    ctx.moveTo(centerX, centerY);
    ctx.lineTo(centerX, centerY - scale);
    ctx.stroke();

    // Z axis (blue) - if 3D view
    if (viewMode === '3d') {
      ctx.strokeStyle = '#3b82f6';
      ctx.beginPath();
      ctx.moveTo(centerX, centerY);
      ctx.lineTo(centerX - scale * 0.7, centerY + scale * 0.7);
      ctx.stroke();
    }
  };

  const drawObstacle = (ctx: CanvasRenderingContext2D, obstacle: any) => {
    const centerX = ctx.canvas.width / 2;
    const centerY = ctx.canvas.height / 2;
    const scale = 5;

    const [x, y, z] = obstacle.position;
    const [scaleX, scaleY, scaleZ] = obstacle.scale;

    let screenX, screenY;
    if (viewMode === '3d') {
      screenX = centerX + (x - y) * scale * Math.cos(Math.PI / 6);
      screenY = centerY + (x + y) * scale * Math.sin(Math.PI / 6) - z * scale;
    } else if (viewMode === 'top') {
      screenX = centerX + x * scale;
      screenY = centerY + y * scale;
    } else {
      screenX = centerX + x * scale;
      screenY = centerY - z * scale;
    }

    ctx.fillStyle = 'rgba(239, 68, 68, 0.6)';
    ctx.strokeStyle = '#ef4444';
    ctx.lineWidth = 2;

    if (obstacle.shape === 'box') {
      const w = scaleX * scale;
      const h = scaleZ * scale;
      ctx.fillRect(screenX - w / 2, screenY - h / 2, w, h);
      ctx.strokeRect(screenX - w / 2, screenY - h / 2, w, h);
    } else if (obstacle.shape === 'sphere') {
      const r = scaleX * scale;
      ctx.beginPath();
      ctx.arc(screenX, screenY, r, 0, Math.PI * 2);
      ctx.fill();
      ctx.stroke();
    }
  };

  const drawRobot = (ctx: CanvasRenderingContext2D, robot: any, selected: boolean) => {
    const centerX = ctx.canvas.width / 2;
    const centerY = ctx.canvas.height / 2;
    const scale = 5;

    const [x, y, z] = robot.position;

    let screenX, screenY;
    if (viewMode === '3d') {
      screenX = centerX + (x - y) * scale * Math.cos(Math.PI / 6);
      screenY = centerY + (x + y) * scale * Math.sin(Math.PI / 6) - z * scale;
    } else if (viewMode === 'top') {
      screenX = centerX + x * scale;
      screenY = centerY + y * scale;
    } else {
      screenX = centerX + x * scale;
      screenY = centerY - z * scale;
    }

    // Robot body
    ctx.fillStyle = selected ? '#22c55e' : '#3b82f6';
    ctx.strokeStyle = selected ? '#16a34a' : '#2563eb';
    ctx.lineWidth = selected ? 3 : 2;

    const size = 15;
    ctx.fillRect(screenX - size / 2, screenY - size / 2, size, size);
    ctx.strokeRect(screenX - size / 2, screenY - size / 2, size, size);

    // Robot label
    ctx.fillStyle = '#ffffff';
    ctx.font = '12px monospace';
    ctx.textAlign = 'center';
    ctx.fillText(robot.name, screenX, screenY - size);

    // Selection indicator
    if (selected) {
      ctx.strokeStyle = '#22c55e';
      ctx.lineWidth = 2;
      ctx.setLineDash([5, 5]);
      ctx.beginPath();
      ctx.arc(screenX, screenY, size, 0, Math.PI * 2);
      ctx.stroke();
      ctx.setLineDash([]);
    }
  };

  const handleStart = async () => {
    try {
      await simulationService.start();
    } catch (error) {
      console.error('Failed to start simulation:', error);
    }
  };

  const handlePause = async () => {
    try {
      await simulationService.pause();
    } catch (error) {
      console.error('Failed to pause simulation:', error);
    }
  };

  const handleReset = async () => {
    try {
      await simulationService.reset();
      setSelectedRobot(null);
    } catch (error) {
      console.error('Failed to reset simulation:', error);
    }
  };

  return (
    <div className="gazebo-simulation">
      <div className="simulation-header">
        <div className="simulation-title">
          <span className="simulation-icon">🤖</span>
          <h2>NAVΛ SIM</h2>
          <span className={`connection-status ${isConnected ? 'connected' : 'disconnected'}`}>
            {isConnected ? '🟢 Connected' : '🔴 Disconnected'}
          </span>
        </div>
        
        <div className="simulation-controls">
          <button 
            onClick={handleStart}
            className="control-btn start"
            disabled={!isConnected || (state?.is_running ?? false)}
          >
            ▶ Start
          </button>
          <button 
            onClick={handlePause}
            className="control-btn pause"
            disabled={!isConnected || !(state?.is_running ?? true)}
          >
            ⏸ Pause
          </button>
          <button 
            onClick={handleReset}
            className="control-btn reset"
            disabled={!isConnected}
          >
            🔄 Reset
          </button>
        </div>
      </div>

      <div className="simulation-content">
        <div className="viewport-container">
          <div className="viewport-header">
            <div className="view-modes">
              <button 
                className={`view-mode-btn ${viewMode === '3d' ? 'active' : ''}`}
                onClick={() => setViewMode('3d')}
              >
                3D View
              </button>
              <button 
                className={`view-mode-btn ${viewMode === 'top' ? 'active' : ''}`}
                onClick={() => setViewMode('top')}
              >
                Top View
              </button>
              <button 
                className={`view-mode-btn ${viewMode === 'side' ? 'active' : ''}`}
                onClick={() => setViewMode('side')}
              >
                Side View
              </button>
            </div>
            
            <div className="simulation-time">
              Time: {state?.time.toFixed(2) ?? '0.00'}s
            </div>
          </div>

          <canvas 
            ref={canvasRef}
            width={1200}
            height={700}
            className="simulation-canvas"
          />

          <div className="viewport-overlay">
            <div className="stats-panel">
              <div className="stat">
                <span className="stat-label">Robots:</span>
                <span className="stat-value">{state?.robots.length ?? 0}</span>
              </div>
              <div className="stat">
                <span className="stat-label">Obstacles:</span>
                <span className="stat-value">{state?.obstacles.length ?? 0}</span>
              </div>
              <div className="stat">
                <span className="stat-label">Status:</span>
                <span className="stat-value">
                  {state?.is_running ? '🟢 Running' : '⏸ Paused'}
                </span>
              </div>
            </div>
          </div>
        </div>

        <div className="simulation-sidebar">
          <div className="robots-panel panel">
            <h3>Robots</h3>
            <div className="robot-list">
              {state?.robots.map(robot => (
                <div 
                  key={robot.id}
                  className={`robot-item ${selectedRobot === robot.id ? 'selected' : ''}`}
                  onClick={() => setSelectedRobot(robot.id)}
                >
                  <div className="robot-name">🤖 {robot.name}</div>
                  <div className="robot-position">
                    Position: ({robot.position[0].toFixed(2)}, {robot.position[1].toFixed(2)}, {robot.position[2].toFixed(2)})
                  </div>
                </div>
              )) ?? <div className="empty-message">No robots in simulation</div>}
            </div>
          </div>

          <div className="obstacles-panel panel">
            <h3>Obstacles</h3>
            <div className="obstacle-list">
              {state?.obstacles.map(obstacle => (
                <div key={obstacle.id} className="obstacle-item">
                  <div className="obstacle-name">
                    {obstacle.shape === 'box' ? '📦' : '⚪'} {obstacle.id}
                  </div>
                  <button 
                    className="remove-btn"
                    onClick={() => simulationService.removeObstacle(obstacle.id)}
                  >
                    ✕
                  </button>
                </div>
              )) ?? <div className="empty-message">No obstacles in world</div>}
            </div>
          </div>
        </div>
      </div>
    </div>
  );
};

export default GazeboSimulation;

