import React, { useEffect, useState } from 'react';
import Viewer from './viewer/Viewer';
import { solve, submitPath, subscribeSSE } from './api';
import './styles.css';

/**
 * Univarm Advanced - Production-Ready Path Planning
 * 
 * Features:
 * - Real Rust backend with solver API
 * - SSE streaming for real-time telemetry
 * - Drive simulation with wheel updates
 * - ROS 2 C++ integration ready
 * - gRPC planner support
 */
export function UnivarmAdvancedApp() {
  const [start, setStart] = useState<[number, number]>([0, 0]);
  const [goal, setGoal] = useState<[number, number]>([5, 3]);
  const [path, setPath] = useState<{ x: number; y: number }[]>([]);
  const [wheels, setWheels] = useState<any>(null);
  const [job, setJob] = useState<string | undefined>();
  const [status, setStatus] = useState<string>('Ready');
  const [connected, setConnected] = useState(false);

  useEffect(() => {
    const close = subscribeSSE((kind, data) => {
      if (kind === 'wheels') {
        setWheels(data);
        setConnected(true);
      }
    });
    return close;
  }, []);

  async function onSolve() {
    try {
      setStatus('Solving path...');
      const r = await solve(start, goal, 200);
      setPath(r.path);
      setStatus(`✓ Path solved - ${r.path.length} points`);
    } catch (error) {
      setStatus(`✗ Error: ${error instanceof Error ? error.message : 'Unknown error'}`);
      console.error('Solve error:', error);
    }
  }

  async function onDrive() {
    if (path.length === 0) {
      setStatus('⚠ Please solve a path first');
      return;
    }
    try {
      setStatus('Starting drive...');
      const r = await submitPath(path);
      setJob(r.job);
      setStatus(`✓ Drive started - Job: ${r.job}`);
    } catch (error) {
      setStatus(`✗ Error: ${error instanceof Error ? error.message : 'Unknown error'}`);
      console.error('Drive error:', error);
    }
  }

  return (
    <div className="univarm-advanced-app">
      <div className="app-header">
        <h2>
          <span style={{ fontSize: 28 }}>🦀</span> Univarm Advanced
        </h2>
        <div className="status-badges">
          <span className={`badge ${connected ? 'connected' : 'disconnected'}`}>
            {connected ? '● Connected' : '○ Disconnected'}
          </span>
          <span className="badge">Rust Backend</span>
          <span className="badge">SSE Streaming</span>
        </div>
      </div>

      <div className="main-grid">
        <div className="control-panel">
          <div className="section">
            <h3>Path Planning</h3>
            
            <div className="input-group">
              <label>
                <span className="label-text">Start Position</span>
                <div className="input-row">
                  <input
                    type="number"
                    value={start[0]}
                    onChange={(e) => setStart([+e.target.value, start[1]])}
                    placeholder="X"
                  />
                  <input
                    type="number"
                    value={start[1]}
                    onChange={(e) => setStart([start[0], +e.target.value])}
                    placeholder="Y"
                  />
                </div>
              </label>
            </div>

            <div className="input-group">
              <label>
                <span className="label-text">Goal Position</span>
                <div className="input-row">
                  <input
                    type="number"
                    value={goal[0]}
                    onChange={(e) => setGoal([+e.target.value, goal[1]])}
                    placeholder="X"
                  />
                  <input
                    type="number"
                    value={goal[1]}
                    onChange={(e) => setGoal([goal[0], +e.target.value])}
                    placeholder="Y"
                  />
                </div>
              </label>
            </div>

            <div className="button-row">
              <button onClick={onSolve} className="btn-primary">
                ⋋ Solve Path
              </button>
              <button onClick={onDrive} className="btn-secondary" disabled={path.length === 0}>
                ▶ Execute Drive
              </button>
            </div>

            <div className="status-display">
              <strong>Status:</strong> {status}
            </div>
          </div>

          <div className="section">
            <h3>Execution Info</h3>
            <div className="info-row">
              <span>Job ID:</span>
              <code>{job ?? '—'}</code>
            </div>
            <div className="info-row">
              <span>Path Points:</span>
              <code>{path.length}</code>
            </div>
            <div className="info-row">
              <span>SSE Status:</span>
              <span className={`led ${connected ? 'led-green' : 'led-gray'}`}></span>
            </div>
          </div>

          <div className="section">
            <h3>Features</h3>
            <ul className="feature-list">
              <li>✅ Real-time path solving</li>
              <li>✅ SSE wheel telemetry</li>
              <li>✅ Constraint satisfaction monitoring</li>
              <li>✅ ROS 2 compatible</li>
              <li>✅ gRPC planner support</li>
            </ul>
          </div>
        </div>

        <div className="viewer-panel">
          <div className="viewer-container">
            <Viewer wheels={wheels} />
          </div>
          {wheels && (
            <div className="telemetry">
              <h4>Live Telemetry</h4>
              <pre>{JSON.stringify(wheels, null, 2)}</pre>
            </div>
          )}
        </div>
      </div>
    </div>
  );
}

export default UnivarmAdvancedApp;

