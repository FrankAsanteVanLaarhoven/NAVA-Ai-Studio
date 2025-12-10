import React from 'react';
import { Toolbar } from './ui/Toolbar';
import './styles.css';

/**
 * Univarm × NAVΛ Starter App
 * 
 * A minimal app with:
 * - One-click "Find optimal path (⋋)" button
 * - Multi-language codegen (Rust/C++/Python/TypeScript)
 * - Mock solver engine (swappable with real NAVΛ engine)
 */
export function UnivarmStarterApp() {
  return (
    <div className="univarm-starter-app">
      <div className="app-shell">
        <div className="left-panel">
          <div className="header">
            <h2 style={{ marginTop: 0, display: 'flex', alignItems: 'center', gap: 8 }}>
              <span style={{ fontSize: 24 }}>🔷</span>
              Univarm × NAVΛ
            </h2>
            <p className="muted">
              Path optimization & cross-language code generation
            </p>
            <div className="badge-container">
              <span className="badge">⋋ actions ready</span>
              <span className="badge">codegen enabled</span>
            </div>
          </div>
          <hr style={{ borderColor: '#1c2633', margin: '16px 0' }} />
          <Toolbar />
        </div>
        
        <div className="main-panel">
          <div className="info-section">
            <h3>🚀 Quick Start</h3>
            <ol>
              <li>Click <b>Find optimal path (⋋)</b> to run the solver</li>
              <li>Set start/goal coordinates and run</li>
              <li>Export generated code in Rust, C++, Python, or TypeScript</li>
            </ol>
          </div>

          <div className="info-section">
            <h3>🔧 API Reference</h3>
            <p className="muted">
              The solver API:
            </p>
            <pre className="code-block">
{`solveOptimalPath({
  start: {x, y, z},
  goal: {x, y, z},
  samples?: number,
  interpreter?: 'auto' | 'tauri-backend' | 'python' | 'wasm' | 'browser-sim',
  energyLandscape?: boolean
}): Promise<{
  points: Vec3[],
  cost: number,
  energy?: number,
  metadata?: { interpreter, executionTime, iterations }
}>`}
            </pre>
          </div>

          <div className="info-section">
            <h3>📚 Features</h3>
            <ul>
              <li>✅ <strong>Real NAVΛ Engine</strong> - Full Van Laarhoven Navigation Calculus support</li>
              <li>✅ <strong>Multiple Interpreters</strong> - Tauri, Python, WASM, Browser simulation</li>
              <li>✅ <strong>VNC Energy Landscape</strong> - Advanced path optimization</li>
              <li>✅ Manifest-backed actions (<code>λopt</code> prefix)</li>
              <li>✅ Real-time path solving with configurable parameters</li>
              <li>✅ Multi-language code generation (Rust, C++, Python, TypeScript)</li>
              <li>✅ Embedded in NAVΛ Studio IDE</li>
              <li>✅ Production-ready architecture</li>
            </ul>
          </div>
        </div>
      </div>
    </div>
  );
}

export default UnivarmStarterApp;

