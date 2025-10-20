/**
 * Simulation Panel
 * 
 * Main panel component with:
 * - Gazebo 3D: Integrated simulation with Models, Robots, Worlds, and Datasets
 * - Legacy Sim: Original simulation interface (for reference/fallback)
 */

import React, { useState } from 'react';
import './SimulationPanel.css';
import { GazeboSimulation } from './GazeboSimulation';
import { GazeboSimulationEnhanced } from './GazeboSimulationEnhanced';

type ActiveTab = 'simulation' | 'gazebo';

export const SimulationPanel: React.FC = () => {
  const [activeTab, setActiveTab] = useState<ActiveTab>('gazebo');

  return (
    <div className="simulation-panel">
      <div className="simulation-tabs">
        <button
          className={`tab-btn ${activeTab === 'gazebo' ? 'active' : ''}`}
          onClick={() => setActiveTab('gazebo')}
        >
          <span className="tab-icon">🚀</span>
          Gazebo 3D
        </button>
        <button
          className={`tab-btn ${activeTab === 'simulation' ? 'active' : ''}`}
          onClick={() => setActiveTab('simulation')}
        >
          <span className="tab-icon">🎮</span>
          Legacy Sim
        </button>
      </div>

      <div className="simulation-tab-content">
        {activeTab === 'gazebo' && (
          <GazeboSimulationEnhanced />
        )}
        {activeTab === 'simulation' && (
          <GazeboSimulation />
        )}
      </div>
    </div>
  );
};

export default SimulationPanel;

