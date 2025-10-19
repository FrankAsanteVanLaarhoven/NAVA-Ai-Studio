/**
 * Simulation Panel
 * 
 * Main panel component that combines Gazebo simulation,
 * robot control, and world management
 */

import React, { useState } from 'react';
import './SimulationPanel.css';
import { GazeboSimulation } from './GazeboSimulation';
import { RobotControlPanel } from './RobotControlPanel';
import { WorldManager } from './WorldManager';
import { DatasetIntegrationPanel } from './DatasetIntegrationPanel';

type ActiveTab = 'simulation' | 'control' | 'world' | 'datasets';

export const SimulationPanel: React.FC = () => {
  const [activeTab, setActiveTab] = useState<ActiveTab>('simulation');
  const [selectedRobot, setSelectedRobot] = useState<string | null>(null);

  return (
    <div className="simulation-panel">
      <div className="simulation-tabs">
        <button
          className={`tab-btn ${activeTab === 'simulation' ? 'active' : ''}`}
          onClick={() => setActiveTab('simulation')}
        >
          <span className="tab-icon">🎮</span>
          Simulation
        </button>
        <button
          className={`tab-btn ${activeTab === 'control' ? 'active' : ''}`}
          onClick={() => setActiveTab('control')}
        >
          <span className="tab-icon">🎛️</span>
          Robot Control
        </button>
        <button
          className={`tab-btn ${activeTab === 'world' ? 'active' : ''}`}
          onClick={() => setActiveTab('world')}
        >
          <span className="tab-icon">🌍</span>
          World Manager
        </button>
        <button
          className={`tab-btn ${activeTab === 'datasets' ? 'active' : ''}`}
          onClick={() => setActiveTab('datasets')}
        >
          <span className="tab-icon">🌟</span>
          Datasets + NIF
        </button>
      </div>

      <div className="simulation-tab-content">
        {activeTab === 'simulation' && (
          <GazeboSimulation />
        )}
        {activeTab === 'control' && (
          <RobotControlPanel 
            robotId={selectedRobot}
          />
        )}
        {activeTab === 'world' && (
          <WorldManager />
        )}
        {activeTab === 'datasets' && (
          <DatasetIntegrationPanel />
        )}
      </div>
    </div>
  );
};

export default SimulationPanel;

