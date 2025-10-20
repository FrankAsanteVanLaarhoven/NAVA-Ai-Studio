/**
 * Enhanced Gazebo Simulation Component
 * Full 3D Gazebo integration similar to The Construct platform
 * 
 * Features:
 * - Real-time 3D visualization with Three.js
 * - Robot spawning and control
 * - World/environment management
 * - Physics simulation
 * - Sensor visualization
 * - Model library
 */

import React, { useState, useEffect } from 'react';
import { 
  Play, Pause, RotateCcw, Plus, Trash2, Download, Upload, 
  Settings, Camera, Zap, Box, Globe, Bot, Eye, Target,
  Grid3x3, Maximize2, GitBranch, ChevronLeft, ChevronRight,
  ArrowUp, ArrowDown, ArrowLeft, ArrowRight, CircleDot,
  Package
} from 'lucide-react';
import './GazeboSimulationEnhanced.css';
import { Gazebo3DViewer } from './Gazebo3DViewer';
import { gazeboService, GazeboWorld, GazeboModel, GazeboRobot } from '../../services/gazebo-service';

type PanelTab = 'models' | 'robots' | 'worlds' | 'datasets';
type ViewMode = '3D' | 'Top' | 'Side' | 'Front';

export const GazeboSimulationEnhanced: React.FC = () => {
  const [connected, setConnected] = useState(false);
  const [worlds, setWorlds] = useState<GazeboWorld[]>([]);
  const [currentWorld, setCurrentWorld] = useState<GazeboWorld | null>(null);
  const [isPlaying, setIsPlaying] = useState(false);
  const [simulationTime, setSimulationTime] = useState(0);
  const [selectedModel, setSelectedModel] = useState<string | null>(null);
  const [selectedRobot, setSelectedRobot] = useState<string | null>(null);
  const [showSpawnMenu, setShowSpawnMenu] = useState(false);
  const [showWorldSelector, setShowWorldSelector] = useState(false);
  const [demoMode, setDemoMode] = useState(true); // Auto-demo on first load
  const [rightPanelTab, setRightPanelTab] = useState<PanelTab>('models'); // Integrated panel tabs
  const [sidebarCollapsed, setSidebarCollapsed] = useState(false); // Sidebar toggle
  const [viewMode, setViewMode] = useState<ViewMode>('3D'); // View mode
  const [showGrid, setShowGrid] = useState(true); // Grid visibility
  const [showAxes, setShowAxes] = useState(true); // Axes visibility
  const [robotVelocity, setRobotVelocity] = useState(1.0); // Robot control velocity

  useEffect(() => {
    // Initialize connection
    gazeboService.connect('ws://localhost:9090')
      .then(() => {
        setConnected(true);
        loadWorlds();
      })
      .catch(() => {
        console.log('Using local simulation mode');
        setConnected(true);
        loadWorlds();
      });

    // Subscribe to events
    gazeboService.on('simulation_update', handleSimulationUpdate);
    gazeboService.on('world_loaded', handleWorldLoaded);
    gazeboService.on('model_spawned', handleModelSpawned);

    return () => {
      gazeboService.off('simulation_update', handleSimulationUpdate);
      gazeboService.off('world_loaded', handleWorldLoaded);
      gazeboService.off('model_spawned', handleModelSpawned);
    };
  }, []);

  // Auto-start demo mode
  useEffect(() => {
    if (demoMode && currentWorld && connected) {
      startDemo();
    }
  }, [demoMode, currentWorld, connected]);

  const loadWorlds = () => {
    const availableWorlds = gazeboService.getWorlds();
    setWorlds(availableWorlds);
    if (availableWorlds.length > 0) {
      loadWorld(availableWorlds[0].id);
    }
  };

  const handleSimulationUpdate = (state: any) => {
    setSimulationTime(state.time);
  };

  const handleWorldLoaded = (world: GazeboWorld) => {
    setCurrentWorld(world);
  };

  const handleModelSpawned = (model: any) => {
    console.log('Model spawned:', model);
  };

  const loadWorld = async (worldId: string) => {
    try {
      const world = await gazeboService.loadWorld(worldId);
      setCurrentWorld(world);
      setIsPlaying(false);
      setSimulationTime(0);
    } catch (error) {
      console.error('Failed to load world:', error);
    }
  };

  const handlePlay = () => {
    gazeboService.play();
    setIsPlaying(true);
  };

  const handlePause = () => {
    gazeboService.pause();
    setIsPlaying(false);
  };

  const handleReset = () => {
    gazeboService.reset();
    setIsPlaying(false);
    setSimulationTime(0);
  };

  const spawnRobot = async (type: string) => {
    const robotTypes = {
      'differential_drive': {
        id: `robot_${Date.now()}`,
        name: `DiffDrive Robot ${Date.now()}`,
        type: 'differential_drive' as const,
        position: [Math.random() * 4 - 2, Math.random() * 4 - 2, 0.2] as [number, number, number],
        velocity: [0, 0, 0] as [number, number, number],
        sensors: [],
        actuators: [],
      },
      'quadcopter': {
        id: `quad_${Date.now()}`,
        name: `Quadcopter ${Date.now()}`,
        type: 'quadcopter' as const,
        position: [0, 0, 1] as [number, number, number],
        velocity: [0, 0, 0] as [number, number, number],
        sensors: [],
        actuators: [],
      },
      'manipulator': {
        id: `arm_${Date.now()}`,
        name: `Robot Arm ${Date.now()}`,
        type: 'manipulator' as const,
        position: [0, 0, 0.5] as [number, number, number],
        velocity: [0, 0, 0] as [number, number, number],
        sensors: [],
        actuators: [],
      },
    };

    const robot = robotTypes[type as keyof typeof robotTypes];
    if (robot) {
      await gazeboService.spawnRobot(robot);
      setShowSpawnMenu(false);
    }
  };

  const spawnObject = async (type: string) => {
    const objectTypes = {
      'box': {
        id: `box_${Date.now()}`,
        name: 'Box',
        type: 'object' as const,
        position: [Math.random() * 4 - 2, Math.random() * 4 - 2, 0.5] as [number, number, number],
        rotation: [0, 0, Math.random() * Math.PI] as [number, number, number],
        scale: [0.5, 0.5, 0.5] as [number, number, number],
        color: '#' + Math.floor(Math.random()*16777215).toString(16),
        physics: {
          mass: 1,
          friction: 0.8,
          restitution: 0.3,
        },
      },
      'sphere': {
        id: `sphere_${Date.now()}`,
        name: 'Sphere',
        type: 'object' as const,
        position: [Math.random() * 4 - 2, Math.random() * 4 - 2, 0.5] as [number, number, number],
        rotation: [0, 0, 0] as [number, number, number],
        scale: [0.3, 0.3, 0.3] as [number, number, number],
        color: '#' + Math.floor(Math.random()*16777215).toString(16),
        physics: {
          mass: 0.5,
          friction: 0.6,
          restitution: 0.7,
        },
      },
      'cylinder': {
        id: `cylinder_${Date.now()}`,
        name: 'Cylinder',
        type: 'object' as const,
        position: [Math.random() * 4 - 2, Math.random() * 4 - 2, 0.5] as [number, number, number],
        rotation: [0, 0, 0] as [number, number, number],
        scale: [0.3, 0.3, 0.6] as [number, number, number],
        color: '#' + Math.floor(Math.random()*16777215).toString(16),
        physics: {
          mass: 0.8,
          friction: 0.7,
          restitution: 0.4,
        },
      },
    };

    const object = objectTypes[type as keyof typeof objectTypes];
    if (object) {
      await gazeboService.spawnModel(object);
      setShowSpawnMenu(false);
    }
  };

  const deleteSelectedModel = async () => {
    if (selectedModel) {
      await gazeboService.deleteModel(selectedModel);
      setSelectedModel(null);
    }
  };

  const startDemo = async () => {
    console.log('🎬 Starting auto-demo mode...');
    
    // Wait a bit for world to fully load
    await new Promise(resolve => setTimeout(resolve, 500));

    try {
      // Spawn a demo robot
      await spawnRobot('differential_drive');
      
      // Spawn some demo objects
      await new Promise(resolve => setTimeout(resolve, 300));
      await spawnObject('box');
      
      await new Promise(resolve => setTimeout(resolve, 300));
      await spawnObject('sphere');
      
      await new Promise(resolve => setTimeout(resolve, 300));
      await spawnObject('cylinder');

      // Start simulation after spawning
      await new Promise(resolve => setTimeout(resolve, 500));
      handlePlay();
      
      setDemoMode(false); // Demo done
      console.log('✅ Demo mode complete!');
    } catch (error) {
      console.error('Demo mode error:', error);
    }
  };

  const exportWorld = () => {
    const sdf = gazeboService.exportWorldSDF();
    const blob = new Blob([sdf], { type: 'application/xml' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = `${currentWorld?.name.replace(/\s+/g, '_') || 'world'}.sdf`;
    document.body.appendChild(a);
    a.click();
    document.body.removeChild(a);
    URL.revokeObjectURL(url);
  };

  const importWorld = async () => {
    const input = document.createElement('input');
    input.type = 'file';
    input.accept = '.sdf,.world';
    input.onchange = async (e: any) => {
      const file = e.target.files[0];
      if (file) {
        const sdf = await file.text();
        try {
          const world = await gazeboService.importWorldSDF(sdf);
          await loadWorld(world.id);
        } catch (error) {
          console.error('Failed to import world:', error);
          alert('Failed to import world. Please check the SDF format.');
        }
      }
    };
    input.click();
  };

  return (
    <div className="gazebo-simulation-enhanced">
      {/* Full-Screen 3D Viewport */}
      <div className="viewport-container">
        <Gazebo3DViewer />
      </div>

      {/* Demo Mode Banner */}
      {demoMode && (
        <div className="demo-banner">
          <span className="demo-icon">🚀</span>
          <span>Loading demo: Spawning robot and objects...</span>
          <span className="demo-spinner">⏳</span>
        </div>
      )}

      {/* Top Toolbar - Thin Overlay */}
      <div className="top-toolbar">
        <div className="toolbar-left">
          <div className="sim-status">
            <div className={`status-dot ${connected ? '' : 'disconnected'}`}></div>
            <span>{currentWorld?.name || 'Empty World'}</span>
          </div>
        </div>

        <div className="toolbar-center">
          <div className="sim-controls">
            <button 
              className={`toolbar-btn ${isPlaying ? 'pause-btn' : 'play-btn'}`}
              onClick={isPlaying ? handlePause : handlePlay}
              title={isPlaying ? 'Pause (Space)' : 'Play (Space)'}
            >
              {isPlaying ? <Pause size={16} /> : <Play size={16} />}
              {isPlaying ? 'Pause' : 'Play'}
            </button>

            <button 
              className="toolbar-btn"
              onClick={handleReset}
              title="Reset (Ctrl+R)"
            >
              <RotateCcw size={16} />
              Reset
            </button>

            <button 
              className="toolbar-btn"
              onClick={() => setShowSpawnMenu(!showSpawnMenu)}
              title="Spawn Model"
            >
              <Plus size={16} />
              Spawn
            </button>
          </div>
        </div>

        <div className="toolbar-right">
          <div className="sim-time">
            Time: {simulationTime.toFixed(2)}s
          </div>
        </div>
      </div>

      {/* Right Sidebar - Collapsible Panel */}
      <div className={`right-sidebar ${sidebarCollapsed ? 'collapsed' : ''}`}>
        <button 
          className="sidebar-toggle"
          onClick={() => setSidebarCollapsed(!sidebarCollapsed)}
          title={sidebarCollapsed ? 'Show Panel' : 'Hide Panel'}
        >
          {sidebarCollapsed ? <ChevronLeft size={22} /> : <ChevronRight size={22} />}
        </button>

        {/* Panel Tabs */}
        <div className="panel-tabs">
          <button
            className={`panel-tab ${rightPanelTab === 'models' ? 'active' : ''}`}
            onClick={() => setRightPanelTab('models')}
            title="Scene Models"
          >
            <Box size={14} />
            <span>Models</span>
            <span className="tab-count">{currentWorld?.models.length || 0}</span>
          </button>
          <button
            className={`panel-tab ${rightPanelTab === 'robots' ? 'active' : ''}`}
            onClick={() => setRightPanelTab('robots')}
            title="Robot Control"
          >
            <Bot size={14} />
            <span>Robots</span>
            <span className="tab-count">{currentWorld?.models.filter(m => m.type === 'robot').length || 0}</span>
          </button>
          <button
            className={`panel-tab ${rightPanelTab === 'worlds' ? 'active' : ''}`}
            onClick={() => setRightPanelTab('worlds')}
            title="World Manager"
          >
            <Globe size={14} />
            <span>Worlds</span>
          </button>
          <button
            className={`panel-tab ${rightPanelTab === 'datasets' ? 'active' : ''}`}
            onClick={() => setRightPanelTab('datasets')}
            title="Datasets & NIF"
          >
            <GitBranch size={14} />
            <span>Datasets</span>
          </button>
        </div>

        {/* Panel Content */}
        <div className="panel-content">
          {/* Models Tab */}
          {rightPanelTab === 'models' && (
            <div className="panel-section models-section">
              <div className="section-header">
                <h3>Scene Models</h3>
                <button className="icon-btn" onClick={() => setShowSpawnMenu(true)} title="Add Model">
                  <Plus size={16} />
                </button>
              </div>
              <div className="models-list">
                {currentWorld?.models.map(model => (
                  <div
                    key={model.id}
                    className={`model-item ${selectedModel === model.id ? 'selected' : ''}`}
                    onClick={() => setSelectedModel(model.id)}
                  >
                    <div className="model-icon">
                      {model.type === 'robot' ? <Bot size={16} /> : 
                       model.type === 'light' ? <Zap size={16} /> :
                       model.type === 'sensor' ? <Eye size={16} /> :
                       <Box size={16} />}
                    </div>
                    <div className="model-info">
                      <div className="model-name">{model.name}</div>
                      <div className="model-type">{model.type}</div>
                    </div>
                    {selectedModel === model.id && (
                      <button 
                        className="delete-btn"
                        onClick={(e) => {
                          e.stopPropagation();
                          deleteSelectedModel();
                        }}
                        title="Delete"
                      >
                        <Trash2 size={12} />
                      </button>
                    )}
                  </div>
                ))}
              </div>

              {/* Selected Model Info */}
              {selectedModel && currentWorld && (() => {
                const model = currentWorld.models.find(m => m.id === selectedModel);
                return model ? (
                  <div className="model-details">
                    <h4>Model Details</h4>
                    <div className="detail-row">
                      <span>Name:</span>
                      <span>{model.name}</span>
                    </div>
                    <div className="detail-row">
                      <span>Type:</span>
                      <span>{model.type}</span>
                    </div>
                    <div className="detail-row">
                      <span>Position:</span>
                      <span>{model.position.map(v => v.toFixed(2)).join(', ')}</span>
                    </div>
                    <div className="detail-row">
                      <span>Rotation:</span>
                      <span>{model.rotation.map(v => (v * 180 / Math.PI).toFixed(1)).join('°, ')}°</span>
                    </div>
                  </div>
                ) : null;
              })()}
            </div>
          )}

          {/* Robots Tab */}
          {rightPanelTab === 'robots' && (
            <div className="panel-section robots-section">
              <div className="section-header">
                <h3>Robot Control</h3>
                <button className="icon-btn" onClick={() => spawnRobot('differential_drive')} title="Add Robot">
                  <Plus size={16} />
                </button>
              </div>

              {/* Robot List */}
              <div className="robots-list">
                {currentWorld?.models.filter(m => m.type === 'robot').map(robot => (
                  <div
                    key={robot.id}
                    className={`robot-item ${selectedRobot === robot.id ? 'selected' : ''}`}
                    onClick={() => setSelectedRobot(robot.id)}
                  >
                    <div className="robot-icon">
                      <Bot size={20} />
                    </div>
                    <div className="robot-info">
                      <div className="robot-name">{robot.name}</div>
                      <div className="robot-status">Active</div>
                    </div>
                  </div>
                ))}
                {(!currentWorld?.models.filter(m => m.type === 'robot').length) && (
                  <div className="empty-state">
                    <Bot size={32} opacity={0.3} />
                    <p>No robots in scene</p>
                    <button onClick={() => spawnRobot('differential_drive')} className="spawn-btn">
                      Spawn Robot
                    </button>
                  </div>
                )}
              </div>

              {/* Robot Controls */}
              {selectedRobot && currentWorld && (() => {
                const robot = currentWorld.models.find(m => m.id === selectedRobot);
                return robot ? (
                  <div className="robot-controls">
                    <h4>Robot Controls</h4>
                    <div className="control-section">
                      <label>Movement</label>
                      <div className="control-grid">
                        <button className="control-btn">↑ Forward</button>
                        <button className="control-btn">↓ Backward</button>
                        <button className="control-btn">← Left</button>
                        <button className="control-btn">→ Right</button>
                      </div>
                    </div>
                    <div className="control-section">
                      <label>Velocity</label>
                      <input type="range" min="0" max="10" defaultValue="5" className="velocity-slider" />
                    </div>
                    <div className="control-section">
                      <label>Position</label>
                      <div className="detail-row">
                        <span>X: {robot.position[0].toFixed(2)}</span>
                        <span>Y: {robot.position[1].toFixed(2)}</span>
                        <span>Z: {robot.position[2].toFixed(2)}</span>
                      </div>
                    </div>
                  </div>
                ) : null;
              })()}
            </div>
          )}

          {/* Worlds Tab */}
          {rightPanelTab === 'worlds' && (
            <div className="panel-section worlds-section">
              <div className="section-header">
                <h3>World Manager</h3>
                <button className="icon-btn" onClick={importWorld} title="Import World">
                  <Upload size={16} />
                </button>
              </div>

              {/* Current World Info */}
              {currentWorld && (
                <div className="current-world-info">
                  <h4>Current World</h4>
                  <div className="world-card active">
                    <Globe size={24} />
                    <div>
                      <div className="world-name">{currentWorld.name}</div>
                      <div className="world-stats">
                        <span>Models: {currentWorld.models.length}</span>
                        <span>Gravity: {currentWorld.gravity[2]}</span>
                      </div>
                    </div>
                  </div>
                </div>
              )}

              {/* Available Worlds */}
              <div className="worlds-list">
                <h4>Available Worlds</h4>
                {worlds.map(world => (
                  <div
                    key={world.id}
                    className={`world-item ${currentWorld?.id === world.id ? 'active' : ''}`}
                    onClick={() => loadWorld(world.id)}
                  >
                    <Globe size={16} />
                    <div className="world-info">
                      <div className="world-name">{world.name}</div>
                      <div className="world-models">{world.models.length} models</div>
                    </div>
                  </div>
                ))}
              </div>

              {/* World Settings */}
              <div className="world-settings">
                <h4>World Settings</h4>
                <div className="setting-row">
                  <label>Physics Engine</label>
                  <select className="setting-select">
                    <option>ODE</option>
                    <option>Bullet</option>
                    <option>Simbody</option>
                  </select>
                </div>
                <div className="setting-row">
                  <label>Gravity (m/s²)</label>
                  <input type="number" defaultValue="-9.81" step="0.1" className="setting-input" />
                </div>
                <div className="setting-row">
                  <label>Real-time Factor</label>
                  <input type="number" defaultValue="1.0" step="0.1" className="setting-input" />
                </div>
                <button className="save-btn">Save Settings</button>
              </div>
            </div>
          )}

          {/* Datasets Tab */}
          {rightPanelTab === 'datasets' && (
            <div className="panel-section datasets-section">
              <div className="section-header">
                <h3>Datasets & NIF</h3>
                <button className="icon-btn" title="Add Dataset">
                  <Plus size={16} />
                </button>
              </div>

              <div className="datasets-info">
                <p className="info-text">
                  Navigate and integrate datasets with your simulation environment.
                </p>

                {/* Dataset Types */}
                <div className="dataset-types">
                  <div className="dataset-type-card">
                    <Camera size={24} />
                    <h4>Camera Data</h4>
                    <p>RGB, Depth, Segmentation</p>
                    <button className="load-btn">Load</button>
                  </div>
                  <div className="dataset-type-card">
                    <Target size={24} />
                    <h4>Sensor Data</h4>
                    <p>LiDAR, IMU, GPS</p>
                    <button className="load-btn">Load</button>
                  </div>
                  <div className="dataset-type-card">
                    <GitBranch size={24} />
                    <h4>NIF Models</h4>
                    <p>Neural Implicit Fields</p>
                    <button className="load-btn">Load</button>
                  </div>
                </div>

                {/* Loaded Datasets */}
                <div className="loaded-datasets">
                  <h4>Loaded Datasets</h4>
                  <div className="empty-state">
                    <GitBranch size={32} opacity={0.3} />
                    <p>No datasets loaded</p>
                  </div>
                </div>
              </div>
            </div>
          )}
        </div>
      </div>

      {/* Floating Robot Control - Bottom Left */}
      {selectedRobot && (
        <div className="robot-control-float">
          <div className="robot-control-header">
            <h4>
              <Bot size={16} />
              Robot Control
            </h4>
          </div>
          <div className="robot-control-body">
            <div className="control-grid">
              <div></div>
              <button className="control-btn" onClick={() => console.log('Forward')} title="Forward">
                <ArrowUp size={20} />
              </button>
              <div></div>
              <button className="control-btn" onClick={() => console.log('Left')} title="Left">
                <ArrowLeft size={20} />
              </button>
              <div className="control-btn center">
                <CircleDot size={20} />
              </div>
              <button className="control-btn" onClick={() => console.log('Right')} title="Right">
                <ArrowRight size={20} />
              </button>
              <div></div>
              <button className="control-btn" onClick={() => console.log('Backward')} title="Backward">
                <ArrowDown size={20} />
              </button>
              <div></div>
            </div>

            <div className="velocity-control">
              <label>
                <span>Velocity</span>
                <span className="velocity-value">{robotVelocity.toFixed(1)} m/s</span>
              </label>
              <input 
                type="range"
                min="0"
                max="5"
                step="0.1"
                value={robotVelocity}
                onChange={(e) => setRobotVelocity(parseFloat(e.target.value))}
                className="velocity-slider"
              />
            </div>
          </div>
        </div>
      )}

      {/* View Controls - Bottom Right */}
      <div className="view-controls">
        <div className="view-mode-buttons">
          <button 
            className={`view-btn ${viewMode === '3D' ? 'active' : ''}`}
            onClick={() => setViewMode('3D')}
            title="3D View"
          >
            3D
          </button>
          <button 
            className={`view-btn ${viewMode === 'Top' ? 'active' : ''}`}
            onClick={() => setViewMode('Top')}
            title="Top View"
          >
            Top
          </button>
          <button 
            className={`view-btn ${viewMode === 'Side' ? 'active' : ''}`}
            onClick={() => setViewMode('Side')}
            title="Side View"
          >
            Side
          </button>
          <button 
            className={`view-btn ${viewMode === 'Front' ? 'active' : ''}`}
            onClick={() => setViewMode('Front')}
            title="Front View"
          >
            Front
          </button>
        </div>
        
        <div className="view-helpers">
          <button 
            className={`view-btn ${showGrid ? 'active' : ''}`}
            onClick={() => setShowGrid(!showGrid)}
            title="Toggle Grid"
          >
            <Grid3x3 size={18} />
          </button>
          <button 
            className={`view-btn ${showAxes ? 'active' : ''}`}
            onClick={() => setShowAxes(!showAxes)}
            title="Toggle Axes"
          >
            <Target size={18} />
          </button>
          <button 
            className="view-btn"
            onClick={() => console.log('Reset camera')}
            title="Reset View"
          >
            <Maximize2 size={18} />
          </button>
        </div>
      </div>
    </div>
  );
};

