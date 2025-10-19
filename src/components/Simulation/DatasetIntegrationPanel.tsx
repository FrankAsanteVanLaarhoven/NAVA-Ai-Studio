/**
 * Dataset Integration Panel
 * 
 * World-class UI for real-time data integration with:
 * - Waymo Open Dataset
 * - KITTI Dataset
 * - Open X-Embodiment (RT-X Models)
 * - NIF (Navigation Integrity Framework)
 * 
 * Patent-worthy features:
 * - Real-time streaming visualization
 * - Live integrity monitoring
 * - Multi-dataset fusion
 * - Automated training pipeline
 */

import React, { useState, useEffect } from 'react';
import './DatasetIntegrationPanel.css';
import { datasetIngestionService, UnifiedDataFrame } from '../../services/dataset-ingestion-service';
import { nifIntegrationService, NIFStatus, OptimizationResult } from '../../services/nif-integration-service';
import { simulationService } from '../../services/simulation-service';

export const DatasetIntegrationPanel: React.FC = () => {
  const [selectedDataset, setSelectedDataset] = useState<'waymo' | 'kitti' | 'rtx' | null>(null);
  const [isStreaming, setIsStreaming] = useState(false);
  const [nifStatus, setNifStatus] = useState<NIFStatus | null>(null);
  const [currentFrame, setCurrentFrame] = useState<UnifiedDataFrame | null>(null);
  const [frameCount, setFrameCount] = useState(0);
  const [optimizationResult, setOptimizationResult] = useState<OptimizationResult | null>(null);
  const [isConnected, setIsConnected] = useState({
    dataset: false,
    nif: false,
    simulation: false,
  });

  useEffect(() => {
    // Check service health
    checkServiceHealth();

    // Subscribe to NIF status updates
    nifIntegrationService.startStatusPolling(500);
    const unsubscribe = nifIntegrationService.onStatusUpdate((status) => {
      setNifStatus(status);
    });

    // Subscribe to dataset frames
    const unsubscribeFrames = datasetIngestionService.onFrame((frame) => {
      setCurrentFrame(frame);
      setFrameCount(prev => prev + 1);
      
      // Send to simulation
      simulationService.addRobot({
        id: `dataset_robot_${frame.timestamp}`,
        name: `${frame.source.toUpperCase()} Robot`,
        model: 'differential_drive',
        max_linear_velocity: 2.0,
        max_angular_velocity: 1.5,
      }).catch(console.error);

      // Send to NIF
      nifIntegrationService.addFrame({
        timestamp: frame.timestamp,
        position: frame.pose.position,
        orientation: frame.pose.orientation,
        lidar: frame.sensor_data.lidar,
        camera: frame.sensor_data.camera,
      }).catch(console.error);
    });

    return () => {
      nifIntegrationService.stopStatusPolling();
      unsubscribe();
      unsubscribeFrames();
      datasetIngestionService.stopStreaming();
    };
  }, []);

  const checkServiceHealth = async () => {
    // Check dataset server
    try {
      const response = await fetch('http://localhost:5050/health');
      setIsConnected(prev => ({ ...prev, dataset: response.ok }));
    } catch {
      setIsConnected(prev => ({ ...prev, dataset: false }));
    }

    // Check NIF
    const nifHealth = await nifIntegrationService.checkHealth();
    setIsConnected(prev => ({ ...prev, nif: nifHealth }));

    // Check simulation
    try {
      await simulationService.getState();
      setIsConnected(prev => ({ ...prev, simulation: true }));
    } catch {
      setIsConnected(prev => ({ ...prev, simulation: false }));
    }
  };

  const handleConnectDataset = async (dataset: 'waymo' | 'kitti' | 'rtx') => {
    try {
      await datasetIngestionService.connectDataset(dataset);
      setSelectedDataset(dataset);
      alert(`✅ Connected to ${dataset.toUpperCase()} dataset!`);
    } catch (error) {
      console.error('Failed to connect:', error);
      alert(`❌ Failed to connect to ${dataset}`);
    }
  };

  const handleStartStreaming = async () => {
    if (!selectedDataset) {
      alert('Please select a dataset first');
      return;
    }

    try {
      await datasetIngestionService.startStreaming(10); // 10 FPS
      await nifIntegrationService.start();
      setIsStreaming(true);
      setFrameCount(0);
      alert('🎬 Started real-time streaming!');
    } catch (error) {
      console.error('Failed to start streaming:', error);
      alert('❌ Failed to start streaming');
    }
  };

  const handleStopStreaming = () => {
    datasetIngestionService.stopStreaming();
    nifIntegrationService.stop();
    setIsStreaming(false);
  };

  const handleOptimize = async () => {
    try {
      const result = await nifIntegrationService.optimize();
      setOptimizationResult(result);
      
      if (result.status === 'success') {
        alert(`✅ Optimization complete!\nIntegrity: ${(result.integrity_score! * 100).toFixed(1)}%\nRMSE: ${result.metrics!.rmse.toFixed(3)}m`);
      }
    } catch (error) {
      console.error('Optimization failed:', error);
      alert('❌ Optimization failed');
    }
  };

  const handleReset = async () => {
    await nifIntegrationService.reset();
    await simulationService.reset();
    setFrameCount(0);
    setCurrentFrame(null);
    setOptimizationResult(null);
    alert('🔄 System reset complete');
  };

  return (
    <div className="dataset-integration-panel">
      <div className="integration-header">
        <h2>🌟 Real-Time Dataset Integration</h2>
        <p>Patent-Worthy: Waymo • KITTI • RT-X + NIF</p>
      </div>

      {/* Service Status */}
      <div className="service-status-grid">
        <div className={`status-card ${isConnected.dataset ? 'connected' : 'disconnected'}`}>
          <div className="status-icon">📊</div>
          <div className="status-info">
            <div className="status-label">Dataset Server</div>
            <div className="status-value">{isConnected.dataset ? '🟢 Connected' : '🔴 Offline'}</div>
          </div>
        </div>

        <div className={`status-card ${isConnected.nif ? 'connected' : 'disconnected'}`}>
          <div className="status-icon">🧮</div>
          <div className="status-info">
            <div className="status-label">NIF Bridge</div>
            <div className="status-value">{isConnected.nif ? '🟢 Active' : '🔴 Offline'}</div>
          </div>
        </div>

        <div className={`status-card ${isConnected.simulation ? 'connected' : 'disconnected'}`}>
          <div className="status-icon">🎮</div>
          <div className="status-info">
            <div className="status-label">Simulation</div>
            <div className="status-value">{isConnected.simulation ? '🟢 Ready' : '🔴 Offline'}</div>
          </div>
        </div>
      </div>

      {/* Dataset Selection */}
      <div className="integration-section">
        <h3>📁 Select Dataset</h3>
        <div className="dataset-buttons">
          <button
            className={`dataset-btn ${selectedDataset === 'waymo' ? 'selected' : ''}`}
            onClick={() => handleConnectDataset('waymo')}
            disabled={!isConnected.dataset}
          >
            <span className="dataset-icon">🚗</span>
            <span className="dataset-name">Waymo</span>
            <span className="dataset-desc">Autonomous Driving</span>
          </button>

          <button
            className={`dataset-btn ${selectedDataset === 'kitti' ? 'selected' : ''}`}
            onClick={() => handleConnectDataset('kitti')}
            disabled={!isConnected.dataset}
          >
            <span className="dataset-icon">🏙️</span>
            <span className="dataset-name">KITTI</span>
            <span className="dataset-desc">Urban Navigation</span>
          </button>

          <button
            className={`dataset-btn ${selectedDataset === 'rtx' ? 'selected' : ''}`}
            onClick={() => handleConnectDataset('rtx')}
            disabled={!isConnected.dataset}
          >
            <span className="dataset-icon">🤖</span>
            <span className="dataset-name">RT-X</span>
            <span className="dataset-desc">Robot Manipulation</span>
          </button>
        </div>
      </div>

      {/* Streaming Controls */}
      <div className="integration-section">
        <h3>🎬 Streaming Control</h3>
        <div className="streaming-controls">
          <button
            className="control-btn stream-btn"
            onClick={handleStartStreaming}
            disabled={!selectedDataset || isStreaming}
          >
            ▶️ Start Streaming
          </button>

          <button
            className="control-btn stop-btn"
            onClick={handleStopStreaming}
            disabled={!isStreaming}
          >
            ⏹️ Stop
          </button>

          <button
            className="control-btn optimize-btn"
            onClick={handleOptimize}
            disabled={!nifStatus || nifStatus.poses < 2}
          >
            🧮 Optimize (NIF)
          </button>

          <button
            className="control-btn reset-btn"
            onClick={handleReset}
          >
            🔄 Reset
          </button>
        </div>

        <div className="streaming-stats">
          <div className="stat">
            <span className="stat-label">Frames Processed:</span>
            <span className="stat-value">{frameCount}</span>
          </div>
          <div className="stat">
            <span className="stat-label">Status:</span>
            <span className="stat-value">{isStreaming ? '🎬 Streaming' : '⏸️ Paused'}</span>
          </div>
        </div>
      </div>

      {/* NIF Status */}
      {nifStatus && (
        <div className="integration-section">
          <h3>🧮 NIF Optimization Status</h3>
          <div className="nif-stats-grid">
            <div className="nif-stat">
              <div className="nif-stat-label">Poses</div>
              <div className="nif-stat-value">{nifStatus.poses}</div>
            </div>
            <div className="nif-stat">
              <div className="nif-stat-label">Observations</div>
              <div className="nif-stat-value">{nifStatus.observations}</div>
            </div>
            <div className="nif-stat">
              <div className="nif-stat-label">Integrity</div>
              <div className="nif-stat-value integrity">
                {(nifStatus.integrity * 100).toFixed(1)}%
              </div>
            </div>
            <div className="nif-stat">
              <div className="nif-stat-label">Status</div>
              <div className="nif-stat-value">
                {nifStatus.optimizing ? '⚙️ Optimizing' : '✅ Ready'}
              </div>
            </div>
          </div>

          {/* Integrity Bar */}
          <div className="integrity-bar-container">
            <div className="integrity-label">Navigation Integrity:</div>
            <div className="integrity-bar">
              <div
                className="integrity-fill"
                style={{
                  width: `${nifStatus.integrity * 100}%`,
                  backgroundColor: nifStatus.integrity > 0.9 ? '#22c55e' : 
                                   nifStatus.integrity > 0.7 ? '#fbbf24' : '#ef4444'
                }}
              />
            </div>
          </div>
        </div>
      )}

      {/* Optimization Results */}
      {optimizationResult && optimizationResult.status === 'success' && (
        <div className="integration-section">
          <h3>📊 Optimization Results</h3>
          <div className="optimization-results">
            <div className="result-item">
              <span className="result-label">RMSE:</span>
              <span className="result-value">{optimizationResult.metrics!.rmse.toFixed(3)} m</span>
            </div>
            <div className="result-item">
              <span className="result-label">Max Error:</span>
              <span className="result-value">{optimizationResult.metrics!.max_error.toFixed(3)} m</span>
            </div>
            <div className="result-item">
              <span className="result-label">CEP95:</span>
              <span className="result-value">{optimizationResult.metrics!.cep95.toFixed(3)}</span>
            </div>
            <div className="result-item">
              <span className="result-label">Integrity Score:</span>
              <span className="result-value">{(optimizationResult.integrity_score! * 100).toFixed(1)}%</span>
            </div>
          </div>
        </div>
      )}

      {/* Current Frame Info */}
      {currentFrame && (
        <div className="integration-section">
          <h3>📍 Current Frame</h3>
          <div className="frame-info">
            <div className="frame-item">
              <span className="frame-label">Source:</span>
              <span className="frame-value">{currentFrame.source.toUpperCase()}</span>
            </div>
            <div className="frame-item">
              <span className="frame-label">Position:</span>
              <span className="frame-value">
                ({currentFrame.pose.position[0].toFixed(2)}, {currentFrame.pose.position[1].toFixed(2)}, {currentFrame.pose.position[2].toFixed(2)})
              </span>
            </div>
            <div className="frame-item">
              <span className="frame-label">Timestamp:</span>
              <span className="frame-value">{currentFrame.timestamp.toFixed(2)} s</span>
            </div>
          </div>
        </div>
      )}

      {/* Setup Instructions */}
      {!isConnected.dataset && (
        <div className="setup-instructions">
          <h4>⚙️ Setup Required</h4>
          <p>Start the dataset server:</p>
          <code>cd simulation-platform && python3 dataset_server.py</code>
          <p style={{ marginTop: '10px' }}>Or use the integrated startup script:</p>
          <code>./start-integrated-system.sh</code>
        </div>
      )}
    </div>
  );
};

export default DatasetIntegrationPanel;

