# NAVA Studio API Documentation

## Overview
This document provides comprehensive API documentation for integrating with the NAVA Studio backend. All APIs are exposed through Tauri commands and can be invoked from the frontend using the `@tauri-apps/api` package.

## Table of Contents
1. [Dataset Integration APIs](#dataset-integration-apis)
2. [Simulation Control APIs](#simulation-control-apis)
3. [ROS2 Middleware APIs](#ros2-middleware-apis)
4. [AI/ML Model APIs](#aiml-model-apis)
5. [Hardware-in-the-Loop APIs](#hardware-in-the-loop-apis)
6. [Cloud Deployment APIs](#cloud-deployment-apis)
7. [UI Rendering APIs](#ui-rendering-apis)
8. [Legacy IDE APIs](#legacy-ide-apis)

---

## Dataset Integration APIs

### Initialize Dataset Integration
Initializes the dataset integration system and loads metadata for Waymo, KITTI, and RT-X datasets.

**Command**: `initialize_dataset_integration`

**TypeScript Usage**:
```typescript
import { invoke } from '@tauri-apps/api/tauri';

async function initializeDatasets() {
  try {
    const result = await invoke<string>('initialize_dataset_integration');
    console.log(result); // "Dataset Integration System Initialized"
  } catch (error) {
    console.error('Failed to initialize datasets:', error);
  }
}
```

**Response**: `string`
- Success: `"Dataset Integration System Initialized"`
- Error: Error message string

**Use Cases**:
- Application startup
- Dataset refresh
- System initialization

---

## Simulation Control APIs

### Start Simulation
Starts the real-time simulation engine.

**Command**: `start_simulation`

**TypeScript Usage**:
```typescript
import { invoke } from '@tauri-apps/api/tauri';

async function startSimulation() {
  try {
    const result = await invoke<string>('start_simulation');
    console.log(result); // "Simulation Started"
  } catch (error) {
    console.error('Failed to start simulation:', error);
  }
}
```

**Response**: `string`
- Success: `"Simulation Started"`
- Error: Error message string

### Stop Simulation
Stops the currently running simulation.

**Command**: `stop_simulation`

**TypeScript Usage**:
```typescript
import { invoke } from '@tauri-apps/api/tauri';

async function stopSimulation() {
  try {
    const result = await invoke<string>('stop_simulation');
    console.log(result); // "Simulation Stopped"
  } catch (error) {
    console.error('Failed to stop simulation:', error);
  }
}
```

**Response**: `string`
- Success: `"Simulation Stopped"`
- Error: Error message string

**Example: Simulation Control Component**:
```typescript
import React, { useState } from 'react';
import { invoke } from '@tauri-apps/api/tauri';

export function SimulationControl() {
  const [isRunning, setIsRunning] = useState(false);
  const [status, setStatus] = useState('');

  const handleStart = async () => {
    try {
      const result = await invoke<string>('start_simulation');
      setStatus(result);
      setIsRunning(true);
    } catch (error) {
      setStatus(`Error: ${error}`);
    }
  };

  const handleStop = async () => {
    try {
      const result = await invoke<string>('stop_simulation');
      setStatus(result);
      setIsRunning(false);
    } catch (error) {
      setStatus(`Error: ${error}`);
    }
  };

  return (
    <div className="simulation-control">
      <h2>Simulation Control</h2>
      <div className="button-group">
        <button onClick={handleStart} disabled={isRunning}>
          Start Simulation
        </button>
        <button onClick={handleStop} disabled={!isRunning}>
          Stop Simulation
        </button>
      </div>
      <div className="status">{status}</div>
    </div>
  );
}
```

---

## ROS2 Middleware APIs

### Initialize ROS2 Middleware
Initializes the ROS2 middleware stack with DDS communication protocols.

**Command**: `initialize_ros_middleware`

**TypeScript Usage**:
```typescript
import { invoke } from '@tauri-apps/api/tauri';

async function initializeROS2() {
  try {
    const result = await invoke<string>('initialize_ros_middleware');
    console.log(result); // "ROS2 Middleware Initialized"
  } catch (error) {
    console.error('Failed to initialize ROS2:', error);
  }
}
```

**Response**: `string`
- Success: `"ROS2 Middleware Initialized"`
- Error: Error message string

**Future Enhancements** (To be implemented):
```typescript
// Publish to ROS2 topic
interface PublishMessageParams {
  topic: string;
  message: any;
  messageType: string;
}

async function publishMessage(params: PublishMessageParams) {
  return await invoke<void>('ros_publish_message', params);
}

// Subscribe to ROS2 topic
interface SubscribeTopicParams {
  topic: string;
  messageType: string;
}

async function subscribeTopic(params: SubscribeTopicParams) {
  return await invoke<void>('ros_subscribe_topic', params);
}
```

---

## AI/ML Model APIs

### Train AI Model
Initiates training for Vision-Language-Action (VLA) models.

**Command**: `train_ai_model`

**TypeScript Usage**:
```typescript
import { invoke } from '@tauri-apps/api/tauri';

async function trainModel() {
  try {
    const result = await invoke<string>('train_ai_model');
    console.log(result); // "AI Model Training Started"
  } catch (error) {
    console.error('Failed to start training:', error);
  }
}
```

**Response**: `string`
- Success: `"AI Model Training Started"`
- Error: Error message string

### Run AI Inference
Executes inference using trained VLA models.

**Command**: `run_ai_inference`

**TypeScript Usage**:
```typescript
import { invoke } from '@tauri-apps/api/tauri';

async function runInference() {
  try {
    const result = await invoke<string>('run_ai_inference');
    console.log(result); // "AI Model Inference Running"
  } catch (error) {
    console.error('Failed to run inference:', error);
  }
}
```

**Response**: `string`
- Success: `"AI Model Inference Running"`
- Error: Error message string

**Example: AI Control Panel**:
```typescript
import React, { useState } from 'react';
import { invoke } from '@tauri-apps/api/tauri';

export function AIControlPanel() {
  const [trainingStatus, setTrainingStatus] = useState('');
  const [inferenceStatus, setInferenceStatus] = useState('');

  const handleTrain = async () => {
    try {
      const result = await invoke<string>('train_ai_model');
      setTrainingStatus(result);
    } catch (error) {
      setTrainingStatus(`Error: ${error}`);
    }
  };

  const handleInference = async () => {
    try {
      const result = await invoke<string>('run_ai_inference');
      setInferenceStatus(result);
    } catch (error) {
      setInferenceStatus(`Error: ${error}`);
    }
  };

  return (
    <div className="ai-control-panel">
      <h2>AI/ML Control</h2>
      <div className="section">
        <h3>Model Training</h3>
        <button onClick={handleTrain}>Start Training</button>
        <p>{trainingStatus}</p>
      </div>
      <div className="section">
        <h3>Model Inference</h3>
        <button onClick={handleInference}>Run Inference</button>
        <p>{inferenceStatus}</p>
      </div>
    </div>
  );
}
```

---

## Hardware-in-the-Loop APIs

### Initialize HIL System
Initializes the Hardware-in-the-Loop system for physical robot integration.

**Command**: `initialize_hil_system`

**TypeScript Usage**:
```typescript
import { invoke } from '@tauri-apps/api/tauri';

async function initializeHIL() {
  try {
    const result = await invoke<string>('initialize_hil_system');
    console.log(result); // "HIL System Initialized"
  } catch (error) {
    console.error('Failed to initialize HIL:', error);
  }
}
```

**Response**: `string`
- Success: `"HIL System Initialized"`
- Error: Error message string

---

## Cloud Deployment APIs

### Deploy to Cloud
Initiates cloud deployment of simulation instances.

**Command**: `deploy_to_cloud`

**TypeScript Usage**:
```typescript
import { invoke } from '@tauri-apps/api/tauri';

async function deployToCloud() {
  try {
    const result = await invoke<string>('deploy_to_cloud');
    console.log(result); // "Deployment to Cloud Started"
  } catch (error) {
    console.error('Failed to deploy:', error);
  }
}
```

**Response**: `string`
- Success: `"Deployment to Cloud Started"`
- Error: Error message string

---

## UI Rendering APIs

### Render UI
Triggers UI rendering for the web-based control interface.

**Command**: `render_ui`

**TypeScript Usage**:
```typescript
import { invoke } from '@tauri-apps/api/tauri';

async function renderUI() {
  try {
    const result = await invoke<string>('render_ui');
    console.log(result); // "UI Rendered"
  } catch (error) {
    console.error('Failed to render UI:', error);
  }
}
```

**Response**: `string`
- Success: `"UI Rendered"`
- Error: Error message string

---

## Legacy IDE APIs

These APIs are part of the existing NAVΛ Studio IDE functionality.

### Initialize LSP
Initializes the NAVΛ Language Server Protocol.

**Command**: `initialize_lsp`

**TypeScript Usage**:
```typescript
import { invoke } from '@tauri-apps/api/tauri';

async function initializeLSP() {
  try {
    const result = await invoke<string>('initialize_lsp');
    console.log(result); // "NAVΛ Language Server initialized with VNC support"
  } catch (error) {
    console.error('Failed to initialize LSP:', error);
  }
}
```

### Parse NAVΛ Code
Parses NAVΛ code and returns the AST.

**Command**: `parse_navlambda_code`

**Parameters**:
- `code`: `string` - The NAVΛ code to parse

**TypeScript Usage**:
```typescript
import { invoke } from '@tauri-apps/api/tauri';

async function parseCode(code: string) {
  try {
    const result = await invoke<string>('parse_navlambda_code', { code });
    console.log(result); // "Parsed successfully: X statements"
  } catch (error) {
    console.error('Parse error:', error);
  }
}
```

### Compile to Target
Compiles NAVΛ code to a target language.

**Command**: `compile_to_target`

**Parameters**:
- `code`: `string` - The NAVΛ code to compile
- `target`: `string` - Target language (`"cpp"`, `"python"`, `"wasm"`, `"glsl"`)

**TypeScript Usage**:
```typescript
import { invoke } from '@tauri-apps/api/tauri';

async function compileCode(code: string, target: string) {
  try {
    const result = await invoke<string>('compile_to_target', { code, target });
    console.log('Compiled code:', result);
  } catch (error) {
    console.error('Compilation error:', error);
  }
}
```

### Get Code Completions
Retrieves code completion suggestions.

**Command**: `get_code_completions`

**Parameters**:
- `code`: `string` - The current code
- `position`: `number` - Cursor position

**TypeScript Usage**:
```typescript
import { invoke } from '@tauri-apps/api/tauri';

async function getCompletions(code: string, position: number) {
  try {
    const completions = await invoke<string[]>('get_code_completions', { code, position });
    return completions;
  } catch (error) {
    console.error('Failed to get completions:', error);
    return [];
  }
}
```

### Get Hover Info
Retrieves hover information for a symbol.

**Command**: `get_hover_info`

**Parameters**:
- `code`: `string` - The current code
- `position`: `number` - Cursor position

**TypeScript Usage**:
```typescript
import { invoke } from '@tauri-apps/api/tauri';

async function getHoverInfo(code: string, position: number) {
  try {
    const info = await invoke<string>('get_hover_info', { code, position });
    return info;
  } catch (error) {
    console.error('Failed to get hover info:', error);
    return '';
  }
}
```

### Visualize Navigation Path
Visualizes the navigation path from NAVΛ code.

**Command**: `visualize_navigation_path`

**Parameters**:
- `code`: `string` - The NAVΛ code to visualize

**TypeScript Usage**:
```typescript
import { invoke } from '@tauri-apps/api/tauri';

async function visualizePath(code: string) {
  try {
    const visualization = await invoke<string>('visualize_navigation_path', { code });
    const data = JSON.parse(visualization);
    return data;
  } catch (error) {
    console.error('Failed to visualize path:', error);
    return null;
  }
}
```

### Run Live Preview
Executes NAVΛ code in live preview mode.

**Command**: `run_live_preview`

**Parameters**:
- `code`: `string` - The NAVΛ code to execute

**TypeScript Usage**:
```typescript
import { invoke } from '@tauri-apps/api/tauri';

async function runPreview(code: string) {
  try {
    const result = await invoke<string>('run_live_preview', { code });
    return result;
  } catch (error) {
    console.error('Execution error:', error);
    throw error;
  }
}
```

---

## Complete Integration Example

Here's a complete example of a React component that integrates multiple APIs:

```typescript
import React, { useState, useEffect } from 'react';
import { invoke } from '@tauri-apps/api/tauri';

export function NAVAStudioDashboard() {
  const [systemStatus, setSystemStatus] = useState({
    datasets: 'Not Initialized',
    simulation: 'Stopped',
    ros2: 'Not Initialized',
    hil: 'Not Initialized',
  });

  useEffect(() => {
    initializeSystem();
  }, []);

  const initializeSystem = async () => {
    try {
      // Initialize all systems
      await invoke('initialize_dataset_integration');
      setSystemStatus(prev => ({ ...prev, datasets: 'Initialized' }));

      await invoke('initialize_ros_middleware');
      setSystemStatus(prev => ({ ...prev, ros2: 'Initialized' }));

      await invoke('initialize_hil_system');
      setSystemStatus(prev => ({ ...prev, hil: 'Initialized' }));

      await invoke('initialize_lsp');
    } catch (error) {
      console.error('System initialization error:', error);
    }
  };

  const handleStartSimulation = async () => {
    try {
      await invoke('start_simulation');
      setSystemStatus(prev => ({ ...prev, simulation: 'Running' }));
    } catch (error) {
      console.error('Failed to start simulation:', error);
    }
  };

  const handleStopSimulation = async () => {
    try {
      await invoke('stop_simulation');
      setSystemStatus(prev => ({ ...prev, simulation: 'Stopped' }));
    } catch (error) {
      console.error('Failed to stop simulation:', error);
    }
  };

  return (
    <div className="dashboard">
      <h1>NAVA Studio Dashboard</h1>
      
      <div className="system-status">
        <h2>System Status</h2>
        <ul>
          <li>Datasets: {systemStatus.datasets}</li>
          <li>Simulation: {systemStatus.simulation}</li>
          <li>ROS2: {systemStatus.ros2}</li>
          <li>HIL: {systemStatus.hil}</li>
        </ul>
      </div>

      <div className="controls">
        <h2>Simulation Control</h2>
        <button onClick={handleStartSimulation}>Start Simulation</button>
        <button onClick={handleStopSimulation}>Stop Simulation</button>
      </div>
    </div>
  );
}
```

---

## Error Handling Best Practices

### Standard Error Format
All API errors follow this format:
```typescript
interface APIError {
  message: string;
  code?: string;
  details?: any;
}
```

### Error Handling Example
```typescript
async function safeInvoke<T>(command: string, args?: any): Promise<T | null> {
  try {
    return await invoke<T>(command, args);
  } catch (error) {
    console.error(`Command ${command} failed:`, error);
    // Show user-friendly error message
    showNotification({
      type: 'error',
      message: `Failed to execute ${command}`,
      details: error,
    });
    return null;
  }
}
```

---

## Performance Considerations

### Debouncing Frequent Calls
```typescript
import { debounce } from 'lodash';

const debouncedParse = debounce(async (code: string) => {
  await invoke('parse_navlambda_code', { code });
}, 300);
```

### Caching Results
```typescript
const cache = new Map<string, any>();

async function cachedInvoke<T>(command: string, args?: any): Promise<T> {
  const cacheKey = `${command}:${JSON.stringify(args)}`;
  
  if (cache.has(cacheKey)) {
    return cache.get(cacheKey);
  }
  
  const result = await invoke<T>(command, args);
  cache.set(cacheKey, result);
  
  return result;
}
```

---

## Future API Enhancements

The following APIs are planned for future releases:

### Dataset Query API
```typescript
interface DatasetQuery {
  dataset: 'waymo' | 'kitti' | 'rtx';
  filters: Record<string, any>;
  limit?: number;
  offset?: number;
}

async function queryDataset(query: DatasetQuery) {
  return await invoke('query_dataset', query);
}
```

### Real-time Metrics API
```typescript
interface SimulationMetrics {
  fps: number;
  latency: number;
  memory_usage: number;
  gpu_utilization: number;
}

async function getMetrics(): Promise<SimulationMetrics> {
  return await invoke('get_simulation_metrics');
}
```

### WebSocket Streaming API
```typescript
import { listen } from '@tauri-apps/api/event';

async function subscribeToSimulationEvents() {
  const unlisten = await listen('simulation-event', (event) => {
    console.log('Simulation event:', event.payload);
  });
  
  return unlisten;
}
```

---

## API Versioning

Current API Version: **v1.0**

All APIs are backward compatible within major versions. Breaking changes will result in a major version bump.

---

## Support and Resources

- **Documentation**: See individual specification documents
- **Examples**: Check the `examples/` directory
- **Issues**: Report bugs on GitHub
- **Community**: Join our Discord server

---

**Document Version**: 1.0  
**Last Updated**: 2025  
**Status**: Production Ready
