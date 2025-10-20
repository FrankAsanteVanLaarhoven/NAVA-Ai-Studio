import React, { useEffect, useRef, useState } from 'react';
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls';
import { gazeboService, GazeboModel, SimulationState } from '../../services/gazebo-service';
import './Gazebo3DViewer.css';

export const Gazebo3DViewer: React.FC = () => {
  const containerRef = useRef<HTMLDivElement>(null);
  const sceneRef = useRef<THREE.Scene | null>(null);
  const cameraRef = useRef<THREE.PerspectiveCamera | null>(null);
  const rendererRef = useRef<THREE.WebGLRenderer | null>(null);
  const controlsRef = useRef<OrbitControls | null>(null);
  const modelsRef = useRef<Map<string, THREE.Object3D>>(new Map());
  const animationIdRef = useRef<number | null>(null);

  const [viewMode, setViewMode] = useState<'3D' | 'Top' | 'Side' | 'Front'>('3D');
  const [showGrid, setShowGrid] = useState(true);
  const [showAxes, setShowAxes] = useState(true);

  useEffect(() => {
    if (!containerRef.current) return;

    // Initialize Three.js scene
    initializeScene();

    // Subscribe to Gazebo events
    gazeboService.on('world_loaded', handleWorldLoaded);
    gazeboService.on('model_spawned', handleModelSpawned);
    gazeboService.on('model_deleted', handleModelDeleted);
    gazeboService.on('simulation_update', handleSimulationUpdate);
    gazeboService.on('model_pose_updated', handleModelPoseUpdated);

    // Start animation loop
    animate();

    // Handle resize
    const handleResize = () => {
      if (!containerRef.current || !cameraRef.current || !rendererRef.current) return;
      
      const width = containerRef.current.clientWidth;
      const height = containerRef.current.clientHeight;
      
      cameraRef.current.aspect = width / height;
      cameraRef.current.updateProjectionMatrix();
      rendererRef.current.setSize(width, height);
    };

    window.addEventListener('resize', handleResize);

    return () => {
      // Cleanup
      if (animationIdRef.current) {
        cancelAnimationFrame(animationIdRef.current);
      }
      if (rendererRef.current) {
        rendererRef.current.dispose();
      }
      gazeboService.off('world_loaded', handleWorldLoaded);
      gazeboService.off('model_spawned', handleModelSpawned);
      gazeboService.off('model_deleted', handleModelDeleted);
      gazeboService.off('simulation_update', handleSimulationUpdate);
      gazeboService.off('model_pose_updated', handleModelPoseUpdated);
      window.removeEventListener('resize', handleResize);
    };
  }, []);

  const initializeScene = () => {
    if (!containerRef.current) return;

    const width = containerRef.current.clientWidth;
    const height = containerRef.current.clientHeight;

    // Scene
    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0x0a0a1a); // Darker blue background
    scene.fog = new THREE.Fog(0x0a0a1a, 100, 500); // Push fog further away
    sceneRef.current = scene;

    // Camera - Better initial position to see everything
    const camera = new THREE.PerspectiveCamera(60, width / height, 0.1, 2000);
    camera.position.set(15, 12, 15); // Higher and further out
    camera.lookAt(0, 1, 0); // Look at center slightly above ground
    cameraRef.current = camera;

    // Renderer
    const renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setSize(width, height);
    renderer.setPixelRatio(window.devicePixelRatio);
    renderer.shadowMap.enabled = true;
    renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    containerRef.current.appendChild(renderer.domElement);
    rendererRef.current = renderer;

    // Controls
    const controls = new OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.dampingFactor = 0.05;
    controls.minDistance = 2;
    controls.maxDistance = 100;
    controlsRef.current = controls;

    // Lighting - Bright and clear
    const ambientLight = new THREE.AmbientLight(0xffffff, 0.8); // Brighter ambient
    scene.add(ambientLight);

    const directionalLight = new THREE.DirectionalLight(0xffffff, 1.0); // Full brightness
    directionalLight.position.set(20, 30, 10);
    directionalLight.castShadow = true;
    directionalLight.shadow.mapSize.width = 2048;
    directionalLight.shadow.mapSize.height = 2048;
    directionalLight.shadow.camera.left = -50;
    directionalLight.shadow.camera.right = 50;
    directionalLight.shadow.camera.top = 50;
    directionalLight.shadow.camera.bottom = -50;
    scene.add(directionalLight);

    // Add hemisphere light for better scene illumination
    const hemiLight = new THREE.HemisphereLight(0x87ceeb, 0x543210, 0.5);
    hemiLight.position.set(0, 50, 0);
    scene.add(hemiLight);

    // Grid Helper - Larger and more visible
    const gridHelper = new THREE.GridHelper(50, 50, 0x00ff00, 0x1a3320);
    gridHelper.name = 'grid';
    gridHelper.position.y = 0.01; // Slightly above ground to avoid z-fighting
    scene.add(gridHelper);

    // Axes Helper - Larger for better visibility
    const axesHelper = new THREE.AxesHelper(8);
    axesHelper.name = 'axes';
    axesHelper.position.y = 0.02;
    scene.add(axesHelper);

    // Load current world
    const currentWorld = gazeboService.getCurrentWorld();
    if (currentWorld) {
      currentWorld.models.forEach(model => {
        addModelToScene(model);
      });
    }
  };

  const addModelToScene = (model: GazeboModel) => {
    if (!sceneRef.current) return;

    console.log('Adding model to scene:', model.name, 'at position:', model.position);

    let object3D: THREE.Object3D;

    switch (model.type) {
      case 'object':
        // Ground plane or generic object
        if (model.name === 'Ground Plane' || model.name === 'Terrain') {
          // Large visible ground plane
          const geometry = new THREE.PlaneGeometry(100, 100);
          const material = new THREE.MeshStandardMaterial({
            color: '#2a2a3a',
            roughness: 0.9,
            metalness: 0.1,
          });
          object3D = new THREE.Mesh(geometry, material);
          object3D.rotation.x = -Math.PI / 2;
          object3D.position.y = 0; // At ground level
          object3D.receiveShadow = true;
        } else {
          // Regular object - use scale from model or default size
          const scale = model.scale && model.scale.length === 3 ? model.scale : [1, 1, 1];
          
          // Create different shapes based on name
          let geometry;
          if (model.name.toLowerCase().includes('sphere')) {
            geometry = new THREE.SphereGeometry(scale[0] / 2, 32, 32);
          } else if (model.name.toLowerCase().includes('cylinder')) {
            geometry = new THREE.CylinderGeometry(scale[0] / 2, scale[0] / 2, scale[1], 32);
          } else {
            // Default box
            geometry = new THREE.BoxGeometry(...scale);
          }
          
          const material = new THREE.MeshStandardMaterial({
            color: model.color || '#4ec9b0',
            roughness: 0.7,
            metalness: 0.3,
            emissive: model.color || '#4ec9b0',
            emissiveIntensity: 0.1,
          });
          object3D = new THREE.Mesh(geometry, material);
          object3D.castShadow = true;
          object3D.receiveShadow = true;
        }
        break;

      case 'robot':
        // Create robot group
        const robotGroup = new THREE.Group();
        
        // Main body - larger and more visible
        const robotGeometry = new THREE.BoxGeometry(1.2, 0.6, 0.8);
        const robotMaterial = new THREE.MeshStandardMaterial({
          color: '#00ff00',
          emissive: '#00ff00',
          emissiveIntensity: 0.3,
          roughness: 0.4,
          metalness: 0.6,
        });
        const body = new THREE.Mesh(robotGeometry, robotMaterial);
        body.castShadow = true;
        body.position.y = 0.3; // Lift body above wheels
        robotGroup.add(body);

        // Add wheels - larger and more visible
        const wheelGeometry = new THREE.CylinderGeometry(0.15, 0.15, 0.1, 16);
        const wheelMaterial = new THREE.MeshStandardMaterial({ 
          color: '#222222',
          roughness: 0.8,
          metalness: 0.2,
        });
        
        const wheelPositions = [
          [-0.5, 0, 0.5],   // Front left
          [-0.5, 0, -0.5],  // Front right
          [0.5, 0, 0.5],    // Back left
          [0.5, 0, -0.5],   // Back right
        ];

        wheelPositions.forEach(pos => {
          const wheel = new THREE.Mesh(wheelGeometry, wheelMaterial);
          wheel.position.set(...pos);
          wheel.rotation.z = Math.PI / 2;
          wheel.castShadow = true;
          robotGroup.add(wheel);
        });
        
        // Add a sensor/indicator on top
        const sensorGeom = new THREE.CylinderGeometry(0.1, 0.1, 0.2, 16);
        const sensorMat = new THREE.MeshStandardMaterial({
          color: '#ff0000',
          emissive: '#ff0000',
          emissiveIntensity: 0.5,
        });
        const sensor = new THREE.Mesh(sensorGeom, sensorMat);
        sensor.position.set(0, 0.7, 0);
        robotGroup.add(sensor);
        
        object3D = robotGroup;
        break;

      case 'light':
        // Create light source
        const pointLight = new THREE.PointLight(0xffffff, 1, 100);
        object3D = pointLight;
        break;

      case 'sensor':
        // Create sensor representation
        const sensorGeometry = new THREE.SphereGeometry(0.05);
        const sensorMaterial = new THREE.MeshBasicMaterial({
          color: '#ff00ff',
          transparent: true,
          opacity: 0.7,
        });
        object3D = new THREE.Mesh(sensorGeometry, sensorMaterial);
        break;

      default:
        // Default box
        const defaultGeometry = new THREE.BoxGeometry(1, 1, 1);
        const defaultMaterial = new THREE.MeshStandardMaterial({ color: '#ffffff' });
        object3D = new THREE.Mesh(defaultGeometry, defaultMaterial);
    }

    object3D.name = model.id;
    object3D.position.set(...model.position);
    object3D.rotation.set(...model.rotation);

    sceneRef.current.add(object3D);
    modelsRef.current.set(model.id, object3D);
  };

  const handleWorldLoaded = (world: any) => {
    // Clear existing models
    modelsRef.current.forEach(obj => {
      if (sceneRef.current) {
        sceneRef.current.remove(obj);
      }
    });
    modelsRef.current.clear();

    // Add new models
    world.models.forEach((model: GazeboModel) => {
      addModelToScene(model);
    });
  };

  const handleModelSpawned = (model: GazeboModel) => {
    addModelToScene(model);
  };

  const handleModelDeleted = (data: { modelId: string }) => {
    const object3D = modelsRef.current.get(data.modelId);
    if (object3D && sceneRef.current) {
      sceneRef.current.remove(object3D);
      modelsRef.current.delete(data.modelId);
    }
  };

  const handleSimulationUpdate = (state: SimulationState) => {
    // Update all model positions
    state.models.forEach(model => {
      const object3D = modelsRef.current.get(model.id);
      if (object3D) {
        object3D.position.set(...model.position);
        object3D.rotation.set(...model.rotation);
      }
    });
  };

  const handleModelPoseUpdated = (data: any) => {
    const object3D = modelsRef.current.get(data.modelId);
    if (object3D) {
      object3D.position.set(...data.position);
      object3D.rotation.set(...data.rotation);
    }
  };

  const animate = () => {
    animationIdRef.current = requestAnimationFrame(animate);

    if (controlsRef.current) {
      controlsRef.current.update();
    }

    if (sceneRef.current && cameraRef.current && rendererRef.current) {
      rendererRef.current.render(sceneRef.current, cameraRef.current);
    }
  };

  const switchViewMode = (mode: typeof viewMode) => {
    setViewMode(mode);
    if (!cameraRef.current || !controlsRef.current) return;

    const targetHeight = 1; // Look at slightly above ground

    switch (mode) {
      case 'Top':
        cameraRef.current.position.set(0, 40, 0);
        cameraRef.current.lookAt(0, 0, 0);
        controlsRef.current.target.set(0, 0, 0);
        break;
      case 'Side':
        cameraRef.current.position.set(30, 10, 0);
        cameraRef.current.lookAt(0, targetHeight, 0);
        controlsRef.current.target.set(0, targetHeight, 0);
        break;
      case 'Front':
        cameraRef.current.position.set(0, 10, 30);
        cameraRef.current.lookAt(0, targetHeight, 0);
        controlsRef.current.target.set(0, targetHeight, 0);
        break;
      default:
        // 3D view - good angle to see everything
        cameraRef.current.position.set(15, 12, 15);
        cameraRef.current.lookAt(0, targetHeight, 0);
        controlsRef.current.target.set(0, targetHeight, 0);
    }
    controlsRef.current.update();
  };

  const toggleGrid = () => {
    const grid = sceneRef.current?.getObjectByName('grid');
    if (grid) {
      grid.visible = !showGrid;
      setShowGrid(!showGrid);
    }
  };

  const toggleAxes = () => {
    const axes = sceneRef.current?.getObjectByName('axes');
    if (axes) {
      axes.visible = !showAxes;
      setShowAxes(!showAxes);
    }
  };

  const resetCamera = () => {
    switchViewMode('3D');
  };

  return (
    <div className="gazebo-3d-viewer">
      <div ref={containerRef} className="viewer-container" />
      
      <div className="viewer-controls">
        <div className="view-mode-buttons">
          <button
            className={viewMode === '3D' ? 'active' : ''}
            onClick={() => switchViewMode('3D')}
            title="3D View"
          >
            3D
          </button>
          <button
            className={viewMode === 'Top' ? 'active' : ''}
            onClick={() => switchViewMode('Top')}
            title="Top View"
          >
            Top
          </button>
          <button
            className={viewMode === 'Side' ? 'active' : ''}
            onClick={() => switchViewMode('Side')}
            title="Side View"
          >
            Side
          </button>
          <button
            className={viewMode === 'Front' ? 'active' : ''}
            onClick={() => switchViewMode('Front')}
            title="Front View"
          >
            Front
          </button>
        </div>

        <div className="display-toggles">
          <button
            className={showGrid ? 'active' : ''}
            onClick={toggleGrid}
            title="Toggle Grid"
          >
            Grid
          </button>
          <button
            className={showAxes ? 'active' : ''}
            onClick={toggleAxes}
            title="Toggle Axes"
          >
            Axes
          </button>
          <button onClick={resetCamera} title="Reset Camera">
            Reset
          </button>
        </div>
      </div>
    </div>
  );
};

