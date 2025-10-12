import React, { useRef, useMemo } from 'react';
import { Canvas, useFrame } from '@react-three/fiber';
import { OrbitControls, Text, Line } from '@react-three/drei';
import * as THREE from 'three';
import { NavigationPath, EnergyLandscape } from '../../types';
import './NavigationVisualizer.css';

interface NavigationVisualizerProps {
  navigationPaths: NavigationPath[];
  energyLandscape?: EnergyLandscape;
  showVncEquations?: boolean;
  realTimeUpdate?: boolean;
}

/**
 * 3D Navigation Visualizer with WebGL
 * 
 * Real-time GPU-accelerated visualization of VNC navigation paths
 */
export const NavigationVisualizer: React.FC<NavigationVisualizerProps> = ({
  navigationPaths,
  energyLandscape,
  showVncEquations = true,
  realTimeUpdate = false,
}) => {
  return (
    <div className="navigation-visualizer">
      <Canvas camera={{ position: [10, 10, 10], fov: 60 }}>
        <ambientLight intensity={0.6} />
        <pointLight position={[10, 10, 10]} intensity={1.0} />
        <pointLight position={[-10, -10, -10]} intensity={0.5} />

        {/* Grid helper */}
        <gridHelper args={[20, 20, 0x444444, 0x222222]} />

        {/* Navigation paths */}
        {navigationPaths.map((path, index) => (
          <NavigationPathMesh
            key={index}
            path={path}
            index={index}
            animate={realTimeUpdate}
          />
        ))}

        {/* Energy landscape */}
        {energyLandscape && <EnergyLandscapeMesh landscape={energyLandscape} />}

        {/* VNC equations in 3D */}
        {showVncEquations && (
          <Text
            position={[0, 8, 0]}
            fontSize={0.8}
            color="#00ff00"
            anchorX="center"
            anchorY="middle"
          >
            𝒩ℐ = ⋋ ⊗ ℰ
          </Text>
        )}

        {/* Interactive controls */}
        <OrbitControls enablePan={true} enableZoom={true} enableRotate={true} />
      </Canvas>

      {/* Navigation metrics panel */}
      <NavigationMetricsPanel paths={navigationPaths} />
    </div>
  );
};

interface NavigationPathMeshProps {
  path: NavigationPath;
  index: number;
  animate: boolean;
}

const NavigationPathMesh: React.FC<NavigationPathMeshProps> = ({ path, index, animate }) => {
  const lineRef = useRef<THREE.Line>(null);

  useFrame((state) => {
    if (lineRef.current && animate) {
      const time = state.clock.elapsedTime;
      // Animate the path with pulsating energy visualization
      const scale = 1 + Math.sin(time * 2 + index) * 0.1;
      lineRef.current.scale.set(scale, scale, scale);
    }
  });

  const points = useMemo(() => {
    return path.waypoints.map((wp) => new THREE.Vector3(wp[0], wp[1], wp[2]));
  }, [path.waypoints]);

  const color = path.optimizationMethod === 'vnc' ? '#00ff00' : '#ff0000';

  return (
    <>
      <Line
        ref={lineRef}
        points={points}
        color={color}
        lineWidth={3}
        dashed={false}
      />
      {/* Start and end markers */}
      {path.waypoints.length > 0 && (
        <>
          <mesh position={path.waypoints[0]}>
            <sphereGeometry args={[0.2, 16, 16]} />
            <meshStandardMaterial color={color} emissive={color} emissiveIntensity={0.5} />
          </mesh>
          <mesh position={path.waypoints[path.waypoints.length - 1]}>
            <sphereGeometry args={[0.3, 16, 16]} />
            <meshStandardMaterial color={color} emissive={color} emissiveIntensity={0.8} />
          </mesh>
        </>
      )}
    </>
  );
};

interface EnergyLandscapeMeshProps {
  landscape: EnergyLandscape;
}

const EnergyLandscapeMesh: React.FC<EnergyLandscapeMeshProps> = ({ landscape }) => {
  const meshRef = useRef<THREE.Mesh>(null);

  useFrame(() => {
    if (meshRef.current) {
      meshRef.current.rotation.z += 0.001;
    }
  });

  return (
    <mesh ref={meshRef} position={[0, -2, 0]}>
      <planeGeometry args={[15, 15, 30, 30]} />
      <meshStandardMaterial
        color="#00ccff"
        wireframe={true}
        transparent={true}
        opacity={0.5}
      />
    </mesh>
  );
};

interface NavigationMetricsPanelProps {
  paths: NavigationPath[];
}

const NavigationMetricsPanel: React.FC<NavigationMetricsPanelProps> = ({ paths }) => {
  const totalEnergy = paths.reduce((sum, path) => sum + path.energy, 0);
  const avgEnergy = paths.length > 0 ? totalEnergy / paths.length : 0;

  return (
    <div className="navigation-metrics-panel">
      <h3>Navigation Metrics</h3>
      <div className="metric">
        <span className="metric-label">Active Paths:</span>
        <span className="metric-value">{paths.length}</span>
      </div>
      <div className="metric">
        <span className="metric-label">Total Energy:</span>
        <span className="metric-value">{totalEnergy.toFixed(4)}</span>
      </div>
      <div className="metric">
        <span className="metric-label">Average Energy:</span>
        <span className="metric-value">{avgEnergy.toFixed(4)}</span>
      </div>
      <div className="metric">
        <span className="metric-label">Optimization:</span>
        <span className="metric-value vnc">VNC</span>
      </div>
    </div>
  );
};

