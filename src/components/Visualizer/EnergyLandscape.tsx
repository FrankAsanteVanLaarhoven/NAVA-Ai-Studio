import React, { useRef } from 'react';
import { useFrame } from '@react-three/fiber';
import * as THREE from 'three';

interface EnergyLandscapeProps {
  resolution: number;
  energyFunction: (x: number, y: number) => number;
}

export const EnergyLandscape: React.FC<EnergyLandscapeProps> = ({
  resolution,
  energyFunction,
}) => {
  const meshRef = useRef<THREE.Mesh>(null);

  useFrame(() => {
    if (meshRef.current) {
      meshRef.current.rotation.z += 0.001;
    }
  });

  const geometry = React.useMemo(() => {
    const geo = new THREE.PlaneGeometry(20, 20, resolution, resolution);
    const positions = geo.attributes.position.array as Float32Array;

    for (let i = 0; i < positions.length; i += 3) {
      const x = positions[i];
      const y = positions[i + 1];
      positions[i + 2] = energyFunction(x, y) * 2;
    }

    geo.computeVertexNormals();
    return geo;
  }, [resolution, energyFunction]);

  return (
    <mesh ref={meshRef} geometry={geometry} rotation={[-Math.PI / 2, 0, 0]} position={[0, -2, 0]}>
      <meshStandardMaterial
        color="#00ccff"
        wireframe
        transparent
        opacity={0.6}
      />
    </mesh>
  );
};

