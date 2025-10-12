import React, { useRef, useState } from 'react';
import { useFrame } from '@react-three/fiber';
import * as THREE from 'three';

interface PathAnimationProps {
  path: [number, number, number][];
  speed: number;
  color: string;
}

export const PathAnimation: React.FC<PathAnimationProps> = ({ path, speed, color }) => {
  const [progress, setProgress] = useState(0);
  const sphereRef = useRef<THREE.Mesh>(null);

  useFrame((_, delta) => {
    setProgress((prev) => {
      const next = prev + delta * speed;
      return next > path.length - 1 ? 0 : next;
    });

    if (sphereRef.current && path.length > 0) {
      const index = Math.floor(progress);
      const t = progress - index;
      const nextIndex = Math.min(index + 1, path.length - 1);

      const pos = new THREE.Vector3(
        THREE.MathUtils.lerp(path[index][0], path[nextIndex][0], t),
        THREE.MathUtils.lerp(path[index][1], path[nextIndex][1], t),
        THREE.MathUtils.lerp(path[index][2], path[nextIndex][2], t)
      );

      sphereRef.current.position.copy(pos);
    }
  });

  return (
    <mesh ref={sphereRef}>
      <sphereGeometry args={[0.3, 16, 16]} />
      <meshStandardMaterial color={color} emissive={color} emissiveIntensity={0.5} />
    </mesh>
  );
};

