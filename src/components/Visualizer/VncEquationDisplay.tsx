import React from 'react';
import { Text } from '@react-three/drei';

interface VncEquationDisplayProps {
  equation: string;
  position: [number, number, number];
  color?: string;
}

export const VncEquationDisplay: React.FC<VncEquationDisplayProps> = ({
  equation,
  position,
  color = '#00ff00',
}) => {
  return (
    <Text
      position={position}
      fontSize={0.8}
      color={color}
      anchorX="center"
      anchorY="middle"
      outlineWidth={0.05}
      outlineColor="#000000"
    >
      {equation}
    </Text>
  );
};

