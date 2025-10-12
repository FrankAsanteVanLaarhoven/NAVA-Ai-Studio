import React, { useEffect, useRef } from 'react';

interface VncMathRendererProps {
  equation: string;
  displayMode?: boolean;
}

export const VncMathRenderer: React.FC<VncMathRendererProps> = ({
  equation,
  displayMode = false,
}) => {
  const containerRef = useRef<HTMLDivElement>(null);

  useEffect(() => {
    if (containerRef.current) {
      // In production, this would use MathJax to render the equation
      containerRef.current.innerHTML = `<span class="math-equation">${equation}</span>`;
    }
  }, [equation]);

  return (
    <div
      ref={containerRef}
      className={`vnc-math-renderer ${displayMode ? 'display-mode' : 'inline-mode'}`}
    />
  );
};

