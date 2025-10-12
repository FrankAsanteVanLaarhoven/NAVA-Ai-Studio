import React, { useState, useEffect } from 'react';

interface IntelliSenseProps {
  position: { x: number; y: number };
  completions: string[];
  onSelect: (completion: string) => void;
  visible: boolean;
}

export const IntelliSense: React.FC<IntelliSenseProps> = ({
  position,
  completions,
  onSelect,
  visible,
}) => {
  const [selectedIndex, setSelectedIndex] = useState(0);

  useEffect(() => {
    setSelectedIndex(0);
  }, [completions]);

  if (!visible || completions.length === 0) {
    return null;
  }

  return (
    <div
      className="intellisense-popup"
      style={{ left: position.x, top: position.y }}
    >
      {completions.map((completion, index) => (
        <div
          key={index}
          className={`completion-item ${index === selectedIndex ? 'selected' : ''}`}
          onClick={() => onSelect(completion)}
          onMouseEnter={() => setSelectedIndex(index)}
        >
          <span className="completion-icon">⋋</span>
          <span className="completion-text">{completion}</span>
        </div>
      ))}
    </div>
  );
};

