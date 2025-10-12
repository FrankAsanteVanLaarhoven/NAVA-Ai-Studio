import React from 'react';
import { List } from 'lucide-react';
import './Outline.css';

interface OutlineItem {
  name: string;
  type: 'function' | 'variable' | 'navigation' | 'operator';
  line: number;
}

const OUTLINE_ITEMS: OutlineItem[] = [
  { name: 'start', type: 'variable', line: 5 },
  { name: 'goal', type: 'variable', line: 6 },
  { name: 'path', type: 'variable', line: 9 },
  { name: 'optimal', type: 'variable', line: 12 },
  { name: 'navigate_to⋋', type: 'navigation', line: 9 },
  { name: 'find_optimal_path⋋', type: 'navigation', line: 12 },
  { name: '𝒩ℐ', type: 'operator', line: 15 },
  { name: 'visualize⋋', type: 'function', line: 18 },
];

export const Outline: React.FC = () => {
  const getIcon = (type: string) => {
    switch (type) {
      case 'function':
        return 'ƒ';
      case 'variable':
        return 'v';
      case 'navigation':
        return '⋋';
      case 'operator':
        return '𝒩';
      default:
        return '•';
    }
  };

  return (
    <div className="outline-panel">
      <div className="outline-header">
        <List size={16} />
        <h3>OUTLINE</h3>
      </div>
      <div className="outline-list">
        {OUTLINE_ITEMS.map((item, index) => (
          <div key={index} className={`outline-item ${item.type}`}>
            <span className="outline-icon">{getIcon(item.type)}</span>
            <span className="outline-name">{item.name}</span>
            <span className="outline-line">:{item.line}</span>
          </div>
        ))}
      </div>
    </div>
  );
};

