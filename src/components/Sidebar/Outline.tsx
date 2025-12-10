import React, { useState, useEffect } from 'react';
import { List } from 'lucide-react';
import { outlineService, type OutlineItem as OutlineItemType } from '../../services/outline-service';
import './Outline.css';

interface OutlineProps {
  hideHeader?: boolean;
  code?: string;
  language?: string;
  onItemClick?: (line: number) => void;
}

export const Outline: React.FC<OutlineProps> = ({ 
  hideHeader = false, 
  code = '',
  language,
  onItemClick,
}) => {
  const [items, setItems] = useState<OutlineItemType[]>([]);

  useEffect(() => {
    if (code) {
      const parsed = outlineService.parseCode(code, language);
      setItems(parsed);
    } else {
      setItems([]);
    }
  }, [code, language]);

  const getIcon = (type: string, customIcon?: string) => {
    if (customIcon) return customIcon;
    switch (type) {
      case 'function':
        return 'f';
      case 'variable':
        return 'v';
      case 'class':
        return 'C';
      case 'interface':
        return 'I';
      case 'navigation':
        return '⋋';
      case 'operator':
        return '𝒩';
      case 'constant':
        return 'c';
      case 'import':
        return '>';
      default:
        return '•';
    }
  };

  const handleItemClick = (line: number) => {
    if (onItemClick) {
      onItemClick(line);
    } else {
      // Emit event to navigate to line
      const event = new CustomEvent('nava:navigate-to-line', {
        detail: { line },
      });
      window.dispatchEvent(event);
    }
  };

  return (
    <div className="outline-panel">
      {!hideHeader && (
        <div className="outline-header">
          <List size={16} />
          <h3>OUTLINE</h3>
        </div>
      )}
      <div className="outline-list">
        {items.length > 0 ? (
          items.map((item, index) => (
            <div 
              key={index} 
              className={`outline-item ${item.type}`}
              onClick={() => handleItemClick(item.line)}
              title={`Go to line ${item.line}`}
            >
              <span className="outline-icon">{getIcon(item.type, item.icon)}</span>
              <span className="outline-name">{item.name}</span>
              <span className="outline-line">:{item.line}</span>
            </div>
          ))
        ) : (
          <div className="outline-empty">
            <p>No code structure found</p>
            <p className="outline-hint">Open a file to see its outline</p>
          </div>
        )}
      </div>
    </div>
  );
};

