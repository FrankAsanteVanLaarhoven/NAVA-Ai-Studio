import React, { useState, useRef, useEffect } from 'react';
import './SplitPanel.css';

interface SplitPanelProps {
  children: [React.ReactNode, React.ReactNode];
  direction?: 'horizontal' | 'vertical';
  defaultSplit?: number; // Percentage (0-100)
  minSize?: number; // Minimum size in pixels
  storageKey?: string;
  titles?: [string, string];
  collapsible?: boolean;
}

export const SplitPanel: React.FC<SplitPanelProps> = ({
  children,
  direction = 'vertical',
  defaultSplit = 50,
  minSize = 100,
  storageKey,
  titles,
  collapsible = false,
}) => {
  const [firstPanel, secondPanel] = children;
  const [firstPanelTitle, secondPanelTitle] = titles || ['', ''];
  
  // Load saved split from localStorage
  const getInitialSplit = () => {
    if (storageKey) {
      const saved = localStorage.getItem(storageKey);
      if (saved) return parseFloat(saved);
    }
    return defaultSplit;
  };

  const [split, setSplit] = useState(getInitialSplit());
  const [isDragging, setIsDragging] = useState(false);
  const [firstCollapsed, setFirstCollapsed] = useState(false);
  const [secondCollapsed, setSecondCollapsed] = useState(false);
  const containerRef = useRef<HTMLDivElement>(null);

  // Save split to localStorage when it changes
  useEffect(() => {
    if (storageKey) {
      localStorage.setItem(storageKey, split.toString());
    }
  }, [split, storageKey]);

  useEffect(() => {
    if (isDragging) {
      const handleMouseMove = (e: MouseEvent) => {
        if (!containerRef.current) return;

        const rect = containerRef.current.getBoundingClientRect();
        let newSplit: number;

        if (direction === 'horizontal') {
          const offset = e.clientX - rect.left;
          newSplit = (offset / rect.width) * 100;
        } else {
          const offset = e.clientY - rect.top;
          newSplit = (offset / rect.height) * 100;
        }

        // Apply min/max constraints
        const minPercent = (minSize / (direction === 'horizontal' ? rect.width : rect.height)) * 100;
        const maxPercent = 100 - minPercent;
        
        newSplit = Math.max(minPercent, Math.min(maxPercent, newSplit));
        setSplit(newSplit);
      };

      const handleMouseUp = () => {
        setIsDragging(false);
        document.body.style.cursor = '';
        document.body.style.userSelect = '';
      };

      document.body.style.cursor = direction === 'horizontal' ? 'col-resize' : 'row-resize';
      document.body.style.userSelect = 'none';

      document.addEventListener('mousemove', handleMouseMove);
      document.addEventListener('mouseup', handleMouseUp);

      return () => {
        document.removeEventListener('mousemove', handleMouseMove);
        document.removeEventListener('mouseup', handleMouseUp);
        document.body.style.cursor = '';
        document.body.style.userSelect = '';
      };
    }
  }, [isDragging, direction, minSize]);

  const handleMouseDown = (e: React.MouseEvent) => {
    e.preventDefault();
    setIsDragging(true);
  };

  const toggleFirstPanel = () => {
    if (firstCollapsed) {
      setFirstCollapsed(false);
    } else {
      setFirstCollapsed(true);
      setSecondCollapsed(false);
    }
  };

  const toggleSecondPanel = () => {
    if (secondCollapsed) {
      setSecondCollapsed(false);
    } else {
      setSecondCollapsed(true);
      setFirstCollapsed(false);
    }
  };

  return (
    <div
      ref={containerRef}
      className={`split-panel ${direction} ${isDragging ? 'dragging' : ''}`}
    >
      {/* First Panel */}
      <div
        className={`split-section first ${firstCollapsed ? 'collapsed' : ''}`}
        style={
          firstCollapsed
            ? { [direction === 'horizontal' ? 'width' : 'height']: '0px' }
            : { [direction === 'horizontal' ? 'width' : 'height']: `${split}%` }
        }
      >
        {firstPanelTitle && (
          <div className="split-section-header">
            <span className="split-section-title">{firstPanelTitle}</span>
            {collapsible && (
              <button
                className="split-collapse-btn"
                onClick={toggleFirstPanel}
                title={firstCollapsed ? 'Expand' : 'Collapse'}
              >
                {firstCollapsed
                  ? direction === 'horizontal' ? '▶' : '▼'
                  : direction === 'horizontal' ? '◀' : '▲'}
              </button>
            )}
          </div>
        )}
        <div className="split-section-content">{firstPanel}</div>
      </div>

      {/* Divider */}
      {!firstCollapsed && !secondCollapsed && (
        <div
          className={`split-divider ${direction}`}
          onMouseDown={handleMouseDown}
        >
          <div className="split-divider-bar" />
        </div>
      )}

      {/* Second Panel */}
      <div
        className={`split-section second ${secondCollapsed ? 'collapsed' : ''}`}
        style={
          secondCollapsed
            ? { [direction === 'horizontal' ? 'width' : 'height']: '0px' }
            : { [direction === 'horizontal' ? 'width' : 'height']: `${100 - split}%` }
        }
      >
        {secondPanelTitle && (
          <div className="split-section-header">
            <span className="split-section-title">{secondPanelTitle}</span>
            {collapsible && (
              <button
                className="split-collapse-btn"
                onClick={toggleSecondPanel}
                title={secondCollapsed ? 'Expand' : 'Collapse'}
              >
                {secondCollapsed
                  ? direction === 'horizontal' ? '◀' : '▲'
                  : direction === 'horizontal' ? '▶' : '▼'}
              </button>
            )}
          </div>
        )}
        <div className="split-section-content">{secondPanel}</div>
      </div>
    </div>
  );
};

