import React, { useState, useRef, useEffect } from 'react';
import './ResizablePanel.css';

interface ResizablePanelProps {
  children: React.ReactNode;
  defaultWidth?: number;
  minWidth?: number;
  maxWidth?: number;
  defaultHeight?: number;
  minHeight?: number;
  maxHeight?: number;
  side: 'left' | 'right' | 'top' | 'bottom';
  orientation?: 'horizontal' | 'vertical';
  isCollapsed?: boolean;
  onToggleCollapse?: () => void;
  onResize?: (size: number) => void;
  title?: string;
  storageKey?: string;
}

export const ResizablePanel: React.FC<ResizablePanelProps> = ({
  children,
  defaultWidth = 300,
  minWidth = 200,
  maxWidth = 800,
  defaultHeight = 300,
  minHeight = 150,
  maxHeight = 600,
  side,
  orientation = side === 'top' || side === 'bottom' ? 'vertical' : 'horizontal',
  isCollapsed = false,
  onToggleCollapse,
  onResize,
  title,
  storageKey
}) => {
  const isVertical = orientation === 'vertical';
  
  // Load saved size from localStorage
  const getInitialSize = () => {
    if (storageKey) {
      const saved = localStorage.getItem(storageKey);
      if (saved) return parseInt(saved, 10);
    }
    return isVertical ? defaultHeight : defaultWidth;
  };

  const [size, setSize] = useState(getInitialSize());
  const [isDragging, setIsDragging] = useState(false);
  const panelRef = useRef<HTMLDivElement>(null);

  // Save size to localStorage when it changes
  useEffect(() => {
    if (storageKey && !isCollapsed) {
      localStorage.setItem(storageKey, size.toString());
    }
  }, [size, storageKey, isCollapsed]);

  useEffect(() => {
    if (isDragging) {
      const handleMouseMove = (e: MouseEvent) => {
        if (!panelRef.current) return;

        const rect = panelRef.current.getBoundingClientRect();
        let newSize: number;

        if (isVertical) {
          // Vertical resizing (for top/bottom panels)
          if (side === 'top') {
            newSize = e.clientY - rect.top;
          } else {
            newSize = rect.bottom - e.clientY;
          }
          newSize = Math.max(minHeight, Math.min(maxHeight, newSize));
        } else {
          // Horizontal resizing (for left/right panels)
          if (side === 'left') {
            newSize = e.clientX - rect.left;
          } else {
            newSize = rect.right - e.clientX;
          }
          newSize = Math.max(minWidth, Math.min(maxWidth, newSize));
        }

        setSize(newSize);
        onResize?.(newSize);
      };

      const handleMouseUp = () => {
        setIsDragging(false);
        document.body.style.cursor = '';
        document.body.style.userSelect = '';
      };

      document.body.style.cursor = isVertical ? 'ns-resize' : 'col-resize';
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
  }, [isDragging, side, minWidth, maxWidth, minHeight, maxHeight, isVertical, onResize]);

  const handleMouseDown = (e: React.MouseEvent) => {
    e.preventDefault();
    setIsDragging(true);
  };

  const getCollapseIcon = () => {
    switch (side) {
      case 'left': return '◀';
      case 'right': return '▶';
      case 'top': return '▲';
      case 'bottom': return '▼';
      default: return '◀';
    }
  };

  const getExpandIcon = () => {
    switch (side) {
      case 'left': return '▶';
      case 'right': return '◀';
      case 'top': return '▼';
      case 'bottom': return '▲';
      default: return '▶';
    }
  };

  // When expanding from collapsed, restore saved size or use default
  const panelStyle = isVertical
    ? { 
        height: isCollapsed ? '0px' : `${size}px`,
        minHeight: isCollapsed ? '0px' : `${minHeight}px`,
        maxHeight: isCollapsed ? '0px' : `${maxHeight}px`,
      }
    : { 
        width: isCollapsed ? '0px' : `${size}px`,
        minWidth: isCollapsed ? '0px' : `${minWidth}px`,
        maxWidth: isCollapsed ? '0px' : `${maxWidth}px`,
      };

  return (
    <div
      ref={panelRef}
      className={`resizable-panel ${side}-panel ${isVertical ? 'vertical' : 'horizontal'} ${isCollapsed ? 'collapsed' : ''} ${isDragging ? 'dragging' : ''}`}
      style={panelStyle}
    >
      {!isCollapsed && (
        <>
          {title && (
            <div className="panel-header">
              <span className="panel-title">{title}</span>
              {onToggleCollapse && (
                <button className="collapse-btn" onClick={onToggleCollapse} title={`Collapse ${side} panel`}>
                  {getCollapseIcon()}
                </button>
              )}
            </div>
          )}
          <div className="panel-content">{children}</div>
        </>
      )}
      
      {/* Expand button when collapsed */}
      {isCollapsed && onToggleCollapse && (
        <button 
          className="expand-btn" 
          onClick={onToggleCollapse} 
          title={`Expand ${side} panel`}
          style={
            isVertical ? {
              position: 'absolute',
              left: '50%',
              [side === 'top' ? 'bottom' : 'top']: '-32px',
              transform: 'translateX(-50%)',
              zIndex: 1000
            } : {
              position: 'absolute',
              [side === 'left' ? 'right' : 'left']: '-32px',
              top: '50%',
              transform: 'translateY(-50%)',
              zIndex: 1000
            }
          }
        >
          {getExpandIcon()}
        </button>
      )}
      
      {!isCollapsed && (
        <div
          className={`resize-handle ${side}-handle ${isVertical ? 'vertical-handle' : 'horizontal-handle'}`}
          onMouseDown={handleMouseDown}
        >
          <div className="resize-handle-bar"></div>
        </div>
      )}
    </div>
  );
};

