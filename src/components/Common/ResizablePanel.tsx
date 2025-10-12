import React, { useState, useRef, useEffect } from 'react';
import './ResizablePanel.css';

interface ResizablePanelProps {
  children: React.ReactNode;
  defaultWidth?: number;
  minWidth?: number;
  maxWidth?: number;
  side: 'left' | 'right';
  isCollapsed?: boolean;
  onToggleCollapse?: () => void;
  title?: string;
}

export const ResizablePanel: React.FC<ResizablePanelProps> = ({
  children,
  defaultWidth = 300,
  minWidth = 200,
  maxWidth = 800,
  side,
  isCollapsed = false,
  onToggleCollapse,
  title
}) => {
  const [width, setWidth] = useState(defaultWidth);
  const [isDragging, setIsDragging] = useState(false);
  const panelRef = useRef<HTMLDivElement>(null);

  useEffect(() => {
    if (isDragging) {
      const handleMouseMove = (e: MouseEvent) => {
        if (!panelRef.current) return;

        const rect = panelRef.current.getBoundingClientRect();
        let newWidth: number;

        if (side === 'left') {
          newWidth = e.clientX - rect.left;
        } else {
          newWidth = rect.right - e.clientX;
        }

        newWidth = Math.max(minWidth, Math.min(maxWidth, newWidth));
        setWidth(newWidth);
      };

      const handleMouseUp = () => {
        setIsDragging(false);
      };

      document.addEventListener('mousemove', handleMouseMove);
      document.addEventListener('mouseup', handleMouseUp);

      return () => {
        document.removeEventListener('mousemove', handleMouseMove);
        document.removeEventListener('mouseup', handleMouseUp);
      };
    }
  }, [isDragging, side, minWidth, maxWidth]);

  const handleMouseDown = (e: React.MouseEvent) => {
    e.preventDefault();
    setIsDragging(true);
  };

  return (
    <div
      ref={panelRef}
      className={`resizable-panel ${side}-panel ${isCollapsed ? 'collapsed' : ''} ${isDragging ? 'dragging' : ''}`}
      style={{ width: isCollapsed ? '0px' : `${width}px` }}
    >
      {!isCollapsed && (
        <>
          {title && (
            <div className="panel-header">
              <span className="panel-title">{title}</span>
              {onToggleCollapse && (
                <button className="collapse-btn" onClick={onToggleCollapse} title="Collapse">
                  <span className="codicon codicon-chevron-left"></span>
                </button>
              )}
            </div>
          )}
          <div className="panel-content">{children}</div>
        </>
      )}
      
      <div
        className={`resize-handle ${side}-handle`}
        onMouseDown={handleMouseDown}
      >
        <div className="resize-handle-bar"></div>
      </div>
    </div>
  );
};

