/**
 * Window Manager Component
 * 
 * Handles draggable windows and blur overlays for the OS Desktop workspace
 */

import React, { useState, useEffect, useCallback, useRef } from 'react';
import './WindowManager.css';

interface Window {
  id: string;
  title: string;
  content: React.ReactNode;
  x: number;
  y: number;
  width: number;
  height: number;
  zIndex: number;
  isMinimized: boolean;
  isMaximized: boolean;
}

interface WindowManagerProps {
  windows?: Window[];
  onWindowUpdate?: (windows: Window[]) => void;
}

/**
 * WindowManager - Manages draggable windows in the desktop
 */
export const WindowManager: React.FC<WindowManagerProps> = ({ 
  windows = [],
  onWindowUpdate 
}) => {
  const [windowList, setWindowList] = useState<Window[]>(windows);
  const [dragging, setDragging] = useState<string | null>(null);
  const dragOffsetRef = useRef({ x: 0, y: 0 });
  const draggingRef = useRef<string | null>(null);

  useEffect(() => {
    if (onWindowUpdate) {
      onWindowUpdate(windowList);
    }
  }, [windowList, onWindowUpdate]);

  const handleMouseDown = useCallback((windowId: string, e: React.MouseEvent) => {
    const window = windowList.find(w => w.id === windowId);
    if (!window) return;

    draggingRef.current = windowId;
    setDragging(windowId);
    dragOffsetRef.current = {
      x: e.clientX - window.x,
      y: e.clientY - window.y,
    };

    // Bring window to front
    const maxZ = Math.max(...windowList.map(w => w.zIndex), 0);
    setWindowList(prev => prev.map(w => 
      w.id === windowId ? { ...w, zIndex: maxZ + 1 } : w
    ));
  }, [windowList]);

  const handleMouseMove = useCallback((e: MouseEvent) => {
    if (!draggingRef.current) return;

    setWindowList(prev => prev.map(w => 
      w.id === draggingRef.current 
        ? { ...w, x: e.clientX - dragOffsetRef.current.x, y: e.clientY - dragOffsetRef.current.y }
        : w
    ));
  }, []);

  const handleMouseUp = useCallback(() => {
    draggingRef.current = null;
    setDragging(null);
  }, []);

  useEffect(() => {
    if (dragging) {
      window.addEventListener('mousemove', handleMouseMove);
      window.addEventListener('mouseup', handleMouseUp);
      return () => {
        window.removeEventListener('mousemove', handleMouseMove);
        window.removeEventListener('mouseup', handleMouseUp);
      };
    }
  }, [dragging, handleMouseMove, handleMouseUp]);

  if (windowList.length === 0) {
    return null;
  }

  return (
    <div className="window-manager">
      {windowList.map(window => (
        <div
          key={window.id}
          className={`window ${window.isMinimized ? 'minimized' : ''} ${window.isMaximized ? 'maximized' : ''}`}
          style={{
            position: 'absolute',
            left: window.x,
            top: window.y,
            width: window.isMaximized ? '100%' : window.width,
            height: window.isMaximized ? '100%' : window.height,
            zIndex: window.zIndex,
            display: window.isMinimized ? 'none' : 'block',
          }}
        >
          <div
            className="window-header"
            onMouseDown={(e) => handleMouseDown(window.id, e)}
          >
            <span className="window-title">{window.title}</span>
            <div className="window-controls">
              <button className="window-btn minimize">−</button>
              <button className="window-btn maximize">□</button>
              <button className="window-btn close">×</button>
            </div>
          </div>
          <div className="window-content">
            {window.content}
          </div>
        </div>
      ))}
    </div>
  );
};

/**
 * BlurOverlay - Provides blur overlay for modals
 */
interface BlurOverlayProps {
  isVisible?: boolean;
  onClick?: () => void;
  zIndex?: number;
}

export const BlurOverlay: React.FC<BlurOverlayProps> = ({ 
  isVisible = false,
  onClick,
  zIndex = 999
}) => {
  if (!isVisible) {
    return null;
  }

  return (
    <div
      className="blur-overlay"
      style={{
        position: 'fixed',
        top: 0,
        left: 0,
        right: 0,
        bottom: 0,
        background: 'rgba(0, 0, 0, 0.5)',
        backdropFilter: 'blur(4px)',
        zIndex,
        pointerEvents: onClick ? 'all' : 'none',
      }}
      onClick={onClick}
    />
  );
};

