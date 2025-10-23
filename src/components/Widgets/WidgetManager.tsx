import React, { useState, useEffect } from 'react';
import { availableWidgets, type Widget } from './index';
import './WidgetManager.css';

interface OpenWidget {
  id: string;
  widgetId: string;
  position: { x: number; y: number };
  size: { width: number; height: number };
  zIndex: number;
}

interface WidgetManagerProps {
  onClose?: () => void;
}

export const WidgetManager: React.FC<WidgetManagerProps> = ({ onClose }) => {
  const [openWidgets, setOpenWidgets] = useState<OpenWidget[]>(() => {
    // Load saved widget positions from localStorage
    const savedWidgets = localStorage.getItem('navlambda-widgets');
    return savedWidgets ? JSON.parse(savedWidgets) : [];
  });
  const [nextZIndex, setNextZIndex] = useState(1000);
  const [showWidgetPicker, setShowWidgetPicker] = useState(false);

  // Save widget positions to localStorage whenever they change
  useEffect(() => {
    localStorage.setItem('navlambda-widgets', JSON.stringify(openWidgets));
  }, [openWidgets]);

  const openWidget = (widget: Widget) => {
    // Check if widget is already open
    const existingWidget = openWidgets.find(w => w.widgetId === widget.id);
    if (existingWidget) {
      // Bring existing widget to front
      bringToFront(existingWidget.id);
      return;
    }

    const newWidget: OpenWidget = {
      id: `${widget.id}-${Date.now()}`,
      widgetId: widget.id,
      position: { 
        x: Math.random() * 200 + 100, 
        y: Math.random() * 100 + 100 
      },
      size: widget.defaultSize || { width: 400, height: 300 },
      zIndex: nextZIndex,
    };

    setOpenWidgets(prev => [...prev, newWidget]);
    setNextZIndex(prev => prev + 1);
    setShowWidgetPicker(false);
  };

  const closeWidget = (widgetId: string) => {
    setOpenWidgets(prev => prev.filter(w => w.id !== widgetId));
  };

  const bringToFront = (widgetId: string) => {
    setOpenWidgets(prev => 
      prev.map(w => 
        w.id === widgetId 
          ? { ...w, zIndex: nextZIndex }
          : w
      )
    );
    setNextZIndex(prev => prev + 1);
  };

  const updateWidgetPosition = (widgetId: string, position: { x: number; y: number }) => {
    setOpenWidgets(prev =>
      prev.map(w =>
        w.id === widgetId ? { ...w, position } : w
      )
    );
  };

  const updateWidgetSize = (widgetId: string, size: { width: number; height: number }) => {
    setOpenWidgets(prev =>
      prev.map(w =>
        w.id === widgetId ? { ...w, size } : w
      )
    );
  };

  const groupedWidgets = availableWidgets.reduce((acc, widget) => {
    if (!acc[widget.category]) {
      acc[widget.category] = [];
    }
    acc[widget.category].push(widget);
    return acc;
  }, {} as Record<string, Widget[]>);

  return (
    <div className="widget-manager">
      {/* Widget Picker */}
      {showWidgetPicker && (
        <div className="widget-picker-overlay" onClick={() => setShowWidgetPicker(false)}>
          <div className="widget-picker" onClick={e => e.stopPropagation()}>
            <div className="widget-picker-header">
              <h3>Add Widget</h3>
              <button 
                className="widget-picker-close"
                onClick={() => setShowWidgetPicker(false)}
              >
                ×
              </button>
            </div>
            
            <div className="widget-categories">
              {Object.entries(groupedWidgets).map(([category, widgets]) => (
                <div key={category} className="widget-category">
                  <h4>{category.charAt(0).toUpperCase() + category.slice(1)}</h4>
                  <div className="widget-grid">
                    {widgets.map(widget => (
                      <button
                        key={widget.id}
                        className="widget-card"
                        onClick={() => openWidget(widget)}
                      >
                        <span className="widget-icon">{widget.icon}</span>
                        <span className="widget-name">{widget.name}</span>
                      </button>
                    ))}
                  </div>
                </div>
              ))}
            </div>
          </div>
        </div>
      )}

      {/* Floating Action Button */}
      <button 
        className="widget-fab"
        onClick={() => setShowWidgetPicker(true)}
        title="Add Widget"
      >
        +
      </button>

      {/* Open Widgets */}
      {openWidgets.map(openWidget => {
        const widget = availableWidgets.find(w => w.id === openWidget.widgetId);
        if (!widget) return null;

        const WidgetComponent = widget.component;

        return (
          <DraggableWidget
            key={openWidget.id}
            widget={openWidget}
            onClose={() => closeWidget(openWidget.id)}
            onFocus={() => bringToFront(openWidget.id)}
            onPositionChange={(position) => updateWidgetPosition(openWidget.id, position)}
            onSizeChange={(size) => updateWidgetSize(openWidget.id, size)}
            resizable={widget.resizable}
          >
            <WidgetComponent onClose={() => closeWidget(openWidget.id)} />
          </DraggableWidget>
        );
      })}

      {/* Widget Manager Controls */}
      <div className="widget-manager-controls">
        <div className="widget-count">
          {openWidgets.length} widget{openWidgets.length !== 1 ? 's' : ''} open
        </div>
        {openWidgets.length > 0 && (
          <button 
            className="close-all-btn"
            onClick={() => setOpenWidgets([])}
          >
            Close All
          </button>
        )}
        {onClose && (
          <button className="widget-manager-close" onClick={onClose}>
            Close Manager
          </button>
        )}
      </div>
    </div>
  );
};

// Draggable Widget Wrapper
interface DraggableWidgetProps {
  widget: OpenWidget;
  children: React.ReactNode;
  onClose: () => void;
  onFocus: () => void;
  onPositionChange: (position: { x: number; y: number }) => void;
  onSizeChange: (size: { width: number; height: number }) => void;
  resizable?: boolean;
}

const DraggableWidget: React.FC<DraggableWidgetProps> = ({
  widget,
  children,
  onClose,
  onFocus,
  onPositionChange,
  onSizeChange,
  resizable = true,
}) => {
  const [isDragging, setIsDragging] = useState(false);
  const [isResizing, setIsResizing] = useState(false);
  const [dragStart, setDragStart] = useState({ x: 0, y: 0 });
  const [resizeStart, setResizeStart] = useState({ x: 0, y: 0, width: 0, height: 0 });

  const handleMouseDown = (e: React.MouseEvent) => {
    if (e.target === e.currentTarget || (e.target as HTMLElement).classList.contains('widget-header')) {
      setIsDragging(true);
      setDragStart({
        x: e.clientX - widget.position.x,
        y: e.clientY - widget.position.y,
      });
      onFocus();
    }
  };

  const handleResizeMouseDown = (e: React.MouseEvent) => {
    e.stopPropagation();
    setIsResizing(true);
    setResizeStart({
      x: e.clientX,
      y: e.clientY,
      width: widget.size.width,
      height: widget.size.height,
    });
    onFocus();
  };

  const handleMouseMove = (e: MouseEvent) => {
    if (isDragging) {
      onPositionChange({
        x: e.clientX - dragStart.x,
        y: e.clientY - dragStart.y,
      });
    } else if (isResizing) {
      const deltaX = e.clientX - resizeStart.x;
      const deltaY = e.clientY - resizeStart.y;
      
      onSizeChange({
        width: Math.max(200, resizeStart.width + deltaX),
        height: Math.max(150, resizeStart.height + deltaY),
      });
    }
  };

  const handleMouseUp = () => {
    setIsDragging(false);
    setIsResizing(false);
  };

  React.useEffect(() => {
    if (isDragging || isResizing) {
      document.addEventListener('mousemove', handleMouseMove);
      document.addEventListener('mouseup', handleMouseUp);
      
      return () => {
        document.removeEventListener('mousemove', handleMouseMove);
        document.removeEventListener('mouseup', handleMouseUp);
      };
    }
  }, [isDragging, isResizing, dragStart, resizeStart]);

  return (
    <div
      className={`draggable-widget ${isDragging ? 'dragging' : ''} ${isResizing ? 'resizing' : ''}`}
      style={{
        position: 'fixed',
        left: widget.position.x,
        top: widget.position.y,
        width: widget.size.width,
        height: widget.size.height,
        zIndex: widget.zIndex,
      }}
      onMouseDown={handleMouseDown}
      onClick={onFocus}
    >
      {children}
      
      {resizable && (
        <div 
          className="resize-handle"
          onMouseDown={handleResizeMouseDown}
        />
      )}
    </div>
  );
};