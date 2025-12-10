/**
 * NAVA Finder Sidebar
 * 
 * Apple Finder-like sidebar with collapsible sections and view modes
 */

import React, { useState, useEffect, useRef } from 'react';
import {
  getSidebarPreferences,
  saveSidebarPreferences,
  toggleSection,
  setSectionViewMode,
  setSidebarWidth,
  type SidebarSection,
  type ViewMode,
  type SidebarPreferences,
} from '../../utils/sidebarStorage';
import './FinderSidebar.css';

interface FinderSidebarProps {
  onItemClick?: (item: { id: string; name: string; route?: string }) => void;
  onCollapse?: () => void;
}

export const FinderSidebar: React.FC<FinderSidebarProps> = ({ onItemClick, onCollapse }) => {
  const [prefs, setPrefs] = useState<SidebarPreferences>(getSidebarPreferences());
  const [resizing, setResizing] = useState(false);
  const [hoveredSection, setHoveredSection] = useState<string | null>(null);
  const sidebarRef = useRef<HTMLDivElement>(null);
  const resizeRef = useRef<HTMLDivElement>(null);

  // Refresh preferences
  const refreshPrefs = () => {
    setPrefs(getSidebarPreferences());
  };

  // Handle section toggle
  const handleToggleSection = (sectionId: string) => {
    toggleSection(sectionId);
    refreshPrefs();
  };

  // Handle view mode change
  const handleViewModeChange = (sectionId: string, viewMode: ViewMode) => {
    setSectionViewMode(sectionId, viewMode);
    refreshPrefs();
  };

  // Handle item click
  const handleItemClick = (item: { id: string; name: string; route?: string }) => {
    if (onItemClick) {
      onItemClick(item);
    } else if (item.route) {
      window.location.href = item.route;
    }
  };

  // Resize handler
  useEffect(() => {
    const handleMouseMove = (e: MouseEvent) => {
      if (!resizing || !sidebarRef.current) return;
      
      const rect = sidebarRef.current.getBoundingClientRect();
      const newWidth = Math.max(200, Math.min(500, e.clientX - rect.left));
      setSidebarWidth(newWidth);
      
      if (sidebarRef.current) {
        sidebarRef.current.style.width = `${newWidth}px`;
      }
    };

    const handleMouseUp = () => {
      setResizing(false);
      refreshPrefs();
    };

    if (resizing) {
      document.addEventListener('mousemove', handleMouseMove);
      document.addEventListener('mouseup', handleMouseUp);
      document.body.style.cursor = 'col-resize';
      document.body.style.userSelect = 'none';
    }

    return () => {
      document.removeEventListener('mousemove', handleMouseMove);
      document.removeEventListener('mouseup', handleMouseUp);
      document.body.style.cursor = '';
      document.body.style.userSelect = '';
    };
  }, [resizing]);

  // Initialize sidebar width
  useEffect(() => {
    if (sidebarRef.current) {
      sidebarRef.current.style.width = `${prefs.sidebarWidth}px`;
    }
  }, []);

  const getViewModeIcon = (viewMode: ViewMode) => {
    switch (viewMode) {
      case 'icons': return '⊞';
      case 'list': return '☰';
      case 'columns': return '▦';
      case 'gallery': return '▣';
      default: return '☰';
    }
  };

  return (
    <div 
      ref={sidebarRef}
      className="finder-sidebar"
      style={{ width: `${prefs.sidebarWidth}px` }}
    >
      {/* Hamburger Menu Button */}
      {onCollapse && (
        <div className="sidebar-header-bar">
          <button 
            className="sidebar-hamburger-btn"
            onClick={onCollapse}
            title="Collapse Sidebar"
          >
            <span className="hamburger-icon">☰</span>
          </button>
        </div>
      )}

      {/* Sections */}
      <div className="sidebar-sections">
        {prefs.sections
          .sort((a, b) => a.order - b.order)
          .map((section) => (
            <div key={section.id} className="sidebar-section-wrapper">
              <div
                className="sidebar-section-header"
                onMouseEnter={() => setHoveredSection(section.id)}
                onMouseLeave={() => setHoveredSection(null)}
              >
                <button
                  className="section-toggle"
                  onClick={() => handleToggleSection(section.id)}
                  title={section.collapsed ? 'Expand' : 'Collapse'}
                >
                  <span className={`toggle-icon ${section.collapsed ? 'collapsed' : ''}`}>
                    ▶
                  </span>
                </button>
                <span className="section-title">{section.title}</span>
                {hoveredSection === section.id && (
                  <div className="section-view-controls" onClick={(e) => e.stopPropagation()}>
                    <button
                      className="view-mode-btn"
                      onClick={() => {
                        const modes: ViewMode[] = ['list', 'icons', 'columns', 'gallery'];
                        const currentIndex = modes.indexOf(section.viewMode);
                        const nextIndex = (currentIndex + 1) % modes.length;
                        handleViewModeChange(section.id, modes[nextIndex]);
                      }}
                      title={`View: ${section.viewMode} (Click to cycle)`}
                    >
                      {getViewModeIcon(section.viewMode)}
                    </button>
                  </div>
                )}
              </div>
              
              {!section.collapsed && (
                <div className={`sidebar-section-items view-${section.viewMode}`}>
                  {section.items.map((item) => (
                    <div
                      key={item.id}
                      className={`sidebar-item item-type-${item.type}`}
                      onClick={() => handleItemClick(item)}
                      title={item.name}
                    >
                      {prefs.showIcons && (
                        <span className="item-icon">{item.icon}</span>
                      )}
                      <span className="item-name">{item.name}</span>
                      {item.type === 'folder' && item.children && item.children.length > 0 && (
                        <span className="item-badge">{item.children.length}</span>
                      )}
                    </div>
                  ))}
                </div>
              )}
            </div>
          ))}
      </div>

      {/* Resize Handle */}
      <div
        ref={resizeRef}
        className="sidebar-resize-handle"
        onMouseDown={() => setResizing(true)}
      />
    </div>
  );
};

