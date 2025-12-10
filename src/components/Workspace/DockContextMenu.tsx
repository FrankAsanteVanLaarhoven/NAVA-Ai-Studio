import React, { useEffect, useRef } from 'react';
import './DockContextMenu.css';

export interface DockAppPreferences {
  openAtLogin: boolean;
  assignToDesktop: 'all' | 'this' | 'none';
  showInFinder?: boolean;
}

interface DockContextMenuProps {
  appId: string;
  appName: string;
  position: { x: number; y: number };
  onClose: () => void;
  onRemoveFromDock: () => void;
  onOpenAtLogin: (enabled: boolean) => void;
  onShowInFinder: () => void;
  onAssignToDesktop: (desktop: 'all' | 'this' | 'none') => void;
  onShowRecents: () => void;
  onOpen: () => void;
  preferences: DockAppPreferences;
}

export const DockContextMenu: React.FC<DockContextMenuProps> = ({
  appId,
  appName,
  position,
  onClose,
  onRemoveFromDock,
  onOpenAtLogin,
  onShowInFinder,
  onAssignToDesktop,
  onShowRecents,
  onOpen,
  preferences,
}) => {
  const menuRef = useRef<HTMLDivElement>(null);
  const submenuRef = useRef<HTMLDivElement>(null);
  const [showOptionsSubmenu, setShowOptionsSubmenu] = React.useState(false);

  // Close menu when clicking outside
  useEffect(() => {
    const handleClickOutside = (event: MouseEvent) => {
      if (menuRef.current && !menuRef.current.contains(event.target as Node)) {
        if (submenuRef.current && submenuRef.current.contains(event.target as Node)) {
          return; // Don't close if clicking in submenu
        }
        onClose();
      }
    };

    const handleEscape = (event: KeyboardEvent) => {
      if (event.key === 'Escape') {
        onClose();
      }
    };

    document.addEventListener('mousedown', handleClickOutside);
    document.addEventListener('keydown', handleEscape);

    return () => {
      document.removeEventListener('mousedown', handleClickOutside);
      document.removeEventListener('keydown', handleEscape);
    };
  }, [onClose]);

  // Position menu above the dock icon
  const menuStyle: React.CSSProperties = {
    position: 'fixed',
    left: `${position.x}px`,
    top: `${position.y - 200}px`, // Position above the icon
    zIndex: 10000,
  };

  const submenuStyle: React.CSSProperties = {
    position: 'fixed',
    left: `${position.x + 200}px`, // Position to the right of main menu
    top: `${position.y - 200}px`,
    zIndex: 10001,
  };

  return (
    <>
      {/* Main Menu */}
      <div ref={menuRef} className="dock-context-menu" style={menuStyle}>
        <div className="context-menu-item" onClick={onOpen}>
          <span>Open</span>
        </div>
        <div className="context-menu-item" onClick={onShowRecents}>
          <span>Show Recents</span>
        </div>
        <div className="context-menu-divider" />
        <div
          className="context-menu-item has-submenu"
          onMouseEnter={() => setShowOptionsSubmenu(true)}
          onMouseLeave={() => setShowOptionsSubmenu(false)}
        >
          <span>Options</span>
          <span className="submenu-arrow">›</span>
        </div>
      </div>

      {/* Options Submenu */}
      {showOptionsSubmenu && (
        <div ref={submenuRef} className="dock-context-menu dock-context-submenu" style={submenuStyle}>
          <div className="context-menu-item" onClick={onRemoveFromDock}>
            <span>Remove from Dock</span>
          </div>
          <div className="context-menu-item" onClick={() => onOpenAtLogin(!preferences.openAtLogin)}>
            <span>Open at Login</span>
            {preferences.openAtLogin && <span className="checkmark">✓</span>}
          </div>
          <div className="context-menu-item" onClick={onShowInFinder}>
            <span>Show in Finder</span>
          </div>
          <div className="context-menu-divider" />
          <div className="context-menu-header">
            <span>Assign To</span>
          </div>
          <div
            className="context-menu-item"
            onClick={() => onAssignToDesktop('all')}
          >
            <span>All Desktops</span>
            {preferences.assignToDesktop === 'all' && <span className="checkmark">✓</span>}
          </div>
          <div
            className="context-menu-item"
            onClick={() => onAssignToDesktop('this')}
          >
            <span>This Desktop</span>
            {preferences.assignToDesktop === 'this' && <span className="checkmark">✓</span>}
          </div>
          <div
            className="context-menu-item"
            onClick={() => onAssignToDesktop('none')}
          >
            <span>None</span>
            {preferences.assignToDesktop === 'none' && <span className="checkmark">✓</span>}
          </div>
        </div>
      )}
    </>
  );
};

