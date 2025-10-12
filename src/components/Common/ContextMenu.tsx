import React, { useEffect, useRef } from 'react';

interface ContextMenuItem {
  label: string;
  icon?: string;
  action: () => void;
  separator?: boolean;
  disabled?: boolean;
}

interface ContextMenuProps {
  x: number;
  y: number;
  items: ContextMenuItem[];
  onClose: () => void;
  visible: boolean;
}

export const ContextMenu: React.FC<ContextMenuProps> = ({
  x,
  y,
  items,
  onClose,
  visible,
}) => {
  const menuRef = useRef<HTMLDivElement>(null);

  useEffect(() => {
    const handleClickOutside = (event: MouseEvent) => {
      if (menuRef.current && !menuRef.current.contains(event.target as Node)) {
        onClose();
      }
    };

    if (visible) {
      document.addEventListener('mousedown', handleClickOutside);
    }

    return () => {
      document.removeEventListener('mousedown', handleClickOutside);
    };
  }, [visible, onClose]);

  if (!visible) return null;

  return (
    <div
      ref={menuRef}
      className="context-menu"
      style={{ left: x, top: y }}
    >
      {items.map((item, index) => (
        <React.Fragment key={index}>
          {item.separator ? (
            <div className="menu-separator" />
          ) : (
            <button
              className={`menu-item ${item.disabled ? 'disabled' : ''}`}
              onClick={() => {
                if (!item.disabled) {
                  item.action();
                  onClose();
                }
              }}
              disabled={item.disabled}
            >
              {item.icon && <span className="menu-icon">{item.icon}</span>}
              <span className="menu-label">{item.label}</span>
            </button>
          )}
        </React.Fragment>
      ))}
    </div>
  );
};

