import React, { useState, useEffect, useRef } from 'react';
import { Search, Command } from 'lucide-react';
import './CommandPalette.css';

interface CommandPaletteProps {
  isOpen: boolean;
  onClose: () => void;
  onExecuteCommand: (command: string) => void;
}

interface Command {
  id: string;
  label: string;
  category: string;
  shortcut?: string;
  action: string;
}

export const CommandPalette: React.FC<CommandPaletteProps> = ({
  isOpen,
  onClose,
  onExecuteCommand,
}) => {
  const [query, setQuery] = useState('');
  const [selectedIndex, setSelectedIndex] = useState(0);
  const inputRef = useRef<HTMLInputElement>(null);

  const commands: Command[] = [
    { id: '1', label: 'Profiles', category: 'Settings', action: 'profiles' },
    { id: '2', label: 'Settings', category: 'Settings', shortcut: '⌘,', action: 'settings' },
    { id: '3', label: 'Extensions', category: 'View', shortcut: '⌘⇧X', action: 'extensions' },
    { id: '4', label: 'Keyboard Shortcuts', category: 'Settings', shortcut: '⌘K ⌘S', action: 'keyboard-shortcuts' },
    { id: '5', label: 'Snippets', category: 'Settings', action: 'snippets' },
    { id: '6', label: 'Tasks', category: 'Settings', action: 'tasks' },
    { id: '7', label: 'Themes', category: 'Settings', action: 'themes' },
    { id: '8', label: 'Backup and Sync Settings...', category: 'Settings', action: 'backup-sync' },
    { id: '9', label: 'Show Update Release Notes', category: 'Help', action: 'release-notes' },
    { id: '10', label: 'Restart to Update', category: 'Help', action: 'restart-update' },
    { id: '11', label: 'Show Opened Editors', category: 'View', action: 'opened-editors' },
    { id: '12', label: 'Close All', category: 'File', shortcut: '⌘K W', action: 'close-all' },
    { id: '13', label: 'Close Saved', category: 'File', shortcut: '⌘K U', action: 'close-saved' },
    { id: '14', label: 'Toggle Preview Editors', category: 'View', action: 'toggle-preview' },
    { id: '15', label: 'Lock Group', category: 'View', action: 'lock-group' },
    { id: '16', label: 'Configure Editors', category: 'Settings', action: 'configure-editors' },
  ];

  const filteredCommands = commands.filter(cmd =>
    cmd.label.toLowerCase().includes(query.toLowerCase()) ||
    cmd.category.toLowerCase().includes(query.toLowerCase())
  );

  useEffect(() => {
    if (isOpen && inputRef.current) {
      inputRef.current.focus();
    }
  }, [isOpen]);

  useEffect(() => {
    const handleKeyDown = (e: KeyboardEvent) => {
      if (!isOpen) return;

      if (e.key === 'Escape') {
        onClose();
      } else if (e.key === 'ArrowDown') {
        e.preventDefault();
        setSelectedIndex(prev => (prev + 1) % filteredCommands.length);
      } else if (e.key === 'ArrowUp') {
        e.preventDefault();
        setSelectedIndex(prev => (prev - 1 + filteredCommands.length) % filteredCommands.length);
      } else if (e.key === 'Enter') {
        e.preventDefault();
        if (filteredCommands[selectedIndex]) {
          onExecuteCommand(filteredCommands[selectedIndex].action);
          onClose();
        }
      }
    };

    document.addEventListener('keydown', handleKeyDown);
    return () => document.removeEventListener('keydown', handleKeyDown);
  }, [isOpen, selectedIndex, filteredCommands, onClose, onExecuteCommand]);

  if (!isOpen) return null;

  return (
    <div className="command-palette-overlay" onClick={onClose}>
      <div className="command-palette" onClick={(e) => e.stopPropagation()}>
        <div className="command-palette-header">
          <Search size={18} />
          <input
            ref={inputRef}
            type="text"
            placeholder="Command Palette..."
            value={query}
            onChange={(e) => setQuery(e.target.value)}
            className="command-palette-input"
          />
          <div className="command-palette-shortcut">
            <Command size={14} />
            <span>⇧P</span>
          </div>
        </div>

        <div className="command-palette-results">
          {filteredCommands.map((cmd, index) => (
            <div
              key={cmd.id}
              className={`command-item ${index === selectedIndex ? 'selected' : ''}`}
              onClick={() => {
                onExecuteCommand(cmd.action);
                onClose();
              }}
              onMouseEnter={() => setSelectedIndex(index)}
            >
              <div className="command-info">
                <span className="command-label">{cmd.label}</span>
                <span className="command-category">{cmd.category}</span>
              </div>
              {cmd.shortcut && (
                <div className="command-shortcut">{cmd.shortcut}</div>
              )}
            </div>
          ))}
        </div>
      </div>
    </div>
  );
};

