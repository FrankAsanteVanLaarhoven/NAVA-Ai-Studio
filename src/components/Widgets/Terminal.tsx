import React, { useState, useEffect, useRef } from 'react';
import './Terminal.css';

interface TerminalCommand {
  command: string;
  output: string;
  timestamp: Date;
}

interface TerminalProps {
  onClose?: () => void;
}

// Mock terminal commands for demonstration
const mockCommands: Record<string, string> = {
  'help': `Available commands:
  help     - Show this help message
  ls       - List directory contents
  pwd      - Print working directory
  cd       - Change directory
  cat      - Display file contents
  echo     - Display text
  date     - Show current date and time
  whoami   - Show current user
  clear    - Clear terminal
  history  - Show command history`,
  
  'ls': `total 24
drwxr-xr-x  8 user  staff   256 Jan 13 10:30 .
drwxr-xr-x  5 user  staff   160 Jan 12 15:45 ..
-rw-r--r--  1 user  staff  1024 Jan 13 09:15 README.md
drwxr-xr-x  4 user  staff   128 Jan 13 10:30 src
-rw-r--r--  1 user  staff  2048 Jan 12 16:20 package.json
drwxr-xr-x  3 user  staff    96 Jan 11 14:10 public`,
  
  'pwd': '/Users/user/projects/navl-studio',
  'whoami': 'user',
  'date': new Date().toString(),
};

export const Terminal: React.FC<TerminalProps> = ({ onClose }) => {
  const [history, setHistory] = useState<TerminalCommand[]>([
    {
      command: 'Welcome to NAVΛ Studio Terminal',
      output: 'Type "help" to see available commands.',
      timestamp: new Date(),
    },
  ]);
  const [currentInput, setCurrentInput] = useState('');
  const [commandHistory, setCommandHistory] = useState<string[]>([]);
  const [historyIndex, setHistoryIndex] = useState(-1);
  const [currentDirectory, setCurrentDirectory] = useState('~/projects/navl-studio');
  
  const terminalRef = useRef<HTMLDivElement>(null);
  const inputRef = useRef<HTMLInputElement>(null);

  // Auto-scroll to bottom when new content is added
  useEffect(() => {
    if (terminalRef.current) {
      terminalRef.current.scrollTop = terminalRef.current.scrollHeight;
    }
  }, [history]);

  // Focus input when terminal is clicked
  useEffect(() => {
    if (inputRef.current) {
      inputRef.current.focus();
    }
  }, []);

  const executeCommand = (command: string) => {
    const trimmedCommand = command.trim();
    if (!trimmedCommand) return;

    // Add to command history
    const newCommandHistory = [...commandHistory, trimmedCommand];
    setCommandHistory(newCommandHistory);
    setHistoryIndex(-1);

    let output = '';
    const lowerCommand = trimmedCommand.toLowerCase();

    if (lowerCommand === 'clear') {
      setHistory([]);
      setCurrentInput('');
      return;
    }

    if (lowerCommand === 'history') {
      output = newCommandHistory.map((cmd, index) => `${index + 1}  ${cmd}`).join('\n');
    } else if (lowerCommand.startsWith('echo ')) {
      output = trimmedCommand.substring(5);
    } else if (lowerCommand.startsWith('cd ')) {
      const path = trimmedCommand.substring(3).trim();
      if (path === '..') {
        const pathParts = currentDirectory.split('/');
        pathParts.pop();
        setCurrentDirectory(pathParts.join('/') || '/');
        output = '';
      } else if (path === '~' || path === '') {
        setCurrentDirectory('~');
        output = '';
      } else {
        setCurrentDirectory(`${currentDirectory}/${path}`.replace('//', '/'));
        output = '';
      }
    } else if (lowerCommand.startsWith('cat ')) {
      const filename = trimmedCommand.substring(4).trim();
      if (filename === 'README.md') {
        output = `# NAVΛ Studio IDE

Welcome to the world's first IDE for Van Laarhoven Navigation Calculus Programming!

## Features
- Advanced code editing
- ROS 2 integration
- Mathematical visualization
- Real-time collaboration`;
      } else if (filename === 'package.json') {
        output = `{
  "name": "navl-studio",
  "version": "1.0.0",
  "description": "IDE for Van Laarhoven Navigation Calculus",
  "main": "src/main.tsx",
  "scripts": {
    "dev": "vite",
    "build": "vite build"
  }
}`;
      } else {
        output = `cat: ${filename}: No such file or directory`;
      }
    } else if (mockCommands[lowerCommand]) {
      output = mockCommands[lowerCommand];
    } else {
      output = `Command not found: ${trimmedCommand}. Type "help" for available commands.`;
    }

    const newCommand: TerminalCommand = {
      command: trimmedCommand,
      output,
      timestamp: new Date(),
    };

    setHistory(prev => [...prev, newCommand]);
    setCurrentInput('');
  };

  const handleKeyDown = (e: React.KeyboardEvent) => {
    if (e.key === 'Enter') {
      executeCommand(currentInput);
    } else if (e.key === 'ArrowUp') {
      e.preventDefault();
      if (commandHistory.length > 0) {
        const newIndex = historyIndex === -1 ? commandHistory.length - 1 : Math.max(0, historyIndex - 1);
        setHistoryIndex(newIndex);
        setCurrentInput(commandHistory[newIndex]);
      }
    } else if (e.key === 'ArrowDown') {
      e.preventDefault();
      if (historyIndex !== -1) {
        const newIndex = historyIndex + 1;
        if (newIndex >= commandHistory.length) {
          setHistoryIndex(-1);
          setCurrentInput('');
        } else {
          setHistoryIndex(newIndex);
          setCurrentInput(commandHistory[newIndex]);
        }
      }
    } else if (e.key === 'Tab') {
      e.preventDefault();
      // Simple tab completion for common commands
      const availableCommands = Object.keys(mockCommands);
      const matches = availableCommands.filter(cmd => cmd.startsWith(currentInput.toLowerCase()));
      if (matches.length === 1) {
        setCurrentInput(matches[0]);
      }
    }
  };

  const formatTimestamp = (date: Date) => {
    return date.toLocaleTimeString('en-US', { 
      hour12: false,
      hour: '2-digit',
      minute: '2-digit',
      second: '2-digit'
    });
  };

  return (
    <div className="terminal-widget">
      <div className="terminal-header">
        <div className="terminal-title">
          <span className="terminal-icon">⚡</span>
          <h3>Terminal</h3>
        </div>
        <div className="terminal-controls">
          <button className="terminal-btn minimize">−</button>
          <button className="terminal-btn maximize">□</button>
          {onClose && (
            <button className="terminal-btn close" onClick={onClose}>
              ×
            </button>
          )}
        </div>
      </div>

      <div 
        className="terminal-content" 
        ref={terminalRef}
        onClick={() => inputRef.current?.focus()}
      >
        {history.map((entry, index) => (
          <div key={index} className="terminal-entry">
            {entry.command !== 'Welcome to NAVΛ Studio Terminal' && (
              <div className="terminal-command">
                <span className="terminal-prompt">
                  user@navl-studio:{currentDirectory}$ 
                </span>
                <span className="terminal-command-text">{entry.command}</span>
                <span className="terminal-timestamp">
                  [{formatTimestamp(entry.timestamp)}]
                </span>
              </div>
            )}
            {entry.output && (
              <div className="terminal-output">
                {entry.output.split('\n').map((line, lineIndex) => (
                  <div key={lineIndex} className="terminal-line">
                    {line}
                  </div>
                ))}
              </div>
            )}
          </div>
        ))}

        <div className="terminal-input-line">
          <span className="terminal-prompt">
            user@navl-studio:{currentDirectory}$ 
          </span>
          <input
            ref={inputRef}
            type="text"
            value={currentInput}
            onChange={(e) => setCurrentInput(e.target.value)}
            onKeyDown={handleKeyDown}
            className="terminal-input"
            placeholder="Type a command..."
            autoComplete="off"
            spellCheck={false}
          />
        </div>
      </div>

      <div className="terminal-footer">
        <div className="terminal-status">
          <span>Ready</span>
          <span>•</span>
          <span>{history.length} commands</span>
          <span>•</span>
          <span>Use ↑↓ for history, Tab for completion</span>
        </div>
      </div>
    </div>
  );
};