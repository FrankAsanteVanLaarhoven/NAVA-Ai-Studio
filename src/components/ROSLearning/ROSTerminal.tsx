import React, { useState, useEffect, useRef } from 'react';
import { rosTerminalService, TerminalOutput } from '../../services/ros-terminal-service';
import './ROSTerminal.css';

interface ROSTerminalProps {
  isOpen: boolean;
  onClose: () => void;
  initialCommand?: string;
}

export const ROSTerminal: React.FC<ROSTerminalProps> = ({ isOpen, onClose, initialCommand }) => {
  const [sessionId, setSessionId] = useState<string>('');
  const [history, setHistory] = useState<TerminalOutput[]>([]);
  const [currentCommand, setCurrentCommand] = useState('');
  const [commandHistory, setCommandHistory] = useState<string[]>([]);
  const [historyIndex, setHistoryIndex] = useState(-1);
  const [isExecuting, setIsExecuting] = useState(false);
  
  const terminalRef = useRef<HTMLDivElement>(null);
  const inputRef = useRef<HTMLInputElement>(null);
  
  // Initialize terminal session
  useEffect(() => {
    if (isOpen && !sessionId) {
      const sid = rosTerminalService.createSession();
      setSessionId(sid);
      setHistory(rosTerminalService.getHistory(sid));
    }
  }, [isOpen, sessionId]);
  
  // Execute initial command if provided
  useEffect(() => {
    if (initialCommand && sessionId && isOpen) {
      executeCommand(initialCommand);
    }
  }, [initialCommand, sessionId, isOpen]);
  
  // Auto-scroll to bottom
  useEffect(() => {
    if (terminalRef.current) {
      terminalRef.current.scrollTop = terminalRef.current.scrollHeight;
    }
  }, [history]);
  
  // Focus input when opened
  useEffect(() => {
    if (isOpen && inputRef.current) {
      inputRef.current.focus();
    }
  }, [isOpen]);
  
  const executeCommand = async (command: string) => {
    if (!command.trim() || !sessionId) return;
    
    setIsExecuting(true);
    
    try {
      const output = await rosTerminalService.executeCommand(command, sessionId);
      setHistory(rosTerminalService.getHistory(sessionId));
      setCommandHistory(rosTerminalService.getCommandHistory());
      setCurrentCommand('');
      setHistoryIndex(-1);
    } catch (error) {
      console.error('Command execution error:', error);
    } finally {
      setIsExecuting(false);
    }
  };
  
  const handleKeyDown = (e: React.KeyboardEvent<HTMLInputElement>) => {
    // Execute command on Enter
    if (e.key === 'Enter' && !isExecuting) {
      e.preventDefault();
      executeCommand(currentCommand);
    }
    
    // Navigate command history with Up/Down arrows
    if (e.key === 'ArrowUp') {
      e.preventDefault();
      if (commandHistory.length > 0) {
        const newIndex = historyIndex < commandHistory.length - 1 ? historyIndex + 1 : historyIndex;
        setHistoryIndex(newIndex);
        setCurrentCommand(commandHistory[commandHistory.length - 1 - newIndex]);
      }
    }
    
    if (e.key === 'ArrowDown') {
      e.preventDefault();
      if (historyIndex > 0) {
        const newIndex = historyIndex - 1;
        setHistoryIndex(newIndex);
        setCurrentCommand(commandHistory[commandHistory.length - 1 - newIndex]);
      } else {
        setHistoryIndex(-1);
        setCurrentCommand('');
      }
    }
    
    // Clear terminal with Ctrl+L
    if (e.ctrlKey && e.key === 'l') {
      e.preventDefault();
      if (sessionId) {
        rosTerminalService.clearHistory(sessionId);
        setHistory(rosTerminalService.getHistory(sessionId));
      }
    }
  };
  
  const handleClear = () => {
    if (sessionId) {
      rosTerminalService.clearHistory(sessionId);
      setHistory(rosTerminalService.getHistory(sessionId));
    }
  };
  
  if (!isOpen) return null;
  
  return (
    <div className="ros-terminal-modal">
      <div className="ros-terminal-container">
        {/* Header */}
        <div className="ros-terminal-header">
          <div className="ros-terminal-title">
            <span className="terminal-icon">🤖</span>
            <span>ROS2 Terminal</span>
            <span className="terminal-status">
              {isExecuting ? '⏳ Executing...' : '✅ Ready'}
            </span>
          </div>
          <div className="ros-terminal-actions">
            <button 
              className="terminal-action-btn"
              onClick={handleClear}
              title="Clear terminal (Ctrl+L)"
            >
              🗑️ Clear
            </button>
            <button 
              className="terminal-close-btn"
              onClick={onClose}
              title="Close terminal"
            >
              ✕
            </button>
          </div>
        </div>
        
        {/* Terminal Output */}
        <div className="ros-terminal-output" ref={terminalRef}>
          {history.map((output, index) => (
            <div key={index} className="terminal-entry">
              {output.command && (
                <div className="terminal-command">
                  <span className="terminal-prompt">$ </span>
                  <span className="terminal-command-text">{output.command}</span>
                </div>
              )}
              <div className={`terminal-output terminal-output-${output.type}`}>
                <pre>{output.output}</pre>
              </div>
            </div>
          ))}
        </div>
        
        {/* Input */}
        <div className="ros-terminal-input">
          <span className="terminal-prompt">$ </span>
          <input
            ref={inputRef}
            type="text"
            value={currentCommand}
            onChange={(e) => setCurrentCommand(e.target.value)}
            onKeyDown={handleKeyDown}
            disabled={isExecuting}
            placeholder="Type a ROS command..."
            className="terminal-input-field"
          />
          <button
            onClick={() => executeCommand(currentCommand)}
            disabled={isExecuting || !currentCommand.trim()}
            className="terminal-execute-btn"
          >
            {isExecuting ? '⏳' : '▶️'}
          </button>
        </div>
        
        {/* Help */}
        <div className="ros-terminal-help">
          <span>💡 Tips:</span>
          <span>↑↓ Navigate history</span>
          <span>Ctrl+L Clear</span>
          <span>Enter Execute</span>
        </div>
      </div>
    </div>
  );
};

