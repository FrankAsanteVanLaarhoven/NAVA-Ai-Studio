import React, { useState, useEffect, useRef } from 'react';
import { v4 as uuidv4 } from 'uuid';
import './PinsNotesEcho.css';

interface Pin {
  id: string;
  x: number;
  y: number;
  content: string;
  color: string;
  timestamp: Date;
  user: string;
}

interface Note {
  id: string;
  title: string;
  content: string;
  tags: string[];
  color: string;
  timestamp: Date;
  user: string;
  pinned: boolean;
}

interface EchoEntry {
  id: string;
  command: string;
  output: string;
  timestamp: Date;
  user: string;
  status: 'success' | 'error' | 'info';
}

const PinsNotesEcho: React.FC = () => {
  const [activeTab, setActiveTab] = useState<'pins' | 'notes' | 'echo'>('pins');
  const [pins, setPins] = useState<Pin[]>([]);
  const [notes, setNotes] = useState<Note[]>([]);
  const [echoHistory, setEchoHistory] = useState<EchoEntry[]>([]);
  const [isCreatingPin, setIsCreatingPin] = useState(false);
  const [isCreatingNote, setIsCreatingNote] = useState(false);
  const [pinPosition, setPinPosition] = useState({ x: 0, y: 0 });
  const canvasRef = useRef<HTMLDivElement>(null);

  // Load data from localStorage on mount
  useEffect(() => {
    const savedPins = localStorage.getItem('navlambda-pins');
    const savedNotes = localStorage.getItem('navlambda-notes');
    const savedEcho = localStorage.getItem('navlambda-echo-history');

    if (savedPins) {
      try {
        const parsedPins = JSON.parse(savedPins).map((pin: any) => ({
          ...pin,
          timestamp: new Date(pin.timestamp)
        }));
        setPins(parsedPins);
      } catch (error) {
        console.error('Failed to load pins:', error);
      }
    }

    if (savedNotes) {
      try {
        const parsedNotes = JSON.parse(savedNotes).map((note: any) => ({
          ...note,
          timestamp: new Date(note.timestamp)
        }));
        setNotes(parsedNotes);
      } catch (error) {
        console.error('Failed to load notes:', error);
      }
    }

    if (savedEcho) {
      try {
        const parsedEcho = JSON.parse(savedEcho).map((entry: any) => ({
          ...entry,
          timestamp: new Date(entry.timestamp)
        }));
        setEchoHistory(parsedEcho);
      } catch (error) {
        console.error('Failed to load echo history:', error);
      }
    }
  }, []);

  // Save data to localStorage whenever state changes
  useEffect(() => {
    localStorage.setItem('navlambda-pins', JSON.stringify(pins));
  }, [pins]);

  useEffect(() => {
    localStorage.setItem('navlambda-notes', JSON.stringify(notes));
  }, [notes]);

  useEffect(() => {
    localStorage.setItem('navlambda-echo-history', JSON.stringify(echoHistory));
  }, [echoHistory]);

  // Handle canvas click for pin creation
  const handleCanvasClick = (event: React.MouseEvent<HTMLDivElement>) => {
    if (!isCreatingPin) return;

    const rect = canvasRef.current?.getBoundingClientRect();
    if (!rect) return;

    const x = event.clientX - rect.left;
    const y = event.clientY - rect.top;

    setPinPosition({ x, y });
    setIsCreatingPin(false);
    // Note: Pin creation modal will be shown by parent component
  };

  // Pin management functions
  const createPin = (content: string, color: string = '#ff6b6b') => {
    const newPin: Pin = {
      id: uuidv4(),
      x: pinPosition.x,
      y: pinPosition.y,
      content,
      color,
      timestamp: new Date(),
      user: 'current-user' // In real app, get from auth context
    };
    setPins(prev => [...prev, newPin]);
  };

  const updatePin = (id: string, updates: Partial<Pin>) => {
    setPins(prev => prev.map(pin =>
      pin.id === id ? { ...pin, ...updates } : pin
    ));
  };

  const deletePin = (id: string) => {
    setPins(prev => prev.filter(pin => pin.id !== id));
  };

  // Note management functions
  const createNote = (title: string, content: string, tags: string[] = [], color: string = '#4ecdc4') => {
    const newNote: Note = {
      id: uuidv4(),
      title,
      content,
      tags,
      color,
      timestamp: new Date(),
      user: 'current-user',
      pinned: false
    };
    setNotes(prev => [...prev, newNote]);
  };

  const updateNote = (id: string, updates: Partial<Note>) => {
    setNotes(prev => prev.map(note =>
      note.id === id ? { ...note, ...updates } : note
    ));
  };

  const deleteNote = (id: string) => {
    setNotes(prev => prev.filter(note => note.id !== id));
  };

  const toggleNotePin = (id: string) => {
    setNotes(prev => prev.map(note =>
      note.id === id ? { ...note, pinned: !note.pinned } : note
    ));
  };

  // Echo management functions
  const addEchoEntry = (command: string, output: string, status: 'success' | 'error' | 'info' = 'info') => {
    const newEntry: EchoEntry = {
      id: uuidv4(),
      command,
      output,
      timestamp: new Date(),
      user: 'current-user',
      status
    };
    setEchoHistory(prev => [newEntry, ...prev.slice(0, 99)]); // Keep last 100 entries
  };

  const clearEchoHistory = () => {
    setEchoHistory([]);
  };

  const exportEchoHistory = () => {
    const data = JSON.stringify(echoHistory, null, 2);
    const blob = new Blob([data], { type: 'application/json' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = `echo-history-${new Date().toISOString().split('T')[0]}.json`;
    a.click();
    URL.revokeObjectURL(url);
  };

  const renderPinsTab = () => (
    <div className="pins-tab">
      <div className="pins-header">
        <h3>Visual Pins</h3>
        <button
          className="create-pin-btn"
          onClick={() => setIsCreatingPin(true)}
          disabled={isCreatingPin}
        >
          {isCreatingPin ? 'Click on canvas to place pin' : 'Create Pin'}
        </button>
      </div>

      <div className="pins-canvas" ref={canvasRef} onClick={handleCanvasClick}>
        {pins.map(pin => (
          <div
            key={pin.id}
            className="pin"
            style={{
              left: pin.x,
              top: pin.y,
              backgroundColor: pin.color
            }}
            title={pin.content}
          >
            📌
            <div className="pin-tooltip">
              <div className="pin-content">{pin.content}</div>
              <div className="pin-meta">
                {pin.user} • {pin.timestamp.toLocaleDateString()}
              </div>
              <div className="pin-actions">
                <button onClick={() => deletePin(pin.id)}>🗑️</button>
              </div>
            </div>
          </div>
        ))}
      </div>

      <div className="pins-list">
        <h4>All Pins ({pins.length})</h4>
        {pins.map(pin => (
          <div key={pin.id} className="pin-item">
            <div className="pin-color" style={{ backgroundColor: pin.color }}></div>
            <div className="pin-details">
              <div className="pin-content">{pin.content}</div>
              <div className="pin-meta">
                Position: ({Math.round(pin.x)}, {Math.round(pin.y)}) •
                {pin.user} • {pin.timestamp.toLocaleString()}
              </div>
            </div>
            <button className="delete-btn" onClick={() => deletePin(pin.id)}>×</button>
          </div>
        ))}
      </div>
    </div>
  );

  const renderNotesTab = () => (
    <div className="notes-tab">
      <div className="notes-header">
        <h3>Notes & Documentation</h3>
        <button
          className="create-note-btn"
          onClick={() => setIsCreatingNote(true)}
        >
          Create Note
        </button>
      </div>

      <div className="notes-grid">
        {notes
          .sort((a, b) => {
            // Pinned notes first, then by timestamp
            if (a.pinned && !b.pinned) return -1;
            if (!a.pinned && b.pinned) return 1;
            return b.timestamp.getTime() - a.timestamp.getTime();
          })
          .map(note => (
            <div key={note.id} className={`note-card ${note.pinned ? 'pinned' : ''}`}>
              <div className="note-header">
                <h4>{note.title}</h4>
                <div className="note-actions">
                  <button
                    className={`pin-btn ${note.pinned ? 'pinned' : ''}`}
                    onClick={() => toggleNotePin(note.id)}
                    title={note.pinned ? 'Unpin note' : 'Pin note'}
                  >
                    📌
                  </button>
                  <button className="delete-btn" onClick={() => deleteNote(note.id)}>×</button>
                </div>
              </div>

              <div className="note-content">{note.content}</div>

              {note.tags.length > 0 && (
                <div className="note-tags">
                  {note.tags.map(tag => (
                    <span key={tag} className="tag">#{tag}</span>
                  ))}
                </div>
              )}

              <div className="note-meta">
                {note.user} • {note.timestamp.toLocaleString()}
              </div>
            </div>
          ))}
      </div>
    </div>
  );

  const renderEchoTab = () => (
    <div className="echo-tab">
      <div className="echo-header">
        <h3>Command Echo History</h3>
        <div className="echo-actions">
          <button className="export-btn" onClick={exportEchoHistory}>
            Export History
          </button>
          <button className="clear-btn" onClick={clearEchoHistory}>
            Clear History
          </button>
        </div>
      </div>

      <div className="echo-history">
        {echoHistory.length === 0 ? (
          <div className="empty-state">
            <p>No command history yet. Commands executed in the terminal will appear here.</p>
          </div>
        ) : (
          echoHistory.map(entry => (
            <div key={entry.id} className={`echo-entry ${entry.status}`}>
              <div className="echo-header">
                <code className="echo-command">{entry.command}</code>
                <div className="echo-meta">
                  <span className={`status-indicator ${entry.status}`}></span>
                  {entry.user} • {entry.timestamp.toLocaleString()}
                </div>
              </div>
              <pre className="echo-output">{entry.output}</pre>
            </div>
          ))
        )}
      </div>
    </div>
  );

  return (
    <div className="pins-notes-echo">
      <div className="tab-navigation">
        <button
          className={`tab-btn ${activeTab === 'pins' ? 'active' : ''}`}
          onClick={() => setActiveTab('pins')}
        >
          📌 Pins
        </button>
        <button
          className={`tab-btn ${activeTab === 'notes' ? 'active' : ''}`}
          onClick={() => setActiveTab('notes')}
        >
          📝 Notes
        </button>
        <button
          className={`tab-btn ${activeTab === 'echo' ? 'active' : ''}`}
          onClick={() => setActiveTab('echo')}
        >
          🖥️ Echo
        </button>
      </div>

      <div className="tab-content">
        {activeTab === 'pins' && renderPinsTab()}
        {activeTab === 'notes' && renderNotesTab()}
        {activeTab === 'echo' && renderEchoTab()}
      </div>

      {/* Pin Creation Modal */}
      {isCreatingPin && (
        <div className="modal-overlay" onClick={() => setIsCreatingPin(false)}>
          <div className="modal-content" onClick={e => e.stopPropagation()}>
            <h3>Create New Pin</h3>
            <p>Click on the canvas above to place your pin, then fill in the details:</p>
            <button onClick={() => setIsCreatingPin(false)}>Cancel</button>
          </div>
        </div>
      )}

      {/* Note Creation Modal */}
      {isCreatingNote && (
        <div className="modal-overlay" onClick={() => setIsCreatingNote(false)}>
          <div className="modal-content" onClick={e => e.stopPropagation()}>
            <h3>Create New Note</h3>
            <form onSubmit={(e) => {
              e.preventDefault();
              const formData = new FormData(e.target as HTMLFormElement);
              const title = formData.get('title') as string;
              const content = formData.get('content') as string;
              const tags = (formData.get('tags') as string)?.split(',').map(t => t.trim()) || [];

              if (title && content) {
                createNote(title, content, tags);
                setIsCreatingNote(false);
              }
            }}>
              <div className="form-group">
                <label htmlFor="note-title">Title:</label>
                <input type="text" id="note-title" name="title" required />
              </div>

              <div className="form-group">
                <label htmlFor="note-content">Content:</label>
                <textarea id="note-content" name="content" required rows={4}></textarea>
              </div>

              <div className="form-group">
                <label htmlFor="note-tags">Tags (comma-separated):</label>
                <input type="text" id="note-tags" name="tags" placeholder="e.g., bug, feature, research" />
              </div>

              <div className="form-actions">
                <button type="submit">Create Note</button>
                <button type="button" onClick={() => setIsCreatingNote(false)}>Cancel</button>
              </div>
            </form>
          </div>
        </div>
      )}
    </div>
  );
};

export default PinsNotesEcho;