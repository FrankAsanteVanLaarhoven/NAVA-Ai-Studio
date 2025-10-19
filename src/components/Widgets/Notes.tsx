import React, { useState, useEffect, useRef } from 'react';
import './Notes.css';

interface Note {
  id: string;
  title: string;
  content: string;
  createdAt: Date;
  updatedAt: Date;
  color: string;
}

interface NotesProps {
  onClose?: () => void;
}

const NOTE_COLORS = [
  '#fef3c7', // yellow
  '#dbeafe', // blue
  '#dcfce7', // green
  '#fce7f3', // pink
  '#f3e8ff', // purple
  '#fed7d7', // red
  '#e0f2fe', // cyan
  '#f0f9ff', // sky
];

export const Notes: React.FC<NotesProps> = ({ onClose }) => {
  const [notes, setNotes] = useState<Note[]>([]);
  const [selectedNote, setSelectedNote] = useState<Note | null>(null);
  const [searchTerm, setSearchTerm] = useState('');
  const textareaRef = useRef<HTMLTextAreaElement>(null);

  // Load notes from localStorage
  useEffect(() => {
    const savedNotes = localStorage.getItem('navl-studio-notes');
    if (savedNotes) {
      const parsedNotes = JSON.parse(savedNotes).map((note: any) => ({
        ...note,
        createdAt: new Date(note.createdAt),
        updatedAt: new Date(note.updatedAt),
      }));
      setNotes(parsedNotes);
    }
  }, []);

  // Save notes to localStorage
  const saveNotes = (updatedNotes: Note[]) => {
    localStorage.setItem('navl-studio-notes', JSON.stringify(updatedNotes));
    setNotes(updatedNotes);
  };

  const createNewNote = () => {
    const newNote: Note = {
      id: Date.now().toString(),
      title: 'New Note',
      content: '',
      createdAt: new Date(),
      updatedAt: new Date(),
      color: NOTE_COLORS[Math.floor(Math.random() * NOTE_COLORS.length)],
    };
    
    const updatedNotes = [newNote, ...notes];
    saveNotes(updatedNotes);
    setSelectedNote(newNote);
  };

  const updateNote = (noteId: string, updates: Partial<Note>) => {
    const updatedNotes = notes.map(note =>
      note.id === noteId
        ? { ...note, ...updates, updatedAt: new Date() }
        : note
    );
    saveNotes(updatedNotes);
    
    if (selectedNote && selectedNote.id === noteId) {
      setSelectedNote({ ...selectedNote, ...updates, updatedAt: new Date() });
    }
  };

  const deleteNote = (noteId: string) => {
    const updatedNotes = notes.filter(note => note.id !== noteId);
    saveNotes(updatedNotes);

    if (selectedNote && selectedNote.id === noteId) {
      setSelectedNote(null);
    }
  };

  const filteredNotes = notes.filter(note =>
    note.title.toLowerCase().includes(searchTerm.toLowerCase()) ||
    note.content.toLowerCase().includes(searchTerm.toLowerCase())
  );

  const formatDate = (date: Date) => {
    return date.toLocaleDateString('en-US', {
      month: 'short',
      day: 'numeric',
      hour: '2-digit',
      minute: '2-digit',
    });
  };

  const handleTitleChange = (value: string) => {
    if (selectedNote) {
      updateNote(selectedNote.id, { title: value });
    }
  };

  const handleContentChange = (value: string) => {
    if (selectedNote) {
      updateNote(selectedNote.id, { content: value });
    }
  };

  const handleColorChange = (color: string) => {
    if (selectedNote) {
      updateNote(selectedNote.id, { color });
    }
  };

  // Auto-resize textarea
  useEffect(() => {
    if (textareaRef.current) {
      textareaRef.current.style.height = 'auto';
      textareaRef.current.style.height = textareaRef.current.scrollHeight + 'px';
    }
  }, [selectedNote?.content]);

  return (
    <div className="notes-widget">
      <div className="notes-header">
        <h3>📝 Notes</h3>
        {onClose && (
          <button className="notes-close" onClick={onClose}>
            ×
          </button>
        )}
      </div>

      <div className="notes-content">
        <div className="notes-sidebar">
          <div className="notes-controls">
            <button className="notes-new-btn" onClick={createNewNote}>
              + New Note
            </button>
            <input
              type="text"
              placeholder="Search notes..."
              value={searchTerm}
              onChange={(e) => setSearchTerm(e.target.value)}
              className="notes-search"
            />
          </div>

          <div className="notes-list">
            {filteredNotes.length === 0 ? (
              <div className="notes-empty">
                {searchTerm ? 'No notes found' : 'No notes yet'}
              </div>
            ) : (
              filteredNotes.map(note => (
                <div
                  key={note.id}
                  className={`notes-item ${selectedNote?.id === note.id ? 'active' : ''}`}
                  style={{ backgroundColor: note.color }}
                  onClick={() => {
                    setSelectedNote(note);
                  }}
                >
                  <div className="notes-item-header">
                    <h4>{note.title}</h4>
                    <button
                      className="notes-delete"
                      onClick={(e) => {
                        e.stopPropagation();
                        deleteNote(note.id);
                      }}
                    >
                      🗑️
                    </button>
                  </div>
                  <p>{note.content.substring(0, 100)}{note.content.length > 100 ? '...' : ''}</p>
                  <small>{formatDate(note.updatedAt)}</small>
                </div>
              ))
            )}
          </div>
        </div>

        <div className="notes-editor">
          {selectedNote ? (
            <>
              <div className="notes-editor-header">
                <input
                  type="text"
                  value={selectedNote.title}
                  onChange={(e) => handleTitleChange(e.target.value)}
                  className="notes-title-input"
                  placeholder="Note title..."
                />
                <div className="notes-color-picker">
                  {NOTE_COLORS.map(color => (
                    <button
                      key={color}
                      className={`notes-color-btn ${selectedNote.color === color ? 'active' : ''}`}
                      style={{ backgroundColor: color }}
                      onClick={() => handleColorChange(color)}
                    />
                  ))}
                </div>
              </div>
              
              <textarea
                ref={textareaRef}
                value={selectedNote.content}
                onChange={(e) => handleContentChange(e.target.value)}
                placeholder="Start writing your note..."
                className="notes-content-input"
              />
              
              <div className="notes-editor-footer">
                <small>
                  Created: {formatDate(selectedNote.createdAt)} • 
                  Updated: {formatDate(selectedNote.updatedAt)}
                </small>
              </div>
            </>
          ) : (
            <div className="notes-placeholder">
              <div className="notes-placeholder-content">
                <h4>📝 Select a note to edit</h4>
                <p>Choose a note from the sidebar or create a new one to get started.</p>
              </div>
            </div>
          )}
        </div>
      </div>
    </div>
  );
};