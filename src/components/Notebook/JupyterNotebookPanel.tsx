import React, { useState, useEffect, useCallback, useRef } from 'react';
import { Code, FileText, Play, Trash2, Plus, ChevronDown, Save, FolderOpen, Download, Upload } from 'lucide-react';
import { jupyterNotebookService, type NotebookOutput } from '../../services/jupyter-notebook-service';
import { multiLanguageExecutionService, type SupportedLanguage } from '../../services/multi-language-execution-service';
import { fileService } from '../../services/file-service';
import './NotebookPanel.css';

type CellLanguage = SupportedLanguage | 'markdown' | 'raw';

interface NotebookCell {
  id: string;
  type: 'code' | 'markdown';
  language: CellLanguage;
  content: string;
  executionCount: number | null;
  outputs: NotebookOutput[];
  isExecuting?: boolean;
  metadata: Record<string, any>;
}

interface JupyterNotebookPanelProps {
  filePath?: string;
  onClose?: () => void;
}

export const JupyterNotebookPanel: React.FC<JupyterNotebookPanelProps> = ({ 
  filePath,
  onClose 
}) => {
  const [cells, setCells] = useState<NotebookCell[]>([]);
  const [activeCell, setActiveCell] = useState<string | null>(null);
  const [currentFilePath, setCurrentFilePath] = useState<string | undefined>(filePath);
  const [isSaving, setIsSaving] = useState(false);
  const [notebookName, setNotebookName] = useState<string>('Untitled.ipynb');
  const [showLanguageMenu, setShowLanguageMenu] = useState<string | null>(null);
  const fileInputRef = useRef<HTMLInputElement>(null);

  const languageOptions: { value: SupportedLanguage; label: string; icon: string }[] = [
    { value: 'python', label: 'Python', icon: '🐍' },
    { value: 'sql', label: 'SQL', icon: '🗄️' },
    { value: 'rust', label: 'Rust', icon: '🦀' },
    { value: 'r', label: 'R', icon: '📊' },
    { value: 'navlambda', label: 'NAVΛ', icon: '⋋' },
    { value: 'vnc', label: 'VNC', icon: '⋋' },
    { value: 'javascript', label: 'JavaScript', icon: 'JS' },
    { value: 'typescript', label: 'TypeScript', icon: 'TS' },
  ];

  const getLanguageInfo = (lang: CellLanguage) => {
    if (lang === 'markdown' || lang === 'raw') {
      return { value: lang, label: lang === 'markdown' ? 'Markdown' : 'Raw', icon: '📝' };
    }
    return languageOptions.find(l => l.value === lang) || languageOptions[0];
  };

  const changeCellLanguage = (id: string, language: SupportedLanguage) => {
    setCells(cells.map(cell => 
      cell.id === id ? { ...cell, language } : cell
    ));
    setShowLanguageMenu(null);
  };

  // Load notebook on mount or when filePath changes
  useEffect(() => {
    if (currentFilePath) {
      loadNotebook(currentFilePath);
    } else {
      // Create empty notebook
      const emptyNotebook = jupyterNotebookService.createEmptyNotebook('python');
      const initialCells = jupyterNotebookService.notebookToCells(emptyNotebook);
      setCells(initialCells.map(cell => ({
        id: cell.id,
        type: cell.type,
        language: cell.language as CellLanguage,
        content: cell.content,
        executionCount: cell.executionCount,
        outputs: cell.outputs,
        metadata: cell.metadata,
      })));
    }
  }, [currentFilePath]);

  // Initialize Python execution service
  useEffect(() => {
    pythonExecutionService.initializePyodide().catch(console.error);
  }, []);

  const loadNotebook = async (path: string) => {
    try {
      const content = await fileService.readFile(path);
      const notebook = jupyterNotebookService.parseNotebook(content);
      const loadedCells = jupyterNotebookService.notebookToCells(notebook);
      
      setCells(loadedCells.map(cell => ({
        id: cell.id,
        type: cell.type,
        language: cell.language as CellLanguage,
        content: cell.content,
        executionCount: cell.executionCount,
        outputs: cell.outputs,
        metadata: cell.metadata,
      })));

      setNotebookName(fileService.getFileName(path));
      setCurrentFilePath(path);
    } catch (error) {
      console.error('Error loading notebook:', error);
      alert(`Failed to load notebook: ${error instanceof Error ? error.message : 'Unknown error'}`);
    }
  };

  const saveNotebook = async (path?: string) => {
    setIsSaving(true);
    try {
      const notebook = jupyterNotebookService.cellsToNotebook(
        cells.map(cell => ({
          id: cell.id,
          type: cell.type,
          language: cell.language,
          content: cell.content,
          executionCount: cell.executionCount,
          outputs: cell.outputs,
          metadata: cell.metadata,
        }))
      );

      const jsonContent = jupyterNotebookService.serializeNotebook(notebook);
      const savePath = path || currentFilePath || `notebooks/${notebookName}`;
      
      await fileService.saveFile(savePath, jsonContent);
      setCurrentFilePath(savePath);
      setIsSaving(false);
    } catch (error) {
      console.error('Error saving notebook:', error);
      alert(`Failed to save notebook: ${error instanceof Error ? error.message : 'Unknown error'}`);
      setIsSaving(false);
    }
  };

  const handleFileOpen = () => {
    fileInputRef.current?.click();
  };

  const handleFileSelect = async (event: React.ChangeEvent<HTMLInputElement>) => {
    const file = event.target.files?.[0];
    if (!file) return;

    try {
      const content = await file.text();
      const notebook = jupyterNotebookService.parseNotebook(content);
      const loadedCells = jupyterNotebookService.notebookToCells(notebook);
      
      setCells(loadedCells.map(cell => ({
        id: cell.id,
        type: cell.type,
        language: cell.language as CellLanguage,
        content: cell.content,
        executionCount: cell.executionCount,
        outputs: cell.outputs,
        metadata: cell.metadata,
      })));

      setNotebookName(file.name);
      setCurrentFilePath(undefined); // Will be set on save
    } catch (error) {
      console.error('Error loading notebook file:', error);
      alert(`Failed to load notebook: ${error instanceof Error ? error.message : 'Unknown error'}`);
    }
  };

  const handleFileDownload = () => {
    const notebook = jupyterNotebookService.cellsToNotebook(
      cells.map(cell => ({
        id: cell.id,
        type: cell.type,
        language: cell.language,
        content: cell.content,
        executionCount: cell.executionCount,
        outputs: cell.outputs,
        metadata: cell.metadata,
      }))
    );

    const jsonContent = jupyterNotebookService.serializeNotebook(notebook);
    const blob = new Blob([jsonContent], { type: 'application/json' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = notebookName;
    a.click();
    URL.revokeObjectURL(url);
  };

  const addCell = (type: 'code' | 'markdown', language: CellLanguage = 'python') => {
    const newCell: NotebookCell = {
      id: `cell-${Date.now()}-${Math.random().toString(36).slice(2, 9)}`,
      type,
      language,
      content: '',
      executionCount: null,
      outputs: [],
      metadata: {},
    };
    setCells([...cells, newCell]);
    setActiveCell(newCell.id);
  };

  const insertCellAfter = (afterId: string, type: 'code' | 'markdown', language: CellLanguage = 'python' as SupportedLanguage) => {
    const index = cells.findIndex(c => c.id === afterId);
    const newCell: NotebookCell = {
      id: `cell-${Date.now()}-${Math.random().toString(36).slice(2, 9)}`,
      type,
      language,
      content: '',
      executionCount: null,
      outputs: [],
      metadata: {},
    };
    const newCells = [...cells];
    newCells.splice(index + 1, 0, newCell);
    setCells(newCells);
    setActiveCell(newCell.id);
  };

  const updateCell = (id: string, content: string) => {
    setCells(cells.map(cell => 
      cell.id === id ? { ...cell, content } : cell
    ));
  };

  const deleteCell = (id: string) => {
    setCells(cells.filter(cell => cell.id !== id));
  };

  const executeCell = async (id: string) => {
    const cell = cells.find(c => c.id === id);
    if (!cell || cell.type !== 'code') return;

    setCells(cells.map(c => 
      c.id === id ? { ...c, isExecuting: true } : c
    ));

    try {
      // Use multi-language execution service
      const language = cell.language as SupportedLanguage;
      const result = await multiLanguageExecutionService.executeCode(
        language,
        cell.content,
        {
          timeout: 30000,
          captureOutput: true,
        }
      );

      setCells(cells.map(c => 
        c.id === id 
          ? { 
              ...c, 
              isExecuting: false,
              executionCount: result.execution_count,
              outputs: result.outputs,
            } 
          : c
      ));
    } catch (error) {
      const errorOutput: NotebookOutput = {
        output_type: 'error',
        ename: 'ExecutionError',
        evalue: error instanceof Error ? error.message : String(error),
        traceback: [String(error)],
      };

      setCells(cells.map(c => 
        c.id === id 
          ? { 
              ...c, 
              isExecuting: false,
              outputs: [errorOutput],
            } 
          : c
      ));
    }
  };

  const executeAllCells = async () => {
    for (const cell of cells) {
      if (cell.type === 'code') {
        await executeCell(cell.id);
      }
    }
  };

  const renderOutput = (output: NotebookOutput, index: number) => {
    const formatted = jupyterNotebookService.formatOutput(output);

    switch (formatted.type) {
      case 'error':
        return (
          <div key={index} className="output-error">
            <pre className="output-content">{formatted.content}</pre>
          </div>
        );

      case 'image':
        return (
          <div key={index} className="output-image">
            <img 
              src={`data:${formatted.mimeType};base64,${formatted.content}`}
              alt="Output"
              style={{ maxWidth: '100%', height: 'auto' }}
            />
          </div>
        );

      case 'html':
        return (
          <div 
            key={index} 
            className="output-html"
            dangerouslySetInnerHTML={{ __html: formatted.content }}
          />
        );

      case 'stream':
        return (
          <div key={index} className="output-stream">
            <pre className={`output-content ${formatted.content.includes('Error') ? 'error' : ''}`}>
              {formatted.content}
            </pre>
          </div>
        );

      default:
        return (
          <div key={index} className="output-text">
            <pre className="output-content">{formatted.content}</pre>
          </div>
        );
    }
  };

  return (
    <div className="notebook-panel">
      <div className="notebook-header">
        <div className="notebook-title">
          <span className="notebook-icon">📓</span>
          <span>{notebookName}</span>
        </div>
        <div className="notebook-actions">
          <button 
            className="notebook-btn" 
            onClick={handleFileOpen}
            title="Open Notebook"
          >
            <FolderOpen size={14} />
            <span>Open</span>
          </button>
          <input
            ref={fileInputRef}
            type="file"
            accept=".ipynb"
            style={{ display: 'none' }}
            onChange={handleFileSelect}
          />
          <button 
            className="notebook-btn" 
            onClick={handleFileDownload}
            title="Download Notebook"
          >
            <Download size={14} />
            <span>Download</span>
          </button>
          <button 
            className="notebook-btn" 
            onClick={() => saveNotebook()}
            disabled={isSaving}
            title="Save Notebook"
          >
            <Save size={14} />
            <span>{isSaving ? 'Saving...' : 'Save'}</span>
          </button>
          <div className="dropdown-group">
            <button className="notebook-btn dropdown-btn" title="Add Code Cell">
              <Code size={14} />
              <span>+ Code</span>
              <ChevronDown size={14} />
            </button>
            <div className="dropdown-menu">
              <button
                onClick={() => addCell('code', 'python')}
                className="dropdown-item"
              >
                <span className="lang-icon">🐍</span>
                <span>Python</span>
              </button>
              <button
                onClick={() => addCell('code', 'sql')}
                className="dropdown-item"
              >
                <span className="lang-icon">🗄️</span>
                <span>SQL</span>
              </button>
              <button
                onClick={() => addCell('code', 'rust')}
                className="dropdown-item"
              >
                <span className="lang-icon">🦀</span>
                <span>Rust</span>
              </button>
              <button
                onClick={() => addCell('code', 'r')}
                className="dropdown-item"
              >
                <span className="lang-icon">📊</span>
                <span>R</span>
              </button>
              <button
                onClick={() => addCell('code', 'navlambda')}
                className="dropdown-item"
              >
                <span className="lang-icon">⋋</span>
                <span>NAVΛ</span>
              </button>
              <button
                onClick={() => addCell('code', 'vnc')}
                className="dropdown-item"
              >
                <span className="lang-icon">⋋</span>
                <span>VNC</span>
              </button>
              <button
                onClick={() => addCell('code', 'javascript')}
                className="dropdown-item"
              >
                <span className="lang-icon">JS</span>
                <span>JavaScript</span>
              </button>
              <button
                onClick={() => addCell('code', 'typescript')}
                className="dropdown-item"
              >
                <span className="lang-icon">TS</span>
                <span>TypeScript</span>
              </button>
            </div>
          </div>
          <button 
            className="notebook-btn" 
            onClick={() => addCell('markdown')}
            title="Add Markdown Cell"
          >
            <FileText size={14} />
            <span>+ Text</span>
          </button>
          <button 
            className="notebook-btn" 
            onClick={executeAllCells}
            title="Run All Cells"
          >
            <Play size={14} />
            <span>Run All</span>
          </button>
        </div>
      </div>

      <div className="notebook-cells">
        {cells.map((cell, index) => (
          <div 
            key={cell.id} 
            className={`notebook-cell ${cell.type}-cell ${activeCell === cell.id ? 'active' : ''} lang-${cell.language}`}
            onClick={() => setActiveCell(cell.id)}
          >
            <div className="cell-toolbar">
              <div className="cell-info">
                <span className="cell-index">
                  {cell.type === 'code' && cell.executionCount !== null
                    ? `[${cell.executionCount}]` 
                    : `[${index + 1}]`}
                </span>
                {cell.type === 'code' && (
                  <div className="cell-language-badge">
                    <button
                      className="language-selector"
                      onClick={(e) => {
                        e.stopPropagation();
                        setShowLanguageMenu(showLanguageMenu === cell.id ? null : cell.id);
                      }}
                    >
                      <span className="lang-icon">{getLanguageInfo(cell.language).icon}</span>
                      <span className="lang-name">{getLanguageInfo(cell.language).label}</span>
                      <ChevronDown size={12} />
                    </button>
                    {showLanguageMenu === cell.id && (
                      <div className="language-dropdown">
                        {languageOptions.map(lang => (
                          <button
                            key={lang.value}
                            onClick={(e) => {
                              e.stopPropagation();
                              changeCellLanguage(cell.id, lang.value);
                            }}
                            className={`lang-option ${lang.value === cell.language ? 'active' : ''}`}
                          >
                            <span className="lang-icon">{lang.icon}</span>
                            <span>{lang.label}</span>
                          </button>
                        ))}
                      </div>
                    )}
                  </div>
                )}
              </div>
              <div className="cell-actions">
                {cell.type === 'code' && (
                  <button 
                    className="cell-btn run-btn"
                    onClick={(e) => {
                      e.stopPropagation();
                      executeCell(cell.id);
                    }}
                    disabled={cell.isExecuting}
                    title="Run Cell (Shift+Enter)"
                  >
                    {cell.isExecuting ? (
                      <span className="spinner">⏳</span>
                    ) : (
                      <Play size={14} />
                    )}
                  </button>
                )}
                <button 
                  className="cell-btn add-btn"
                  onClick={(e) => {
                    e.stopPropagation();
                    const newLang = cell.type === 'code' ? cell.language : 'python';
                    insertCellAfter(cell.id, 'code', newLang as SupportedLanguage);
                  }}
                  title="Add Cell Below"
                >
                  <Plus size={14} />
                </button>
                <button 
                  className="cell-btn delete-btn"
                  onClick={(e) => {
                    e.stopPropagation();
                    deleteCell(cell.id);
                  }}
                  title="Delete Cell"
                >
                  <Trash2 size={14} />
                </button>
              </div>
            </div>

            <div className="cell-input">
              {cell.type === 'markdown' ? (
                <textarea
                  value={cell.content}
                  onChange={(e) => updateCell(cell.id, e.target.value)}
                  placeholder="Enter markdown..."
                  className="cell-textarea markdown-input"
                  spellCheck={false}
                />
              ) : (
                <textarea
                  value={cell.content}
                  onChange={(e) => updateCell(cell.id, e.target.value)}
                  placeholder={`Enter ${getLanguageInfo(cell.language).label} code...`}
                  className={`cell-textarea code-input lang-${cell.language}`}
                  spellCheck={false}
                  onKeyDown={(e) => {
                    if (e.shiftKey && e.key === 'Enter') {
                      e.preventDefault();
                      executeCell(cell.id);
                    }
                  }}
                />
              )}
            </div>

            {cell.outputs && cell.outputs.length > 0 && (
              <div className="cell-output">
                <div className="output-header">
                  <span className="output-label">Output:</span>
                </div>
                <div className="output-content-wrapper">
                  {cell.outputs.map((output, idx) => renderOutput(output, idx))}
                </div>
              </div>
            )}
          </div>
        ))}
      </div>
    </div>
  );
};

