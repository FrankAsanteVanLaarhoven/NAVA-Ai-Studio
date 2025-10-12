import React, { useState } from 'react';
import { Code, FileText, Play, Trash2, Plus, ChevronDown } from 'lucide-react';
import './NotebookPanel.css';

type CellLanguage = 'vnc' | 'rust' | 'python' | 'sql' | 'javascript' | 'typescript' | 'bash' | 'html' | 'css' | 'json';

interface NotebookCell {
  id: string;
  type: 'code' | 'markdown';
  language: CellLanguage;
  content: string;
  output?: string;
  isExecuting?: boolean;
  metadata?: {
    executionCount?: number;
    lastExecuted?: Date;
  };
}

export const NotebookPanel: React.FC = () => {
  const [cells, setCells] = useState<NotebookCell[]>([
    {
      id: '1',
      type: 'markdown',
      language: 'vnc',
      content: '# NAVΛ Studio Notebook\n\nInteractive multi-language computational notebook\n\n**Supported Languages:** VNC, Rust, Python, SQL, JavaScript, TypeScript, Bash',
    },
    {
      id: '2',
      type: 'code',
      language: 'vnc',
      content: '⋋ φ = navigation_field(manifold: M)\nφ.compute_optimal_path(start, goal)',
      output: '',
      metadata: { executionCount: 0 },
    },
    {
      id: '3',
      type: 'code',
      language: 'python',
      content: 'import numpy as np\nimport matplotlib.pyplot as plt\n\n# Generate navigation data\npoints = np.random.rand(100, 2)\nprint(f"Generated {len(points)} navigation points")',
      output: '',
      metadata: { executionCount: 0 },
    },
  ]);

  const [activeCell, setActiveCell] = useState<string | null>(null);
  const [showLanguageMenu, setShowLanguageMenu] = useState<string | null>(null);

  const languageOptions: { value: CellLanguage; label: string; icon: string }[] = [
    { value: 'vnc', label: 'NAVΛ (VNC)', icon: '⋋' },
    { value: 'rust', label: 'Rust', icon: '🦀' },
    { value: 'python', label: 'Python', icon: '🐍' },
    { value: 'sql', label: 'SQL', icon: '🗄️' },
    { value: 'javascript', label: 'JavaScript', icon: 'JS' },
    { value: 'typescript', label: 'TypeScript', icon: 'TS' },
    { value: 'bash', label: 'Bash', icon: '💻' },
    { value: 'html', label: 'HTML', icon: '🌐' },
    { value: 'css', label: 'CSS', icon: '🎨' },
    { value: 'json', label: 'JSON', icon: '{}' },
  ];

  const addCell = (type: 'code' | 'markdown', language: CellLanguage = 'vnc') => {
    const newCell: NotebookCell = {
      id: Date.now().toString(),
      type,
      language,
      content: '',
      metadata: type === 'code' ? { executionCount: 0 } : undefined,
    };
    setCells([...cells, newCell]);
    setActiveCell(newCell.id);
  };

  const insertCellAfter = (afterId: string, type: 'code' | 'markdown', language: CellLanguage = 'vnc') => {
    const index = cells.findIndex(c => c.id === afterId);
    const newCell: NotebookCell = {
      id: Date.now().toString(),
      type,
      language,
      content: '',
      metadata: type === 'code' ? { executionCount: 0 } : undefined,
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

  const changeCellLanguage = (id: string, language: CellLanguage) => {
    setCells(cells.map(cell => 
      cell.id === id ? { ...cell, language } : cell
    ));
    setShowLanguageMenu(null);
  };

  const executeCell = async (id: string) => {
    const cell = cells.find(c => c.id === id);
    if (!cell || cell.type !== 'code') return;

    setCells(cells.map(c => 
      c.id === id ? { ...c, isExecuting: true } : c
    ));

    // Simulate execution based on language
    await new Promise(resolve => setTimeout(resolve, 800 + Math.random() * 400));

    let output = '';
    const executionCount = (cell.metadata?.executionCount || 0) + 1;

    switch (cell.language) {
      case 'vnc':
        output = '✓ Navigation field computed\n→ Optimal path: [p₀, p₁, ..., pₙ]\n→ Energy cost: 2.47 units\n→ Optimization: VNC';
        break;
      case 'python':
        output = 'Generated 100 navigation points\n[array([[0.123, 0.456], [0.789, 0.012], ...])]\nExecution time: 0.043s';
        break;
      case 'rust':
        output = 'Compiled successfully\nRunning navigation algorithm...\nResult: Ok(NavigationPath { points: 42, energy: 1.23 })\nExecution time: 12ms';
        break;
      case 'sql':
        output = '✓ Query executed successfully\n→ 156 rows returned\n→ Execution time: 0.023s';
        break;
      case 'javascript':
        output = '> "Execution successful"\n> Object { paths: Array(10), energy: 3.14 }';
        break;
      case 'typescript':
        output = '✓ Type checking passed\n> NavigationResult { status: "success", data: [...] }';
        break;
      case 'bash':
        output = '$ Command executed\nnavigation_system_v1.0\nStatus: Running\nProcessed: 1024 nodes';
        break;
      default:
        output = `✓ ${cell.language} code executed successfully`;
    }

    setCells(cells.map(c => 
      c.id === id 
        ? { 
            ...c, 
            isExecuting: false,
            output,
            metadata: { 
              executionCount, 
              lastExecuted: new Date() 
            }
          } 
        : c
    ));
  };

  const executeAllCells = async () => {
    for (const cell of cells) {
      if (cell.type === 'code') {
        await executeCell(cell.id);
      }
    }
  };

  const deleteCell = (id: string) => {
    setCells(cells.filter(cell => cell.id !== id));
  };

  const getLanguageInfo = (lang: CellLanguage) => {
    return languageOptions.find(l => l.value === lang) || languageOptions[0];
  };

  return (
    <div className="notebook-panel">
      <div className="notebook-header">
        <div className="notebook-title">
          <span className="notebook-icon">📓</span>
          <span>Multi-Language Notebook</span>
        </div>
        <div className="notebook-actions">
          <div className="dropdown-group">
            <button className="notebook-btn dropdown-btn" title="Add Code Cell">
              <Code size={14} />
              <span>+ Code</span>
              <ChevronDown size={14} />
            </button>
            <div className="dropdown-menu">
              {languageOptions.map(lang => (
                <button
                  key={lang.value}
                  onClick={() => addCell('code', lang.value)}
                  className="dropdown-item"
                >
                  <span className="lang-icon">{lang.icon}</span>
                  <span>{lang.label}</span>
                </button>
              ))}
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
        {cells.map((cell, index) => {
          const langInfo = getLanguageInfo(cell.language);
          return (
            <div 
              key={cell.id} 
              className={`notebook-cell ${cell.type}-cell ${activeCell === cell.id ? 'active' : ''} lang-${cell.language}`}
              onClick={() => setActiveCell(cell.id)}
            >
              <div className="cell-toolbar">
                <div className="cell-info">
                  <span className="cell-index">
                    {cell.type === 'code' && cell.metadata?.executionCount 
                      ? `[${cell.metadata.executionCount}]` 
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
                        <span className="lang-icon">{langInfo.icon}</span>
                        <span className="lang-name">{langInfo.label}</span>
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
                      insertCellAfter(cell.id, 'code', cell.language);
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
                <textarea
                  value={cell.content}
                  onChange={(e) => updateCell(cell.id, e.target.value)}
                  placeholder={
                    cell.type === 'code' 
                      ? `Enter ${langInfo.label} code...` 
                      : 'Enter markdown...'
                  }
                  className={`cell-textarea ${cell.type}-input lang-${cell.language}`}
                  spellCheck={false}
                  onKeyDown={(e) => {
                    if (e.shiftKey && e.key === 'Enter' && cell.type === 'code') {
                      e.preventDefault();
                      executeCell(cell.id);
                    }
                  }}
                />
              </div>

              {cell.output && (
                <div className="cell-output">
                  <div className="output-header">
                    <span className="output-label">Output:</span>
                    {cell.metadata?.lastExecuted && (
                      <span className="output-time">
                        {cell.metadata.lastExecuted.toLocaleTimeString()}
                      </span>
                    )}
                  </div>
                  <pre className={`output-content lang-${cell.language}`}>{cell.output}</pre>
                </div>
              )}
            </div>
          );
        })}
      </div>
    </div>
  );
};

