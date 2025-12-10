import React, { useEffect, useRef, useState } from 'react';
import * as monaco from 'monaco-editor';
import { Terminal, Code, Search, LayoutGrid, ChevronDown, MessageSquare, PanelRight, Sidebar, Settings } from 'lucide-react';
import { registerNavLambdaLanguage } from '../../services/navlambda-language';
import { navaSemanticService } from '../../services/nava-semantic-service';
import './MonacoEditor.css';

interface MonacoEditorProps {
  initialCode: string;
  onCodeChange: (code: string) => void;
  onCursorPositionChange?: (position: monaco.Position) => void;
  filePath?: string;
  language?: string; // Add language prop for multi-language support
  onToggleTerminal?: () => void;
  onToggleFileReference?: (symbol?: string) => void;
  showTerminal?: boolean;
  showFileReference?: boolean;
  onToggleChat?: () => void;
  onTogglePanel?: () => void;
  onToggleSidebar?: () => void;
  showChat?: boolean;
  showPanel?: boolean;
  showSidebar?: boolean;
  onSearch?: (query: string) => void;
}

/**
 * Monaco Editor with Native ⋋ Symbol Support and Multi-Language Capabilities
 *
 * Revolutionary code editor for Van Laarhoven Navigation Calculus and all major programming languages
 */
export const NavLambdaMonacoEditor: React.FC<MonacoEditorProps> = ({
  initialCode,
  onCodeChange,
  onCursorPositionChange,
  filePath,
  language = 'navlambda', // Default to NAVΛ but allow other languages
  onToggleTerminal,
  onToggleFileReference,
  showTerminal = false,
  showFileReference = false,
  onToggleChat,
  onTogglePanel,
  onToggleSidebar,
  showChat = false,
  showPanel = false,
  showSidebar = true,
  onSearch,
}) => {
  const [searchQuery, setSearchQuery] = useState('');
  const [showLayoutMenu, setShowLayoutMenu] = useState(false);
  const [sidebarPosition, setSidebarPosition] = useState<'left' | 'right'>('left');
  const [showDiffs, setShowDiffs] = useState(true);
  const layoutMenuRef = useRef<HTMLDivElement>(null);

  // Close layout menu when clicking outside
  useEffect(() => {
    const handleClickOutside = (event: MouseEvent) => {
      if (layoutMenuRef.current && !layoutMenuRef.current.contains(event.target as Node)) {
        setShowLayoutMenu(false);
      }
    };

    if (showLayoutMenu) {
      document.addEventListener('mousedown', handleClickOutside);
      return () => document.removeEventListener('mousedown', handleClickOutside);
    }
  }, [showLayoutMenu]);
  const editorRef = useRef<monaco.editor.IStandaloneCodeEditor | null>(null);
  const containerRef = useRef<HTMLDivElement>(null);
  const isLanguageRegistered = useRef(false);

  useEffect(() => {
    if (!containerRef.current) return;

    // Register NAVΛ language once
    if (!isLanguageRegistered.current) {
      registerNavLambdaLanguage();
      registerNAVASemanticProviders();
      isLanguageRegistered.current = true;
    }

    // Create editor instance with dynamic language support
    editorRef.current = monaco.editor.create(containerRef.current, {
      value: initialCode,
      language: language, // Use dynamic language
      theme: language === 'navlambda' ? 'vnc-theme' : 'vs-dark',
      fontSize: 14,
      fontFamily: 'JetBrains Mono, Consolas, monospace',
      lineNumbers: 'on',
      roundedSelection: false,
      scrollBeyondLastLine: false,
      automaticLayout: true,
      minimap: { enabled: true },
      quickSuggestions: {
        other: true,
        comments: true,
        strings: true,
      },
      suggestOnTriggerCharacters: true,
      tabSize: 2,
      insertSpaces: true,
      wordWrap: 'on',
      folding: true,
      foldingStrategy: 'indentation',
      showFoldingControls: 'always',
      bracketPairColorization: {
        enabled: true,
      },
      guides: {
        bracketPairs: true,
        indentation: true,
      },
    });

    // Handle code changes
    editorRef.current.onDidChangeModelContent(() => {
      const code = editorRef.current?.getValue() || '';
      onCodeChange(code);
    });

    // Handle cursor position changes
    if (onCursorPositionChange) {
      editorRef.current.onDidChangeCursorPosition((e) => {
        onCursorPositionChange(e.position);
      });
    }

    // Listen for external update events
    const handleEditorUpdate = (event: CustomEvent) => {
      if (event.detail && event.detail.path === filePath && editorRef.current) {
        const newContent = event.detail.content || '';
        editorRef.current.setValue(newContent);
        editorRef.current.setPosition({ lineNumber: 1, column: 1 });
        editorRef.current.focus();
      }
    };
    
    window.addEventListener('nava:editor-update', handleEditorUpdate as EventListener);

    // Add custom key bindings for ⋋ symbols (only for NAVΛ language)
    if (language === 'navlambda') {
      editorRef.current.addCommand(
        monaco.KeyMod.Alt | monaco.KeyCode.KeyL,
        () => {
          editorRef.current?.trigger('keyboard', 'type', { text: '⋋' });
        }
      );

      editorRef.current.addCommand(
        monaco.KeyMod.Alt | monaco.KeyMod.Shift | monaco.KeyCode.KeyT,
        () => {
          editorRef.current?.trigger('keyboard', 'type', { text: '⊗⋋' });
        }
      );

      editorRef.current.addCommand(
        monaco.KeyMod.Alt | monaco.KeyMod.Shift | monaco.KeyCode.KeyS,
        () => {
          editorRef.current?.trigger('keyboard', 'type', { text: '⊕⋋' });
        }
      );

      editorRef.current.addCommand(
        monaco.KeyMod.Alt | monaco.KeyMod.Shift | monaco.KeyCode.KeyM,
        () => {
          editorRef.current?.trigger('keyboard', 'type', { text: '𝒩ℐ' });
        }
      );

      editorRef.current.addCommand(
        monaco.KeyMod.Alt | monaco.KeyMod.Shift | monaco.KeyCode.KeyE,
        () => {
          editorRef.current?.trigger('keyboard', 'type', { text: 'ℰ' });
        }
      );
    }

    // Add save command (Ctrl+S / Cmd+S)
    editorRef.current.addCommand(
      monaco.KeyMod.CtrlCmd | monaco.KeyCode.KeyS,
      () => {
        // Trigger save event
        const event = new CustomEvent('editor-save', {
          detail: { code: editorRef.current?.getValue() },
        });
        window.dispatchEvent(event);
      }
    );

    // Handle navigate to line events
    const handleNavigateToLine = (event: CustomEvent) => {
      const { line } = event.detail;
      if (editorRef.current && line > 0) {
        editorRef.current.setPosition({ lineNumber: line, column: 1 });
        editorRef.current.revealLineInCenter(line);
        editorRef.current.focus();
      }
    };

    window.addEventListener('nava:navigate-to-line', handleNavigateToLine as EventListener);

    // Handle editor commands from menu bar
    const handleEditorUndo = () => {
      if (editorRef.current) {
        editorRef.current.trigger('keyboard', 'undo', {});
      }
    };

    const handleEditorRedo = () => {
      if (editorRef.current) {
        editorRef.current.trigger('keyboard', 'redo', {});
      }
    };

    const handleEditorFind = () => {
      if (editorRef.current) {
        editorRef.current.trigger('keyboard', 'actions.find', {});
      }
    };

    const handleEditorReplace = () => {
      if (editorRef.current) {
        editorRef.current.trigger('keyboard', 'editor.action.startFindReplaceAction', {});
      }
    };

    const handleEditorToggleComment = () => {
      if (editorRef.current) {
        editorRef.current.trigger('keyboard', 'editor.action.commentLine', {});
      }
    };

    const handleEditorSelectAll = () => {
      if (editorRef.current) {
        editorRef.current.trigger('keyboard', 'editor.action.selectAll', {});
      }
    };

    const handleEditorToggleWordWrap = () => {
      if (editorRef.current) {
        const currentValue = editorRef.current.getOption(monaco.editor.Option.wordWrap);
        editorRef.current.updateOptions({
          wordWrap: currentValue === 'on' ? 'off' : 'on',
        });
      }
    };

    const handleGoToDefinition = () => {
      if (editorRef.current) {
        editorRef.current.trigger('keyboard', 'editor.action.revealDefinition', {});
      }
    };

    const handleToggleBreakpoint = () => {
      if (editorRef.current) {
        editorRef.current.trigger('keyboard', 'editor.debug.action.toggleBreakpoint', {});
      }
    };

    window.addEventListener('nava:editor-undo', handleEditorUndo);
    window.addEventListener('nava:editor-redo', handleEditorRedo);
    window.addEventListener('nava:editor-find', handleEditorFind);
    window.addEventListener('nava:editor-replace', handleEditorReplace);
    window.addEventListener('nava:editor-toggle-comment', handleEditorToggleComment);
    window.addEventListener('nava:editor-select-all', handleEditorSelectAll);
    window.addEventListener('nava:editor-toggle-word-wrap', handleEditorToggleWordWrap);
    window.addEventListener('nava:go-to-definition', handleGoToDefinition);
    window.addEventListener('nava:toggle-breakpoint', handleToggleBreakpoint);

    const handleEditorCut = () => {
      if (editorRef.current) {
        editorRef.current.trigger('keyboard', 'editor.action.clipboardCutAction', {});
      }
    };

    const handleEditorCopy = () => {
      if (editorRef.current) {
        editorRef.current.trigger('keyboard', 'editor.action.clipboardCopyAction', {});
      }
    };

    const handleEditorPaste = () => {
      if (editorRef.current) {
        editorRef.current.trigger('keyboard', 'editor.action.clipboardPasteAction', {});
      }
    };

    window.addEventListener('nava:editor-cut', handleEditorCut);
    window.addEventListener('nava:editor-copy', handleEditorCopy);
    window.addEventListener('nava:editor-paste', handleEditorPaste);

    // Cleanup function
    return () => {
      window.removeEventListener('nava:editor-update', handleEditorUpdate as EventListener);
      window.removeEventListener('nava:navigate-to-line', handleNavigateToLine as EventListener);
      window.removeEventListener('nava:editor-undo', handleEditorUndo);
      window.removeEventListener('nava:editor-redo', handleEditorRedo);
      window.removeEventListener('nava:editor-find', handleEditorFind);
      window.removeEventListener('nava:editor-replace', handleEditorReplace);
      window.removeEventListener('nava:editor-toggle-comment', handleEditorToggleComment);
      window.removeEventListener('nava:editor-select-all', handleEditorSelectAll);
      window.removeEventListener('nava:editor-toggle-word-wrap', handleEditorToggleWordWrap);
      window.removeEventListener('nava:go-to-definition', handleGoToDefinition);
      window.removeEventListener('nava:toggle-breakpoint', handleToggleBreakpoint);
      window.removeEventListener('nava:editor-cut', handleEditorCut);
      window.removeEventListener('nava:editor-copy', handleEditorCopy);
      window.removeEventListener('nava:editor-paste', handleEditorPaste);
      if (editorRef.current) {
        editorRef.current.dispose();
        editorRef.current = null;
      }
    };
  }, [language, filePath]); // Re-create editor when language or file path changes

  // Register NAVA semantic providers (go-to-definition, hover, etc.)
  function registerNAVASemanticProviders() {
    if (language !== 'navlambda') return;

    // Hover provider with semantic info
    monaco.languages.registerHoverProvider('navlambda', {
      provideHover: (model, position) => {
        const code = model.getValue();
        const line = position.lineNumber - 1;
        const column = position.column - 1;
        const info = navaSemanticService.getHoverInfo(code, line, column);
        
        if (info) {
          return {
            range: new monaco.Range(position.lineNumber, position.column, position.lineNumber, position.column),
            contents: [{ value: info, isTrusted: true }],
          };
        }
        return null;
      },
    });

    // Go-to-definition provider
    monaco.languages.registerDefinitionProvider('navlambda', {
      provideDefinition: (model, position) => {
        const code = model.getValue();
        const line = position.lineNumber - 1;
        const column = position.column - 1;
        const symbol = navaSemanticService.findDefinition(code, line, column);
        
        if (symbol && symbol.definition.range) {
          return {
            uri: model.uri,
            range: new monaco.Range(
              symbol.definition.range.line + 1,
              symbol.definition.range.column + 1,
              symbol.definition.range.line + 1,
              symbol.definition.range.column + symbol.definition.name!.length + 1
            ),
          };
        }
        return null;
      },
    });

    // References provider
    monaco.languages.registerReferenceProvider('navlambda', {
      provideReferences: (model, position) => {
        const code = model.getValue();
        const line = position.lineNumber - 1;
        const column = position.column - 1;
        const symbol = navaSemanticService.findDefinition(code, line, column);
        
        if (symbol) {
          // Trigger file reference viewer
          if (onToggleFileReference && symbol.name) {
            setTimeout(() => {
              onToggleFileReference(symbol.name);
            }, 100);
          }
          
          const refs = symbol.references.map(ref => ({
            uri: model.uri,
            range: new monaco.Range(ref.line + 1, ref.column + 1, ref.line + 1, ref.column + symbol.name.length + 1),
          }));
          
          // Include definition
          if (symbol.definition.range) {
            refs.unshift({
              uri: model.uri,
              range: new monaco.Range(
                symbol.definition.range.line + 1,
                symbol.definition.range.column + 1,
                symbol.definition.range.line + 1,
                symbol.definition.range.column + symbol.name.length + 1
              ),
            });
          }
          
          return refs;
        }
        return [];
      },
    });

    // Enhanced completion provider with semantic suggestions
    monaco.languages.registerCompletionItemProvider('navlambda', {
      triggerCharacters: ['.', ':', '(', '[', ' '],
      provideCompletionItems: (model, position) => {
        const code = model.getValue();
        const line = position.lineNumber - 1;
        const column = position.column - 1;
        const semanticCompletions = navaSemanticService.getCompletions(code, line, column);
        
        const word = model.getWordUntilPosition(position);
        const range = new monaco.Range(
          position.lineNumber,
          word.startColumn,
          position.lineNumber,
          word.endColumn
        );

        const suggestions: monaco.languages.CompletionItem[] = semanticCompletions.map(comp => ({
          label: comp.label,
          kind: comp.kind === 'function' ? monaco.languages.CompletionItemKind.Function :
                comp.kind === 'variable' ? monaco.languages.CompletionItemKind.Variable :
                comp.kind === 'manifold' ? monaco.languages.CompletionItemKind.Class :
                comp.kind === 'field' ? monaco.languages.CompletionItemKind.Field :
                monaco.languages.CompletionItemKind.Text,
          detail: comp.detail,
          insertText: comp.insertText,
          range,
          documentation: { value: comp.detail, isTrusted: true },
        }));

        return { suggestions };
      },
    });

    // Document symbol provider (for outline)
    monaco.languages.registerDocumentSymbolProvider('navlambda', {
      provideDocumentSymbols: (model) => {
        const code = model.getValue();
        const semantic = navaSemanticService.getSemanticInfo(code);
        const symbols: monaco.languages.DocumentSymbol[] = [];

        semantic.symbols.forEach((symbol, name) => {
          if (symbol.definition.range) {
            symbols.push({
              name,
              kind: symbol.type === 'function' ? monaco.languages.SymbolKind.Function :
                    symbol.type === 'variable' ? monaco.languages.SymbolKind.Variable :
                    symbol.type === 'manifold' ? monaco.languages.SymbolKind.Class :
                    monaco.languages.SymbolKind.Variable,
              range: new monaco.Range(
                symbol.definition.range.line + 1,
                symbol.definition.range.column + 1,
                symbol.definition.range.line + 1,
                symbol.definition.range.column + name.length + 1
              ),
              selectionRange: new monaco.Range(
                symbol.definition.range.line + 1,
                symbol.definition.range.column + 1,
                symbol.definition.range.line + 1,
                symbol.definition.range.column + name.length + 1
              ),
              detail: symbol.type,
            });
          }
        });

        return symbols;
      },
    });
  }

  // Update editor content when initialCode or filePath changes
  useEffect(() => {
    if (editorRef.current && initialCode !== undefined) {
      const currentValue = editorRef.current.getValue();
      // Always update if content is different or file path changed
      if (initialCode !== currentValue || (filePath && currentValue === '')) {
        const newContent = initialCode || '';
        editorRef.current.setValue(newContent);
        // Move cursor to top
        editorRef.current.setPosition({ lineNumber: 1, column: 1 });
        // Focus editor
        editorRef.current.focus();
        console.log('[MonacoEditor] Updated content for file:', filePath, 'Length:', newContent.length);
      }
    }
  }, [initialCode, filePath]);

  // Update language when prop changes
  useEffect(() => {
    if (editorRef.current) {
      const model = editorRef.current.getModel();
      if (model) {
        monaco.editor.setModelLanguage(model, language);
        // Update theme based on language
        monaco.editor.setTheme(language === 'navlambda' ? 'vnc-theme' : 'vs-dark');
      }
    }
  }, [language]);

  const handleSymbolClick = (symbol: string) => {
    const position = editorRef.current?.getPosition();
    if (position && editorRef.current) {
      editorRef.current.executeEdits('insert-symbol', [{
        range: new monaco.Range(position.lineNumber, position.column, position.lineNumber, position.column),
        text: symbol,
      }]);
    }
  };

  const handleFindReferences = () => {
    if (editorRef.current) {
      const position = editorRef.current.getPosition();
      if (position) {
        const model = editorRef.current.getModel();
        if (model) {
          const word = model.getWordAtPosition(position);
          if (word && onToggleFileReference) {
            onToggleFileReference(word.word);
          }
        }
      }
    }
  };

  const handleSearch = (e: React.FormEvent) => {
    e.preventDefault();
    if (onSearch && searchQuery.trim()) {
      onSearch(searchQuery);
    }
  };

  return (
    <div className="monaco-editor-wrapper">
      {/* NAVLAMBDA Badge - Top Only */}
      <div className="navlambda-badge-top">
        <div className="navlambda-badge-content">
          <span className="navlambda-badge-text">NAV<span className="navlambda-lambda">Λ</span>LAMBDA</span>
          
          {/* VS Code/Cursor Style Toolbar */}
          <div className="editor-top-toolbar">
            {/* Search Bar */}
            <form onSubmit={handleSearch} className="editor-search-form">
              <Search size={14} className="search-icon" />
              <input
                type="text"
                placeholder="Search..."
                value={searchQuery}
                onChange={(e) => setSearchQuery(e.target.value)}
                className="editor-search-input"
                onKeyDown={(e) => {
                  if (e.key === 'Escape') {
                    setSearchQuery('');
                  }
                }}
              />
            </form>

            {/* Symbol Buttons (NAVΛ specific) */}
            {language === 'navlambda' && (
              <div className="vnc-symbols-toolbar">
                <button
                  className="vnc-symbol-toolbar-btn"
                  onClick={() => handleSymbolClick('⋋')}
                  title="Lambda Navigation"
                >
                  ⋋
                </button>
                <button
                  className="vnc-symbol-toolbar-btn"
                  onClick={() => handleSymbolClick('⊕⋋')}
                  title="Navigation Sum"
                >
                  ⊕⋋
                </button>
                <button
                  className="vnc-symbol-toolbar-btn"
                  onClick={() => handleSymbolClick('∪⋋')}
                  title="Union"
                >
                  ∪⋋
                </button>
                <button
                  className="vnc-symbol-toolbar-btn"
                  onClick={() => handleSymbolClick('∩⋋')}
                  title="Intersection"
                >
                  ∩⋋
                </button>
                <button
                  className="vnc-symbol-toolbar-btn"
                  onClick={() => handleSymbolClick('∇⋋')}
                  title="Gradient"
                >
                  ∇⋋
                </button>
                <button
                  className="vnc-symbol-toolbar-btn"
                  onClick={() => handleSymbolClick('𝒩ℐ')}
                  title="Master Operator"
                >
                  𝒩ℐ
                </button>
                <button
                  className="vnc-symbol-toolbar-btn"
                  onClick={() => handleSymbolClick('ℰ')}
                  title="Evolution"
                >
                  ℰ
                </button>
              </div>
            )}

            {/* Layout Controls */}
            <div className="editor-layout-controls">
              <div className="layout-menu-container" ref={layoutMenuRef}>
                <button
                  className="layout-menu-btn"
                  onClick={() => setShowLayoutMenu(!showLayoutMenu)}
                  title="Custom Layout"
                >
                  <LayoutGrid size={14} />
                  <ChevronDown size={12} />
                </button>
                {showLayoutMenu && (
                  <div className="layout-menu-dropdown" onClick={(e) => e.stopPropagation()}>
                    <div className="layout-menu-section">
                      <div className="layout-menu-item" onClick={() => { onToggleChat?.(); setShowLayoutMenu(false); }}>
                        <MessageSquare size={14} />
                        <span>Chat</span>
                        <span className="layout-shortcut">⌥⌘B</span>
                        <div className={`layout-toggle ${showChat ? 'on' : 'off'}`}>
                          <div className="toggle-slider"></div>
                        </div>
                      </div>
                      <div className="layout-menu-item" onClick={() => { onTogglePanel?.(); setShowLayoutMenu(false); }}>
                        <PanelRight size={14} />
                        <span>Panel</span>
                        <span className="layout-shortcut">⌘J</span>
                        <div className={`layout-toggle ${showPanel ? 'on' : 'off'}`}>
                          <div className="toggle-slider"></div>
                        </div>
                      </div>
                      <div className="layout-menu-item" onClick={() => { onToggleSidebar?.(); setShowLayoutMenu(false); }}>
                        <Sidebar size={14} />
                        <span>Sidebar</span>
                        <span className="layout-shortcut">⌘B</span>
                        <div className={`layout-toggle ${showSidebar ? 'on' : 'off'}`}>
                          <div className="toggle-slider"></div>
                        </div>
                      </div>
                    </div>
                    <div className="layout-menu-divider"></div>
                    <div className="layout-menu-section">
                      <div className="layout-menu-item">
                        <span>Sidebar Position</span>
                        <span>{sidebarPosition}</span>
                        <ChevronDown size={12} />
                      </div>
                      <div className="layout-menu-item">
                        <span>Show Diffs</span>
                        <span>{showDiffs ? 'On' : 'Off'}</span>
                        <ChevronDown size={12} />
                      </div>
                    </div>
                    <div className="layout-menu-divider"></div>
                    <div className="layout-menu-section">
                      <div className="layout-menu-item">
                        <Settings size={14} />
                        <span>Cursor Settings</span>
                        <span className="layout-shortcut">⇧⌘J</span>
                      </div>
                    </div>
                  </div>
                )}
              </div>
            </div>

            {/* Action Buttons */}
            <div className="editor-action-buttons">
              <button
                className={`editor-action-btn ${showFileReference ? 'active' : ''}`}
                onClick={handleFindReferences}
                title="Find All References (Shift+F12)"
              >
                <Code size={14} />
              </button>
              {onToggleTerminal && (
                <button
                  className={`editor-action-btn ${showTerminal ? 'active' : ''}`}
                  onClick={onToggleTerminal}
                  title="Toggle Terminal (Ctrl+`)"
                >
                  <Terminal size={14} />
                </button>
              )}
            </div>
          </div>
        </div>
      </div>
      
      {/* File Info Bar - Minimal */}
      <div className="monaco-editor-toolbar">
        <div className="file-info">
          {filePath && (
            <span className="file-path" title={filePath}>
              {filePath.split('/').pop()}
            </span>
          )}
          <span className="language-indicator">
            {language.toUpperCase()}
          </span>
        </div>
      </div>
      <div ref={containerRef} className="monaco-editor-container" />
    </div>
  );
};
