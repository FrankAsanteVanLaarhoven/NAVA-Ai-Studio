import React, { useEffect, useRef } from 'react';
import * as monaco from 'monaco-editor';
import './MonacoEditor.css';

interface MonacoEditorProps {
  initialCode: string;
  onCodeChange: (code: string) => void;
  onCursorPositionChange?: (position: monaco.Position) => void;
}

/**
 * Monaco Editor with Native ⋋ Symbol Support
 * 
 * Revolutionary code editor for Van Laarhoven Navigation Calculus
 */
export const NavLambdaMonacoEditor: React.FC<MonacoEditorProps> = ({
  initialCode,
  onCodeChange,
  onCursorPositionChange,
}) => {
  const editorRef = useRef<monaco.editor.IStandaloneCodeEditor | null>(null);
  const containerRef = useRef<HTMLDivElement>(null);

  useEffect(() => {
    if (!containerRef.current) return;

    // Register NAVΛ language with Monaco
    monaco.languages.register({ id: 'navlambda' });

    // Define NAVΛ syntax highlighting with ⋋ symbols
    monaco.languages.setMonarchTokensProvider('navlambda', {
      tokenizer: {
        root: [
          // Van Laarhoven symbols
          [/⋋/, 'vnc-lambda-nav'],
          [/⊗⋋/, 'vnc-nav-tensor'],
          [/⊕⋋/, 'vnc-nav-sum'],
          [/∪⋋/, 'vnc-nav-union'],
          [/∩⋋/, 'vnc-nav-intersection'],
          [/[↑↓→←]⋋/, 'vnc-nav-direction'],

          // Master operators
          [/𝒩ℐ/, 'vnc-master-operator'],
          [/ℰ/, 'vnc-evolution-operator'],

          // Navigation keywords
          [/\bnavigate_to⋋\b/, 'keyword'],
          [/\bfind_optimal_path⋋\b/, 'keyword'],
          [/\benergy_landscape⋋\b/, 'keyword'],

          // Standard tokens
          [/\b(let|fn|return|if|else|while|for)\b/, 'keyword'],
          [/\b\d+(\.\d+)?\b/, 'number'],
          [/"([^"\\]|\\.)*"/, 'string'],
          [/'([^'\\]|\\.)*'/, 'string'],
          [/\/\/.*$/, 'comment'],
          [/\/\*/, 'comment', '@comment'],
        ],
        comment: [
          [/[^/*]+/, 'comment'],
          [/\*\//, 'comment', '@pop'],
          [/[/*]/, 'comment'],
        ],
      },
    });

    // Define VNC theme with distinctive colors
    monaco.editor.defineTheme('vnc-theme', {
      base: 'vs-dark',
      inherit: true,
      rules: [
        { token: 'vnc-lambda-nav', foreground: '00ff00', fontStyle: 'bold' },
        { token: 'vnc-nav-tensor', foreground: '00ccff' },
        { token: 'vnc-nav-sum', foreground: '00ccff' },
        { token: 'vnc-nav-union', foreground: 'ff6600' },
        { token: 'vnc-nav-intersection', foreground: 'ff6600' },
        { token: 'vnc-nav-direction', foreground: 'ffff00' },
        { token: 'vnc-master-operator', foreground: 'ff0099', fontStyle: 'bold' },
        { token: 'vnc-evolution-operator', foreground: 'cc00ff' },
        { token: 'keyword', foreground: '569cd6', fontStyle: 'bold' },
        { token: 'comment', foreground: '6a9955' },
        { token: 'string', foreground: 'ce9178' },
        { token: 'number', foreground: 'b5cea8' },
      ],
      colors: {
        'editor.background': '#1a1a1a',
        'editor.foreground': '#ffffff',
        'editorCursor.foreground': '#00ff00',
        'editor.lineHighlightBackground': '#2a2a2a',
        'editorLineNumber.foreground': '#858585',
        'editor.selectionBackground': '#264f78',
        'editor.inactiveSelectionBackground': '#3a3d41',
      },
    });

    // Create editor instance
    editorRef.current = monaco.editor.create(containerRef.current, {
      value: initialCode,
      language: 'navlambda',
      theme: 'vnc-theme',
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

    // Add custom key bindings for ⋋ symbols
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

    return () => {
      editorRef.current?.dispose();
    };
  }, [initialCode, onCodeChange, onCursorPositionChange]);

  return (
    <div className="monaco-editor-wrapper">
      <div className="monaco-editor-toolbar">
        <div className="vnc-symbols-palette">
          <button
            className="vnc-symbol-btn"
            onClick={() => editorRef.current?.trigger('keyboard', 'type', { text: '⋋' })}
            title="Lambda Navigation (Alt+L)"
          >
            ⋋
          </button>
          <button
            className="vnc-symbol-btn"
            onClick={() => editorRef.current?.trigger('keyboard', 'type', { text: '⊗⋋' })}
            title="Tensor Product (Alt+Shift+T)"
          >
            ⊗⋋
          </button>
          <button
            className="vnc-symbol-btn"
            onClick={() => editorRef.current?.trigger('keyboard', 'type', { text: '⊕⋋' })}
            title="Navigation Sum (Alt+Shift+S)"
          >
            ⊕⋋
          </button>
          <button
            className="vnc-symbol-btn"
            onClick={() => editorRef.current?.trigger('keyboard', 'type', { text: '∪⋋' })}
          >
            ∪⋋
          </button>
          <button
            className="vnc-symbol-btn"
            onClick={() => editorRef.current?.trigger('keyboard', 'type', { text: '∩⋋' })}
          >
            ∩⋋
          </button>
          <button
            className="vnc-symbol-btn"
            onClick={() => editorRef.current?.trigger('keyboard', 'type', { text: '𝒩ℐ' })}
            title="Master Operator (Alt+Shift+M)"
          >
            𝒩ℐ
          </button>
          <button
            className="vnc-symbol-btn"
            onClick={() => editorRef.current?.trigger('keyboard', 'type', { text: 'ℰ' })}
          >
            ℰ
          </button>
        </div>
      </div>
      <div ref={containerRef} className="monaco-editor-container" />
    </div>
  );
};

