import React, { useEffect, useRef } from 'react';
import * as monaco from 'monaco-editor';
import { registerNavLambdaLanguage } from '../../services/navlambda-language';
import './MonacoEditor.css';

interface MonacoEditorProps {
  initialCode: string;
  onCodeChange: (code: string) => void;
  onCursorPositionChange?: (position: monaco.Position) => void;
  filePath?: string;
  language?: string; // Add language prop for multi-language support
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
}) => {
  const editorRef = useRef<monaco.editor.IStandaloneCodeEditor | null>(null);
  const containerRef = useRef<HTMLDivElement>(null);
  const isLanguageRegistered = useRef(false);

  useEffect(() => {
    if (!containerRef.current) return;

    // Register NAVΛ language once
    if (!isLanguageRegistered.current) {
      registerNavLambdaLanguage();
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

    return () => {
      editorRef.current?.dispose();
    };
  }, [language]); // Re-create editor when language changes

  // Update editor content when initialCode changes
  useEffect(() => {
    if (editorRef.current && initialCode !== editorRef.current.getValue()) {
      editorRef.current.setValue(initialCode);
    }
  }, [initialCode]);

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

  return (
    <div className="monaco-editor-wrapper">
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
        {language === 'navlambda' && (
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
              title="Union"
            >
              ∪⋋
            </button>
            <button
              className="vnc-symbol-btn"
              onClick={() => editorRef.current?.trigger('keyboard', 'type', { text: '∩⋋' })}
              title="Intersection"
            >
              ∩⋋
            </button>
            <button
              className="vnc-symbol-btn"
              onClick={() => editorRef.current?.trigger('keyboard', 'type', { text: '∇⋋' })}
              title="Gradient"
            >
              ∇⋋
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
              title="Evolution (Alt+Shift+E)"
            >
              ℰ
            </button>
          </div>
        )}
      </div>
      <div ref={containerRef} className="monaco-editor-container" />
    </div>
  );
};
