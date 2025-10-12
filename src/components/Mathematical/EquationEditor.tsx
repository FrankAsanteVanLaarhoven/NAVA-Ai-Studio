import React, { useState } from 'react';
import { VncMathRenderer } from './VncMathRenderer';

interface EquationEditorProps {
  initialEquation?: string;
  onSave: (equation: string) => void;
}

export const EquationEditor: React.FC<EquationEditorProps> = ({
  initialEquation = '',
  onSave,
}) => {
  const [equation, setEquation] = useState(initialEquation);
  const [preview, setPreview] = useState(true);

  const handleInsertSymbol = (symbol: string) => {
    setEquation((prev) => prev + symbol);
  };

  const vncSymbols = ['⋋', '⊗⋋', '⊕⋋', '∪⋋', '∩⋋', '𝒩ℐ', 'ℰ'];

  return (
    <div className="equation-editor">
      <div className="editor-toolbar">
        {vncSymbols.map((symbol) => (
          <button key={symbol} onClick={() => handleInsertSymbol(symbol)}>
            {symbol}
          </button>
        ))}
        <button onClick={() => setPreview(!preview)}>
          {preview ? 'Hide' : 'Show'} Preview
        </button>
        <button onClick={() => onSave(equation)} className="save-btn">
          Save
        </button>
      </div>
      <textarea
        value={equation}
        onChange={(e) => setEquation(e.target.value)}
        className="equation-input"
        placeholder="Enter VNC equation..."
      />
      {preview && (
        <div className="equation-preview">
          <VncMathRenderer equation={equation} displayMode />
        </div>
      )}
    </div>
  );
};

