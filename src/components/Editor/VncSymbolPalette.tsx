import React from 'react';
import { VncSymbol } from '../../types';

interface VncSymbolPaletteProps {
  onSymbolInsert: (symbol: string) => void;
}

const VNC_SYMBOLS: VncSymbol[] = [
  { symbol: '⋋', name: 'Lambda Navigation', documentation: 'Core VNC operator', insertText: '⋋' },
  { symbol: '⊗⋋', name: 'Tensor Product', documentation: 'Navigation tensor product', insertText: '⊗⋋' },
  { symbol: '⊕⋋', name: 'Navigation Sum', documentation: 'Parallel paths', insertText: '⊕⋋' },
  { symbol: '∪⋋', name: 'Navigation Union', documentation: 'Merge spaces', insertText: '∪⋋' },
  { symbol: '∩⋋', name: 'Navigation Intersection', documentation: 'Common paths', insertText: '∩⋋' },
  { symbol: '↑⋋', name: 'North Navigation', documentation: 'Northward constraint', insertText: '↑⋋' },
  { symbol: '↓⋋', name: 'South Navigation', documentation: 'Southward constraint', insertText: '↓⋋' },
  { symbol: '→⋋', name: 'East Navigation', documentation: 'Eastward constraint', insertText: '→⋋' },
  { symbol: '←⋋', name: 'West Navigation', documentation: 'Westward constraint', insertText: '←⋋' },
  { symbol: '𝒩ℐ', name: 'Master Operator', documentation: 'Ultimate VNC optimization', insertText: '𝒩ℐ' },
  { symbol: 'ℰ', name: 'Evolution Operator', documentation: 'Time evolution', insertText: 'ℰ' },
];

export const VncSymbolPalette: React.FC<VncSymbolPaletteProps> = ({ onSymbolInsert }) => {
  return (
    <div className="vnc-symbol-palette-panel">
      <h3>VNC Symbols</h3>
      <div className="symbol-grid">
        {VNC_SYMBOLS.map((sym) => (
          <button
            key={sym.symbol}
            className="symbol-btn"
            onClick={() => onSymbolInsert(sym.insertText)}
            title={`${sym.name}\n${sym.documentation}`}
          >
            <span className="symbol">{sym.symbol}</span>
            <span className="name">{sym.name}</span>
          </button>
        ))}
      </div>
    </div>
  );
};

