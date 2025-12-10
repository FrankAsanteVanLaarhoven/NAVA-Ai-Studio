// Widget Components
import React from 'react';
import { Calculator } from './Calculator';
import { Notes } from './Notes';
import { FileExplorer } from './FileExplorer';
import { Terminal } from './Terminal';
import { WidgetManager } from './WidgetManager';

export { Calculator, Notes, FileExplorer, Terminal, WidgetManager };

// Widget Types
export interface Widget {
  id: string;
  name: string;
  icon: string;
  component: React.ComponentType<any>;
  defaultSize?: { width: number; height: number };
  resizable?: boolean;
  category: 'productivity' | 'development' | 'utility';
}

// Available Widgets Configuration
export const availableWidgets: Widget[] = [
  {
    id: 'calculator',
    name: 'Calculator',
    icon: '🧮',
    component: Calculator,
    defaultSize: { width: 280, height: 400 },
    resizable: false,
    category: 'utility',
  },
  {
    id: 'notes',
    name: 'Notes',
    icon: '📝',
    component: Notes,
    defaultSize: { width: 800, height: 600 },
    resizable: true,
    category: 'productivity',
  },
  {
    id: 'file-explorer',
    name: 'File Explorer',
    icon: '📁',
    component: FileExplorer,
    defaultSize: { width: 600, height: 500 },
    resizable: true,
    category: 'development',
  },
  {
    id: 'terminal',
    name: 'Terminal',
    icon: '⚡',
    component: Terminal,
    defaultSize: { width: 700, height: 500 },
    resizable: true,
    category: 'development',
  },
];