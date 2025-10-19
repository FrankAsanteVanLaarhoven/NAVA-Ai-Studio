import * as monaco from 'monaco-editor';

/**
 * Van Laarhoven Navigation Calculus Language Definition
 * Complete language support for NAVΛ programming
 */

export interface NavLambdaLanguageConfig {
  keywords: string[];
  operators: string[];
  symbols: string[];
  builtinFunctions: string[];
}

export const navLambdaConfig: NavLambdaLanguageConfig = {
  keywords: [
    // Control flow
    'let', 'fn', 'return', 'if', 'else', 'while', 'for', 'match', 'case',
    'break', 'continue', 'yield', 'async', 'await',

    // Navigation keywords
    'navigate_to⋋', 'find_optimal_path⋋', 'energy_landscape⋋',
    'compute_gradient⋋', 'optimize_trajectory⋋', 'minimize_energy⋋',
    'maximize_flow⋋', 'balance_forces⋋', 'stabilize_system⋋',

    // Type keywords
    'type', 'struct', 'enum', 'trait', 'impl', 'where',
    'pub', 'priv', 'const', 'static', 'mut', 'ref',

    // Navigation types
    'Path⋋', 'Trajectory⋋', 'EnergyField⋋', 'GradientFlow⋋',
    'NavigationSpace⋋', 'OptimalRoute⋋', 'Waypoint⋋',

    // Module system
    'import', 'export', 'module', 'use', 'as', 'from',
  ],

  operators: [
    // Van Laarhoven operators
    '⋋', '⊗⋋', '⊕⋋', '∪⋋', '∩⋋', '↑⋋', '↓⋋', '→⋋', '←⋋',
    '𝒩ℐ', 'ℰ', '∇⋋', '∂⋋', '∫⋋', '∑⋋', '∏⋋',

    // Standard operators
    '+', '-', '*', '/', '%', '**',
    '=', '==', '!=', '<', '>', '<=', '>=',
    '&&', '||', '!', '&', '|', '^', '~',
    '<<', '>>', '++', '--',
    '+=', '-=', '*=', '/=', '%=',
    '->', '=>', '::', '..', '...', '?',
  ],

  symbols: [
    // Mathematical symbols
    '∞', '∅', '∈', '∉', '⊂', '⊃', '⊆', '⊇',
    '∀', '∃', '∄', '∧', '∨', '¬', '⊤', '⊥',
    '≈', '≠', '≤', '≥', '≡', '≢',

    // Greek letters (commonly used in math)
    'α', 'β', 'γ', 'δ', 'ε', 'ζ', 'η', 'θ',
    'λ', 'μ', 'ν', 'ξ', 'π', 'ρ', 'σ', 'τ',
    'φ', 'χ', 'ψ', 'ω',

    // Navigation symbols
    '⋋', '⊗', '⊕', '∪', '∩', '↑', '↓', '→', '←',
    '⇒', '⇐', '⇔', '↔', '↕',
  ],

  builtinFunctions: [
    // Navigation functions
    'navigate', 'optimize', 'minimize', 'maximize',
    'gradient', 'divergence', 'curl', 'laplacian',
    'integrate', 'differentiate', 'transform',

    // Energy functions
    'compute_energy', 'energy_gradient', 'potential_field',
    'kinetic_energy', 'total_energy', 'energy_conservation',

    // Path functions
    'shortest_path', 'optimal_path', 'smooth_path',
    'interpolate_path', 'sample_path', 'evaluate_path',

    // Utility functions
    'print', 'debug', 'assert', 'panic', 'log',
    'map', 'filter', 'reduce', 'fold', 'scan',
    'zip', 'enumerate', 'range', 'collect',
  ],
};

/**
 * Monaco Monarch Language Definition for NAVΛ
 */
export const navLambdaMonarchLanguage: monaco.languages.IMonarchLanguage = {
  defaultToken: '',
  tokenPostfix: '.navlambda',

  keywords: navLambdaConfig.keywords,
  operators: navLambdaConfig.operators,
  builtinFunctions: navLambdaConfig.builtinFunctions,
  symbols: /[=><!~?:&|+\-*\/\^%⋋⊗⊕∪∩↑↓→←𝒩ℐℰ∇∂∫∑∏]+/,

  escapes: /\\(?:[abfnrtv\\"']|x[0-9A-Fa-f]{1,4}|u[0-9A-Fa-f]{4}|U[0-9A-Fa-f]{8})/,

  tokenizer: {
    root: [
      // Van Laarhoven Navigation symbols (highest priority)
      [/⋋/, 'vnc-lambda-nav'],
      [/⊗⋋/, 'vnc-nav-tensor'],
      [/⊕⋋/, 'vnc-nav-sum'],
      [/∪⋋/, 'vnc-nav-union'],
      [/∩⋋/, 'vnc-nav-intersection'],
      [/[↑↓→←]⋋/, 'vnc-nav-direction'],
      [/∇⋋/, 'vnc-nav-gradient'],
      [/∂⋋/, 'vnc-nav-partial'],
      [/∫⋋/, 'vnc-nav-integral'],
      [/∑⋋/, 'vnc-nav-sum-op'],
      [/∏⋋/, 'vnc-nav-product'],

      // Master operators
      [/𝒩ℐ/, 'vnc-master-operator'],
      [/ℰ/, 'vnc-evolution-operator'],

      // Identifiers and keywords
      [/[a-zA-Z_]\w*⋋/, {
        cases: {
          '@keywords': 'keyword',
          '@default': 'identifier.nav'
        }
      }],

      [/[a-zA-Z_]\w*/, {
        cases: {
          '@keywords': 'keyword',
          '@builtinFunctions': 'function.builtin',
          '@default': 'identifier'
        }
      }],

      // Whitespace
      { include: '@whitespace' },

      // Delimiters and operators
      [/[{}()\[\]]/, '@brackets'],
      [/[<>](?!@symbols)/, '@brackets'],
      [/@symbols/, {
        cases: {
          '@operators': 'operator',
          '@default': ''
        }
      }],

      // Numbers
      [/\d*\.\d+([eE][\-+]?\d+)?/, 'number.float'],
      [/0[xX][0-9a-fA-F]+/, 'number.hex'],
      [/0[oO][0-7]+/, 'number.octal'],
      [/0[bB][01]+/, 'number.binary'],
      [/\d+/, 'number'],

      // Delimiter: after number because of .\d floats
      [/[;,.]/, 'delimiter'],

      // Strings
      [/"([^"\\]|\\.)*$/, 'string.invalid'],
      [/"/, { token: 'string.quote', bracket: '@open', next: '@string' }],

      // Characters
      [/'[^\\']'/, 'string'],
      [/(')(@escapes)(')/, ['string', 'string.escape', 'string']],
      [/'/, 'string.invalid'],
    ],

    comment: [
      [/[^\/*]+/, 'comment'],
      [/\/\*/, 'comment', '@push'],
      ['\\*/', 'comment', '@pop'],
      [/[\/*]/, 'comment']
    ],

    string: [
      [/[^\\"]+/, 'string'],
      [/@escapes/, 'string.escape'],
      [/\\./, 'string.escape.invalid'],
      [/"/, { token: 'string.quote', bracket: '@close', next: '@pop' }]
    ],

    whitespace: [
      [/[ \t\r\n]+/, 'white'],
      [/\/\*/, 'comment', '@comment'],
      [/\/\/.*$/, 'comment'],
    ],
  },
};

/**
 * Monaco Theme for NAVΛ
 */
export const navLambdaTheme: monaco.editor.IStandaloneThemeData = {
  base: 'vs-dark',
  inherit: true,
  rules: [
    // Van Laarhoven Navigation symbols
    { token: 'vnc-lambda-nav', foreground: '00ff00', fontStyle: 'bold' },
    { token: 'vnc-nav-tensor', foreground: '00ccff', fontStyle: 'bold' },
    { token: 'vnc-nav-sum', foreground: '00ccff', fontStyle: 'bold' },
    { token: 'vnc-nav-union', foreground: 'ff6600', fontStyle: 'bold' },
    { token: 'vnc-nav-intersection', foreground: 'ff6600', fontStyle: 'bold' },
    { token: 'vnc-nav-direction', foreground: 'ffff00', fontStyle: 'bold' },
    { token: 'vnc-nav-gradient', foreground: 'ff00ff', fontStyle: 'bold' },
    { token: 'vnc-nav-partial', foreground: 'ff00ff' },
    { token: 'vnc-nav-integral', foreground: 'ff00ff' },
    { token: 'vnc-nav-sum-op', foreground: 'ff00ff' },
    { token: 'vnc-nav-product', foreground: 'ff00ff' },

    // Master operators
    { token: 'vnc-master-operator', foreground: 'ff0099', fontStyle: 'bold' },
    { token: 'vnc-evolution-operator', foreground: 'cc00ff', fontStyle: 'bold' },

    // Standard tokens
    { token: 'keyword', foreground: '569cd6', fontStyle: 'bold' },
    { token: 'identifier', foreground: 'd4d4d4' },
    { token: 'identifier.nav', foreground: '4ec9b0', fontStyle: 'italic' },
    { token: 'function.builtin', foreground: 'dcdcaa' },
    { token: 'comment', foreground: '6a9955', fontStyle: 'italic' },
    { token: 'string', foreground: 'ce9178' },
    { token: 'string.escape', foreground: 'd7ba7d' },
    { token: 'number', foreground: 'b5cea8' },
    { token: 'number.float', foreground: 'b5cea8' },
    { token: 'number.hex', foreground: 'b5cea8' },
    { token: 'operator', foreground: 'd4d4d4' },
    { token: 'delimiter', foreground: 'd4d4d4' },
  ],
  colors: {
    'editor.background': '#1a1a1a',
    'editor.foreground': '#ffffff',
    'editorCursor.foreground': '#00ff00',
    'editor.lineHighlightBackground': '#2a2a2a',
    'editorLineNumber.foreground': '#858585',
    'editorLineNumber.activeForeground': '#c6c6c6',
    'editor.selectionBackground': '#264f78',
    'editor.inactiveSelectionBackground': '#3a3d41',
    'editorIndentGuide.background': '#404040',
    'editorIndentGuide.activeBackground': '#707070',
    'editorWhitespace.foreground': '#404040',
  },
};

/**
 * Autocomplete suggestions for NAVΛ
 */
export const navLambdaCompletionProvider: monaco.languages.CompletionItemProvider = {
  provideCompletionItems: (model, position) => {
    const word = model.getWordUntilPosition(position);
    const range = {
      startLineNumber: position.lineNumber,
      endLineNumber: position.lineNumber,
      startColumn: word.startColumn,
      endColumn: word.endColumn,
    };

    const suggestions: monaco.languages.CompletionItem[] = [];

    // Navigation keywords
    const navigationKeywords = [
      {
        label: 'navigate_to⋋',
        kind: monaco.languages.CompletionItemKind.Keyword,
        insertText: 'navigate_to⋋(${1:target})',
        insertTextRules: monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
        documentation: 'Navigate to a target location using Van Laarhoven calculus',
        range,
      },
      {
        label: 'find_optimal_path⋋',
        kind: monaco.languages.CompletionItemKind.Keyword,
        insertText: 'find_optimal_path⋋(${1:start}, ${2:end})',
        insertTextRules: monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
        documentation: 'Find the optimal path between two points',
        range,
      },
      {
        label: 'energy_landscape⋋',
        kind: monaco.languages.CompletionItemKind.Keyword,
        insertText: 'energy_landscape⋋(${1:space})',
        insertTextRules: monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
        documentation: 'Compute the energy landscape of a navigation space',
        range,
      },
    ];

    // Van Laarhoven symbols
    const vncSymbols = [
      {
        label: '⋋ (Lambda Navigation)',
        kind: monaco.languages.CompletionItemKind.Operator,
        insertText: '⋋',
        documentation: 'Van Laarhoven lambda navigation operator',
        range,
      },
      {
        label: '⊗⋋ (Tensor Product)',
        kind: monaco.languages.CompletionItemKind.Operator,
        insertText: '⊗⋋',
        documentation: 'Navigation tensor product operator',
        range,
      },
      {
        label: '⊕⋋ (Navigation Sum)',
        kind: monaco.languages.CompletionItemKind.Operator,
        insertText: '⊕⋋',
        documentation: 'Navigation sum operator',
        range,
      },
      {
        label: '𝒩ℐ (Master Operator)',
        kind: monaco.languages.CompletionItemKind.Operator,
        insertText: '𝒩ℐ',
        documentation: 'Navigation Institute master operator',
        range,
      },
      {
        label: 'ℰ (Evolution)',
        kind: monaco.languages.CompletionItemKind.Operator,
        insertText: 'ℰ',
        documentation: 'Evolution operator for dynamic systems',
        range,
      },
    ];

    // Built-in functions
    const builtinFunctions = navLambdaConfig.builtinFunctions.map(fn => ({
      label: fn,
      kind: monaco.languages.CompletionItemKind.Function,
      insertText: `${fn}($1)`,
      insertTextRules: monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
      documentation: `Built-in function: ${fn}`,
      range,
    }));

    suggestions.push(...navigationKeywords, ...vncSymbols, ...builtinFunctions);

    return { suggestions };
  },
};

/**
 * Hover provider for NAVΛ
 */
export const navLambdaHoverProvider: monaco.languages.HoverProvider = {
  provideHover: (model, position) => {
    const word = model.getWordAtPosition(position);
    if (!word) return null;

    const hoverInfo: Record<string, string> = {
      '⋋': '**Lambda Navigation Operator**\n\nCore operator for Van Laarhoven navigation calculus.',
      '⊗⋋': '**Tensor Product Operator**\n\nCombines navigation spaces using tensor product.',
      '⊕⋋': '**Navigation Sum Operator**\n\nSums navigation paths or energy fields.',
      '𝒩ℐ': '**Master Operator**\n\nNavigation Institute master transformation operator.',
      'ℰ': '**Evolution Operator**\n\nEvolves systems through navigation space.',
      'navigate_to⋋': '**Navigate To**\n\nNavigates to a target using optimal path finding.',
      'find_optimal_path⋋': '**Find Optimal Path**\n\nComputes the energy-minimizing path between points.',
      'energy_landscape⋋': '**Energy Landscape**\n\nComputes the energy field over a navigation space.',
    };

    const info = hoverInfo[word.word];
    if (info) {
      return {
        contents: [{ value: info }],
      };
    }

    return null;
  },
};

/**
 * Register NAVΛ language with Monaco
 */
export function registerNavLambdaLanguage() {
  // Register language
  monaco.languages.register({ id: 'navlambda' });

  // Set language configuration
  monaco.languages.setLanguageConfiguration('navlambda', {
    comments: {
      lineComment: '//',
      blockComment: ['/*', '*/'],
    },
    brackets: [
      ['{', '}'],
      ['[', ']'],
      ['(', ')'],
    ],
    autoClosingPairs: [
      { open: '{', close: '}' },
      { open: '[', close: ']' },
      { open: '(', close: ')' },
      { open: '"', close: '"' },
      { open: "'", close: "'" },
    ],
    surroundingPairs: [
      { open: '{', close: '}' },
      { open: '[', close: ']' },
      { open: '(', close: ')' },
      { open: '"', close: '"' },
      { open: "'", close: "'" },
    ],
  });

  // Set tokens provider
  monaco.languages.setMonarchTokensProvider('navlambda', navLambdaMonarchLanguage);

  // Define theme
  monaco.editor.defineTheme('vnc-theme', navLambdaTheme);

  // Register completion provider
  monaco.languages.registerCompletionItemProvider('navlambda', {
    provideCompletionItems: (model, position) => {
      const word = model.getWordUntilPosition(position);
      const range = {
        startLineNumber: position.lineNumber,
        endLineNumber: position.lineNumber,
        startColumn: word.startColumn,
        endColumn: word.endColumn,
      };

      const keywords = navLambdaConfig.keywords;
      const operators = navLambdaConfig.operators;
      const types = navLambdaConfig.keywords.filter(k => /^[A-Z]/.test(k));
      const builtins = navLambdaConfig.builtinFunctions;

      const suggestions: monaco.languages.CompletionItem[] = [
        // Keywords
        ...keywords.map((keyword) => ({
          label: keyword,
          kind: monaco.languages.CompletionItemKind.Keyword,
          insertText: keyword,
          range,
          documentation: `NAVΛ keyword: ${keyword}`,
        })),
        // Operators
        ...operators.slice(0, 5).map(op => ({
          label: op,
          kind: monaco.languages.CompletionItemKind.Operator,
          insertText: op,
          insertTextRules: monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
          range,
          documentation: `Operator: ${op}`,
        })),
        // Types
        ...types.map((type) => ({
          label: type,
          kind: monaco.languages.CompletionItemKind.Class,
          insertText: type,
          range,
          documentation: `NAVΛ type: ${type}`,
        })),
        // Built-in functions
        ...builtins.map((builtin) => ({
          label: builtin,
          kind: monaco.languages.CompletionItemKind.Function,
          insertText: `${builtin}($1)`,
          insertTextRules: monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
          range,
          documentation: `Built-in function: ${builtin}`,
        })),
      ];

      return { suggestions };
    },
  });

  // Register hover provider
  monaco.languages.registerHoverProvider('navlambda', navLambdaHoverProvider);
}
