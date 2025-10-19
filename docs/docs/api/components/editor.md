---
id: editor
title: Editor API
sidebar_label: Editor
---

# Editor API Reference

The Editor API provides comprehensive access to Monaco Editor functionality with custom NAVΛ language support.

## 📝 Overview

The Editor API wraps Monaco Editor with specialized functionality for Van Laarhoven Navigation Calculus programming, including custom syntax highlighting, completion providers, and VNC-specific features.

## 🔧 Core Interfaces

### EditorOptions
```typescript
interface EditorOptions {
  language?: string;
  theme?: string;
  fontSize?: number;
  fontFamily?: string;
  lineNumbers?: 'on' | 'off' | 'relative';
  minimap?: { enabled: boolean };
  scrollBeyondLastLine?: boolean;
  automaticLayout?: boolean;
  readOnly?: boolean;
  vncMode?: boolean;
}
```

### VNCCompletionItem
```typescript
interface VNCCompletionItem {
  label: string;
  kind: CompletionItemKind;
  detail: string;
  documentation: string;
  insertText: string;
  sortText: string;
  filterText: string;
}
```

### TokenDefinition
```typescript
interface TokenDefinition {
  token: string;
  foreground: string;
  fontStyle?: string;
}
```

## 📋 API Methods

### createEditor(container, options)
Creates a new Monaco Editor instance with NAVΛ support.

```typescript
static createEditor(container: HTMLElement, options: EditorOptions): MonacoEditor
```

**Parameters:**
- `container`: DOM element to host the editor
- `options`: Editor configuration options

**Returns:** MonacoEditor instance

**Example:**
```typescript
const editor = EditorAPI.createEditor(document.getElementById('editor'), {
  language: 'navlambda',
  theme: 'vs-dark',
  fontSize: 14,
  minimap: { enabled: true },
  vncMode: true
});
```

### setVNCMode(editor)
Enables Van Laarhoven Navigation Calculus mode with custom features.

```typescript
static setVNCMode(editor: MonacoEditor): void
```

**Parameters:**
- `editor`: MonacoEditor instance

**Example:**
```typescript
EditorAPI.setVNCMode(editor);
```

### registerCompletionProvider(language, provider)
Registers a custom completion provider for the specified language.

```typescript
static registerCompletionProvider(
  language: string, 
  provider: CompletionProvider
): IDisposable
```

**Parameters:**
- `language`: Language identifier (e.g., 'navlambda')
- `provider`: Completion provider implementation

**Returns:** Disposable object for cleanup

**Example:**
```typescript
const provider = {
  provideCompletionItems: (model, position) => {
    return {
      suggestions: [
        {
          label: '⋋',
          kind: monaco.languages.CompletionItemKind.Function,
          detail: 'Lambda Navigation Calculus',
          documentation: 'Navigation calculus operator',
          insertText: '⋋'
        }
      ]
    };
  }
};

EditorAPI.registerCompletionProvider('navlambda', provider);
```

### registerSyntaxHighlighter(language, tokens)
Registers custom syntax highlighting rules.

```typescript
static registerSyntaxHighlighter(
  language: string, 
  tokens: TokenDefinition[]
): void
```

**Parameters:**
- `language`: Language identifier
- `tokens`: Array of token definitions

**Example:**
```typescript
const vncTokens = [
  { token: 'keyword.navlambda', foreground: '#569CD6' },
  { token: 'operator.lambda', foreground: '#D4D4D4', fontStyle: 'bold' },
  { token: 'function.navigation', foreground: '#4EC9B0' },
  { token: 'constant.energy', foreground: '#B5CEA8' }
];

EditorAPI.registerSyntaxHighlighter('navlambda', vncTokens);
```

### getEditorContent(editor)
Retrieves the current content of the editor.

```typescript
static getEditorContent(editor: MonacoEditor): string
```

**Parameters:**
- `editor`: MonacoEditor instance

**Returns:** Current editor content as string

**Example:**
```typescript
const content = EditorAPI.getEditorContent(editor);
console.log(content);
```

### setEditorContent(editor, content)
Sets the content of the editor.

```typescript
static setEditorContent(editor: MonacoEditor, content: string): void
```

**Parameters:**
- `editor`: MonacoEditor instance
- `content`: New content to set

**Example:**
```typescript
EditorAPI.setEditorContent(editor, '⋋(x, y) = navigate(x, y, energy)');
```

### addAction(editor, action)
Adds a custom action to the editor.

```typescript
static addAction(
  editor: MonacoEditor, 
  action: IActionDescriptor
): void
```

**Parameters:**
- `editor`: MonacoEditor instance
- `action`: Action descriptor object

**Example:**
```typescript
EditorAPI.addAction(editor, {
  id: 'insert-lambda',
  label: 'Insert ⋋ Symbol',
  keybindings: [monaco.KeyMod.CtrlCmd | monaco.KeyCode.KeyL],
  run: (editor) => {
    editor.trigger('keyboard', 'type', { text: '⋋' });
  }
});
```

### createModel(content, language)
Creates a new text model for the editor.

```typescript
static createModel(content: string, language: string): ITextModel
```

**Parameters:**
- `content`: Initial content
- `language`: Language identifier

**Returns:** Text model instance

**Example:**
```typescript
const model = EditorAPI.createModel('⋋(x) = x + 1', 'navlambda');
editor.setModel(model);
```

### disposeEditor(editor)
Disposes of an editor instance and cleans up resources.

```typescript
static disposeEditor(editor: MonacoEditor): void
```

**Parameters:**
- `editor`: MonacoEditor instance to dispose

**Example:**
```typescript
EditorAPI.disposeEditor(editor);
```

## 🔹 VNC-Specific Features

### Lambda Symbol Support
Automatic support for the ⋋ symbol with keyboard shortcuts and visual rendering.

### Navigation Function Recognition
Special highlighting and completion for navigation functions like `navigate()`, `energy()`, and `path()`.

### Energy Landscape Syntax
Custom syntax highlighting for energy function definitions and landscape descriptions.

### Multi-Target Compilation Hints
Inline hints and suggestions for compilation targets (C++, Python, WebAssembly, etc.).

## 📋 Code Examples

### Complete Editor Setup
```typescript
import { EditorAPI } from '@navlambda/studio';

// Create editor with VNC support
const editor = EditorAPI.createEditor(document.getElementById('editor'), {
  language: 'navlambda',
  theme: 'vs-dark',
  fontSize: 14,
  minimap: { enabled: true },
  automaticLayout: true,
  vncMode: true
});

// Register VNC completion provider
EditorAPI.registerCompletionProvider('navlambda', {
  provideCompletionItems: (model, position) => {
    const suggestions = [
      {
        label: 'navigate',
        kind: monaco.languages.CompletionItemKind.Function,
        detail: 'Navigation function',
        documentation: 'Creates a navigation path between points',
        insertText: 'navigate(${1:start}, ${2:end}, ${3:energy})'
      },
      {
        label: '⋋',
        kind: monaco.languages.CompletionItemKind.Operator,
        detail: 'Lambda navigation calculus',
        documentation: 'Navigation calculus operator',
        insertText: '⋋'
      }
    ];
    return { suggestions };
  }
});

// Add custom action for ⋋ symbol
EditorAPI.addAction(editor, {
  id: 'insert-lambda-nav',
  label: 'Insert ⋋ Navigation Symbol',
  keybindings: [monaco.KeyMod.CtrlCmd | monaco.KeyCode.KeyN],
  run: (editor) => {
    editor.trigger('keyboard', 'type', { text: '⋋' });
  }
});
```

### Content Management
```typescript
// Set initial content
const vncCode = `
⋋(x, y) = navigate(x, y, energy_function)
  where energy_function = λ(p) → p.distance + p.obstacle_penalty

path = ⋋(start_point, end_point)
optimized_path = optimize(path, gradient_descent)
`;

EditorAPI.setEditorContent(editor, vncCode);

// Get content on demand
const currentContent = EditorAPI.getEditorContent(editor);
console.log('Current VNC code:', currentContent);
```

## 🔧 Error Handling

### Common Errors
```typescript
try {
  const editor = EditorAPI.createEditor(container, options);
} catch (error) {
  if (error instanceof EditorAPI.InvalidContainerError) {
    console.error('Invalid container element');
  } else if (error instanceof EditorAPI.UnsupportedLanguageError) {
    console.error('Language not supported');
  } else {
    console.error('Unknown error:', error);
  }
}
```

### Validation
```typescript
// Validate container
if (!container || !container.tagName) {
  throw new EditorAPI.InvalidContainerError('Container must be a valid DOM element');
}

// Validate options
if (options.language && !isSupportedLanguage(options.language)) {
  throw new EditorAPI.UnsupportedLanguageError(`Language ${options.language} not supported`);
}
```

## 📈 Performance Considerations

### Memory Management
Always dispose of editor instances when no longer needed:
```typescript
// Cleanup when component unmounts
useEffect(() => {
  return () => {
    EditorAPI.disposeEditor(editor);
  };
}, []);
```

### Large Files
For files larger than 10MB, consider:
- Disabling minimap
- Reducing font size
- Using virtual scrolling
- Implementing pagination

### Syntax Highlighting
Custom syntax highlighting can impact performance:
- Limit token complexity
- Use efficient regex patterns
- Consider debouncing updates
- Profile rendering performance

## 🔀 Related APIs

- [Parser API](../utils/parser) - VNC syntax parsing
- [Validator API](../utils/validator) - Code validation
- [Formatter API](../utils/formatter) - Code formatting
- [LSP API](../components/lsp) - Language Server Protocol

---

*For more examples and advanced usage, see the [Editor Examples](./examples) page.*