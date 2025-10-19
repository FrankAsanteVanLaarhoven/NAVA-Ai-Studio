# NAVΛ Studio IDE - Enhanced Features Documentation

## Overview

This document describes the enhanced Monaco editor integration with comprehensive file system operations, project management, and custom syntax highlighting for Van Laarhoven Navigation Calculus.

## Features Implemented

### 1. Van Laarhoven Navigation Calculus Language Support

**File:** `src/services/navlambda-language.ts`

The IDE now includes full language support for NAVΛ with:

#### Syntax Highlighting
- **Keywords:** `nav`, `tensor`, `sum`, `union`, `intersection`, `gradient`, `master`, `evolution`, `compose`, `parallel`, `sequential`, `feedback`, `loop`, `if`, `then`, `else`, `let`, `in`, `type`, `module`, `import`, `export`
- **Operators:** `⋋`, `⊗⋋`, `⊕⋋`, `∪⋋`, `∩⋋`, `∇⋋`, `𝒩ℐ`, `ℰ`, `→`, `⇒`, `∘`, `⊗`, `⊕`
- **Types:** `Navigation`, `Tensor`, `Sum`, `Product`, `State`, `Action`, `Observation`
- **Built-in Functions:** `map`, `filter`, `reduce`, `fold`, `scan`, `zip`, `unzip`

#### Code Completion
- Keyword suggestions
- Operator suggestions with descriptions
- Type suggestions
- Function suggestions with parameter hints

#### Language Features
- Bracket matching and auto-closing
- Comment support (line and block)
- Indentation rules
- Folding regions
- Hover information

#### Custom Theme
- Dark theme optimized for NAVΛ syntax
- Color-coded operators and keywords
- High contrast for readability

### 2. File System Operations

**File:** `src/services/file-service.ts`

Comprehensive file system service with:

#### File Operations
- **Open File:** Read file contents from disk
- **Save File:** Write file contents to disk
- **Create File:** Create new files with default content
- **Delete File:** Remove files from disk
- **Rename File:** Rename files and update references

#### Directory Operations
- **Read Directory:** List all files and subdirectories
- **Create Directory:** Create new folders
- **Delete Directory:** Remove folders and contents
- **Watch Directory:** Monitor for file changes

#### Project Management
- **Create Project:** Initialize new NAVΛ projects with structure
- **Open Project:** Load existing projects
- **Save Project:** Persist project state
- **Close Project:** Clean up project resources
- **Recent Files:** Track and display recently opened files

#### Utility Functions
- File extension detection
- Path manipulation
- File name extraction
- Directory name extraction

### 3. File Explorer Component

**File:** `src/components/Editor/FileExplorer.tsx`

VS Code-style file explorer with:

#### Features
- **Tree View:** Hierarchical file/folder display
- **Expand/Collapse:** Toggle folder visibility
- **File Selection:** Click to open files
- **Context Menu:** Right-click for operations
- **Drag & Drop:** (Future enhancement)

#### Context Menu Actions
- New File
- New Folder
- Rename
- Delete
- Copy Path
- Reveal in Explorer

#### Visual Design
- Dark theme matching VS Code
- Folder/file icons
- Hover effects
- Selection highlighting
- Smooth animations

### 4. Project Manager Component

**File:** `src/components/Editor/ProjectManager.tsx`

Project management interface with:

#### Features
- **Create New Project:** Initialize projects with templates
- **Open Existing Project:** Browse and open folders
- **Recent Files:** Quick access to recent work
- **Quick Start:** Templates and examples

#### Project Structure
```
my-project/
├── src/
│   ├── main.navλ
│   └── utils.navλ
├── tests/
│   └── test_main.navλ
├── docs/
│   └── README.md
└── project.json
```

### 5. Enhanced Monaco Editor

**File:** `src/components/Editor/MonacoEditor.tsx`

Upgraded editor with:

#### Features
- **File Path Display:** Shows current file name
- **Symbol Palette:** Quick access to NAVΛ symbols
- **Keyboard Shortcuts:**
  - `Alt+L`: Insert ⋋
  - `Alt+Shift+T`: Insert ⊗⋋
  - `Alt+Shift+S`: Insert ⊕⋋
  - `Alt+Shift+M`: Insert 𝒩ℐ
  - `Alt+Shift+E`: Insert ℰ
  - `Ctrl+S` / `Cmd+S`: Save file
- **Auto-save:** Optional automatic saving
- **Minimap:** Code overview
- **Bracket Colorization:** Visual bracket matching
- **Code Folding:** Collapse/expand code blocks

## Usage Guide

### Starting a New Project

1. Open the IDE
2. Click "Project Manager" or press `Ctrl+Shift+P`
3. Enter project name
4. Click "Create Project"
5. Start coding!

### Opening Existing Files

1. Click "Open Folder" in Project Manager
2. Navigate to your project directory
3. Select the folder
4. Files will appear in the File Explorer

### Working with Files

#### Creating Files
1. Right-click in File Explorer
2. Select "New File"
3. Enter file name (e.g., `navigation.navλ`)
4. Press Enter

#### Editing Files
1. Click file in File Explorer
2. Edit in Monaco editor
3. Save with `Ctrl+S` or auto-save

#### Deleting Files
1. Right-click file in File Explorer
2. Select "Delete"
3. Confirm deletion

### Using NAVΛ Symbols

#### Via Keyboard
- Press `Alt+L` for ⋋
- Press `Alt+Shift+T` for ⊗⋋
- Press `Alt+Shift+S` for ⊕⋋

#### Via Symbol Palette
- Click symbol buttons in toolbar
- Symbols insert at cursor position

### Code Completion

1. Start typing a keyword (e.g., `nav`)
2. Press `Ctrl+Space` for suggestions
3. Select from dropdown
4. Press `Enter` to insert

## Example NAVΛ Code

```navlambda
// Define a navigation operator
nav pathfinder(start: State, goal: State) -> Navigation {
  let distance = ∇⋋(start, goal)
  let path = 𝒩ℐ(distance)
  
  // Compose navigation steps
  compose(
    tensor(start ⊗⋋ path),
    evolution(ℰ(path)),
    feedback(goal)
  )
}

// Use the operator
let result = pathfinder(
  State { x: 0, y: 0 },
  State { x: 10, y: 10 }
)
```

## File Types

### `.navλ` Files
Primary NAVΛ source files with full language support.

### `.navlambda` Files
Alternative extension for NAVΛ files.

### `.vnc` Files
Van Laarhoven Navigation Calculus files.

## Configuration

### Editor Settings
Located in Monaco editor initialization:
- Font: JetBrains Mono, Consolas
- Font Size: 14px
- Tab Size: 2 spaces
- Word Wrap: On
- Minimap: Enabled

### Theme
Custom "vnc-theme" with:
- Background: #1e1e1e
- Foreground: #d4d4d4
- Keywords: #569cd6
- Operators: #00ff00
- Strings: #ce9178
- Comments: #6a9955

## Keyboard Shortcuts

| Shortcut | Action |
|----------|--------|
| `Ctrl+S` / `Cmd+S` | Save file |
| `Ctrl+N` | New file |
| `Ctrl+O` | Open file |
| `Ctrl+Shift+P` | Project Manager |
| `Alt+L` | Insert ⋋ |
| `Alt+Shift+T` | Insert ⊗⋋ |
| `Alt+Shift+S` | Insert ⊕⋋ |
| `Alt+Shift+M` | Insert 𝒩ℐ |
| `Alt+Shift+E` | Insert ℰ |
| `Ctrl+Space` | Code completion |
| `Ctrl+/` | Toggle comment |
| `Ctrl+F` | Find |
| `Ctrl+H` | Replace |

## Architecture

### Component Hierarchy
```
App
├── ProjectManager
│   ├── Create Project Form
│   ├── Open Project Dialog
│   └── Recent Files List
├── FileExplorer
│   ├── Tree View
│   ├── Context Menu
│   └── File Operations
└── MonacoEditor
    ├── Editor Instance
    ├── Symbol Palette
    └── File Info Display
```

### Service Layer
```
FileService
├── File Operations
├── Directory Operations
├── Project Management
└── Recent Files Tracking
```

### Language Support
```
NavLambdaLanguage
├── Tokenizer
├── Completion Provider
├── Hover Provider
├── Theme Definition
└── Configuration
```

## Future Enhancements

### Planned Features
- [ ] Git integration
- [ ] Multi-file search
- [ ] Refactoring tools
- [ ] Debugging support
- [ ] Terminal integration
- [ ] Extension system
- [ ] Collaborative editing
- [ ] Cloud sync
- [ ] Mobile support

### Language Features
- [ ] Type checking
- [ ] Error diagnostics
- [ ] Code formatting
- [ ] Linting
- [ ] Documentation generation
- [ ] Test runner
- [ ] Performance profiler

## Troubleshooting

### Editor Not Loading
1. Check browser console for errors
2. Verify Monaco editor is installed
3. Clear browser cache
4. Restart development server

### Files Not Saving
1. Check file permissions
2. Verify file path is valid
3. Check browser storage quota
4. Try saving to different location

### Syntax Highlighting Not Working
1. Verify file extension is `.navλ` or `.navlambda`
2. Check language registration in console
3. Reload editor
4. Clear Monaco cache

### Context Menu Not Appearing
1. Check if right-click is enabled
2. Verify context menu component is mounted
3. Check z-index conflicts
4. Try clicking on different elements

## Support

For issues, questions, or contributions:
- GitHub: [Your Repository]
- Documentation: [Your Docs Site]
- Discord: [Your Community]

## License

MIT OR Apache-2.0

---

**Built with ❤️ for the NAVΛ community**
