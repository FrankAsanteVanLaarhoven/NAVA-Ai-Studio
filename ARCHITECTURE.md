# NAVΛ Studio IDE - Architecture Diagram

```
┌─────────────────────────────────────────────────────────────────────┐
│                         NAVΛ Studio IDE                              │
│                    (React + TypeScript + Monaco)                     │
└─────────────────────────────────────────────────────────────────────┘
                                    │
                    ┌───────────────┴───────────────┐
                    │                               │
        ┌───────────▼──────────┐       ┌───────────▼──────────┐
        │   UI Components      │       │   Services Layer     │
        └──────────────────────┘       └──────────────────────┘
                    │                               │
        ┌───────────┴───────────┐       ┌───────────┴───────────┐
        │                       │       │                       │
┌───────▼────────┐  ┌──────────▼──────┐│  ┌────────────────────▼─────┐
│ ProjectManager │  │  FileExplorer   ││  │   navlambda-language.ts  │
│                │  │                 ││  │                          │
│ - Create       │  │ - Tree View     ││  │ - Tokenizer              │
│ - Open         │  │ - Context Menu  ││  │ - Completion Provider    │
│ - Recent Files │  │ - File Ops      ││  │ - Hover Provider         │
└────────────────┘  └─────────────────┘│  │ - Theme Definition       │
                                        │  └──────────────────────────┘
        ┌───────────────────────────────┘
        │
┌───────▼────────────────────────────┐  ┌──────────────────────────┐
│     MonacoEditor Component         │  │   file-service.ts        │
│                                    │  │                          │
│ - Editor Instance                  │  │ - File Operations        │
│ - Symbol Palette                   │  │ - Directory Operations   │
│ - File Path Display                │  │ - Project Management     │
│ - Keyboard Shortcuts               │  │ - Recent Files Tracking  │
│ - Auto-save                        │  │ - Storage Management     │
└────────────────────────────────────┘  └──────────────────────────┘
                    │
        ┌───────────┴───────────┐
        │                       │
┌───────▼────────┐  ┌──────────▼──────┐
│ Monaco Editor  │  │  Browser APIs   │
│                │  │                 │
│ - Syntax       │  │ - localStorage  │
│ - Completion   │  │ - File System   │
│ - Hover        │  │ - IndexedDB     │
│ - Folding      │  │ - Clipboard     │
└────────────────┘  └─────────────────┘
```

## Component Hierarchy

```
App
├── IDE Container
│   ├── Toolbar
│   │   ├── Project Manager Button
│   │   ├── Save Button
│   │   └── Project Name Display
│   │
│   ├── Content Area
│   │   ├── Sidebar (250px)
│   │   │   └── FileExplorer
│   │   │       ├── Header
│   │   │       ├── Tree View
│   │   │       │   ├── Folder Items
│   │   │       │   └── File Items
│   │   │       └── Context Menu
│   │   │
│   │   └── Editor Area (flex: 1)
│   │       └── MonacoEditor
│   │           ├── Toolbar
│   │           │   ├── File Path
│   │           │   └── Symbol Palette
│   │           └── Editor Instance
│   │
│   └── Modals
│       └── ProjectManager
│           ├── Create Project Form
│           ├── Open Project Button
│           ├── Recent Files List
│           └── Quick Start Cards
```

## Data Flow

```
┌─────────────┐
│    User     │
└──────┬──────┘
       │
       │ (1) Interaction
       ▼
┌─────────────────────┐
│   UI Components     │
│  (React Components) │
└──────┬──────────────┘
       │
       │ (2) Action
       ▼
┌─────────────────────┐
│   Services Layer    │
│  (Business Logic)   │
└──────┬──────────────┘
       │
       │ (3) Operation
       ▼
┌─────────────────────┐
│   Browser APIs      │
│  (Storage, FS, etc) │
└──────┬──────────────┘
       │
       │ (4) Result
       ▼
┌─────────────────────┐
│   State Update      │
│  (React State)      │
└──────┬──────────────┘
       │
       │ (5) Re-render
       ▼
┌─────────────────────┐
│   UI Update         │
│  (Visual Feedback)  │
└─────────────────────┘
```

## File Operation Flow

```
User Action (e.g., "Open File")
       │
       ▼
FileExplorer.onFileSelect(filePath)
       │
       ▼
fileService.readFile(filePath)
       │
       ├─► localStorage.getItem(filePath)
       │
       ▼
Return file content
       │
       ▼
Update editor state
       │
       ▼
MonacoEditor.setValue(content)
       │
       ▼
Display in editor
```

## Project Creation Flow

```
User clicks "Create Project"
       │
       ▼
ProjectManager.handleCreateProject()
       │
       ▼
fileService.createProject(name, path)
       │
       ├─► Create project structure
       ├─► Create default files
       ├─► Save to localStorage
       │
       ▼
Return project object
       │
       ▼
Update UI state
       │
       ├─► Load FileExplorer
       ├─► Open first file
       │
       ▼
Display project in IDE
```

## Language Registration Flow

```
App initialization
       │
       ▼
MonacoEditor component mount
       │
       ▼
registerNavLambdaLanguage()
       │
       ├─► monaco.languages.register()
       ├─► monaco.languages.setMonarchTokensProvider()
       ├─► monaco.languages.registerCompletionItemProvider()
       ├─► monaco.languages.registerHoverProvider()
       ├─► monaco.editor.defineTheme()
       │
       ▼
Language ready for use
       │
       ▼
Create editor with language: 'navlambda'
       │
       ▼
Syntax highlighting active
```

## State Management

```
┌─────────────────────────────────────┐
│         Application State           │
├─────────────────────────────────────┤
│                                     │
│  currentProject: Project | null     │
│  currentFile: string | null         │
│  code: string                       │
│  recentFiles: string[]              │
│  showProjectManager: boolean        │
│                                     │
└─────────────────────────────────────┘
           │
           ├─► ProjectManager
           ├─► FileExplorer
           └─► MonacoEditor
```

## Storage Architecture

```
Browser Storage (localStorage)
├── projects/
│   ├── project-1/
│   │   ├── metadata.json
│   │   └── files/
│   │       ├── src/main.navλ
│   │       └── src/utils.navλ
│   │
│   └── project-2/
│       └── ...
│
├── recent-files
│   └── [array of file paths]
│
└── settings
    └── {user preferences}
```

## Event Flow

```
Keyboard Shortcut (Ctrl+S)
       │
       ▼
MonacoEditor.addCommand()
       │
       ▼
Dispatch 'editor-save' event
       │
       ▼
IDE.handleSave()
       │
       ▼
fileService.saveFile(path, content)
       │
       ▼
localStorage.setItem(path, content)
       │
       ▼
Show success notification
```

## Module Dependencies

```
MonacoEditor.tsx
├── monaco-editor
├── navlambda-language.ts
└── MonacoEditor.css

FileExplorer.tsx
├── file-service.ts
└── FileExplorer.css

ProjectManager.tsx
├── file-service.ts
└── ProjectManager.css

navlambda-language.ts
└── monaco-editor

file-service.ts
└── (no dependencies)
```

## API Surface

### FileService API
```typescript
class FileService {
  // File Operations
  readFile(path: string): Promise<string>
  saveFile(path: string, content: string): Promise<void>
  createFile(path: string, content: string): Promise<void>
  deleteFile(path: string): Promise<void>
  
  // Directory Operations
  readDirectory(path: string): Promise<FileEntry[]>
  createDirectory(path: string): Promise<void>
  deleteDirectory(path: string): Promise<void>
  
  // Project Management
  createProject(name: string, path: string): Promise<Project>
  openProject(path: string): Promise<Project | null>
  saveProject(project: Project): Promise<void>
  closeProject(project: Project): Promise<void>
  
  // Recent Files
  getRecentFiles(): string[]
  addRecentFile(path: string): void
  clearRecentFiles(): void
  
  // Utilities
  getFileExtension(path: string): string
  getFileName(path: string): string
  getDirectoryName(path: string): string
}
```

### Language API
```typescript
function registerNavLambdaLanguage(): void {
  // Registers language with Monaco
  // Sets up tokenizer, completion, hover, theme
}
```

## Performance Considerations

```
┌─────────────────────────────────────┐
│      Performance Optimizations      │
├─────────────────────────────────────┤
│                                     │
│ ✓ Lazy loading of files             │
│ ✓ Virtual scrolling in tree view    │
│ ✓ Debounced auto-save               │
│ ✓ Memoized component renders        │
│ ✓ Efficient state updates           │
│ ✓ Cached language definitions       │
│                                     │
└─────────────────────────────────────┘
```

## Security Model

```
┌─────────────────────────────────────┐
│         Security Layers             │
├─────────────────────────────────────┤
│                                     │
│ 1. Browser Sandbox                  │
│    └─► localStorage isolation       │
│                                     │
│ 2. Input Validation                 │
│    └─► Path sanitization            │
│                                     │
│ 3. Content Security Policy          │
│    └─► XSS prevention               │
│                                     │
│ 4. CORS Protection                  │
│    └─► Same-origin policy           │
│                                     │
└─────────────────────────────────────┘
```

---

**This architecture provides a solid foundation for the NAVΛ Studio IDE! 🏗️**
