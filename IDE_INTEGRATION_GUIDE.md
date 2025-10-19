# IDE Integration Guide

## Quick Start

This guide shows how to integrate the enhanced IDE components into your NAVΛ Studio application.

## Step 1: Import Components

```tsx
import { NavLambdaMonacoEditor } from './components/Editor/MonacoEditor';
import { FileExplorer } from './components/Editor/FileExplorer';
import { ProjectManager } from './components/Editor/ProjectManager';
import { fileService } from './services/file-service';
```

## Step 2: Create Main IDE Component

```tsx
import React, { useState } from 'react';
import { NavLambdaMonacoEditor } from './components/Editor/MonacoEditor';
import { FileExplorer } from './components/Editor/FileExplorer';
import { ProjectManager } from './components/Editor/ProjectManager';
import { fileService, Project } from './services/file-service';
import './IDE.css';

export const IDE: React.FC = () => {
  const [code, setCode] = useState('// Welcome to NAVΛ Studio\n');
  const [currentFile, setCurrentFile] = useState<string | null>(null);
  const [currentProject, setCurrentProject] = useState<Project | null>(null);
  const [showProjectManager, setShowProjectManager] = useState(false);

  const handleFileOpen = async (filePath: string) => {
    try {
      const content = await fileService.readFile(filePath);
      setCode(content);
      setCurrentFile(filePath);
    } catch (error) {
      console.error('Error opening file:', error);
      alert('Failed to open file');
    }
  };

  const handleFileSave = async () => {
    if (!currentFile) {
      alert('No file open');
      return;
    }

    try {
      await fileService.saveFile(currentFile, code);
      console.log('File saved successfully');
    } catch (error) {
      console.error('Error saving file:', error);
      alert('Failed to save file');
    }
  };

  const handleProjectOpen = (project: Project) => {
    setCurrentProject(project);
    if (project.files.length > 0) {
      handleFileOpen(project.files[0].path);
    }
  };

  // Listen for save events from editor
  React.useEffect(() => {
    const handleSave = () => {
      handleFileSave();
    };

    window.addEventListener('editor-save', handleSave);
    return () => window.removeEventListener('editor-save', handleSave);
  }, [currentFile, code]);

  return (
    <div className="ide-container">
      {/* Top Toolbar */}
      <div className="ide-toolbar">
        <button onClick={() => setShowProjectManager(true)}>
          📁 Project Manager
        </button>
        <button onClick={handleFileSave} disabled={!currentFile}>
          💾 Save
        </button>
        <div className="toolbar-spacer" />
        <span className="project-name">
          {currentProject?.name || 'No Project'}
        </span>
      </div>

      {/* Main Content */}
      <div className="ide-content">
        {/* File Explorer Sidebar */}
        <div className="ide-sidebar">
          <FileExplorer
            rootPath={currentProject?.path || '/'}
            onFileSelect={handleFileOpen}
          />
        </div>

        {/* Editor Area */}
        <div className="ide-editor">
          <NavLambdaMonacoEditor
            initialCode={code}
            onCodeChange={setCode}
            filePath={currentFile || undefined}
          />
        </div>
      </div>

      {/* Project Manager Modal */}
      {showProjectManager && (
        <ProjectManager
          onProjectOpen={handleProjectOpen}
          onClose={() => setShowProjectManager(false)}
        />
      )}
    </div>
  );
};
```

## Step 3: Add IDE Styles

Create `IDE.css`:

```css
.ide-container {
  display: flex;
  flex-direction: column;
  height: 100vh;
  width: 100vw;
  background: #1e1e1e;
  color: #cccccc;
  overflow: hidden;
}

.ide-toolbar {
  display: flex;
  align-items: center;
  gap: 8px;
  padding: 8px 12px;
  background: #2a2a2a;
  border-bottom: 1px solid #3e3e42;
}

.ide-toolbar button {
  padding: 6px 12px;
  background: #3a3a3a;
  border: 1px solid #555;
  border-radius: 4px;
  color: #cccccc;
  cursor: pointer;
  font-size: 13px;
  transition: all 0.2s;
}

.ide-toolbar button:hover:not(:disabled) {
  background: #4a4a4a;
  border-color: #0e639c;
}

.ide-toolbar button:disabled {
  opacity: 0.5;
  cursor: not-allowed;
}

.toolbar-spacer {
  flex: 1;
}

.project-name {
  font-size: 13px;
  color: #858585;
}

.ide-content {
  display: flex;
  flex: 1;
  overflow: hidden;
}

.ide-sidebar {
  width: 250px;
  border-right: 1px solid #3e3e42;
  overflow: hidden;
}

.ide-editor {
  flex: 1;
  overflow: hidden;
}
```

## Step 4: Update Your App

Replace your existing editor component with the new IDE:

```tsx
// App.tsx
import React from 'react';
import { IDE } from './components/IDE';

function App() {
  return <IDE />;
}

export default App;
```

## Step 5: Test the Integration

1. Start your development server:
   ```bash
   npm start
   ```

2. Open http://localhost:3000

3. Test the features:
   - Click "Project Manager" to create a project
   - Use File Explorer to navigate files
   - Edit code in Monaco editor
   - Save with Ctrl+S
   - Use symbol palette for NAVΛ operators

## Advanced Features

### Custom File Operations

```tsx
// Create a new file
const createFile = async (path: string, content: string) => {
  await fileService.createFile(path, content);
};

// Delete a file
const deleteFile = async (path: string) => {
  await fileService.deleteFile(path);
};

// Rename a file
const renameFile = async (oldPath: string, newPath: string) => {
  const content = await fileService.readFile(oldPath);
  await fileService.createFile(newPath, content);
  await fileService.deleteFile(oldPath);
};
```

### Project Templates

```tsx
const createProjectFromTemplate = async (name: string, template: string) => {
  const project = await fileService.createProject(name, `/projects/${name}`);
  
  // Add template files
  const templates = {
    basic: `// Basic NAVΛ Project\nnav main() {\n  // Your code here\n}`,
    advanced: `// Advanced NAVΛ Project\nimport { Navigation } from 'navλ'\n\nnav main() {\n  // Your code here\n}`,
  };
  
  const content = templates[template] || templates.basic;
  await fileService.createFile(`${project.path}/src/main.navλ`, content);
  
  return project;
};
```

### Recent Files Management

```tsx
// Get recent files
const recentFiles = fileService.getRecentFiles();

// Clear recent files
fileService.clearRecentFiles();

// Add file to recent
fileService.addRecentFile('/path/to/file.navλ');
```

## Keyboard Shortcuts

Add global keyboard shortcuts:

```tsx
useEffect(() => {
  const handleKeyDown = (e: KeyboardEvent) => {
    // Ctrl+S / Cmd+S - Save
    if ((e.ctrlKey || e.metaKey) && e.key === 's') {
      e.preventDefault();
      handleFileSave();
    }
    
    // Ctrl+N - New File
    if ((e.ctrlKey || e.metaKey) && e.key === 'n') {
      e.preventDefault();
      // Handle new file
    }
    
    // Ctrl+O - Open File
    if ((e.ctrlKey || e.metaKey) && e.key === 'o') {
      e.preventDefault();
      // Handle open file
    }
    
    // Ctrl+Shift+P - Project Manager
    if ((e.ctrlKey || e.metaKey) && e.shiftKey && e.key === 'P') {
      e.preventDefault();
      setShowProjectManager(true);
    }
  };

  window.addEventListener('keydown', handleKeyDown);
  return () => window.removeEventListener('keydown', handleKeyDown);
}, [currentFile, code]);
```

## Browser Storage

The file service uses browser storage. For production, consider:

1. **IndexedDB** for larger files
2. **File System Access API** for native file access
3. **Backend API** for server-side storage

Example with File System Access API:

```tsx
const openFileNative = async () => {
  try {
    const [fileHandle] = await window.showOpenFilePicker({
      types: [
        {
          description: 'NAVΛ Files',
          accept: {
            'text/plain': ['.navλ', '.navlambda', '.vnc'],
          },
        },
      ],
    });
    
    const file = await fileHandle.getFile();
    const content = await file.text();
    setCode(content);
    setCurrentFile(file.name);
  } catch (error) {
    console.error('Error opening file:', error);
  }
};
```

## Troubleshooting

### Monaco Editor Not Loading
- Ensure `monaco-editor` is installed: `npm install monaco-editor`
- Check webpack configuration for Monaco
- Verify language registration

### File Operations Failing
- Check browser storage permissions
- Verify file paths are valid
- Check console for errors

### Syntax Highlighting Not Working
- Verify file extension is `.navλ` or `.navlambda`
- Check language registration in console
- Reload editor

## Next Steps

1. Add more language features (type checking, linting)
2. Implement debugging support
3. Add terminal integration
4. Create extension system
5. Add collaborative editing

## Resources

- [Monaco Editor Documentation](https://microsoft.github.io/monaco-editor/)
- [File System Access API](https://developer.mozilla.org/en-US/docs/Web/API/File_System_Access_API)
- [IndexedDB API](https://developer.mozilla.org/en-US/docs/Web/API/IndexedDB_API)

---

**Happy Coding with NAVΛ Studio! 🚀**
