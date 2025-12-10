import { useEffect, useState, useCallback } from 'react';

import './App.css';
import { ResizablePanel } from './components/Common/ResizablePanel';
import { SplitPanel } from './components/Common/SplitPanel';
import { MenuBar } from './components/Common/MenuBar';
import { TraditionalMenuBar } from './components/Common/TraditionalMenuBar';
import { FileExplorer } from './components/Editor/FileExplorer';
import { NavLambdaMonacoEditor } from './components/Editor/MonacoEditor';
import { NavigationVisualizer } from './components/Visualizer/NavigationVisualizer';
import { ROSLearningCenter } from './components/ROSLearning/ROSLearningCenter';
import { WidgetManager } from './components/Widgets';
import { ActivityBar, type ActivityType } from './components/ActivityBar/ActivityBar';
import { SourceControlPanel } from './components/ActivityPanels/SourceControlPanel';
import { DebugPanel } from './components/ActivityPanels/DebugPanel';
import { ExtensionsPanel } from './components/ActivityPanels/ExtensionsPanel';
import { ChatHistoryPanel } from './components/ActivityPanels/ChatHistoryPanel';
import { MCPToolkitPanel } from './components/ActivityPanels/MCPToolkitPanel';
import { DiscoveryPanel } from './components/ActivityPanels/DiscoveryPanel';
import { SimulationPanel } from './components/Simulation/SimulationPanel';
import { NotebookPanel } from './components/Notebook/NotebookPanel';
import { JupyterNotebookPanel } from './components/Notebook/JupyterNotebookPanel';
import { AIPanePanel } from './components/AI/AIPanePanel';
import { CollaborationPanel } from './components/Collaboration/CollaborationPanel';
import { CommandPalette } from './components/CommandPalette/CommandPalette';
import { SettingsPanel } from './components/Settings/SettingsPanel';
import { Outline } from './components/Sidebar/Outline';
import { Timeline } from './components/Sidebar/Timeline';
import { OSDesktop } from './components/Workspace/OSDesktop';
import { AppCentre } from './components/AppCentre/AppCentre';
import { ProjectManager } from './components/Editor/ProjectManager';
import { PWAInstallPrompt } from './components/PWA/PWAInstallPrompt';
import { SearchPanel } from './components/Search/SearchPanel';
import { FileReferenceViewer } from './components/FileReference/FileReferenceViewer';
import { Terminal } from './components/Terminal/Terminal';
import { EditorTabs } from './components/Editor/EditorTabs';
import { timelineService } from './services/timeline-service';
import { initializeBackends } from './services/llm-hub/backend-initializer';
import { fileService } from './services/file-service';
import { debugService, type DebugConfiguration } from './services/debug-service';
import { taskService, type Task } from './services/task-service';
import { workspaceService, type WorkspaceFolder } from './services/workspace-service';
import type { NavigationPath } from './types';

// Runtime check to ensure fileService is available
if (!fileService) {
  console.error('[App] Critical: fileService is not available!');
}
import { useNavLambdaLsp } from './hooks/useNavLambdaLsp';
import { useMultiTargetCompilation } from './hooks/useMultiTargetCompilation';
import { pwaService } from './services/pwa-service';
import { errorMonitorService } from './services/error-monitor-service';
import { voiceService } from './services/voice-service';
import { KernelProvider } from './kernel/kernel';
// Register apps on import
import './apps';
// Import Univarm apps
import { App as UnivarmStarterApp } from './apps/univarm-starter';
import { App as UnivarmAdvancedApp } from './apps/univarm-advanced';

const initialCode = `// Welcome to NAVΛ Studio IDE
// Van Laarhoven Navigation Calculus Programming

// Define a navigation path with VNC optimization
let start = [0.0, 0.0, 0.0]
let goal = [5.0, 5.0, 5.0]

// Use the ⋋ operator for navigation (Alt+L)
let path = navigate_to⋋(start, goal)

// Find optimal path through energy landscape
let optimal = find_optimal_path⋋(start, goal, energy_landscape⋋)

// Master navigation operator
let result = 𝒩ℐ(optimal)

// Visualize the navigation path
visualize⋋(result)
`;

function App() {
  const [code, setCode] = useState(initialCode);
  const [currentFile, setCurrentFile] = useState<string>('');
  const [openTabs, setOpenTabs] = useState<Array<{ path: string; name: string; content: string; isModified: boolean }>>([]);
  const [showTerminal, setShowTerminal] = useState(false);
  const [showFileReference, setShowFileReference] = useState(false);
  const [referenceSymbol, setReferenceSymbol] = useState<string>('');
  const [hasActiveDebugSession, setHasActiveDebugSession] = useState(false);
  const [runningTasks, setRunningTasks] = useState<Array<{ id: string; label: string; status: string }>>([]);
  const [recentWorkspaces, setRecentWorkspaces] = useState<Array<{ id: string; name: string }>>([]);
  const [autoSaveEnabled, setAutoSaveEnabled] = useState(false);
  
  // File operations handlers
  const handleNewFile = useCallback(() => {
      // Create a new file
      setCode('');
    setCurrentFile('');
    // Add to tabs
    const newTab = { path: '', name: 'Untitled', content: '', isModified: false };
    setOpenTabs(prev => [...prev, newTab]);
      // Focus on editor
      setTimeout(() => {
        const editor = document.querySelector('.monaco-editor');
        if (editor) {
          (editor as HTMLElement).click();
        }
      }, 100);
  }, []);

  const handleOpenFile = useCallback(() => {
      // Trigger file explorer by setting activity
        setActiveActivity('explorer');
  }, []);
  
  // Handle welcome screen actions
  useEffect(() => {

    const handleCloneRepo = () => {
      // Show clone repository dialog or trigger git clone
      const repoUrl = prompt('Enter Git repository URL:');
      if (repoUrl) {
        // In production, this would trigger actual git clone
        alert(`Cloning repository: ${repoUrl}\n\nThis feature will be implemented with Git integration.`);
      }
    };

    const handleOpenSettings = () => {
      setShowSettings(true);
    };

    // Handle code insertion from AI panel
    const handleInsertCode = (event: CustomEvent) => {
      const { code, language, explanation, files } = event.detail;
      
      if (files && files.length > 0) {
        // Multiple files - show file picker or create all
        const fileList = files.map((f: any) => f.path).join('\n');
        if (confirm(`AI generated ${files.length} file(s):\n${fileList}\n\nInsert first file into editor?`)) {
          setCode(files[0].content);
        }
      } else if (code) {
        // Single code block - insert into editor
        setCode(code);
        // Show explanation if available
        if (explanation) {
          console.log('[AI Code Generator]', explanation);
        }
      }
    };

    // Handle keyboard shortcuts display
    const handleShowShortcuts = () => {
      const shortcuts = [
        '⌘K ⌘S - Keyboard Shortcuts',
        '⌘, - Settings',
        '⌘N - New File',
        '⌘O - Open File',
        '⌘S - Save',
        '⌘⇧P - Command Palette',
        '⌘/ - Toggle Comment',
        '⌘F - Find',
        '⌘⇧F - Find in Files',
      ];
      alert('Keyboard Shortcuts:\n\n' + shortcuts.join('\n'));
    };

    // Handle terminal/CLI opening
    const handleOpenTerminal = () => {
      // Open terminal panel or window
      setShowPanel(true);
      // Could also open a dedicated terminal component
      const event = new CustomEvent('nava:focus-terminal');
      window.dispatchEvent(event);
    };

    const handleNewFileEvent = () => handleNewFile();
    const handleOpenFileEvent = () => handleOpenFile();
    window.addEventListener('nava:new-file', handleNewFileEvent);
    window.addEventListener('nava:open-file', handleOpenFileEvent);
    window.addEventListener('nava:clone-repo', handleCloneRepo);
    window.addEventListener('nava:open-settings', handleOpenSettings);
    window.addEventListener('nava:insert-code', handleInsertCode as EventListener);
    window.addEventListener('nava:show-shortcuts', handleShowShortcuts);
    window.addEventListener('nava:open-terminal', handleOpenTerminal);

    return () => {
      window.removeEventListener('nava:new-file', handleNewFileEvent);
      window.removeEventListener('nava:open-file', handleOpenFileEvent);
      window.removeEventListener('nava:clone-repo', handleCloneRepo);
      window.removeEventListener('nava:open-settings', handleOpenSettings);
      window.removeEventListener('nava:insert-code', handleInsertCode as EventListener);
      window.removeEventListener('nava:show-shortcuts', handleShowShortcuts);
      window.removeEventListener('nava:open-terminal', handleOpenTerminal);
    };
  }, []);
  const [cursorLine, setCursorLine] = useState(1);
  const [cursorColumn, setCursorColumn] = useState(1);
  const [status, setStatus] = useState<'ready' | 'compiling' | 'running' | 'error'>('ready');
  const [showVisualizer, setShowVisualizer] = useState(false);
  const [showCollaboration, setShowCollaboration] = useState(false);
  const [showSidebar, setShowSidebar] = useState(true);
  const [showPanel, setShowPanel] = useState(true); // Show notebook by default
  const [showAIPane, setShowAIPane] = useState(false);
  const [notebookFilePath, setNotebookFilePath] = useState<string | undefined>(undefined);
  const [showProjectManager, setShowProjectManager] = useState(false);
  // Initialize activity state - prioritize URL over localStorage
  const [activeActivity, setActiveActivity] = useState<ActivityType | null>(() => {
    // If we're on workspace.html, force workspace activity
    if (window.location.pathname === '/workspace.html' || window.location.pathname.includes('workspace.html')) {
      console.log(`[APP] On workspace.html, forcing workspace activity`);
      return 'workspace';
    }
    
    // Check URL parameters FIRST (for navigation)
    const urlParams = new URLSearchParams(window.location.search);
    const activity = urlParams.get('activity');
    if (activity && ['workspace', 'ros-learning', 'simulation', 'explorer', 'search', 'source-control', 'debug', 'extensions', 'chat-history', 'mcp-toolkit', 'univarm-starter', 'univarm-advanced'].includes(activity)) {
      console.log(`[APP] Initializing from URL: ${activity}`);
      return activity as ActivityType;
    }
    // Fallback to localStorage if no URL param
    const savedActivity = (() => {
      try {
        return localStorage.getItem('navlambda-active-activity');
      } catch {
        return null;
      }
    })();
    if (savedActivity && ['workspace', 'explorer', 'search', 'source-control', 'debug', 'simulation', 'remote-explorer', 'extensions', 'containers', 'stripe', 'github', 'chat-history', 'ros-learning', 'mcp-toolkit', 'univarm-starter', 'univarm-advanced'].includes(savedActivity)) {
      console.log(`[APP] Initializing from localStorage: ${savedActivity}`);
      return savedActivity as ActivityType;
    }
    // ALWAYS default to workspace (OS Desktop) - this is the main entry point
    console.log(`[APP] Initializing with default: workspace (OS Desktop is the main entry point)`);
    return 'workspace';
  });
  const [showCommandPalette, setShowCommandPalette] = useState(false);
  const [showSettings, setShowSettings] = useState(false);
  const [showWidgetManager, setShowWidgetManager] = useState(false);
  // For workspace.html, initialize immediately to prevent flash
  // workspace.html is the MAIN ENTRY POINT - always initialize immediately
  const [isInitialized, setIsInitialized] = useState(() => {
    // If on workspace.html or root, initialize immediately (workspace is main entry)
    if (window.location.pathname === '/workspace.html' || 
        window.location.pathname.includes('workspace.html') ||
        window.location.pathname === '/' ||
        window.location.pathname === '/index.html') {
      return true;
    }
    return false;
  });
  // Removed unused state variables: currentFilePath, fileStatus, isLoading, isCompiling

  const { parseCode } = useNavLambdaLsp(code);
  const { compileToTarget } = useMultiTargetCompilation();

  const handleCursorPositionChange = (position: { lineNumber: number; column: number }) => {
    setCursorLine(position.lineNumber);
    setCursorColumn(position.column);
  };

  // Initialize backends and PWA on app startup
  useEffect(() => {
    // Load current project from file service
    if (fileService) {
      try {
        const currentProject = fileService.getCurrentProject();
        if (currentProject) {
          console.log('[App] Restored project from storage:', currentProject.name);
          // Trigger explorer refresh
          const refreshEvent = new CustomEvent('nava:refresh-explorer');
          window.dispatchEvent(refreshEvent);
        }
      } catch (error) {
        console.error('[App] Error loading current project:', error);
      }
    } else {
      console.warn('[App] fileService is not available');
    }

    // Load recent workspaces
    const recent = workspaceService.getRecentWorkspaces();
    setRecentWorkspaces(recent.map(r => ({ id: r.workspace.id, name: r.workspace.name })));

    // Listen for debug session changes
    const handleDebugSessionChanged = (event: CustomEvent) => {
      const session = event.detail.session;
      setHasActiveDebugSession(session.status === 'running' || session.status === 'paused');
    };

    // Listen for running tasks changes
    const handleRunningTasksChanged = (event: CustomEvent) => {
      const tasks = event.detail.runningTasks;
      setRunningTasks(tasks.map((t: any) => ({ id: t.id, label: t.task.label, status: t.status })));
    };

    window.addEventListener('nava:debug-session-changed', handleDebugSessionChanged as EventListener);
    window.addEventListener('nava:running-tasks-changed', handleRunningTasksChanged as EventListener);
    
    // Initialize LLM backends (all models available)
    console.log('[App] Initializing LLM backends...');
    try {
      initializeBackends();
      console.log('[App] ✅ LLM backends initialized successfully');
    } catch (error) {
      console.error('[App] ❌ Failed to initialize LLM backends:', error);
    }

    // Start automatic backend health checking (silent, persistent)
    console.log('[App] Starting automatic backend health checks...');
    import('./services/backend-auto-start').then(({ backendAutoStartService }) => {
      backendAutoStartService.startAutoCheck((isRunning) => {
        if (isRunning) {
          console.log('[App] ✅ NAVΛ RS1 backend is running');
        } else {
          console.log('[App] ⚠️ NAVΛ RS1 backend is not running (auto-checking in background)');
        }
      });
    });
    
    // For workspace, app-centre, and simulation, initialize immediately (no delay)
    if (activeActivity === 'workspace' || activeActivity === 'app-centre' || activeActivity === 'simulation') {
      setIsInitialized(true);
    }
    
    // Initialize PWA in background (non-blocking)
    // Service worker is disabled in dev mode to prevent fetch errors
    pwaService.initialize().catch((error) => {
      // Ignore PWA errors, they don't affect desktop UI
      console.warn('PWA initialization warning (non-critical):', error);
    });
    
    // For other activities (explorer, etc.), initialize after a brief delay
    let timer: NodeJS.Timeout | null = null;
    if (activeActivity !== 'workspace' && activeActivity !== 'app-centre' && activeActivity !== 'simulation') {
      // Small delay to prevent flash on other pages
      timer = setTimeout(() => {
        setIsInitialized(true);
      }, 50);
    }

    return () => {
      window.removeEventListener('nava:debug-session-changed', handleDebugSessionChanged as EventListener);
      window.removeEventListener('nava:running-tasks-changed', handleRunningTasksChanged as EventListener);
      if (timer) {
        clearTimeout(timer);
      }
    };
  }, [activeActivity]);

  // Listen for URL changes to update activity
  useEffect(() => {
    const handleUrlChange = () => {
      const urlParams = new URLSearchParams(window.location.search);
      const activity = urlParams.get('activity');
      console.log(`[APP] URL changed, activity from URL: ${activity}`);
      if (activity && ['workspace', 'ros-learning', 'simulation', 'explorer', 'search', 'source-control', 'debug', 'extensions', 'chat-history', 'mcp-toolkit', 'univarm-starter', 'univarm-advanced', 'app-centre'].includes(activity)) {
        console.log(`[APP] Setting activeActivity to: ${activity}`);
        setActiveActivity(activity as ActivityType);
        // Update localStorage to match URL
        localStorage.setItem('navlambda-active-activity', activity);
      } else if (!activity) {
        // No activity in URL, default to workspace
        console.log(`[APP] No activity in URL, defaulting to workspace`);
        setActiveActivity('workspace');
        localStorage.setItem('navlambda-active-activity', 'workspace');
      }
    };

    // Listen for popstate (back/forward) and custom activitychange event
    window.addEventListener('popstate', handleUrlChange);
    window.addEventListener('activitychange', handleUrlChange as EventListener);
    
    // Also check URL on mount
    handleUrlChange();

    return () => {
      window.removeEventListener('popstate', handleUrlChange);
      window.removeEventListener('activitychange', handleUrlChange as EventListener);
    };
  }, []);

  // Persist activity changes to localStorage
  useEffect(() => {
    if (isInitialized && activeActivity) {
      localStorage.setItem('navlambda-active-activity', activeActivity);
    }
  }, [activeActivity, isInitialized]);

  // Keyboard shortcuts for menu bar actions
  useEffect(() => {
    const handleKeyDown = (e: KeyboardEvent) => {
      const isMac = navigator.platform.toUpperCase().indexOf('MAC') >= 0;
      const cmdOrCtrl = isMac ? e.metaKey : e.ctrlKey;

      // Command Palette (Ctrl+Shift+P or Cmd+Shift+P)
      if (cmdOrCtrl && e.shiftKey && e.key === 'P') {
        e.preventDefault();
        setShowCommandPalette(true);
      }
      // Settings (Ctrl+, or Cmd+,)
      if (cmdOrCtrl && e.key === ',') {
        e.preventDefault();
        setShowSettings(true);
      }
      // Widget Manager (Ctrl+Shift+W or Cmd+Shift+W)
      if (cmdOrCtrl && e.shiftKey && e.key === 'W') {
        e.preventDefault();
        setShowWidgetManager(true);
      }
      // New File (Cmd+N or Ctrl+N)
      if (cmdOrCtrl && e.key === 'n' && !e.shiftKey) {
        e.preventDefault();
        handleNewFile();
      }
      // Open File (Cmd+O or Ctrl+O)
      if (cmdOrCtrl && e.key === 'o' && !e.shiftKey) {
        e.preventDefault();
        setActiveActivity('explorer');
      }
      // Save (Cmd+S or Ctrl+S) - handled by Monaco Editor
      // Save As (Cmd+Shift+S or Ctrl+Shift+S)
      if (cmdOrCtrl && e.shiftKey && e.key === 'S') {
        e.preventDefault();
        const fileName = prompt('Enter file name:');
        if (fileName && currentFile) {
          const path = `/projects/${fileName}`;
          fileService.saveFile(path, code);
          setCurrentFile(path);
        }
      }
      // Close Editor (Cmd+W or Ctrl+W)
      if (cmdOrCtrl && e.key === 'w' && !e.shiftKey) {
        e.preventDefault();
        handleCloseEditor();
      }
      // Find (Cmd+F or Ctrl+F) - handled by Monaco Editor
      // Find in Files (Cmd+Shift+F or Ctrl+Shift+F)
      if (cmdOrCtrl && e.shiftKey && e.key === 'F') {
        e.preventDefault();
        setActiveActivity('search');
      }
      // Toggle Terminal (Ctrl+` or Cmd+`)
      if ((e.ctrlKey || e.metaKey) && e.key === '`') {
        e.preventDefault();
        setShowTerminal(!showTerminal);
      }
      // Toggle Explorer (Cmd+Shift+E or Ctrl+Shift+E)
      if (cmdOrCtrl && e.shiftKey && e.key === 'E') {
        e.preventDefault();
        if (activeActivity === 'explorer') {
          setShowSidebar(!showSidebar);
        } else {
          setActiveActivity('explorer');
        }
      }
      // Go to File (Cmd+P or Ctrl+P)
      if (cmdOrCtrl && e.key === 'p' && !e.shiftKey) {
        e.preventDefault();
        setShowCommandPalette(true);
      }
      // Go to Symbol (Cmd+T or Ctrl+T)
      if (cmdOrCtrl && e.key === 't' && !e.shiftKey) {
        e.preventDefault();
        setShowCommandPalette(true);
      }
      // Go to Line (Ctrl+G or Cmd+G)
      if ((e.ctrlKey || e.metaKey) && e.key === 'g' && !e.shiftKey) {
        e.preventDefault();
        const line = prompt('Enter line number:');
        if (line) {
          const event = new CustomEvent('nava:navigate-to-line', { detail: { line: parseInt(line) } });
          window.dispatchEvent(event);
        }
      }
    };

    window.addEventListener('keydown', handleKeyDown);
    return () => window.removeEventListener('keydown', handleKeyDown);
  }, [handleNewFile, activeActivity, showSidebar, showTerminal, currentFile, code]);

  // Handle activity changes with proper state management
  const handleActivityChange = (activity: ActivityType) => {
    // Prevent unnecessary re-renders if already on this activity
    if (activeActivity === activity) {
      return;
    }
    // Use functional update to prevent race conditions
    setActiveActivity((prev) => {
      if (prev === activity) return prev;
      return activity;
    });
    setShowSidebar(true);
    // Clear any pending state that might cause flashing
    setShowVisualizer(false);
    setShowAIPane(false);
    setShowCollaboration(false);
  };

  // Sample navigation paths for visualization
  const navigationPaths: NavigationPath[] = [
    {
      waypoints: [
        [0, 0, 0],
        [2, 2, 1],
        [4, 3, 2],
        [5, 5, 5],
      ],
      energy: 1.234,
      optimizationMethod: 'vnc',
    },
  ];

  const handleRun = async () => {
    setStatus('running');
    voiceService.announceStatus('Running code');
    try {
      await parseCode();
      setShowVisualizer(true);
      voiceService.speak('Code executed successfully');
      setTimeout(() => setStatus('ready'), 1000);
    } catch (error) {
      setStatus('error');
      voiceService.readError('Failed to run code');
      console.error('Run error:', error);
    }
  };

  const handleCompile = async () => {
    setStatus('compiling');
    voiceService.announceStatus('Compiling code');
    try {
      await compileToTarget(code, 'cpp');
      voiceService.speak('Compilation successful');
      setTimeout(() => setStatus('ready'), 1000);
    } catch (error) {
      setStatus('error');
      voiceService.readError('Compilation failed');
      console.error('Compilation error:', error);
    }
  };

  const handleVisualize = () => {
    setShowVisualizer(!showVisualizer);
  };

  const handleDeploy = () => {
    console.log('Deploy functionality coming soon!');
  };

  const handleSettings = () => {
    setShowSettings(true);
  };

  const handleToggleSidebar = () => {
    setShowSidebar(!showSidebar);
  };

  const handleTogglePanel = () => {
    setShowPanel(!showPanel);
  };

  const handleToggleAIPane = () => {
    setShowAIPane(!showAIPane);
  };

  const handleNavigateToWorkspace = () => {
    setActiveActivity('workspace');
    // Also update URL if needed
    const url = new URL(window.location.href);
    url.pathname = '/workspace.html';
    url.search = '';
    window.location.href = url.toString();
  };

  const handleNewWindow = () => {
    window.open(window.location.href, '_blank');
  };

  const handleExportChat = () => {
    console.log('Exporting conversation...');
    // Implement export logic
  };

  const handleDeleteChat = () => {
    console.log('Deleting chat...');
    // Implement delete logic
  };

  const handleSignIn = () => {
    console.log('Sign in requested...');
    // Implement sign-in logic
  };

  const handleAddContext = () => {
    console.log('Add context...');
    // Implement add context logic
  };

  const handleCustomInstructions = () => {
    setShowSettings(true);
  };

  const handleExecuteCommand = (command: string) => {
    switch (command) {
      case 'settings':
        setShowSettings(true);
        break;
      case 'extensions':
        setActiveActivity('extensions');
        setShowSidebar(true);
        break;
      case 'keyboard-shortcuts':
        setShowSettings(true);
        break;
      case 'themes':
        setShowSettings(true);
        break;
      case 'profiles':
        setShowSettings(true);
        break;
      case 'widgets':
      case 'widget-manager':
        setShowWidgetManager(true);
        break;
      case 'close-all':
        console.log('Close all editors');
        break;
      case 'configure-editors':
        setShowSettings(true);
        break;
      default:
        console.log('Execute command:', command);
    }
  };

  const renderActivityPanel = () => {
    // Prevent rendering before initialization to avoid flashing
    if (!isInitialized || !activeActivity) {
      return (
        <div className="empty-panel" style={{ padding: '20px', textAlign: 'center', color: '#94a3b8' }}>
          <div style={{ fontSize: '14px' }}>Loading...</div>
        </div>
      );
    }

    switch (activeActivity) {
      case 'workspace':
        // Workspace activity - OS Desktop is rendered in main area, not in sidebar
        // Return null since OS Desktop is full-screen and doesn't need sidebar content
        return null;
      case 'explorer':
        return (
          <SplitPanel
            direction="vertical"
            defaultSplit={60}
            minSize={100}
            storageKey="nav-studio-explorer-split"
            titles={['EXPLORER', 'OUTLINE & TIMELINE']}
            collapsible={true}
          >
            <FileExplorer
              onFileSelect={async (path, content) => {
                console.log('[FileExplorer] File selected:', path, 'Content length:', content?.length);
                
                // Track file open event
                timelineService.addEvent('File opened', path, 'file');
                
                // Check if it's a notebook file
                if (path.endsWith('.ipynb')) {
                  setNotebookFilePath(path);
                  setShowPanel(true);
                } else {
                  // Ensure we have content - try multiple methods
                  let fileContent = content;
                  
                  if (!fileContent || fileContent.trim() === '') {
                    console.warn('[FileExplorer] Empty content, trying to reload...');
                    try {
                      fileContent = await fileService.readFile(path);
                      console.log('[FileExplorer] Reloaded content, length:', fileContent.length);
                    } catch (error) {
                      console.error('[FileExplorer] Failed to reload file:', error);
                      // Try one more time with normalized path
                      try {
                        const normalizedPath = path.startsWith('/') ? path.substring(1) : path;
                        fileContent = await fileService.readFile(normalizedPath);
                        console.log('[FileExplorer] Reloaded with normalized path');
                      } catch (error2) {
                        console.error('[FileExplorer] All reload attempts failed');
                        alert(`Unable to read file: ${path}\n\nPlease try refreshing the explorer or re-importing the file.`);
                        return;
                      }
                    }
                  }
                  
                  // Ensure we have valid content (even if empty)
                  const finalContent = fileContent || '';
                  
                  // Update editor state - force update
                  setCurrentFile(path);
                  setCode(finalContent);
                  setNotebookFilePath(undefined);
                  
                  // Add to tabs if not already open
                  const fileName = fileService.getFileName(path);
                  setOpenTabs(prev => {
                    const existing = prev.find(t => t.path === path);
                    if (existing) {
                      // Update existing tab with fresh content
                      return prev.map(t => 
                        t.path === path 
                          ? { ...t, content: finalContent, isModified: false } 
                          : t
                      );
                    }
                    return [...prev, { path, name: fileName, content: finalContent, isModified: false }];
                  });
                  
                  console.log('[FileExplorer] File opened successfully:', path, 'Content length:', finalContent.length);
                  
                  // Force editor update
                  setTimeout(() => {
                    const event = new CustomEvent('nava:editor-update', { detail: { path, content: finalContent } });
                    window.dispatchEvent(event);
                  }, 100);
                }
              }}
              currentFilePath={currentFile}
              onOpenProject={() => setShowProjectManager(true)}
            />
            <SplitPanel
              direction="vertical"
              defaultSplit={50}
              minSize={80}
              storageKey="nav-studio-outline-timeline-split"
              titles={['OUTLINE', 'TIMELINE']}
              collapsible={true}
            >
              <Outline 
                hideHeader={true} 
                code={code}
                language={currentFile ? fileService.getFileExtension(currentFile) : undefined}
                onItemClick={(line) => {
                  // Navigate to line in editor
                  const event = new CustomEvent('nava:navigate-to-line', {
                    detail: { line },
                  });
                  window.dispatchEvent(event);
                }}
              />
              <Timeline hideHeader={true} />
            </SplitPanel>
          </SplitPanel>
        );
      case 'search':
        return <SearchPanel />;
      case 'source-control':
        return <SourceControlPanel />;
      case 'debug':
        return <DebugPanel />;
      case 'discovery':
        return <DiscoveryPanel />;
      case 'extensions':
        return <ExtensionsPanel />;
      case 'chat-history':
        return <ChatHistoryPanel />;
      case 'ros-learning':
        // ROS Learning Center will be rendered in main area, not sidebar
        return (
          <div className="empty-panel" style={{ padding: '20px', color: '#94a3b8' }}>
            <h3 style={{ color: '#10b981', marginBottom: '10px' }}>🤖 ROS Learning Center</h3>
            <p style={{ fontSize: '14px' }}>Use the main area to access courses and content</p>
          </div>
        );
      case 'mcp-toolkit':
        return <MCPToolkitPanel />;
      case 'app-centre':
        // App Centre is rendered full-screen, not in sidebar
        return null;
      case 'simulation':
        // Simulation now renders in main area, show info in sidebar
        return (
          <div className="simulation-sidebar-info" style={{ padding: '20px', color: '#94a3b8' }}>
            <h3 style={{ color: '#3b82f6', marginBottom: '10px' }}>🎮 Simulation Active</h3>
            <p style={{ fontSize: '14px', lineHeight: '1.6' }}>
              Full simulation interface is displayed in the main workspace area.
            </p>
            <p style={{ fontSize: '13px', marginTop: '10px', opacity: 0.7 }}>
              Use the tabs in the main area to switch between Simulation, Robot Control, and World Manager.
            </p>
          </div>
        );
      case 'remote-explorer':
        return (
          <div className="empty-panel" style={{ padding: '20px', color: '#94a3b8' }}>
            <h3 style={{ color: '#3b82f6', marginBottom: '10px' }}>💾 Remote Explorer</h3>
            <p style={{ fontSize: '14px' }}>Remote file explorer functionality coming soon...</p>
          </div>
        );
      case 'containers':
        return (
          <div className="empty-panel" style={{ padding: '20px', color: '#94a3b8' }}>
            <h3 style={{ color: '#3b82f6', marginBottom: '10px' }}>🐳 Containers</h3>
            <p style={{ fontSize: '14px' }}>Container management functionality coming soon...</p>
          </div>
        );
      case 'stripe':
        return (
          <div className="empty-panel" style={{ padding: '20px', color: '#94a3b8' }}>
            <h3 style={{ color: '#3b82f6', marginBottom: '10px' }}>💳 Stripe Integration</h3>
            <p style={{ fontSize: '14px' }}>Stripe integration functionality coming soon...</p>
          </div>
        );
      case 'github':
        return (
          <div className="empty-panel" style={{ padding: '20px', color: '#94a3b8' }}>
            <h3 style={{ color: '#3b82f6', marginBottom: '10px' }}>🐙 GitHub</h3>
            <p style={{ fontSize: '14px' }}>GitHub integration functionality coming soon...</p>
          </div>
        );
      case 'univarm-starter':
        return (
          <div className="univarm-starter-fullscreen" style={{ flex: 1, width: '100%', height: '100%', overflow: 'hidden' }}>
            <UnivarmStarterApp />
          </div>
        );
      case 'univarm-advanced':
        return (
          <div className="univarm-advanced-fullscreen" style={{ flex: 1, width: '100%', height: '100%', overflow: 'hidden' }}>
            <UnivarmAdvancedApp />
          </div>
        );
      default:
        return (
          <div className="empty-panel" style={{ padding: '20px', color: '#94a3b8' }}>
            <h3 style={{ color: '#3b82f6', marginBottom: '10px' }}>Select an activity</h3>
            <p style={{ fontSize: '14px' }}>Choose an activity from the sidebar to get started.</p>
          </div>
        );
    }
  };

  const handleCloseEditor = () => {
    if (openTabs.length > 0) {
      if (currentFile) {
        setOpenTabs(prev => prev.filter(t => t.path !== currentFile));
        const remaining = openTabs.filter(t => t.path !== currentFile);
        if (remaining.length > 0) {
          const nextTab = remaining[0];
          setCurrentFile(nextTab.path);
          setCode(nextTab.content);
        } else {
          setCurrentFile('');
          setCode('');
        }
      }
    }
  };

  return (
    <div className={`app ${isInitialized ? '' : 'loading'} ${activeActivity === 'workspace' ? 'workspace-mode' : ''} ${activeActivity === 'app-centre' ? 'app-centre-mode' : ''}`}>
      {/* Traditional Menu Bar - macOS style */}
      {activeActivity !== 'workspace' && activeActivity !== 'app-centre' && (
        <TraditionalMenuBar
          onNewFile={handleNewFile}
          onOpenFile={() => setActiveActivity('explorer')}
          onSave={() => {
            if (currentFile) {
              fileService.saveFile(currentFile, code);
              setOpenTabs(prev => prev.map(t => t.path === currentFile ? { ...t, isModified: false } : t));
            }
          }}
          onSaveAs={() => {
            const fileName = prompt('Enter file name:');
            if (fileName) {
              const path = `/projects/${fileName}`;
              fileService.saveFile(path, code);
              setCurrentFile(path);
            }
          }}
          onSaveAll={() => {
            openTabs.forEach(tab => {
              if (tab.isModified) {
                fileService.saveFile(tab.path, tab.content);
              }
            });
            setOpenTabs(prev => prev.map(t => ({ ...t, isModified: false })));
          }}
          onCloseEditor={handleCloseEditor}
          onCloseWindow={() => window.close()}
          onUndo={() => {
            const event = new CustomEvent('nava:editor-undo');
            window.dispatchEvent(event);
          }}
          onRedo={() => {
            const event = new CustomEvent('nava:editor-redo');
            window.dispatchEvent(event);
          }}
          onCut={() => {
            const event = new CustomEvent('nava:editor-cut');
            window.dispatchEvent(event);
          }}
          onCopy={() => {
            const event = new CustomEvent('nava:editor-copy');
            window.dispatchEvent(event);
          }}
          onPaste={() => {
            const event = new CustomEvent('nava:editor-paste');
            window.dispatchEvent(event);
          }}
          onFind={() => {
            const event = new CustomEvent('nava:editor-find');
            window.dispatchEvent(event);
          }}
          onReplace={() => {
            const event = new CustomEvent('nava:editor-replace');
            window.dispatchEvent(event);
          }}
          onFindInFiles={() => setActiveActivity('search')}
          onReplaceInFiles={() => setActiveActivity('search')}
          onToggleLineComment={() => {
            const event = new CustomEvent('nava:editor-toggle-comment');
            window.dispatchEvent(event);
          }}
          onSelectAll={() => {
            const event = new CustomEvent('nava:editor-select-all');
            window.dispatchEvent(event);
          }}
          onCommandPalette={() => setShowCommandPalette(true)}
          onToggleExplorer={() => {
            if (activeActivity === 'explorer') {
              setShowSidebar(!showSidebar);
            } else {
              setActiveActivity('explorer');
            }
          }}
          onToggleSearch={() => setActiveActivity('search')}
          onToggleSourceControl={() => setActiveActivity('source-control')}
          onToggleRun={() => setActiveActivity('debug')}
          onToggleExtensions={() => setActiveActivity('extensions')}
          onToggleChat={() => setShowAIPane(!showAIPane)}
          onToggleProblems={() => {
            // Toggle Problems panel - show in debug activity
            if (activeActivity === 'debug') {
              setShowSidebar(!showSidebar);
            } else {
              setActiveActivity('debug');
            }
          }}
          onToggleOutput={() => {
            // Toggle Output panel - show in bottom panel
            setShowPanel(!showPanel);
          }}
          onToggleDebugConsole={() => {
            // Toggle Debug Console - show in bottom panel
            setShowPanel(!showPanel);
          }}
          onToggleTerminal={() => setShowTerminal(!showTerminal)}
          onToggleWordWrap={() => {
            const event = new CustomEvent('nava:editor-toggle-word-wrap');
            window.dispatchEvent(event);
          }}
          onGoToFile={() => setShowCommandPalette(true)}
          onGoToSymbol={() => setShowCommandPalette(true)}
          onGoToDefinition={() => {
            const event = new CustomEvent('nava:go-to-definition');
            window.dispatchEvent(event);
          }}
          onGoToLine={() => {
            const line = prompt('Enter line number:');
            if (line) {
              const event = new CustomEvent('nava:navigate-to-line', { detail: { line: parseInt(line) } });
              window.dispatchEvent(event);
            }
          }}
          onStartDebugging={() => {
            const configs = debugService.getConfigurations();
            if (configs.length > 0) {
              const session = debugService.startDebugging(configs[0]);
              setHasActiveDebugSession(true);
            } else {
              // Create default config and start
              const defaultConfig: DebugConfiguration = {
                name: 'Launch',
                type: 'navlambda',
                request: 'launch',
                program: currentFile || '${workspaceFolder}/main.navλ',
              };
              debugService.addConfiguration(defaultConfig);
              const session = debugService.startDebugging(defaultConfig);
              setHasActiveDebugSession(true);
            }
          }}
          onRunWithoutDebugging={handleRun}
          onStopDebugging={() => {
            debugService.stopDebugging();
            setHasActiveDebugSession(false);
          }}
          onRestartDebugging={() => {
            const session = debugService.restartDebugging();
            if (session) {
              setHasActiveDebugSession(true);
            }
          }}
          onAddConfiguration={() => {
            const name = prompt('Configuration name:');
            const type = prompt('Type (navlambda/python/node):', 'navlambda');
            if (name && type) {
              const config: DebugConfiguration = {
                name,
                type,
                request: 'launch',
                program: currentFile || '${workspaceFolder}/main.navλ',
              };
              debugService.addConfiguration(config);
            }
          }}
          onOpenConfigurations={() => {
            const configs = debugService.getConfigurations();
            alert(`Debug Configurations:\n\n${configs.map(c => `• ${c.name} (${c.type})`).join('\n')}`);
          }}
          onStepOver={() => debugService.stepOver()}
          onStepInto={() => debugService.stepInto()}
          onStepOut={() => debugService.stepOut()}
          onContinue={() => debugService.continueDebugging()}
          onToggleBreakpoint={() => {
            if (currentFile) {
              // Get current line from editor (would need to get from Monaco)
              const line = prompt('Enter line number for breakpoint:');
              if (line) {
                debugService.toggleBreakpoint(currentFile, parseInt(line));
              }
            }
          }}
          onNewBreakpoint={() => {
            if (currentFile) {
              const line = prompt('Enter line number for breakpoint:');
              if (line) {
                debugService.addBreakpoint(currentFile, parseInt(line));
              }
            }
          }}
          onEnableAllBreakpoints={() => debugService.enableAllBreakpoints()}
          onDisableAllBreakpoints={() => debugService.disableAllBreakpoints()}
          onRemoveAllBreakpoints={() => {
            if (confirm('Remove all breakpoints?')) {
              debugService.removeAllBreakpoints();
            }
          }}
          onNewTerminal={() => setShowTerminal(true)}
          onSplitTerminal={() => {
            // Would create a split terminal view
            console.log('Split terminal');
          }}
          onNewTerminalWindow={() => {
            window.open(window.location.href + '?terminal=true', '_blank');
          }}
          onRunTask={async () => {
            const tasks = taskService.getTasks();
            if (tasks.length === 0) {
              alert('No tasks configured. Use "Configure Tasks..." to add tasks.');
              return;
            }
            const taskNames = tasks.map(t => `${t.id}: ${t.label}`).join('\n');
            const taskId = prompt(`Select task:\n${taskNames}\n\nEnter task ID:`);
            if (taskId) {
              try {
                await taskService.runTask(taskId);
              } catch (error) {
                alert(`Failed to run task: ${error}`);
              }
            }
          }}
          onRunBuildTask={async () => {
            try {
              await taskService.runBuildTask();
            } catch (error) {
              alert(`Failed to run build task: ${error}`);
            }
          }}
          onRunActiveFile={async () => {
            if (currentFile) {
              try {
                await taskService.runActiveFile(currentFile);
              } catch (error) {
                alert(`Failed to run file: ${error}`);
              }
            } else {
              alert('No file is currently open.');
            }
          }}
          onRunSelectedText={() => {
            // Would run selected text in terminal
            console.log('Run selected text');
          }}
          onShowRunningTasks={() => {
            const tasks = runningTasks;
            if (tasks.length === 0) {
              alert('No tasks are currently running.');
            } else {
              alert(`Running Tasks:\n\n${tasks.map(t => `• ${t.label} (${t.status})`).join('\n')}`);
            }
          }}
          onRestartRunningTask={async () => {
            if (runningTasks.length === 0) {
              alert('No tasks are currently running.');
              return;
            }
            const taskId = prompt(`Restart task:\n${runningTasks.map(t => `${t.id}: ${t.label}`).join('\n')}\n\nEnter task ID:`);
            if (taskId) {
              try {
                await taskService.restartTask(taskId);
              } catch (error) {
                alert(`Failed to restart task: ${error}`);
              }
            }
          }}
          onTerminateTask={() => {
            if (runningTasks.length === 0) {
              alert('No tasks are currently running.');
              return;
            }
            const taskId = prompt(`Terminate task:\n${runningTasks.map(t => `${t.id}: ${t.label}`).join('\n')}\n\nEnter task ID:`);
            if (taskId) {
              taskService.stopTask(taskId);
            }
          }}
          onConfigureTasks={() => {
            const tasks = taskService.getTasks();
            const taskList = tasks.length > 0 
              ? tasks.map(t => `• ${t.label} (${t.type}): ${t.command}`).join('\n')
              : 'No tasks configured.';
            alert(`Tasks:\n\n${taskList}\n\nUse the Settings panel to add/edit tasks.`);
          }}
          onConfigureDefaultBuildTask={() => {
            const tasks = taskService.getTasks().filter(t => t.group?.kind === 'build');
            if (tasks.length === 0) {
              alert('No build tasks found. Create a task with group "build" first.');
              return;
            }
            const taskId = prompt(`Set default build task:\n${tasks.map(t => `${t.id}: ${t.label}`).join('\n')}\n\nEnter task ID:`);
            if (taskId) {
              taskService.setDefaultBuildTask(taskId);
              alert('Default build task set!');
            }
          }}
          onAddFolderToWorkspace={() => {
            const folderPath = prompt('Enter folder path:');
            if (folderPath) {
              workspaceService.addFolderToWorkspace(folderPath);
              alert('Folder added to workspace!');
            }
          }}
          onSaveWorkspaceAs={() => {
            const name = prompt('Enter workspace name:');
            if (name) {
              try {
                workspaceService.saveWorkspaceAs(name);
                alert('Workspace saved!');
              } catch (error) {
                alert(`Failed to save workspace: ${error}`);
              }
            }
          }}
          onDuplicateWorkspace={() => {
            try {
              workspaceService.duplicateWorkspace();
              alert('Workspace duplicated!');
            } catch (error) {
              alert(`Failed to duplicate workspace: ${error}`);
            }
          }}
          onOpenRecent={(workspaceId) => {
            const workspace = workspaceService.openRecentWorkspace(workspaceId);
            if (workspace) {
              alert(`Opened workspace: ${workspace.name}`);
            }
          }}
          onShareWorkspace={async () => {
            const current = workspaceService.getCurrentWorkspace();
            if (current) {
              try {
                const shareId = await workspaceService.shareWorkspace(current.id);
                alert(`Workspace shared! Share ID: ${shareId}\n\n(In production, this would generate a shareable link)`);
              } catch (error) {
                alert(`Failed to share workspace: ${error}`);
              }
            } else {
              alert('No workspace to share. Create or open a workspace first.');
            }
          }}
          onAutoSave={() => {
            setAutoSaveEnabled(!autoSaveEnabled);
            alert(`Auto Save ${autoSaveEnabled ? 'disabled' : 'enabled'}`);
          }}
          onRevertFile={() => {
            if (currentFile) {
              if (confirm('Revert file to last saved version? Unsaved changes will be lost.')) {
                fileService.readFile(currentFile).then(content => {
                  setCode(content);
                  setOpenTabs(prev => prev.map(t => 
                    t.path === currentFile ? { ...t, content, isModified: false } : t
                  ));
                }).catch(error => {
                  alert(`Failed to revert file: ${error}`);
                });
              }
            }
          }}
          recentWorkspaces={recentWorkspaces}
          runningTasks={runningTasks}
          hasActiveDebugSession={hasActiveDebugSession}
          onMinimize={() => {
            if ((window as any).electron) {
              (window as any).electron.minimize();
            }
          }}
          onZoom={() => {
            if ((window as any).electron) {
              (window as any).electron.maximize();
            }
          }}
          onShowSettings={() => setShowSettings(true)}
        />
      )}
      {/* Menu Bar - Hide when in workspace or app-centre mode */}
      {activeActivity !== 'workspace' && activeActivity !== 'app-centre' && (
        <MenuBar
            onRun={handleRun}
            onCompile={handleCompile}
            onVisualize={handleVisualize}
            onDeploy={handleDeploy}
            onSettings={handleSettings}
            onTogglePanel={handleTogglePanel}
            showPanel={showPanel}
            onToggleAIPane={handleToggleAIPane}
            showAIPane={showAIPane}
          onNavigateToWorkspace={handleNavigateToWorkspace}
          currentFile={currentFile || 'No file open'}
          fileCount={12}
          isSaved={true}
        />
      )}

      <div className="app-main">
        <div className="app-content">
          {/* Hide Activity Bar and sidebar when in workspace mode */}
          {activeActivity !== 'workspace' && (
            <>
              {/* Activity Bar */}
              <ActivityBar
                activeActivity={activeActivity}
                onActivityChange={handleActivityChange}
                onNewWindow={handleNewWindow}
                onExportChat={handleExportChat}
                onDeleteChat={handleDeleteChat}
                onSignIn={handleSignIn}
                onAddContext={handleAddContext}
                onCustomInstructions={handleCustomInstructions}
              />

              {/* Left Sidebar - Resizable */}
              <ResizablePanel
                side="left"
                defaultWidth={300}
                minWidth={200}
                maxWidth={600}
                isCollapsed={!showSidebar}
                onToggleCollapse={handleToggleSidebar}
                title={activeActivity?.toUpperCase() || 'PANEL'}
                storageKey="nav-studio-left-panel-width"
              >
                {renderActivityPanel()}
              </ResizablePanel>
            </>
          )}

          {/* Main Editor Area or Full Simulation or OS Desktop */}
          {activeActivity === 'workspace' ? (
              // Full-screen OS Desktop Workspace - Render immediately, no initialization check
              <div className="workspace-fullscreen" style={{ flex: 1, width: '100%', height: '100%', overflow: 'hidden' }}>
                <KernelProvider>
                  <OSDesktop />
                </KernelProvider>
              </div>
            ) : activeActivity === 'app-centre' ? (
              // Full-screen App Centre - Render immediately
              <div className="app-centre-fullscreen" style={{ flex: 1, width: '100%', height: '100%', overflow: 'hidden' }}>
                <AppCentre />
              </div>
            ) : activeActivity === 'simulation' ? (
              // Full-screen Simulation Experience - Render immediately, no initialization check
              <div className="simulation-fullscreen" style={{ flex: 1, width: '100%', height: '100%', overflow: 'hidden' }}>
                <SimulationPanel />
              </div>
            ) : isInitialized ? (
            activeActivity === 'univarm-starter' ? (
              // Full-screen Univarm Starter
              <div className="univarm-starter-fullscreen" style={{ flex: 1, width: '100%', height: '100%', overflow: 'hidden' }}>
                <UnivarmStarterApp />
              </div>
            ) : activeActivity === 'univarm-advanced' ? (
              // Full-screen Univarm Advanced
              <div className="univarm-advanced-fullscreen" style={{ flex: 1, width: '100%', height: '100%', overflow: 'hidden' }}>
                <UnivarmAdvancedApp />
              </div>
            ) : activeActivity === 'ros-learning' ? (
              // ROS Learning Center with resizable panels
              <div className="ros-learning-container" style={{ flex: 1, display: 'flex', flexDirection: 'row', height: '100%', overflow: 'hidden' }}>
                {/* Main ROS Learning Content - Resizable */}
                <ResizablePanel
                  side="left"
                  defaultWidth={600}
                  minWidth={400}
                  maxWidth={1200}
                  isCollapsed={false}
                  title="ROS LEARNING"
                  storageKey="nav-studio-ros-learning-width"
                >
                  <ROSLearningCenter />
                </ResizablePanel>

                {/* Right Panel - Notebook/Editor - Resizable */}
                {showPanel && (
                  <ResizablePanel
                    side="right"
                    defaultWidth={500}
                    minWidth={300}
                    maxWidth={800}
                    isCollapsed={!showPanel}
                    onToggleCollapse={handleTogglePanel}
                    title="NOTEBOOK"
                    storageKey="nav-studio-ros-notebook-width"
                  >
                    {notebookFilePath ? (
                      <JupyterNotebookPanel filePath={notebookFilePath} />
                    ) : (
                      <NotebookPanel />
                    )}
                  </ResizablePanel>
                )}
              </div>
            ) : (
              // Normal Editor View
              <div className="editor-container">
                {/* Editor Tabs - Window Viewer */}
                {openTabs.length > 0 && (
                  <EditorTabs
                    tabs={openTabs.map(tab => ({
                      path: tab.path,
                      name: tab.name,
                      isModified: tab.isModified,
                      isActive: tab.path === currentFile,
                    }))}
                    activeTab={currentFile}
                    onTabClick={async (path) => {
                      const tab = openTabs.find(t => t.path === path);
                      if (tab) {
                        setCurrentFile(path);
                        setCode(tab.content);
                      } else {
                        // Load file if not in tabs
                        try {
                          const content = await fileService.readFile(path);
                          setCurrentFile(path);
                          setCode(content);
                          const fileName = fileService.getFileName(path);
                          setOpenTabs(prev => [...prev, { path, name: fileName, content, isModified: false }]);
                        } catch (error) {
                          console.error('Error loading file:', error);
                        }
                      }
                    }}
                    onTabClose={(path, e) => {
                      e.stopPropagation();
                      if (openTabs.length === 1) {
                        // Last tab - clear editor
                        setCurrentFile('');
                        setCode('');
                        setOpenTabs([]);
                      } else {
                        // Remove tab
                        setOpenTabs(prev => prev.filter(t => t.path !== path));
                        if (currentFile === path) {
                          // Switch to another tab
                          const remaining = openTabs.filter(t => t.path !== path);
                          if (remaining.length > 0) {
                            const nextTab = remaining[0];
                            setCurrentFile(nextTab.path);
                            setCode(nextTab.content);
                          }
                        }
                      }
                    }}
                  />
                )}
                
                <div className="editor-main-area" style={{ display: 'flex', flexDirection: 'row', flex: 1, overflow: 'hidden' }}>
                  <div className="editor-panel" style={{ flex: 1, display: 'flex', flexDirection: 'column', minWidth: 0 }}>
                  <NavLambdaMonacoEditor
                    key={currentFile || 'new-file'} // Force re-render when file changes
                    initialCode={code}
                    onCodeChange={(newCode) => {
                      setCode(newCode);
                      // Mark tab as modified
                      if (currentFile) {
                        setOpenTabs(prev => prev.map(t => 
                          t.path === currentFile ? { ...t, content: newCode, isModified: true } : t
                        ));
                      }
                    }}
                    onCursorPositionChange={handleCursorPositionChange}
                    filePath={currentFile}
                    language={currentFile ? fileService.getFileExtension(currentFile) : 'navlambda'}
                    onToggleTerminal={() => setShowTerminal(!showTerminal)}
                    onToggleFileReference={(symbol) => {
                      if (symbol) {
                        setReferenceSymbol(symbol);
                      }
                      setShowFileReference(!showFileReference);
                    }}
                    showTerminal={showTerminal}
                    showFileReference={showFileReference}
                    onToggleChat={() => setShowCollaboration(!showCollaboration)}
                    onTogglePanel={() => handleTogglePanel()}
                    onToggleSidebar={() => setShowSidebar(!showSidebar)}
                    showChat={showCollaboration}
                    showPanel={showPanel}
                    showSidebar={showSidebar}
                    onSearch={(query) => {
                      // Trigger global search
                      setActiveActivity('search');
                      // Dispatch search event
                      const event = new CustomEvent('nava:search', { detail: { query } });
                      window.dispatchEvent(event);
                    }}
                  />
                </div>

                  {/* Right Panel - File Reference Viewer */}
                  {showFileReference && (
                    <ResizablePanel
                      side="right"
                      defaultWidth={400}
                      minWidth={300}
                      maxWidth={800}
                      isCollapsed={!showFileReference}
                      onToggleCollapse={() => setShowFileReference(false)}
                      title="REFERENCES"
                      storageKey="nav-studio-file-reference-width"
                    >
                      <FileReferenceViewer
                        symbol={referenceSymbol}
                        filePath={currentFile}
                        onClose={() => setShowFileReference(false)}
                      />
                    </ResizablePanel>
                  )}
                </div>

                {/* Bottom Panel - Resizable (Terminal or Notebook) */}
                {(showTerminal || showPanel) && (
                  <ResizablePanel
                    side="bottom"
                    defaultHeight={showTerminal ? 300 : 300}
                    minHeight={150}
                    maxHeight={600}
                    isCollapsed={!showTerminal && !showPanel}
                    onToggleCollapse={() => {
                      if (showTerminal) {
                        setShowTerminal(false);
                      } else {
                        handleTogglePanel();
                      }
                    }}
                    title={showTerminal ? "TERMINAL" : "NOTEBOOK"}
                    storageKey={showTerminal ? "nav-studio-terminal-height" : "nav-studio-bottom-panel-height"}
                  >
                    {showTerminal ? (
                      <Terminal />
                    ) : notebookFilePath ? (
                      <JupyterNotebookPanel filePath={notebookFilePath} />
                    ) : (
                      <NotebookPanel />
                    )}
                  </ResizablePanel>
                )}
              </div>
            )
          ) : (
            // Loading state to prevent flash (only for non-workspace activities)
            <div className="editor-container" style={{ display: 'flex', alignItems: 'center', justifyContent: 'center' }}>
              <div style={{ color: '#94a3b8', fontSize: '14px' }}>Loading IDE...</div>
            </div>
          )}

          {/* Right Panel - Resizable (Visualizer / AI / Collaboration) - Hide during simulation, workspace, and app-centre */}
          {activeActivity !== 'simulation' && activeActivity !== 'workspace' && activeActivity !== 'app-centre' && (showVisualizer || showAIPane || showCollaboration) && (
            <ResizablePanel
              side="right"
              defaultWidth={450}
              minWidth={350}
              maxWidth={800}
              isCollapsed={!(showVisualizer || showAIPane || showCollaboration)}
              onToggleCollapse={() => {
                setShowVisualizer(false);
                setShowAIPane(false);
                setShowCollaboration(false);
              }}
              title="PANELS"
              storageKey="nav-studio-right-panel-width"
            >
              {showVisualizer && (
                <div className="visualizer-panel">
                  <NavigationVisualizer
                    navigationPaths={navigationPaths}
                    showVncEquations={true}
                    realTimeUpdate={true}
                  />
                </div>
              )}
              {showAIPane && (
                <div className="ai-pane-container">
                  <AIPanePanel 
                    currentCode={code}
                    currentFile={currentFile}
                  />
                </div>
              )}
              {showCollaboration && (
                <div className="collaboration-container">
                  <CollaborationPanel />
                </div>
              )}
            </ResizablePanel>
          )}
        </div>
      </div>




      {/* Command Palette */}
      <CommandPalette
        isOpen={showCommandPalette}
        onClose={() => setShowCommandPalette(false)}
        onExecuteCommand={handleExecuteCommand}
      />

      {/* Settings Panel */}
      <SettingsPanel
        isOpen={showSettings}
        onClose={() => setShowSettings(false)}
      />

      {/* Voice Assistant - Now integrated into AI Pane Panel */}
      {/* <VoiceAssistant
        onCommand={(command) => {
          console.log('Voice command:', command);
          handleExecuteCommand(command.toLowerCase());
        }}
      /> */}

      {/* Widget Manager */}
      {showWidgetManager && (
        <WidgetManager onClose={() => setShowWidgetManager(false)} />
      )}

      {/* Project Manager Modal */}
      {showProjectManager && (
        <ProjectManager
          onProjectOpen={(project) => {
            timelineService.addEvent('Project opened', project.path, 'file', project.name);
            setShowProjectManager(false);
            // Reload file explorer
            const event = new CustomEvent('nava:refresh-explorer');
            window.dispatchEvent(event);
          }}
          onClose={() => setShowProjectManager(false)}
        />
      )}
      
      {/* PWA Install Prompt */}
      <PWAInstallPrompt />
    </div>
  );
}

export default App;

