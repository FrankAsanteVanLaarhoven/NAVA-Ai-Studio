import { useEffect, useState } from 'react';

import './App.css';
import { Toolbar } from './components/Common/Toolbar';
import { StatusBar } from './components/Common/StatusBar';
import { ProjectStatusBar } from './components/Common/ProjectStatusBar';
import { ResizablePanel } from './components/Common/ResizablePanel';
import { SplitPanel } from './components/Common/SplitPanel';
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
import { SimulationPanel } from './components/Simulation/SimulationPanel';
import { NotebookPanel } from './components/Notebook/NotebookPanel';
import { AIPanePanel } from './components/AI/AIPanePanel';
import { CollaborationPanel } from './components/Collaboration/CollaborationPanel';
import { CommandPalette } from './components/CommandPalette/CommandPalette';
import { SettingsPanel } from './components/Settings/SettingsPanel';
import { Outline } from './components/Sidebar/Outline';
import { Timeline } from './components/Sidebar/Timeline';
import { OSDesktop } from './components/Workspace/OSDesktop';
import type { NavigationPath } from './types';
import { useNavLambdaLsp } from './hooks/useNavLambdaLsp';
import { useMultiTargetCompilation } from './hooks/useMultiTargetCompilation';
import { pwaService } from './services/pwa-service';
import { voiceService } from './services/voice-service';
import { KernelProvider } from './kernel/kernel';
// Register apps on import
import './apps';

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
  const [cursorLine, setCursorLine] = useState(1);
  const [cursorColumn, setCursorColumn] = useState(1);
  const [status, setStatus] = useState<'ready' | 'compiling' | 'running' | 'error'>('ready');
  const [showVisualizer, setShowVisualizer] = useState(false);
  const [showCollaboration, setShowCollaboration] = useState(false);
  const [showSidebar, setShowSidebar] = useState(true);
  const [showPanel, setShowPanel] = useState(true); // Show notebook by default
  const [showAIPane, setShowAIPane] = useState(false);
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
    if (activity && ['workspace', 'ros-learning', 'simulation', 'explorer', 'search', 'source-control', 'debug', 'extensions', 'chat-history', 'mcp-toolkit'].includes(activity)) {
      console.log(`[APP] Initializing from URL: ${activity}`);
      return activity as ActivityType;
    }
    // Fallback to localStorage if no URL param
    const savedActivity = localStorage.getItem('navlambda-active-activity');
    if (savedActivity && ['workspace', 'explorer', 'search', 'source-control', 'debug', 'simulation', 'remote-explorer', 'extensions', 'containers', 'stripe', 'github', 'chat-history', 'ros-learning', 'mcp-toolkit'].includes(savedActivity)) {
      console.log(`[APP] Initializing from localStorage: ${savedActivity}`);
      return savedActivity as ActivityType;
    }
    // Default to workspace (OS Desktop)
    console.log(`[APP] Initializing with default: workspace`);
    return 'workspace';
  });
  const [showCommandPalette, setShowCommandPalette] = useState(false);
  const [showSettings, setShowSettings] = useState(false);
  const [showWidgetManager, setShowWidgetManager] = useState(false);
  // For workspace.html, initialize immediately to prevent flash
  const [isInitialized, setIsInitialized] = useState(() => {
    // If on workspace.html, initialize immediately
    if (window.location.pathname === '/workspace.html' || window.location.pathname.includes('workspace.html')) {
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

  // Initialize PWA and mark as initialized
  useEffect(() => {
    // For workspace, ensure it's initialized immediately (no delay)
    if (activeActivity === 'workspace') {
      setIsInitialized(true);
    }
    
    // Initialize PWA in background (non-blocking for workspace)
    pwaService.initialize().catch(() => {
      // Ignore PWA errors, they don't affect desktop UI
    });
    
    // If not already initialized (for non-workspace pages), set it after a brief delay
    if (!isInitialized && activeActivity !== 'workspace') {
      // Small delay to prevent flash on other pages
      const timer = setTimeout(() => {
        setIsInitialized(true);
      }, 50);
      return () => clearTimeout(timer);
    }
  }, [activeActivity]);

  // Listen for URL changes to update activity
  useEffect(() => {
    const handleUrlChange = () => {
      const urlParams = new URLSearchParams(window.location.search);
      const activity = urlParams.get('activity');
      console.log(`[APP] URL changed, activity from URL: ${activity}`);
      if (activity && ['workspace', 'ros-learning', 'simulation', 'explorer', 'search', 'source-control', 'debug', 'extensions', 'chat-history', 'mcp-toolkit'].includes(activity)) {
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

  // Keyboard shortcuts
  useEffect(() => {
    const handleKeyDown = (e: KeyboardEvent) => {
      // Command Palette (Ctrl+Shift+P or Cmd+Shift+P)
      if ((e.ctrlKey || e.metaKey) && e.shiftKey && e.key === 'P') {
        e.preventDefault();
        setShowCommandPalette(true);
      }
      // Settings (Ctrl+, or Cmd+,)
      if ((e.ctrlKey || e.metaKey) && e.key === ',') {
        e.preventDefault();
        setShowSettings(true);
      }
      // Widget Manager (Ctrl+Shift+W or Cmd+Shift+W)
      if ((e.ctrlKey || e.metaKey) && e.shiftKey && e.key === 'W') {
        e.preventDefault();
        setShowWidgetManager(true);
      }
    };

    window.addEventListener('keydown', handleKeyDown);
    return () => window.removeEventListener('keydown', handleKeyDown);
  }, []);

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
              onFileSelect={(path, content) => {
                setCurrentFile(path);
                setCode(content);
              }}
              currentFilePath={currentFile}
            />
            <SplitPanel
              direction="vertical"
              defaultSplit={50}
              minSize={80}
              storageKey="nav-studio-outline-timeline-split"
              titles={['OUTLINE', 'TIMELINE']}
              collapsible={true}
            >
              <Outline hideHeader={true} />
              <Timeline hideHeader={true} />
            </SplitPanel>
          </SplitPanel>
        );
      case 'search':
        return (
          <div className="empty-panel" style={{ padding: '20px', color: '#94a3b8' }}>
            <h3 style={{ color: '#3b82f6', marginBottom: '10px' }}>🔍 Search</h3>
            <p style={{ fontSize: '14px' }}>Global search functionality coming soon...</p>
          </div>
        );
      case 'source-control':
        return <SourceControlPanel />;
      case 'debug':
        return <DebugPanel />;
      case 'extensions':
        return <ExtensionsPanel />;
      case 'chat-history':
        return <ChatHistoryPanel />;
      case 'ros-learning':
        return <ROSLearningCenter />;
      case 'mcp-toolkit':
        return <MCPToolkitPanel />;
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
      default:
        return (
          <div className="empty-panel" style={{ padding: '20px', color: '#94a3b8' }}>
            <h3 style={{ color: '#3b82f6', marginBottom: '10px' }}>Select an activity</h3>
            <p style={{ fontSize: '14px' }}>Choose an activity from the sidebar to get started.</p>
          </div>
        );
    }
  };

  return (
    <div className={`app ${isInitialized ? '' : 'loading'} ${activeActivity === 'workspace' ? 'workspace-mode' : ''}`}>
      {/* Hide IDE toolbar and status bar when in workspace mode */}
      {activeActivity !== 'workspace' && (
        <>
          <Toolbar
            onRun={handleRun}
            onCompile={handleCompile}
            onVisualize={handleVisualize}
            onDeploy={handleDeploy}
            onSettings={handleSettings}
            onToggleSidebar={handleToggleSidebar}
            showSidebar={showSidebar}
            onTogglePanel={handleTogglePanel}
            showPanel={showPanel}
            onToggleAIPane={handleToggleAIPane}
            showAIPane={showAIPane}
          />

          {/* Project Status Bar */}
          <ProjectStatusBar
            projectName="NAVΛ STUDIO IDE"
            currentFile="No file open"
            fileStatus="saved"
            totalFiles={12}
            modifiedFiles={0}
          />
        </>
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
          {            activeActivity === 'workspace' ? (
              // Full-screen OS Desktop Workspace - Render immediately, no initialization check
              <div className="workspace-fullscreen" style={{ flex: 1, width: '100%', height: '100%', overflow: 'hidden' }}>
                <KernelProvider>
                  <OSDesktop />
                </KernelProvider>
              </div>
            ) : isInitialized ? (
            activeActivity === 'simulation' ? (
              // Full-screen Simulation Experience
              <div className="simulation-fullscreen" style={{ flex: 1, width: '100%', height: '100%', overflow: 'hidden' }}>
                <SimulationPanel />
              </div>
            ) : (
              // Normal Editor View
              <div className="editor-container">
                <div className="editor-panel">
                  <NavLambdaMonacoEditor
                    initialCode={initialCode}
                    onCodeChange={setCode}
                    onCursorPositionChange={handleCursorPositionChange}
                  />
                </div>

                {/* Bottom Panel - Resizable (Notebook) */}
                {showPanel && (
                  <ResizablePanel
                    side="bottom"
                    defaultHeight={300}
                    minHeight={150}
                    maxHeight={600}
                    isCollapsed={!showPanel}
                    onToggleCollapse={handleTogglePanel}
                    title="NOTEBOOK"
                    storageKey="nav-studio-bottom-panel-height"
                  >
                    <NotebookPanel />
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

          {/* Right Panel - Resizable (Visualizer / AI / Collaboration) - Hide during simulation and workspace */}
          {activeActivity !== 'simulation' && activeActivity !== 'workspace' && (showVisualizer || showAIPane || showCollaboration) && (
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
                  <AIPanePanel />
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



      {/* Hide IDE status bar when in workspace mode */}
      {activeActivity !== 'workspace' && (
        <StatusBar
          line={cursorLine}
          column={cursorColumn}
          language="NAVΛ"
          status={status}
        />
      )}

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
    </div>
  );
}

export default App;

