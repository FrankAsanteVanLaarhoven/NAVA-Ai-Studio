import { useState, useEffect } from 'react';
import { NavLambdaMonacoEditor } from './components/Editor/MonacoEditor';
import { NavigationVisualizer } from './components/Visualizer/NavigationVisualizer';
import { Toolbar } from './components/Common/Toolbar';
import { StatusBar } from './components/Common/StatusBar';
import { ProjectStatusBar } from './components/Common/ProjectStatusBar';
import { FileExplorer } from './components/Sidebar/FileExplorer';
import { Outline } from './components/Sidebar/Outline';
import { Timeline } from './components/Sidebar/Timeline';
import { CollaborationPanel } from './components/Collaboration/CollaborationPanel';
import { ResizablePanel } from './components/Common/ResizablePanel';
import { SplitPanel } from './components/Common/SplitPanel';
import { NotebookPanel } from './components/Notebook/NotebookPanel';
import { AIPanePanel } from './components/AI/AIPanePanel';
import { ActivityBar, ActivityType } from './components/ActivityBar/ActivityBar';
import { SourceControlPanel } from './components/ActivityPanels/SourceControlPanel';
import { DebugPanel } from './components/ActivityPanels/DebugPanel';
import { ExtensionsPanel } from './components/ActivityPanels/ExtensionsPanel';
import { ChatHistoryPanel } from './components/ActivityPanels/ChatHistoryPanel';
import { MCPToolkitPanel } from './components/ActivityPanels/MCPToolkitPanel';
import { CommandPalette } from './components/CommandPalette/CommandPalette';
import { SettingsPanel } from './components/Settings/SettingsPanel';
import { VoiceAssistant } from './components/Voice/VoiceAssistant';
import { ROSLearningCenter } from './components/ROSLearning/ROSLearningCenter';
import { SimulationPanel } from './components/Simulation/SimulationPanel';
import { WidgetManager } from './components/Widgets/WidgetManager';
import { useNavLambdaLsp } from './hooks/useNavLambdaLsp';
import { useMultiTargetCompilation } from './hooks/useMultiTargetCompilation';
import { pwaService } from './services/pwa-service';
import { voiceService } from './services/voice-service';
import { NavigationPath } from './types';
import './App.css';

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
  const [cursorLine, setCursorLine] = useState(1);
  const [cursorColumn, setCursorColumn] = useState(1);
  const [status, setStatus] = useState<'ready' | 'compiling' | 'running' | 'error'>('ready');
  const [showVisualizer, setShowVisualizer] = useState(false);
  const [showCollaboration, setShowCollaboration] = useState(false);
  const [showSidebar, setShowSidebar] = useState(true);
  const [showPanel, setShowPanel] = useState(true); // Show notebook by default
  const [showAIPane, setShowAIPane] = useState(false);
  const [activeActivity, setActiveActivity] = useState<ActivityType | null>('explorer');
  const [showCommandPalette, setShowCommandPalette] = useState(false);
  const [showSettings, setShowSettings] = useState(false);
  const [showWidgetManager, setShowWidgetManager] = useState(false);
  const [currentFilePath, setCurrentFilePath] = useState<string>('No file open');
  const [fileStatus, setFileStatus] = useState<'saved' | 'unsaved' | 'error'>('saved');

  const { parseCode, isLoading } = useNavLambdaLsp(code);
  const { compileToTarget, isCompiling } = useMultiTargetCompilation();

  // Initialize PWA
  useEffect(() => {
    pwaService.initialize();
  }, []);

  // Handle URL parameters for deep linking
  useEffect(() => {
    const urlParams = new URLSearchParams(window.location.search);
    const activity = urlParams.get('activity');
    if (activity && activity === 'ros-learning') {
      setActiveActivity('ros-learning');
      setShowSidebar(true);
    }
  }, []);

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

  // Handle activity changes
  const handleActivityChange = (activity: ActivityType) => {
    setActiveActivity(activity);
    setShowSidebar(true);
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

  const handleCursorPositionChange = (position: any) => {
    setCursorLine(position.lineNumber);
    setCursorColumn(position.column);
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
    switch (activeActivity) {
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
            <FileExplorer />
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
      default:
        return <div className="empty-panel">Select an activity</div>;
    }
  };

  return (
    <div className="app">
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
        currentFile={currentFilePath}
        fileStatus={fileStatus}
        totalFiles={12}
        modifiedFiles={0}
      />

      <div className="app-main">
        <div className="app-content">
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

          {/* Main Editor Area or Full Simulation */}
          {activeActivity === 'simulation' ? (
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
          )}

          {/* Right Panel - Resizable (Visualizer / AI / Collaboration) - Hide during simulation */}
          {activeActivity !== 'simulation' && (showVisualizer || showAIPane || showCollaboration) && (
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



      <StatusBar
        line={cursorLine}
        column={cursorColumn}
        language="NAVΛ"
        status={status}
      />

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

