import React, { useState, useRef, useEffect } from 'react';
import './TraditionalMenuBar.css';

interface MenuItem {
  label: string;
  shortcut?: string;
  action?: () => void;
  disabled?: boolean;
  submenu?: MenuItem[];
  separator?: boolean;
}

interface TraditionalMenuBarProps {
  onNewFile?: () => void;
  onOpenFile?: () => void;
  onSave?: () => void;
  onSaveAs?: () => void;
  onSaveAll?: () => void;
  onCloseEditor?: () => void;
  onCloseWindow?: () => void;
  onUndo?: () => void;
  onRedo?: () => void;
  onCut?: () => void;
  onCopy?: () => void;
  onPaste?: () => void;
  onFind?: () => void;
  onReplace?: () => void;
  onFindInFiles?: () => void;
  onReplaceInFiles?: () => void;
  onToggleLineComment?: () => void;
  onSelectAll?: () => void;
  onCommandPalette?: () => void;
  onToggleExplorer?: () => void;
  onToggleSearch?: () => void;
  onToggleSourceControl?: () => void;
  onToggleRun?: () => void;
  onToggleExtensions?: () => void;
  onToggleChat?: () => void;
  onToggleProblems?: () => void;
  onToggleOutput?: () => void;
  onToggleDebugConsole?: () => void;
  onToggleTerminal?: () => void;
  onToggleWordWrap?: () => void;
  onGoToFile?: () => void;
  onGoToSymbol?: () => void;
  onGoToDefinition?: () => void;
  onGoToLine?: () => void;
  onStartDebugging?: () => void;
  onRunWithoutDebugging?: () => void;
  onToggleBreakpoint?: () => void;
  onNewTerminal?: () => void;
  onRunTask?: () => void;
  onRunBuildTask?: () => void;
  onMinimize?: () => void;
  onZoom?: () => void;
  onShowSettings?: () => void;
  onAddConfiguration?: () => void;
  onOpenConfigurations?: () => void;
  onStepOver?: () => void;
  onStepInto?: () => void;
  onStepOut?: () => void;
  onContinue?: () => void;
  onStopDebugging?: () => void;
  onRestartDebugging?: () => void;
  onEnableAllBreakpoints?: () => void;
  onDisableAllBreakpoints?: () => void;
  onRemoveAllBreakpoints?: () => void;
  onNewBreakpoint?: () => void;
  onSplitTerminal?: () => void;
  onNewTerminalWindow?: () => void;
  onRunActiveFile?: () => void;
  onRunSelectedText?: () => void;
  onShowRunningTasks?: () => void;
  onRestartRunningTask?: () => void;
  onTerminateTask?: () => void;
  onConfigureTasks?: () => void;
  onConfigureDefaultBuildTask?: () => void;
  onAddFolderToWorkspace?: () => void;
  onSaveWorkspaceAs?: () => void;
  onDuplicateWorkspace?: () => void;
  onOpenRecent?: (workspaceId: string) => void;
  onShareWorkspace?: () => void;
  onAutoSave?: () => void;
  onRevertFile?: () => void;
  recentWorkspaces?: Array<{ id: string; name: string }>;
  runningTasks?: Array<{ id: string; label: string; status: string }>;
  hasActiveDebugSession?: boolean;
}

export const TraditionalMenuBar: React.FC<TraditionalMenuBarProps> = ({
  onNewFile,
  onOpenFile,
  onSave,
  onSaveAs,
  onSaveAll,
  onCloseEditor,
  onCloseWindow,
  onUndo,
  onRedo,
  onCut,
  onCopy,
  onPaste,
  onFind,
  onReplace,
  onFindInFiles,
  onReplaceInFiles,
  onToggleLineComment,
  onSelectAll,
  onCommandPalette,
  onToggleExplorer,
  onToggleSearch,
  onToggleSourceControl,
  onToggleRun,
  onToggleExtensions,
  onToggleChat,
  onToggleProblems,
  onToggleOutput,
  onToggleDebugConsole,
  onToggleTerminal,
  onToggleWordWrap,
  onGoToFile,
  onGoToSymbol,
  onGoToDefinition,
  onGoToLine,
  onStartDebugging,
  onRunWithoutDebugging,
  onToggleBreakpoint,
  onNewTerminal,
  onRunTask,
  onRunBuildTask,
  onMinimize,
  onZoom,
  onShowSettings,
  onAddConfiguration,
  onOpenConfigurations,
  onStepOver,
  onStepInto,
  onStepOut,
  onContinue,
  onStopDebugging,
  onRestartDebugging,
  onEnableAllBreakpoints,
  onDisableAllBreakpoints,
  onRemoveAllBreakpoints,
  onNewBreakpoint,
  onSplitTerminal,
  onNewTerminalWindow,
  onRunActiveFile,
  onRunSelectedText,
  onShowRunningTasks,
  onRestartRunningTask,
  onTerminateTask,
  onConfigureTasks,
  onConfigureDefaultBuildTask,
  onAddFolderToWorkspace,
  onSaveWorkspaceAs,
  onDuplicateWorkspace,
  onOpenRecent,
  onShareWorkspace,
  onAutoSave,
  onRevertFile,
  recentWorkspaces = [],
  runningTasks = [],
  hasActiveDebugSession = false,
}) => {
  const [activeMenu, setActiveMenu] = useState<string | null>(null);
  const menuRefs = useRef<{ [key: string]: HTMLDivElement | null }>({});

  useEffect(() => {
    const handleClickOutside = (event: MouseEvent) => {
      if (activeMenu) {
        const menuElement = menuRefs.current[activeMenu];
        if (menuElement && !menuElement.contains(event.target as Node)) {
          setActiveMenu(null);
        }
      }
    };

    if (activeMenu) {
      document.addEventListener('mousedown', handleClickOutside);
      return () => document.removeEventListener('mousedown', handleClickOutside);
    }
  }, [activeMenu]);

  const handleMenuClick = (menuName: string) => {
    setActiveMenu(activeMenu === menuName ? null : menuName);
  };

  const handleMenuItemClick = (action?: () => void) => {
    if (action) {
      action();
    }
    setActiveMenu(null);
  };

  const fileMenu: MenuItem[] = [
    { label: 'New Text File', shortcut: '⌘N', action: onNewFile },
    { label: 'New File...', shortcut: '⇧⌘N', action: onNewFile },
    { label: 'New Window', shortcut: '⇧⌘N', action: () => window.open(window.location.href) },
    { label: 'New Window with Profile', submenu: [] },
    { separator: true },
    { label: 'Open...', shortcut: '⌘O', action: onOpenFile },
    { label: 'Open Folder...', action: onOpenFile },
    { label: 'Open Workspace from File...', action: onOpenFile },
    { 
      label: 'Open Recent', 
      submenu: (recentWorkspaces || []).length > 0 
        ? (recentWorkspaces || []).map(ws => ({ 
            label: ws.name, 
            action: () => onOpenRecent?.(ws.id) 
          }))
        : [{ label: 'No recent workspaces', disabled: true }]
    },
    { separator: true },
    { label: 'Add Folder to Workspace...', action: onAddFolderToWorkspace },
    { label: 'Save Workspace As...', action: onSaveWorkspaceAs },
    { label: 'Duplicate Workspace', action: onDuplicateWorkspace },
    { separator: true },
    { label: 'Save', shortcut: '⌘S', action: onSave },
    { label: 'Save As...', shortcut: '⇧⌘S', action: onSaveAs },
    { label: 'Save All', shortcut: '⌥⌘S', action: onSaveAll },
    { separator: true },
    { 
      label: 'Share', 
      submenu: [
        { label: 'Share Workspace...', action: onShareWorkspace },
        { label: 'Share via Link...', action: onShareWorkspace },
      ]
    },
    { separator: true },
    { label: 'Auto Save', action: onAutoSave },
    { separator: true },
    { label: 'Revert File', action: onRevertFile },
    { label: 'Close Editor', shortcut: '⌘W', action: onCloseEditor },
    { label: 'Close Window', shortcut: '⇧⌘W', action: onCloseWindow },
  ];

  const editMenu: MenuItem[] = [
    { label: 'Undo', shortcut: '⌘Z', action: onUndo },
    { label: 'Redo', shortcut: '⇧⌘Z', action: onRedo },
    { separator: true },
    { label: 'Cut', shortcut: '⌘X', action: onCut },
    { label: 'Copy', shortcut: '⌘C', action: onCopy },
    { label: 'Paste', shortcut: '⌘V', action: onPaste },
    { separator: true },
    { label: 'Find', shortcut: '⌘F', action: onFind },
    { label: 'Replace', shortcut: '⌥⌘F', action: onReplace },
    { separator: true },
    { label: 'Find in Files', shortcut: '⇧⌘F', action: onFindInFiles },
    { label: 'Replace in Files', shortcut: '⇧⌘H', action: onReplaceInFiles },
    { separator: true },
    { label: 'Toggle Line Comment', shortcut: '⌘/', action: onToggleLineComment },
    { label: 'Toggle Block Comment', shortcut: '⌥⇧⌘A' },
    { label: 'Emmet: Expand Abbreviation', submenu: [] },
    { separator: true },
    { label: 'Writing Tools', submenu: [] },
    { label: 'AutoFill', submenu: [] },
    { label: 'Start Dictation', disabled: true },
    { label: 'Emoji & Symbols', shortcut: '⌃⌘Space' },
  ];

  const selectionMenu: MenuItem[] = [
    { label: 'Select All', shortcut: '⌘A', action: onSelectAll },
    { label: 'Expand Selection', shortcut: '⇧⌥⌘►' },
    { label: 'Shrink Selection', shortcut: '⇧⌥⌘◄' },
    { separator: true },
    { label: 'Copy Line Up', shortcut: '⌥⇧▲' },
    { label: 'Copy Line Down', shortcut: '⌥⇧▼' },
    { label: 'Move Line Up', shortcut: '⌥▲' },
    { label: 'Move Line Down', shortcut: '⌥▼' },
    { label: 'Duplicate Selection' },
    { separator: true },
    { label: 'Add Cursor Above', shortcut: '⌥⌘▲' },
    { label: 'Add Cursor Below', shortcut: '⌥⌘▼' },
    { label: 'Add Cursors to Line Ends', shortcut: '⌥⇧I' },
    { label: 'Add Next Occurrence', shortcut: '⌘D' },
    { label: 'Add Previous Occurrence' },
    { label: 'Select All Occurrences' },
    { separator: true },
    { label: 'Switch to Cmd+Click for Multi-Cursor' },
    { label: 'Column Selection Mode' },
  ];

  const viewMenu: MenuItem[] = [
    { label: 'Command Palette...', shortcut: '⇧⌘P', action: onCommandPalette },
    { label: 'Open View...' },
    { separator: true },
    { label: 'Appearance', submenu: [] },
    { label: 'Editor Layout', submenu: [] },
    { separator: true },
    { label: 'Explorer', shortcut: '⇧⌘E', action: onToggleExplorer },
    { label: 'Search', shortcut: '⇧⌘F', action: onToggleSearch },
    { label: 'Source Control', shortcut: '⌃⇧⌘G', action: onToggleSourceControl },
    { label: 'Run', shortcut: '⇧⌘D', action: onToggleRun },
    { label: 'Extensions', shortcut: '⇧⌘X', action: onToggleExtensions },
    { separator: true },
    { label: 'Chat', shortcut: '⌃⌘I', action: onToggleChat },
    { separator: true },
    { label: 'Problems', shortcut: '⇧⌘M', action: onToggleProblems },
    { label: 'Output', shortcut: '⇧⌘U', action: onToggleOutput },
    { label: 'Debug Console', shortcut: '⇧⌘Y', action: onToggleDebugConsole },
    { label: 'Terminal', shortcut: '⌃`', action: onToggleTerminal },
    { separator: true },
    { label: 'Word Wrap', shortcut: '⌥Z', action: onToggleWordWrap },
  ];

  const goMenu: MenuItem[] = [
    { label: 'Back', shortcut: '^ -', disabled: true },
    { label: 'Forward', shortcut: '^⇧ -', disabled: true },
    { label: 'Last Edit Location', shortcut: '⌘K ⌘Q' },
    { separator: true },
    { label: 'Switch Editor', submenu: [] },
    { label: 'Switch Group', submenu: [] },
    { separator: true },
    { label: 'Go to File...', shortcut: '⌘P', action: onGoToFile },
    { label: 'Go to Symbol in Workspace...', shortcut: '⌘T', action: onGoToSymbol },
    { separator: true },
    { label: 'Go to Symbol in Editor...', shortcut: '⇧⌘O' },
    { label: 'Go to Definition', shortcut: 'F12', action: onGoToDefinition },
    { label: 'Go to Declaration' },
    { label: 'Go to Type Definition' },
    { label: 'Go to Implementations', shortcut: '⇧F12' },
    { label: 'Go to References', shortcut: '⇧F12' },
    { separator: true },
    { label: 'Go to Line/Column...', shortcut: '^G', action: onGoToLine },
    { label: 'Go to Bracket', shortcut: '⇧⌘\\' },
    { separator: true },
    { label: 'Next Problem', shortcut: 'F8' },
    { label: 'Previous Problem', shortcut: '⇧F8' },
    { separator: true },
    { label: 'Next Change', shortcut: '⌥F3' },
    { label: 'Previous Change', shortcut: '⇧⌥F3' },
  ];

  const runMenu: MenuItem[] = [
    { label: 'Start Debugging', shortcut: 'F5', action: onStartDebugging },
    { label: 'Run Without Debugging', shortcut: '⇧F5', action: onRunWithoutDebugging },
    { 
      label: 'Stop Debugging', 
      shortcut: '⇧F5', 
      disabled: !hasActiveDebugSession,
      action: onStopDebugging 
    },
    { 
      label: 'Restart Debugging', 
      shortcut: '⇧⌘F5', 
      disabled: !hasActiveDebugSession,
      action: onRestartDebugging 
    },
    { separator: true },
    { 
      label: 'Open Configurations', 
      disabled: !hasActiveDebugSession,
      action: onOpenConfigurations 
    },
    { label: 'Add Configuration...', action: onAddConfiguration },
    { separator: true },
    { 
      label: 'Step Over', 
      shortcut: 'F10', 
      disabled: !hasActiveDebugSession,
      action: onStepOver 
    },
    { 
      label: 'Step Into', 
      shortcut: 'F11', 
      disabled: !hasActiveDebugSession,
      action: onStepInto 
    },
    { 
      label: 'Step Out', 
      shortcut: '⇧F11', 
      disabled: !hasActiveDebugSession,
      action: onStepOut 
    },
    { 
      label: 'Continue', 
      shortcut: 'F5', 
      disabled: !hasActiveDebugSession,
      action: onContinue 
    },
    { separator: true },
    { label: 'Toggle Breakpoint', shortcut: 'F9', action: onToggleBreakpoint },
    { label: 'New Breakpoint', submenu: [] },
    { separator: true },
    { label: 'Enable All Breakpoints' },
    { label: 'Disable All Breakpoints' },
    { label: 'Remove All Breakpoints' },
    { separator: true },
    { label: 'Install Additional Debuggers...' },
  ];

  const terminalMenu: MenuItem[] = [
    { label: 'New Terminal', shortcut: '^⇧`', action: onNewTerminal },
    { label: 'Split Terminal', shortcut: '⌘\\', action: onSplitTerminal },
    { label: 'New Terminal Window', shortcut: '^⇧`', action: onNewTerminalWindow },
    { separator: true },
    { label: 'Run Task...', action: onRunTask },
    { label: 'Run Build Task...', shortcut: '⇧⌘B', action: onRunBuildTask },
    { label: 'Run Active File', action: onRunActiveFile },
    { label: 'Run Selected Text', action: onRunSelectedText },
    { separator: true },
    { 
      label: 'Show Running Tasks...', 
      disabled: (runningTasks || []).length === 0,
      action: onShowRunningTasks 
    },
    { 
      label: 'Restart Running Task...', 
      disabled: (runningTasks || []).length === 0,
      action: onRestartRunningTask 
    },
    { 
      label: 'Terminate Task...', 
      disabled: (runningTasks || []).length === 0,
      action: onTerminateTask 
    },
    { separator: true },
    { label: 'Configure Tasks...', action: onConfigureTasks },
    { label: 'Configure Default Build Task...', action: onConfigureDefaultBuildTask },
  ];

  const windowMenu: MenuItem[] = [
    { label: 'Minimize', shortcut: '⌘M', action: onMinimize },
    { label: 'Zoom', action: onZoom },
    { label: 'Fill', shortcut: '^🌐F' },
    { label: 'Centre', shortcut: '^🌐C' },
    { separator: true },
    { label: 'Move & Resize', submenu: [] },
    { label: 'Full-Screen Tile', submenu: [] },
    { separator: true },
    { label: 'Remove Window from Set', disabled: true },
    { separator: true },
    { label: 'Move to Frank\'s iPad pro 12 (2)' },
    { label: 'Switch Window...' },
    { label: 'Bring All to Front' },
  ];

  const helpMenu: MenuItem[] = [
    { label: 'Welcome' },
    { label: 'Show All Commands', shortcut: '⇧⌘P' },
    { label: 'Documentation' },
    { label: 'Release Notes' },
    { separator: true },
    { label: 'Keyboard Shortcuts', shortcut: '⌘K ⌘S' },
    { label: 'Video Tutorials' },
    { separator: true },
    { label: 'Tips and Tricks' },
    { separator: true },
    { label: 'Report Issue' },
    { label: 'Feature Request' },
    { separator: true },
    { label: 'About NAVΛ Studio IDE' },
  ];

  const menus = [
    { name: 'File', items: fileMenu },
    { name: 'Edit', items: editMenu },
    { name: 'Selection', items: selectionMenu },
    { name: 'View', items: viewMenu },
    { name: 'Go', items: goMenu },
    { name: 'Run', items: runMenu },
    { name: 'Terminal', items: terminalMenu },
    { name: 'Window', items: windowMenu },
    { name: 'Help', items: helpMenu },
  ];

  const renderMenuItem = (item: MenuItem, index: number) => {
    if (item.separator) {
      return <div key={`sep-${index}`} className="menu-separator" />;
    }

    return (
      <div
        key={index}
        className={`menu-item ${item.disabled ? 'disabled' : ''} ${item.submenu ? 'has-submenu' : ''}`}
        onClick={() => !item.disabled && handleMenuItemClick(item.action)}
      >
        <span className="menu-item-label">{item.label}</span>
        {item.shortcut && <span className="menu-item-shortcut">{item.shortcut}</span>}
        {item.submenu && <span className="menu-item-arrow">›</span>}
      </div>
    );
  };

  return (
    <div className="traditional-menu-bar">
      <div className="menu-bar-items">
        {menus.map((menu) => (
          <div key={menu.name} className="menu-bar-item-container">
            <button
              className={`menu-bar-item ${activeMenu === menu.name ? 'active' : ''}`}
              onClick={() => handleMenuClick(menu.name)}
            >
              {menu.name}
            </button>
            {activeMenu === menu.name && (
              <div
                ref={(el) => (menuRefs.current[menu.name] = el)}
                className="menu-dropdown"
              >
                {menu.items.map((item, index) => renderMenuItem(item, index))}
              </div>
            )}
          </div>
        ))}
      </div>
    </div>
  );
};

