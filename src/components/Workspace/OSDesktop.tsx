/**
 * NAVA OS Desktop Workspace
 * 
 * The complete OS desktop workspace matching the screenshots:
 * - NAVA OS branding and app icons
 * - Left sidebar with navigation
 * - Right sidebar with widgets (Weather, World Clock, News, Currency, Stocks)
 * - Bottom dock
 * - Top menu bar with dropdown menus
 */

import React, { useState, useRef, useEffect } from 'react';
import { WorldWidgets } from './WorldWidgets';
import { WindowManager } from '../WindowManager';
import { BlurOverlay } from '../WindowManager';
import CIIndicatorWidget from '../../apps/ci-indicator/Widget';
import './OSDesktop.css';

interface AppIcon {
  id: string;
  name: string;
  icon: string;
  color?: string;
  route?: string;
  onClick?: () => void;
}

interface MenuItem {
  label?: string;
  action?: () => void;
  route?: string;
  divider?: boolean;
}

export const OSDesktop: React.FC = () => {
  const [selectedApp, setSelectedApp] = useState<string | null>(null);
  const [activeMenu, setActiveMenu] = useState<string | null>(null);
  const [activeDockApp, setActiveDockApp] = useState<string | null>(null);
  const [hoveredDockApp, setHoveredDockApp] = useState<string | null>(null);
  const menuRefs = useRef<{ [key: string]: HTMLDivElement | null }>({});

  const desktopApps: AppIcon[] = [
    { id: 'record', name: 'Record', icon: '🔴', color: '#ef4444', route: '/app.html?activity=simulation' },
    { id: 'camera', name: 'Camera', icon: '📷', route: '/app.html?activity=simulation' },
    { id: 'blueprint', name: 'Blueprint', icon: '📐', color: '#3b82f6', route: '/app.html?activity=explorer' },
    { id: 'go-live', name: 'Go Live', icon: '▶️', color: '#22c55e', route: '/app.html?activity=simulation' },
    { id: 'ad-manager', name: 'Ad Manager', icon: '📊', route: '/app.html' },
    { id: 'youtube', name: 'YouTube', icon: '▶️', color: '#ff0000', route: 'https://youtube.com' },
    { id: 'twitch', name: '.twitch Twitch', icon: '🎮', color: '#9146ff', route: 'https://twitch.tv' },
    { id: 'video-editor', name: 'Video Editor', icon: '🎬', route: '/app.html' },
    { id: 'image-editor', name: 'Image Editor', icon: '🖼️', route: '/app.html' },
    { id: 'web-builder', name: 'Web Builder', icon: '🌐', route: '/app.html' },
    { id: 'slides', name: 'Slides', icon: '📊', route: '/app.html' },
    { id: 'calculator', name: 'Calculator', icon: '🔢', color: '#3b82f6', route: '/app.html?activity=explorer' },
  ];

  // Dock apps matching screenshot exactly: House, Factory, Lambda, Folder, Books, Robot, Monitor, Globe, Grid (9 icons)
  // Added Univarm Starter (⚡) as 10th icon
  const dockApps = [
    { id: 'home', icon: '🏠', name: 'Home', route: '/app.html?activity=workspace', description: 'NAVA OS Desktop - Main Hub' },
    { id: 'factory', icon: '🏭', name: 'Factory', route: '/app.html?activity=simulation', description: 'Simulation & Factory Tools' },
    { id: 'nava', icon: '⋋', name: 'NAVA IDE', route: '/app.html', description: 'Full IDE with Code Editor' },
    { id: 'folder', icon: '📁', name: 'Explorer', route: '/app.html?activity=explorer', description: 'File Explorer' },
    { id: 'books', icon: '📚', name: 'ROS Learning', route: '/app.html?activity=ros-learning', description: 'ROS Learning Center' },
    { id: 'robot', icon: '🤖', name: 'Simulation', route: '/app.html?activity=simulation', description: 'Robot Simulation' },
    { id: 'monitor', icon: '🖥️', name: 'CLI', route: '/app.html?activity=explorer', description: 'Command Line Interface' },
    { id: 'globe', icon: '🌐', name: 'Browser', route: '/app.html', description: 'Web Browser' },
    { id: 'univarm-starter', icon: '⚡', name: 'Univarm ⋋', route: '/app.html?activity=univarm-starter', description: 'Path Optimizer & Code Generator' },
    { id: 'grid', icon: '⊞', name: 'Extensions', route: '/app.html?activity=extensions', description: 'Extensions Manager' },
  ];

  // Debug: Log counts on mount (after arrays are defined)
  useEffect(() => {
    console.log('[OS DESKTOP] ========== COMPONENT MOUNTED ==========');
    console.log(`[OS DESKTOP] Desktop Apps: ${desktopApps.length} (should be 12)`);
    console.log(`[OS DESKTOP] Dock Apps: ${dockApps.length} (should be 9)`);
    console.log(`[OS DESKTOP] Sidebar sections: 6 (Community Projects, My Rosjects, Favorites, Achievements, ROS 2 Learning Center, My Portfolio)`);
    console.log(`[OS DESKTOP] Total sidebar items: 18`);
    console.log(`[OS DESKTOP] Widgets: 5 types (World Clocks, Weather, News, Currency, Stocks)`);
    console.log('[OS DESKTOP] All features should be visible!');
    console.log('[OS DESKTOP] ======================================');
  }, []);

  const menuItems: { [key: string]: MenuItem[] } = {
    file: [
      { label: 'New File', route: '/app.html', action: () => console.log('New File') },
      { label: 'New Window', route: '/app.html', action: () => window.open('/app.html', '_blank') },
      { divider: true },
      { label: 'Open File...', route: '/app.html?activity=explorer', action: () => navigate('/app.html?activity=explorer') },
      { label: 'Open Folder...', route: '/app.html?activity=explorer', action: () => navigate('/app.html?activity=explorer') },
      { divider: true },
      { label: 'Save', action: () => console.log('Save') },
      { label: 'Save As...', action: () => console.log('Save As') },
      { divider: true },
      { label: 'Exit', action: () => console.log('Exit') },
    ],
    edit: [
      { label: 'Undo', action: () => console.log('Undo') },
      { label: 'Redo', action: () => console.log('Redo') },
      { divider: true },
      { label: 'Cut', action: () => console.log('Cut') },
      { label: 'Copy', action: () => console.log('Copy') },
      { label: 'Paste', action: () => console.log('Paste') },
      { divider: true },
      { label: 'Find', action: () => console.log('Find') },
      { label: 'Replace', action: () => console.log('Replace') },
    ],
    view: [
      { label: 'Desktop', route: '/app.html?activity=workspace', action: () => navigate('/app.html?activity=workspace') },
      { label: 'Explorer', route: '/app.html?activity=explorer', action: () => navigate('/app.html?activity=explorer') },
      { label: 'Search', route: '/app.html?activity=search', action: () => navigate('/app.html?activity=search') },
      { label: 'Source Control', route: '/app.html?activity=source-control', action: () => navigate('/app.html?activity=source-control') },
      { divider: true },
      { label: 'NAVA IDE', route: '/app.html?activity=explorer', action: () => navigate('/app.html?activity=explorer') },
      { label: 'CLI', route: '/app.html?activity=explorer', action: () => navigate('/app.html?activity=explorer') },
      { label: 'Browser', route: '/app.html', action: () => navigate('/app.html') },
      { divider: true },
      { label: 'Run', route: '/app.html?activity=simulation', action: () => navigate('/app.html?activity=simulation') },
      { label: 'Debug', route: '/app.html?activity=debug', action: () => navigate('/app.html?activity=debug') },
      { divider: true },
      { label: 'Extensions', route: '/app.html?activity=extensions', action: () => navigate('/app.html?activity=extensions') },
      { label: 'Settings', action: () => console.log('Settings') },
    ],
    window: [
      { label: 'New Window', action: () => window.open('/app.html', '_blank') },
      { label: 'Close Window', action: () => console.log('Close Window') },
      { divider: true },
      { label: 'Minimize', action: () => console.log('Minimize') },
      { label: 'Zoom', action: () => console.log('Zoom') },
      { divider: true },
      { label: 'Workspace', route: '/app.html?activity=workspace', action: () => navigate('/app.html?activity=workspace') },
      { label: 'IDE', route: '/app.html', action: () => navigate('/app.html') },
    ],
    help: [
      { label: 'Welcome', route: '/app.html?activity=ros-learning', action: () => navigate('/app.html?activity=ros-learning') },
      { label: 'Documentation', route: '/docs.html', action: () => navigate('/docs.html') },
      { label: 'Quick Start Guide', route: '/app.html?activity=ros-learning', action: () => navigate('/app.html?activity=ros-learning') },
      { label: 'API Reference', route: '/docs.html', action: () => navigate('/docs.html') },
      { divider: true },
      { label: 'Keyboard Shortcuts', action: () => console.log('Keyboard Shortcuts') },
      { divider: true },
      { label: 'About NAVA OS', action: () => console.log('About NAVA OS') },
    ],
  };

  const navigate = (route: string) => {
    console.log(`[NAVIGATE] Function called with route: ${route}`);
    console.log(`[NAVIGATE] Current location: ${window.location.href}`);
    
    if (!route) {
      console.error('[NAVIGATE] No route provided!');
      return;
    }
    
    if (route.startsWith('http')) {
      console.log('[NAVIGATE] External URL, opening in new tab');
      window.open(route, '_blank');
      return;
    }
    
    // Ensure route starts with / for absolute paths
    const absoluteRoute = route.startsWith('/') ? route : `/${route}`;
    const fullUrl = `${window.location.origin}${absoluteRoute}`;
    console.log(`[NAVIGATE] Absolute route: ${absoluteRoute}`);
    console.log(`[NAVIGATE] Full URL: ${fullUrl}`);
    
    // Extract activity from route to update localStorage before navigation
    const urlParams = new URLSearchParams(absoluteRoute.split('?')[1] || '');
    const activity = urlParams.get('activity');
    if (activity) {
      console.log(`[NAVIGATE] Updating localStorage with activity: ${activity}`);
      localStorage.setItem('navlambda-active-activity', activity);
    } else if (absoluteRoute === '/app.html') {
      // No activity specified, clear it
      console.log(`[NAVIGATE] No activity in route, clearing localStorage`);
      localStorage.removeItem('navlambda-active-activity');
    }
    
    // Check current activity vs target activity
    const currentActivity = new URLSearchParams(window.location.search).get('activity') || '';
    const targetActivity = activity || '';
    
    console.log(`[NAVIGATE] Current activity: "${currentActivity}", Target activity: "${targetActivity}"`);
    
    // Only skip if we're on the exact same route AND same activity
    const currentUrl = window.location.href;
    if (currentUrl === fullUrl && currentActivity === targetActivity) {
      console.log(`[NAVIGATE] Already on this exact route with same activity, skipping navigation`);
      // Even if same, force a reload to ensure React picks up the state
      console.log(`[NAVIGATE] Forcing reload to ensure state sync`);
      window.location.reload();
      return;
    }
    
    // Always navigate - force navigation to ensure React picks up the change
    console.log(`[NAVIGATE] Navigating to: ${absoluteRoute}`);
    console.log(`[NAVIGATE] Activity change: ${currentActivity} → ${targetActivity}`);
    
    // Use window.location.href for navigation
    try {
      window.location.href = absoluteRoute;
    } catch (error) {
      console.error('[NAVIGATE] Error during navigation:', error);
      // Fallback
      window.location.assign(absoluteRoute);
    }
  };

  const handleAppClick = (app: AppIcon) => {
    setSelectedApp(app.id);
    if (app.route) {
      navigate(app.route);
    } else if (app.onClick) {
      app.onClick();
    }
  };

  const handleDockClick = (app: typeof dockApps[0], event?: React.MouseEvent | React.KeyboardEvent) => {
    console.log(`[DOCK CLICK] ========== DOCK ICON CLICKED ==========`);
    console.log(`[DOCK CLICK] App: ${app.name}`);
    console.log(`[DOCK CLICK] Route: ${app.route}`);
    console.log(`[DOCK CLICK] Event:`, event);
    
    if (event) {
      event.preventDefault();
      if ('stopPropagation' in event) {
        event.stopPropagation();
      }
      console.log(`[DOCK CLICK] Event prevented and stopped`);
    }
    
    console.log(`[DOCK CLICK] Setting active dock app to: ${app.id}`);
    setActiveDockApp(app.id);
    
    if (app.route) {
      console.log(`[DOCK CLICK] Route exists, calling navigate...`);
      console.log(`[DOCK CLICK] Navigate will be called with: ${app.route}`);
      // Navigate immediately - no delay
      navigate(app.route);
    } else {
      console.error(`[DOCK CLICK] ERROR: No route defined for dock app: ${app.name}`);
    }
  };

  // Set active dock app based on current route
  const updateActiveDockApp = () => {
    const currentPath = window.location.pathname;
    const currentActivity = new URLSearchParams(window.location.search).get('activity') || '';
    
    // Determine active dock app based on current route
    if (currentPath.includes('app.html')) {
      if (currentActivity === 'workspace' || !currentActivity) {
        setActiveDockApp('home');
      } else if (currentActivity === 'simulation') {
        setActiveDockApp('factory');
      } else if (currentActivity === 'explorer') {
        setActiveDockApp('folder');
      } else if (currentActivity === 'ros-learning') {
        setActiveDockApp('books');
      } else if (currentActivity === 'extensions') {
        setActiveDockApp('grid');
      } else {
        setActiveDockApp('nava');
      }
    } else if (currentPath.includes('download.html')) {
      setActiveDockApp('globe'); // Downloads page -> Globe icon
    } else {
      setActiveDockApp('home');
    }
  };

  useEffect(() => {
    updateActiveDockApp();
    
    // Listen for route changes (popstate for back/forward, custom event for programmatic navigation)
    const handleRouteChange = () => {
      setTimeout(updateActiveDockApp, 100);
    };
    
    window.addEventListener('popstate', handleRouteChange);
    window.addEventListener('hashchange', handleRouteChange);
    
    // Also check periodically in case navigation happens without events
    const interval = setInterval(updateActiveDockApp, 500);
    
    return () => {
      window.removeEventListener('popstate', handleRouteChange);
      window.removeEventListener('hashchange', handleRouteChange);
      clearInterval(interval);
    };
  }, []);

  const handleSidebarClick = (item: string) => {
    console.log(`[SIDEBAR] ========== SIDEBAR ITEM CLICKED ==========`);
    console.log(`[SIDEBAR] Item: ${item}`);
    
    const routes: { [key: string]: string } = {
      // COMMUNITY PROJECTS
      'Community Projects': '/app.html?activity=explorer',
      'ROS 2 Tutorials': '/app.html?activity=ros-learning',
      'Navigation Demos': '/app.html?activity=simulation',
      // MY ROSJECTS
      'Navigation Bot': '/app.html?activity=simulation',
      'SLAM Mapping': '/app.html?activity=simulation',
      'Path Planning': '/app.html?activity=simulation',
      'Sensor Fusion': '/app.html?activity=simulation',
      // FAVORITES
      'Navigation Guide': '/app.html?activity=ros-learning',
      'ROS2 Cheat Sheet': '/app.html?activity=ros-learning',
      'Calculus Reference': '/app.html?activity=explorer',
      // ACHIEVEMENTS
      'ROS 2 Master': '/app.html?activity=ros-learning',
      'Navigation Expert': '/app.html?activity=simulation',
      'Code Ninja': '/app.html?activity=explorer',
      // ROS 2 LEARNING CENTER
      'Interactive Courses': '/app.html?activity=ros-learning',
      'Robot Simulator': '/app.html?activity=simulation',
      'ROS Workspace': '/app.html?activity=ros-learning',
      // MY PORTFOLIO
      'Projects': '/app.html?activity=explorer',
      'Certifications': '/app.html?activity=ros-learning',
      // LEGACY (for backward compatibility)
      'NAVA Desktop': '/app.html?activity=workspace',
      'AI Models': '/app.html?activity=simulation',
      'Portfolio': '/app.html?activity=explorer',
      'Experiments': '/app.html?activity=simulation',
      'README.md': '/app.html?activity=explorer',
      'Quick Start Guide': '/app.html?activity=ros-learning',
      'API Documentation': '/docs.html',
      'Downloads': '/download.html',
      'Desktop': '/app.html?activity=workspace',
      'Applications': '/app.html?activity=extensions',
    };
    
    const route = routes[item];
    if (route) {
      console.log(`[SIDEBAR] Route found: ${item} -> ${route}`);
      console.log(`[SIDEBAR] Calling navigate with: ${route}`);
      navigate(route);
    } else {
      console.error(`[SIDEBAR] ERROR: No route defined for: ${item}`);
      console.warn(`[SIDEBAR] Available routes:`, Object.keys(routes));
    }
  };

  const handleMenuClick = (menuName: string) => {
    setActiveMenu(activeMenu === menuName ? null : menuName);
  };

  const handleDownloadInterface = () => {
    navigate('/download.html');
  };

  // Close dropdowns when clicking outside
  useEffect(() => {
    const handleClickOutside = (event: MouseEvent) => {
      if (activeMenu && menuRefs.current[activeMenu]) {
        const menuElement = menuRefs.current[activeMenu];
        if (menuElement && !menuElement.contains(event.target as Node)) {
          setActiveMenu(null);
        }
      }
    };

    document.addEventListener('mousedown', handleClickOutside);
    return () => document.removeEventListener('mousedown', handleClickOutside);
  }, [activeMenu]);

  return (
    <div className="os-desktop">
      {/* Top Menu Bar */}
      <div className="desktop-top-bar">
        <div className="desktop-menu">
          {Object.keys(menuItems).map((menuName) => (
            <div key={menuName} className="menu-item-container" ref={(el) => (menuRefs.current[menuName] = el)}>
              <span
                className={`menu-item ${activeMenu === menuName ? 'active' : ''}`}
                onClick={() => handleMenuClick(menuName)}
              >
                {menuName.charAt(0).toUpperCase() + menuName.slice(1)}
              </span>
              {activeMenu === menuName && (
                <div className="menu-dropdown">
                  {menuItems[menuName].map((item, index) => (
                    <React.Fragment key={index}>
                      {item.divider ? (
                        <div className="menu-divider" />
                      ) : (
                        <div
                          className="menu-dropdown-item"
                          onClick={() => {
                            if (item.route) {
                              navigate(item.route);
                            } else if (item.action) {
                              item.action();
                            }
                            setActiveMenu(null);
                          }}
                        >
                          {item.label}
                        </div>
                      )}
                    </React.Fragment>
                  ))}
                </div>
              )}
            </div>
          ))}
        </div>
        <div className="desktop-logo">
          <span className="logo-icon">⋋</span>
          <div className="logo-text">
            <div className="logo-main">NAVA OS</div>
            <div className="logo-subtitle">THE FUTURE OF AI DESKTOP COMPUTING</div>
          </div>
        </div>
        <button className="download-interface-btn" onClick={handleDownloadInterface}>
          Download Interface
        </button>
      </div>

      {/* Main Desktop Content */}
      <div className="desktop-content">
        {/* Left Sidebar */}
        <div className="desktop-sidebar-left">
          {/* Community Projects */}
          <div className="sidebar-section">
            <h3 className="sidebar-title">COMMUNITY PROJECTS</h3>
            <div className="sidebar-items">
              <div 
                className="sidebar-item" 
                onClick={(e) => {
                  e.preventDefault();
                  e.stopPropagation();
                  console.log('[SIDEBAR] Click detected on: Community Projects');
                  handleSidebarClick('Community Projects');
                }}
              >
                🌐 Community Projects
              </div>
              <div 
                className="sidebar-item" 
                onClick={(e) => {
                  e.preventDefault();
                  e.stopPropagation();
                  console.log('[SIDEBAR] Click detected on: ROS 2 Tutorials');
                  handleSidebarClick('ROS 2 Tutorials');
                }}
              >
                📚 ROS 2 Tutorials
              </div>
              <div 
                className="sidebar-item" 
                onClick={(e) => {
                  e.preventDefault();
                  e.stopPropagation();
                  console.log('[SIDEBAR] Click detected on: Navigation Demos');
                  handleSidebarClick('Navigation Demos');
                }}
              >
                🕐 Navigation Demos
              </div>
            </div>
          </div>

          {/* My Rosjects */}
          <div className="sidebar-section">
            <h3 className="sidebar-title">MY ROSJECTS</h3>
            <div className="sidebar-items">
              <div 
                className="sidebar-item" 
                onClick={(e) => {
                  e.preventDefault();
                  e.stopPropagation();
                  console.log('[SIDEBAR] Click detected on: Navigation Bot');
                  handleSidebarClick('Navigation Bot');
                }}
              >
                🤖 Navigation Bot
              </div>
              <div 
                className="sidebar-item" 
                onClick={(e) => {
                  e.preventDefault();
                  e.stopPropagation();
                  console.log('[SIDEBAR] Click detected on: SLAM Mapping');
                  handleSidebarClick('SLAM Mapping');
                }}
              >
                🗺️ SLAM Mapping
              </div>
              <div 
                className="sidebar-item" 
                onClick={(e) => {
                  e.preventDefault();
                  e.stopPropagation();
                  console.log('[SIDEBAR] Click detected on: Path Planning');
                  handleSidebarClick('Path Planning');
                }}
              >
                🧭 Path Planning
              </div>
              <div 
                className="sidebar-item" 
                onClick={(e) => {
                  e.preventDefault();
                  e.stopPropagation();
                  console.log('[SIDEBAR] Click detected on: Sensor Fusion');
                  handleSidebarClick('Sensor Fusion');
                }}
              >
                📡 Sensor Fusion
              </div>
            </div>
          </div>

          {/* Favorites */}
          <div className="sidebar-section">
            <h3 className="sidebar-title">⭐ FAVORITES</h3>
            <div className="sidebar-items">
              <div 
                className="sidebar-item" 
                onClick={(e) => {
                  e.preventDefault();
                  e.stopPropagation();
                  console.log('[SIDEBAR] Click detected on: Navigation Guide');
                  handleSidebarClick('Navigation Guide');
                }}
              >
                📄 Navigation Guide
              </div>
              <div 
                className="sidebar-item" 
                onClick={(e) => {
                  e.preventDefault();
                  e.stopPropagation();
                  console.log('[SIDEBAR] Click detected on: ROS2 Cheat Sheet');
                  handleSidebarClick('ROS2 Cheat Sheet');
                }}
              >
                📄 ROS2 Cheat Sheet
              </div>
              <div 
                className="sidebar-item" 
                onClick={(e) => {
                  e.preventDefault();
                  e.stopPropagation();
                  console.log('[SIDEBAR] Click detected on: Calculus Reference');
                  handleSidebarClick('Calculus Reference');
                }}
              >
                📐 Calculus Reference
              </div>
            </div>
          </div>

          {/* Achievements */}
          <div className="sidebar-section">
            <h3 className="sidebar-title">🏆 ACHIEVEMENTS</h3>
            <div className="sidebar-items">
              <div 
                className="sidebar-item" 
                onClick={(e) => {
                  e.preventDefault();
                  e.stopPropagation();
                  console.log('[SIDEBAR] Click detected on: ROS 2 Master');
                  handleSidebarClick('ROS 2 Master');
                }}
              >
                🏅 ROS 2 Master
              </div>
              <div 
                className="sidebar-item" 
                onClick={(e) => {
                  e.preventDefault();
                  e.stopPropagation();
                  console.log('[SIDEBAR] Click detected on: Navigation Expert');
                  handleSidebarClick('Navigation Expert');
                }}
              >
                🧭 Navigation Expert
              </div>
              <div 
                className="sidebar-item" 
                onClick={(e) => {
                  e.preventDefault();
                  e.stopPropagation();
                  console.log('[SIDEBAR] Click detected on: Code Ninja');
                  handleSidebarClick('Code Ninja');
                }}
              >
                🥷 Code Ninja
              </div>
            </div>
          </div>

          {/* ROS 2 Learning Center - Highlighted */}
          <div className="sidebar-section sidebar-section-highlighted">
            <h3 className="sidebar-title sidebar-title-highlighted">ROS 2 LEARNING CENTER</h3>
            <div className="sidebar-items">
              <div 
                className="sidebar-item" 
                onClick={(e) => {
                  e.preventDefault();
                  e.stopPropagation();
                  console.log('[SIDEBAR] Click detected on: Interactive Courses');
                  handleSidebarClick('Interactive Courses');
                }}
              >
                📚 Interactive Courses
              </div>
              <div 
                className="sidebar-item" 
                onClick={(e) => {
                  e.preventDefault();
                  e.stopPropagation();
                  console.log('[SIDEBAR] Click detected on: Robot Simulator');
                  handleSidebarClick('Robot Simulator');
                }}
              >
                🤖 Robot Simulator
              </div>
              <div 
                className="sidebar-item" 
                onClick={(e) => {
                  e.preventDefault();
                  e.stopPropagation();
                  console.log('[SIDEBAR] Click detected on: ROS Workspace');
                  handleSidebarClick('ROS Workspace');
                }}
              >
                ⚙️ ROS Workspace
              </div>
            </div>
          </div>

          {/* My Portfolio */}
          <div className="sidebar-section">
            <h3 className="sidebar-title">MY PORTFOLIO</h3>
            <div className="sidebar-items">
              <div 
                className="sidebar-item" 
                onClick={(e) => {
                  e.preventDefault();
                  e.stopPropagation();
                  console.log('[SIDEBAR] Click detected on: Projects');
                  handleSidebarClick('Projects');
                }}
              >
                📁 Projects
              </div>
              <div 
                className="sidebar-item" 
                onClick={(e) => {
                  e.preventDefault();
                  e.stopPropagation();
                  console.log('[SIDEBAR] Click detected on: Certifications');
                  handleSidebarClick('Certifications');
                }}
              >
                🏆 Certifications
              </div>
            </div>
          </div>
        </div>

        {/* Main Desktop Area */}
        <div className="desktop-main-area">
          {/* Window Manager - Handles draggable windows */}
          <div className="window-manager-container" style={{ position: 'absolute', inset: 0, zIndex: 100 }}>
            <WindowManager />
          </div>
          
          {/* The NAVA Navigation Institute Branding */}
          <div className="desktop-branding">
            <div className="branding-logo">⋋</div>
            <div className="branding-text">
              <h1 className="branding-title">The NAVA</h1>
              <p className="branding-subtitle">NAVIGATION INSTITUTE</p>
            </div>
          </div>

          {/* Blueprint Canvas Control Bar */}
          <div className="blueprint-control-bar">
            <div className="control-bar-left">
              <button className="control-btn record-btn" title="Record">
                <span className="control-dot"></span>
                <span>Record</span>
              </button>
              <button className="control-btn" title="Camera">
                📷 Camera
              </button>
              <button className="control-btn" title="Fullscreen">
                ⛶ Fullscreen
              </button>
            </div>
            <div className="control-bar-tabs">
              <button className="tab-btn active">Blueprint</button>
              <button className="tab-btn">
                <span className="tab-dot"></span>
                Live Ad
              </button>
              <button className="tab-btn">YouTube</button>
              <button className="tab-btn">Twitch</button>
              <button className="tab-btn">Video</button>
              <button className="tab-btn">Image</button>
              <button className="tab-btn">Web</button>
              <button className="tab-btn">Slides</button>
            </div>
          </div>

          {/* Blueprint Canvas/Grid Area */}
          <div className="blueprint-canvas">
            <div className="canvas-grid">
              {/* Grid background with faint NAVA logo */}
              <div className="grid-background">
                <div className="grid-logo">⋋</div>
              </div>
              {/* Grid lines and coordinate system */}
              <svg className="grid-svg" viewBox="0 0 800 600">
                <defs>
                  <pattern id="grid" width="40" height="40" patternUnits="userSpaceOnUse">
                    <path d="M 40 0 L 0 0 0 40" fill="none" stroke="rgba(59, 130, 246, 0.2)" strokeWidth="1"/>
                  </pattern>
                </defs>
                <rect width="100%" height="100%" fill="url(#grid)" />
                {/* Coordinate axes */}
                <line x1="50" y1="550" x2="750" y2="550" stroke="rgba(59, 130, 246, 0.5)" strokeWidth="2" />
                <line x1="50" y1="550" x2="50" y2="50" stroke="rgba(59, 130, 246, 0.5)" strokeWidth="2" />
                {/* Sample path visualization */}
                <path d="M 100 500 Q 350 300 600 500" fill="none" stroke="#22c55e" strokeWidth="3" />
                <circle cx="100" cy="500" r="5" fill="#22c55e" />
                <circle cx="350" cy="300" r="5" fill="#3b82f6" />
                <circle cx="600" cy="500" r="5" fill="#22c55e" />
              </svg>
            </div>
          </div>

          {/* Desktop App Icons Grid (Alternative view - can be toggled) */}
          <div className="desktop-apps-grid" style={{ display: 'none' }}>
            {desktopApps.map((app) => (
              <div
                key={app.id}
                className={`desktop-app-icon ${selectedApp === app.id ? 'selected' : ''}`}
                onClick={() => handleAppClick(app)}
                style={{ borderColor: app.color }}
                title={app.name}
              >
                <div className="app-icon-image" style={{ color: app.color }}>
                  {app.icon}
                </div>
                <div className="app-icon-name">{app.name}</div>
              </div>
            ))}
          </div>
        </div>

        {/* Right Sidebar - Widgets */}
        <div className="desktop-sidebar-right">
          <WorldWidgets />
        </div>
      </div>

      {/* Bottom Dock - 9 icons */}
      <div className="desktop-dock" style={{ display: 'flex', visibility: 'visible', opacity: 1 }}>
        {dockApps.map((app, index) => {
          console.log(`[OS DESKTOP] Rendering dock icon ${index + 1}/${dockApps.length}: ${app.name}`);
          return (
            <div
              key={app.id}
              className={`dock-icon ${activeDockApp === app.id ? 'active' : ''} ${hoveredDockApp === app.id ? 'hovered' : ''}`}
              title={app.name}
              style={{ display: 'flex', visibility: 'visible', opacity: 1, cursor: 'pointer' }}
              onClick={(e) => {
                console.log(`[DOCK] Click detected on: ${app.name}`);
                handleDockClick(app, e);
              }}
              onMouseDown={(e) => {
                console.log(`[DOCK] MouseDown on: ${app.name}`);
              }}
              onMouseEnter={() => {
                setHoveredDockApp(app.id);
                console.log(`[DOCK] MouseEnter: ${app.name}`);
              }}
              onMouseLeave={() => {
                setHoveredDockApp(null);
              }}
              role="button"
              tabIndex={0}
              onKeyDown={(e) => {
                if (e.key === 'Enter' || e.key === ' ') {
                  e.preventDefault();
                  console.log(`[DOCK] Keyboard activation: ${app.name}`);
                  handleDockClick(app, e);
                }
              }}
            >
            <div className="dock-icon-badge">
              <span className="dock-icon-emoji">{app.icon}</span>
            </div>
            {activeDockApp === app.id && <div className="dock-active-indicator" />}
            <div className="dock-tooltip">{app.name}</div>
          </div>
          );
        })}
      </div>

      {/* Bottom Status Bar */}
      <div className="desktop-status-bar">
        <div className="status-bar-left">
          <span className="status-icon">💡</span>
          <span className="status-icon">⚙️</span>
        </div>
        <div className="status-bar-center">
          <span className="status-time">{new Date().toLocaleTimeString('en-US', { hour: '2-digit', minute: '2-digit' })}</span>
          <span className="status-date">{new Date().toLocaleDateString('en-GB', { day: '2-digit', month: '2-digit', year: 'numeric' })}</span>
        </div>
        <div className="status-bar-right">
          <span className="status-language">EN</span>
          <span className="status-icon">⚡</span>
        </div>
      </div>

      {/* Blur Overlay for modals */}
      <BlurOverlay />
      
      {/* CI Indicator Widget */}
      <CIIndicatorWidget />
    </div>
  );
};

export default OSDesktop;

