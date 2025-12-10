/**
 * NAVΛ OS Desktop Workspace
 * 
 * The complete OS desktop workspace matching the screenshots:
 * - NAVΛ OS branding and app icons
 * - Left sidebar with navigation
 * - Right sidebar with widgets (Weather, World Clock, News, Currency, Stocks)
 * - Bottom dock
 * - Top menu bar with dropdown menus
 */

import React, { useState, useRef, useEffect, useCallback } from 'react';
import { WorldWidgets } from './WorldWidgets';
import { WindowManager } from '../WindowManager';
import { BlurOverlay } from '../WindowManager';
import { MediaCentre } from '../MediaCentre/MediaCentre';
import { FinderSidebar } from './FinderSidebar';
import { DesktopBranding } from './DesktopBranding';
import { TrashBin } from '../TrashBin/TrashBin';
import { DownloadsStack } from './DownloadsStack';
import { DockContextMenu, type DockAppPreferences } from './DockContextMenu';
import { IDEDownloadButton } from './IDEDownloadButton';
import { initializeBrowserCompatibility, safeLocalStorage, safeWindowOpen } from '../../utils/browser-compatibility';
import { hasTrashItems } from '../../utils/trashStorage';
import { removeFromDock } from '../../utils/appStorage';
import { getDownloadCount } from '../../utils/downloadStorage';
import './OSDesktop.css';

// CIIndicatorWidget - Removed per user request

interface AppIcon {
  id: string;
  name: string;
  icon: string;
  color?: string;
  route?: string | null;
  onClick?: () => void;
}

interface DockApp {
  id: string;
  icon: string;
  name: string;
  route?: string | null;
  description: string;
  onClick?: () => void;
}

interface MenuItem {
  label?: string;
  action?: () => void;
  route?: string;
  divider?: boolean;
  submenu?: MenuItem[];
  shortcut?: string;
}

// Branding Controls Panel Component
const BrandingControlsPanel: React.FC<{ onClose: () => void; onUpdate: () => void }> = ({ onClose, onUpdate }) => {
  const [elements, setElements] = useState<{ [key: string]: { visible: boolean } }>(() => {
    try {
      const saved = localStorage.getItem('nava-desktop-branding');
      if (saved) {
        const parsed = JSON.parse(saved);
        return {
          logo: { visible: parsed.logo?.visible !== false },
          career: { visible: parsed.career?.visible !== false },
          display: { visible: parsed.display?.visible !== false },
        };
      }
    } catch {}
    return {
      logo: { visible: true },
      career: { visible: true },
      display: { visible: true },
    };
  });

  const videoOptions = [
    { url: 'https://www.youtube.com/embed/Eu5mYMavctM', label: 'Figure 03', id: 'Eu5mYMavctM' },
    { url: 'https://www.youtube.com/embed/W1Ftle-w8HQ', label: 'Tesla Optimus Gen 3', id: 'W1Ftle-w8HQ' },
    { url: 'https://www.youtube.com/embed/6c8M0_EYa1c', label: 'Unitree Robots Full Lineup', id: '6c8M0_EYa1c' },
  ];

  const [displaySettings, setDisplaySettings] = useState<{
    videoUrl: string;
    autoplay: boolean;
    loop: boolean;
    muted: boolean;
    showControls: boolean;
  }>(() => {
    try {
      const saved = localStorage.getItem('nava-display-settings');
      if (saved) {
        return JSON.parse(saved);
      }
    } catch {}
    return {
      videoUrl: 'https://www.youtube.com/embed/Eu5mYMavctM',
      autoplay: true,
      loop: true,
      muted: false,
      showControls: true,
    };
  });

  const toggleElement = (id: string) => {
    try {
      const saved = localStorage.getItem('nava-desktop-branding');
      const parsed = saved ? JSON.parse(saved) : {};
      
      if (!parsed[id]) {
        // Create default element if it doesn't exist
        const width = window.innerWidth;
        const height = window.innerHeight;
        if (id === 'logo') {
          parsed[id] = { id, x: 50, y: 80, visible: true };
        } else if (id === 'career') {
          parsed[id] = { id, x: Math.max(50, width - 400), y: Math.max(80, height - 200), visible: true };
        } else if (id === 'display') {
          parsed[id] = { id, x: Math.max(50, width / 2 - 400), y: Math.max(80, height / 2 - 225), visible: true };
        }
      }
      
      parsed[id].visible = !parsed[id].visible;
      localStorage.setItem('nava-desktop-branding', JSON.stringify(parsed));
      
      setElements({
        logo: { visible: parsed.logo?.visible !== false },
        career: { visible: parsed.career?.visible !== false },
        display: { visible: parsed.display?.visible !== false },
      });
      
      // Dispatch custom event to update DesktopBranding
      window.dispatchEvent(new CustomEvent('nava:branding-update'));
      
      // Trigger update in parent
      onUpdate();
    } catch (error) {
      console.error('Error toggling element:', error);
    }
  };

  const resetPositions = () => {
    const width = window.innerWidth;
    const height = window.innerHeight;
    const defaultElements = {
      logo: { id: 'logo', x: 50, y: 80, visible: true },
      career: { id: 'career', x: Math.max(50, width - 400), y: Math.max(80, height - 200), visible: true },
      display: { id: 'display', x: Math.max(50, width / 2 - 400), y: Math.max(80, height / 2 - 225), visible: true },
    };
    localStorage.setItem('nava-desktop-branding', JSON.stringify(defaultElements));
    setElements({
      logo: { visible: true },
      career: { visible: true },
      display: { visible: true },
    });
    
    // Dispatch custom event to update DesktopBranding
    window.dispatchEvent(new CustomEvent('nava:branding-update'));
    
    onUpdate();
  };

  return (
    <div className="branding-controls-panel">
      <div className="branding-controls-content">
        <div className="branding-controls-header">
          <h3>Branding Controls</h3>
          <button 
            className="branding-controls-close"
            onClick={onClose}
          >
            ×
          </button>
        </div>
        <div className="branding-controls-body">
          <p style={{ color: '#94a3b8', fontSize: '13px', marginBottom: '16px' }}>
            Drag elements on the desktop to reposition them. Use these controls to show/hide elements.
          </p>
          <div className="branding-control-item">
            <label>
              <input 
                type="checkbox" 
                checked={elements.logo.visible}
                onChange={() => toggleElement('logo')}
              />
              <span>NAVA Navigation Institute Logo</span>
            </label>
          </div>
          <div className="branding-control-item">
            <label>
              <input 
                type="checkbox" 
                checked={elements.career.visible}
                onChange={() => toggleElement('career')}
              />
              <span>Career Happens Text</span>
            </label>
          </div>
          <div className="branding-control-item">
            <label>
              <input 
                type="checkbox" 
                checked={elements.display.visible}
                onChange={() => toggleElement('display')}
              />
              <span>Display Screen (16:9)</span>
            </label>
          </div>

          {/* Video Controls Section */}
          {elements.display.visible && (
            <div style={{ marginTop: '20px', paddingTop: '16px', borderTop: '1px solid rgba(59, 130, 246, 0.2)' }}>
              <h4 style={{ color: '#ffffff', fontSize: '14px', fontWeight: '600', marginBottom: '12px' }}>
                Video Settings
              </h4>
              
              <div className="branding-control-item" style={{ marginBottom: '12px' }}>
                <label style={{ display: 'flex', flexDirection: 'column', gap: '6px' }}>
                  <span style={{ color: '#e2e8f0', fontSize: '13px' }}>Select Video</span>
                  <select
                    value={displaySettings.videoUrl}
                    onChange={(e) => {
                      const newSettings = { ...displaySettings, videoUrl: e.target.value };
                      setDisplaySettings(newSettings);
                      localStorage.setItem('nava-display-settings', JSON.stringify(newSettings));
                      window.dispatchEvent(new CustomEvent('nava:display-update'));
                    }}
                    style={{
                      padding: '8px 12px',
                      background: 'rgba(15, 23, 42, 0.6)',
                      border: '1px solid rgba(59, 130, 246, 0.3)',
                      borderRadius: '6px',
                      color: '#ffffff',
                      fontSize: '12px',
                      cursor: 'pointer',
                    }}
                  >
                    {videoOptions.map((option) => (
                      <option key={option.id} value={option.url}>
                        {option.label}
                      </option>
                    ))}
                  </select>
                </label>
              </div>

              <div className="branding-control-item" style={{ marginBottom: '12px' }}>
                <label style={{ display: 'flex', flexDirection: 'column', gap: '6px' }}>
                  <span style={{ color: '#e2e8f0', fontSize: '13px' }}>Or Enter Custom YouTube URL</span>
                  <input
                    type="text"
                    value={displaySettings.videoUrl}
                    onChange={(e) => {
                      const newSettings = { ...displaySettings, videoUrl: e.target.value };
                      setDisplaySettings(newSettings);
                      localStorage.setItem('nava-display-settings', JSON.stringify(newSettings));
                      window.dispatchEvent(new CustomEvent('nava:display-update'));
                    }}
                    placeholder="https://www.youtube.com/embed/..."
                    style={{
                      padding: '8px 12px',
                      background: 'rgba(15, 23, 42, 0.6)',
                      border: '1px solid rgba(59, 130, 246, 0.3)',
                      borderRadius: '6px',
                      color: '#ffffff',
                      fontSize: '12px',
                    }}
                  />
                </label>
              </div>

              <div className="branding-control-item">
                <label>
                  <input 
                    type="checkbox" 
                    checked={displaySettings.autoplay}
                    onChange={(e) => {
                      const newSettings = { ...displaySettings, autoplay: e.target.checked };
                      setDisplaySettings(newSettings);
                      localStorage.setItem('nava-display-settings', JSON.stringify(newSettings));
                      window.dispatchEvent(new CustomEvent('nava:display-update'));
                    }}
                  />
                  <span>Autoplay</span>
                </label>
              </div>

              <div className="branding-control-item">
                <label>
                  <input 
                    type="checkbox" 
                    checked={displaySettings.loop}
                    onChange={(e) => {
                      const newSettings = { ...displaySettings, loop: e.target.checked };
                      setDisplaySettings(newSettings);
                      localStorage.setItem('nava-display-settings', JSON.stringify(newSettings));
                      window.dispatchEvent(new CustomEvent('nava:display-update'));
                    }}
                  />
                  <span>Loop</span>
                </label>
              </div>

              <div className="branding-control-item">
                <label>
                  <input 
                    type="checkbox" 
                    checked={displaySettings.muted}
                    onChange={(e) => {
                      const newSettings = { ...displaySettings, muted: e.target.checked };
                      setDisplaySettings(newSettings);
                      localStorage.setItem('nava-display-settings', JSON.stringify(newSettings));
                      window.dispatchEvent(new CustomEvent('nava:display-update'));
                    }}
                  />
                  <span>Muted</span>
                </label>
              </div>

              <div className="branding-control-item">
                <label>
                  <input 
                    type="checkbox" 
                    checked={displaySettings.showControls}
                    onChange={(e) => {
                      const newSettings = { ...displaySettings, showControls: e.target.checked };
                      setDisplaySettings(newSettings);
                      localStorage.setItem('nava-display-settings', JSON.stringify(newSettings));
                      window.dispatchEvent(new CustomEvent('nava:display-update'));
                    }}
                  />
                  <span>Show Video Controls</span>
                </label>
              </div>
            </div>
          )}

          {/* Picture-in-Picture Controls Section */}
          {elements.display.visible && (
            <div style={{ marginTop: '20px', paddingTop: '16px', borderTop: '1px solid rgba(59, 130, 246, 0.2)' }}>
              <h4 style={{ color: '#ffffff', fontSize: '14px', fontWeight: '600', marginBottom: '12px' }}>
                Picture-in-Picture Settings
              </h4>
              
              <div className="branding-control-item">
                <label>
                  <input 
                    type="checkbox" 
                    checked={(() => {
                      try {
                        const saved = localStorage.getItem('nava-pip-settings');
                        return saved ? JSON.parse(saved).enabled : false;
                      } catch {
                        return false;
                      }
                    })()}
                    onChange={(e) => {
                      try {
                        const saved = localStorage.getItem('nava-pip-settings');
                        const pip = saved ? JSON.parse(saved) : {
                          enabled: false,
                          videoUrl: 'https://www.youtube.com/embed/Eu5mYMavctM',
                          x: window.innerWidth - 420,
                          y: 100,
                          width: 400,
                          height: 225,
                          autoplay: true,
                          loop: true,
                          muted: false,
                          showControls: true,
                        };
                        pip.enabled = e.target.checked;
                        localStorage.setItem('nava-pip-settings', JSON.stringify(pip));
                        window.dispatchEvent(new CustomEvent('nava:display-update'));
                        window.location.reload();
                      } catch {}
                    }}
                  />
                  <span>Enable Picture-in-Picture</span>
                </label>
              </div>

              {(() => {
                try {
                  const saved = localStorage.getItem('nava-pip-settings');
                  const pip = saved ? JSON.parse(saved) : null;
                  if (!pip || !pip.enabled) return null;
                  
                  return (
                    <>
                      <div className="branding-control-item" style={{ marginTop: '12px', marginBottom: '12px' }}>
                        <label style={{ display: 'flex', flexDirection: 'column', gap: '6px' }}>
                          <span style={{ color: '#e2e8f0', fontSize: '13px' }}>PiP Video</span>
                          <select
                            value={pip.videoUrl || 'https://www.youtube.com/embed/Eu5mYMavctM'}
                            onChange={(e) => {
                              pip.videoUrl = e.target.value;
                              localStorage.setItem('nava-pip-settings', JSON.stringify(pip));
                              window.dispatchEvent(new CustomEvent('nava:display-update'));
                              window.location.reload();
                            }}
                            style={{
                              padding: '8px 12px',
                              background: 'rgba(15, 23, 42, 0.6)',
                              border: '1px solid rgba(59, 130, 246, 0.3)',
                              borderRadius: '6px',
                              color: '#ffffff',
                              fontSize: '12px',
                              cursor: 'pointer',
                            }}
                          >
                            {videoOptions.map((option) => (
                              <option key={option.id} value={option.url}>
                                {option.label}
                              </option>
                            ))}
                          </select>
                        </label>
                      </div>

                      <div className="branding-control-item">
                        <label>
                          <input 
                            type="checkbox" 
                            checked={pip.autoplay !== false}
                            onChange={(e) => {
                              pip.autoplay = e.target.checked;
                              localStorage.setItem('nava-pip-settings', JSON.stringify(pip));
                              window.dispatchEvent(new CustomEvent('nava:display-update'));
                              window.location.reload();
                            }}
                          />
                          <span>PiP Autoplay</span>
                        </label>
                      </div>

                      <div className="branding-control-item">
                        <label>
                          <input 
                            type="checkbox" 
                            checked={pip.muted !== false}
                            onChange={(e) => {
                              pip.muted = e.target.checked;
                              localStorage.setItem('nava-pip-settings', JSON.stringify(pip));
                              window.dispatchEvent(new CustomEvent('nava:display-update'));
                              window.location.reload();
                            }}
                          />
                          <span>PiP Muted</span>
                        </label>
                      </div>
                    </>
                  );
                } catch {
                  return null;
                }
              })()}
            </div>
          )}

          <div style={{ marginTop: '20px', paddingTop: '16px', borderTop: '1px solid rgba(59, 130, 246, 0.2)' }}>
            <button
              onClick={resetPositions}
              style={{
                width: '100%',
                padding: '10px 16px',
                background: 'rgba(59, 130, 246, 0.2)',
                border: '1px solid rgba(59, 130, 246, 0.4)',
                borderRadius: '8px',
                color: '#60a5fa',
                fontSize: '13px',
                fontWeight: '500',
                cursor: 'pointer',
                transition: 'all 0.2s',
              }}
              onMouseEnter={(e) => {
                e.currentTarget.style.background = 'rgba(59, 130, 246, 0.3)';
              }}
              onMouseLeave={(e) => {
                e.currentTarget.style.background = 'rgba(59, 130, 246, 0.2)';
              }}
            >
              Reset Positions
            </button>
          </div>
        </div>
      </div>
    </div>
  );
};

export const OSDesktop: React.FC = () => {
  const [selectedApp, setSelectedApp] = useState<string | null>(null);
  const [activeMenu, setActiveMenu] = useState<string | null>(null);
  const [activeDockApp, setActiveDockApp] = useState<string | null>(null);
  const [hoveredDockApp, setHoveredDockApp] = useState<string | null>(null);
  const [contextMenu, setContextMenu] = useState<{
    appId: string;
    appName: string;
    position: { x: number; y: number };
  } | null>(null);
  const [dockPreferences, setDockPreferences] = useState<Map<string, DockAppPreferences>>(new Map());
  const [showNavaMenu, setShowNavaMenu] = useState(false);
  const [hoveredSubmenu, setHoveredSubmenu] = useState<string | null>(null);
  // LOCKED: NAVΛ RS1 overlay state - always available
  const [showRobotisOverlay, setShowRobotisOverlay] = useState(false);
  
  // LOCKED: NAVΛ RS1 open handler - guaranteed to work
  const handleOpenNavARS1 = useCallback(() => {
    console.log('[NAVΛ RS1] Opening overlay - LOCKED ROUTE');
    setShowRobotisOverlay(true);
    setRobotisError(null);
    setRobotisLoading(true);
    // Force state update
    setTimeout(() => {
      setShowRobotisOverlay(true);
    }, 0);
  }, []);
  const [robotisError, setRobotisError] = useState<string | null>(null);
  const [robotisLoading, setRobotisLoading] = useState(true);
  const [robotisRetryCount, setRobotisRetryCount] = useState(0);
  const robotisCheckIntervalRef = useRef<NodeJS.Timeout | null>(null);
  const [showMediaCentre, setShowMediaCentre] = useState(false);
  const [showBrandingControls, setShowBrandingControls] = useState(false);
  const [showTrashBin, setShowTrashBin] = useState(false);
  const [trashHasItems, setTrashHasItems] = useState(false);
  const [showDownloadsStack, setShowDownloadsStack] = useState(false);
  const [downloadsCount, setDownloadsCount] = useState(0);
  const [rightSidebarCollapsed, setRightSidebarCollapsed] = useState<boolean>(() => {
    try {
      const saved = localStorage.getItem('nava-right-sidebar-collapsed');
      return saved === 'true';
    } catch {
      return false;
    }
  });
  const [leftSidebarCollapsed, setLeftSidebarCollapsed] = useState<boolean>(() => {
    try {
      const saved = localStorage.getItem('nava-left-sidebar-collapsed');
      return saved === 'true';
    } catch {
      return false;
    }
  });
  const [robotisTheme, setRobotisTheme] = useState<'dark' | 'light'>(() => {
    try {
      const saved = localStorage.getItem('robotis-overlay-theme');
      return (saved === 'light' || saved === 'dark') ? saved : 'dark';
    } catch {
      return 'dark';
    }
  });
  const menuRefs = useRef<{ [key: string]: HTMLDivElement | null }>({});

  // Apps are now managed in App Centre - removed from desktop
  // Media apps are now managed in Media Centre

  // Dock apps - Unified with all latest features
  const dockApps: DockApp[] = [
    { id: 'home', icon: '🏠', name: 'Home', route: '/app.html?activity=workspace', description: 'NAVΛ OS Desktop - Main Hub' },
    { id: 'factory', icon: '🏭', name: 'Factory', route: '/app.html?activity=simulation', description: 'Simulation & Factory Tools' },
    { id: 'nava', icon: '⋋', name: 'NAVΛ IDE', route: '/app.html?activity=explorer', description: 'Full IDE with Code Editor & Notebook Support' },
    { id: 'folder', icon: '📁', name: 'Explorer', route: '/app.html?activity=explorer', description: 'File Explorer' },
    { id: 'books', icon: '📚', name: 'ROS Learning', route: '/app.html?activity=ros-learning', description: 'ROS Learning Center' },
    { id: 'robot', icon: '🤖', name: 'Simulation', route: '/app.html?activity=simulation', description: 'Robot Simulation' },
    { 
      id: 'monitor', 
      icon: '🖥️', 
      name: 'CLI', 
      route: '/app.html?activity=explorer', 
      description: 'Command Line Interface',
      onClick: () => {
        // Open terminal/CLI in a new window or overlay
        const event = new CustomEvent('nava:open-terminal');
        window.dispatchEvent(event);
        // Also navigate to explorer for file access
        navigate('/app.html?activity=explorer');
      }
    },
    { id: 'globe', icon: '🌐', name: 'Browser', route: '/app.html', description: 'Web Browser' },
    { id: 'app-centre', icon: '📱', name: 'App Centre', route: '/app.html?activity=app-centre', description: 'NAVΛ App Centre - Manage All Apps' },
    { 
      id: 'robotis-systemic', 
      icon: '🔷', 
      name: 'NAVΛ RS1', 
      route: null, 
      description: 'NAVΛ RS1 Platform', 
      onClick: handleOpenNavARS1 // LOCKED: Always uses the guaranteed handler
    },
    { id: 'univarm-starter', icon: '⚡', name: 'Univarm ⋋', route: '/app.html?activity=univarm-starter', description: 'Path Optimizer & Code Generator' },
    { id: 'univarm-advanced', icon: '🦀', name: 'Univarm Pro', route: '/app.html?activity=univarm-advanced', description: 'Production Path Planning with Rust Backend' },
    { id: 'grid', icon: '⊞', name: 'Extensions', route: '/app.html?activity=extensions', description: 'Extensions Manager' },
    { 
      id: 'downloads', 
      icon: '📥', 
      name: 'Downloads', 
      route: null, 
      description: 'Recent Downloads',
      onClick: () => {
        setShowDownloadsStack(true);
      }
    },
    { 
      id: 'trash', 
      icon: trashHasItems ? '🗑️' : '🗑️', 
      name: 'Trash', 
      route: null, 
      description: trashHasItems ? 'Trash Bin - Contains Items' : 'Trash Bin - Empty',
      onClick: () => {
        setShowTrashBin(true);
      }
    },
  ];

  // Initialize browser compatibility checks
  useEffect(() => {
    initializeBrowserCompatibility();
  }, []);

  // Load dock app preferences
  useEffect(() => {
    try {
      const saved = localStorage.getItem('nava-dock-preferences');
      if (saved) {
        const prefs = JSON.parse(saved);
        const prefsMap = new Map<string, DockAppPreferences>();
        Object.entries(prefs).forEach(([appId, pref]) => {
          prefsMap.set(appId, pref as DockAppPreferences);
        });
        setDockPreferences(prefsMap);
      }
    } catch (error) {
      console.error('[OSDesktop] Error loading dock preferences:', error);
    }
  }, []);

  // Save dock preferences
  const saveDockPreferences = (appId: string, preferences: DockAppPreferences) => {
    const newPrefs = new Map(dockPreferences);
    newPrefs.set(appId, preferences);
    setDockPreferences(newPrefs);
    try {
      const prefsObj: Record<string, DockAppPreferences> = {};
      newPrefs.forEach((pref, id) => {
        prefsObj[id] = pref;
      });
      localStorage.setItem('nava-dock-preferences', JSON.stringify(prefsObj));
    } catch (error) {
      console.error('[OSDesktop] Error saving dock preferences:', error);
    }
  };

  // Get preferences for an app (with defaults)
  const getAppPreferences = (appId: string): DockAppPreferences => {
    return dockPreferences.get(appId) || {
      openAtLogin: false,
      assignToDesktop: 'none',
    };
  };

  // Track downloads count for badge
  useEffect(() => {
    const updateDownloadsCount = () => {
      setDownloadsCount(getDownloadCount());
    };
    updateDownloadsCount();
    
    const handleDownload = () => {
      updateDownloadsCount();
    };
    window.addEventListener('nava:download-complete', handleDownload as EventListener);
    return () => {
      window.removeEventListener('nava:download-complete', handleDownload as EventListener);
    };
  }, []);

  // ROBOTIS Backend Health Check Function
  // Uses no-cors mode to avoid CORS issues - if fetch succeeds, backend is running
  const checkRobotisBackend = useCallback(async (): Promise<boolean> => {
    try {
      // Use no-cors mode to avoid CORS policy issues
      // If the fetch doesn't throw an error, the backend is reachable
      const controller = new AbortController();
      const timeoutId = setTimeout(() => controller.abort(), 2000); // 2 second timeout
      
      try {
        // Use no-cors mode - we can't read the response, but if it doesn't error, server is up
        await fetch('http://localhost:3000', { 
          method: 'HEAD',
          mode: 'no-cors',
          cache: 'no-cache',
          signal: controller.signal
        });
        
        clearTimeout(timeoutId);
        return true; // No error means backend is reachable
      } catch (fetchError: any) {
        clearTimeout(timeoutId);
        // Check if it's a network error (backend down) vs CORS error (backend up but CORS issue)
        // In no-cors mode, CORS errors don't throw - only network errors do
        if (fetchError.name === 'AbortError' || fetchError.message?.includes('Failed to fetch')) {
          return false; // Network error - backend not running
        }
        // If we get here, it might be a different error, but assume backend is down
        return false;
      }
    } catch (error) {
      // Any other error - assume backend not running
      return false;
    }
  }, []);

  // Auto-retry ROBOTIS backend connection with persistent checking
  useEffect(() => {
    if (!showRobotisOverlay) {
      // Clear any running checks when overlay is closed
      if (robotisCheckIntervalRef.current) {
        clearInterval(robotisCheckIntervalRef.current);
        robotisCheckIntervalRef.current = null;
      }
      return;
    }

    // Start checking immediately
    const startChecking = async () => {
      setRobotisLoading(true);
      setRobotisError(null);
      setRobotisRetryCount(0);

      // Check immediately
      const isRunning = await checkRobotisBackend();
      if (isRunning) {
        setRobotisLoading(false);
        setRobotisError(null);
        return; // Backend is running, stop checking
      }

      // If not running, start persistent retry loop (check indefinitely)
      let retryCount = 0;
      const maxRetriesBeforeError = 30; // Show error after 60 seconds (30 * 2 seconds)
      let hasShownError = false;

      robotisCheckIntervalRef.current = setInterval(async () => {
        retryCount++;
        setRobotisRetryCount(retryCount);

        const isRunning = await checkRobotisBackend();
        if (isRunning) {
          // Backend is now running!
          setRobotisLoading(false);
          setRobotisError(null);
          hasShownError = false;
          // Reload iframe to connect
          const iframe = document.querySelector('iframe[title="ROBOTIS-SYSTEMIC Platform"]') as HTMLIFrameElement;
          if (iframe) {
            iframe.src = iframe.src;
          }
          if (robotisCheckIntervalRef.current) {
            clearInterval(robotisCheckIntervalRef.current);
            robotisCheckIntervalRef.current = null;
          }
        } else if (retryCount >= maxRetriesBeforeError && !hasShownError) {
          // Show error after max retries, but keep checking in background
          setRobotisError('The NAVΛ RS1 backend server is not running on http://localhost:3000. The system is automatically trying to start it. Please wait...');
          setRobotisLoading(true); // Keep loading state to show we're still trying
          hasShownError = true;
        }
        // Continue checking indefinitely - don't stop
      }, 2000); // Check every 2 seconds
    };

    startChecking();

    // Cleanup on unmount or when overlay closes
    return () => {
      if (robotisCheckIntervalRef.current) {
        clearInterval(robotisCheckIntervalRef.current);
        robotisCheckIntervalRef.current = null;
      }
    };
  }, [showRobotisOverlay, checkRobotisBackend]);

  // LOCKED: Listen for App Centre events to open NAVΛ RS1 overlay
  useEffect(() => {
    const handleOpenRobotis = () => {
      console.log('[NAVΛ RS1] Received request to open overlay from App Centre - LOCKED ROUTE');
      handleOpenNavARS1();
    };

    window.addEventListener('nava:open-robotis-overlay', handleOpenRobotis);
    return () => {
      window.removeEventListener('nava:open-robotis-overlay', handleOpenRobotis);
    };
  }, [handleOpenNavARS1]);

  // Debug: Log counts on mount (after arrays are defined)
  useEffect(() => {
    console.log('[OS DESKTOP] ========== COMPONENT MOUNTED ==========');
    console.log(`[OS DESKTOP] Dock Apps: ${dockApps.length}`);
    console.log(`[OS DESKTOP] Sidebar sections: 6 (Community Projects, My Rosjects, Favorites, Achievements, ROS 2 Learning Center, My Portfolio)`);
    console.log(`[OS DESKTOP] Total sidebar items: 18`);
    console.log(`[OS DESKTOP] Widgets: 5 types (World Clocks, Weather, News, Currency, Stocks)`);
    console.log('[OS DESKTOP] Apps in App Centre, Media in Media Centre');
    console.log('[OS DESKTOP] ======================================');
  }, []);

  // Recent items for the NAVA menu
  const recentItems: MenuItem[] = [
    { label: 'Show "App Centre.app" in Finder', action: () => navigate('/app.html?activity=app-centre') },
    { label: 'Show "Media Centre.app" in Finder', action: () => navigate('/app.html?activity=media-centre') },
      { label: 'Show "NAVΛ IDE.app" in Finder', action: () => navigate('/app.html?activity=explorer') },
    { label: 'Show "NAVΛ RS1.app" in Finder', action: () => setShowRobotisOverlay(true) },
    { label: 'Show "Univarm.app" in Finder', action: () => navigate('/app.html?activity=univarm') },
    { label: 'Show "Simulation.app" in Finder', action: () => navigate('/app.html?activity=simulation') },
  ];

  const navaMenuItems: MenuItem[] = [
    { label: 'About NAVΛ Studio', action: () => alert('NAVΛ Studio IDE\nVersion 1.0.0\n\nThe Future of AI Desktop Computing') },
    { label: 'System Settings...', action: () => console.log('System Settings') },
    { label: 'App Store', action: () => navigate('/app.html?activity=app-centre') },
    { 
      label: 'Recent Items', 
      submenu: recentItems,
    },
    { divider: true },
    { label: 'Force Quit Applications...', shortcut: '⌥⌘⎋', action: () => console.log('Force Quit') },
    { divider: true },
    { label: 'Sleep', action: () => console.log('Sleep') },
    { label: 'Restart...', action: () => {
      if (confirm('Are you sure you want to restart NAVΛ Studio?')) {
        window.location.reload();
      }
    }},
    { label: 'Shut Down...', action: () => {
      if (confirm('Are you sure you want to shut down NAVA Studio?')) {
        window.close();
      }
    }},
    { divider: true },
    { label: 'Lock Screen', shortcut: '⌃⌘Q', action: () => console.log('Lock Screen') },
    { label: 'Log Out...', shortcut: '⇧⌘Q', action: () => {
      if (confirm('Are you sure you want to log out?')) {
        localStorage.clear();
        window.location.reload();
      }
    }},
  ];

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
      { label: 'NAVΛ IDE (with Notebook)', route: '/app.html?activity=explorer', action: () => navigate('/app.html?activity=explorer') },
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
      { label: 'NAVΛ IDE (with Notebook)', route: '/app.html?activity=explorer', action: () => navigate('/app.html?activity=explorer') },
    ],
    help: [
      { label: 'Welcome', route: '/app.html?activity=ros-learning', action: () => navigate('/app.html?activity=ros-learning') },
      { label: 'Documentation', route: '/docs.html', action: () => navigate('/docs.html') },
      { label: 'Quick Start Guide', route: '/app.html?activity=ros-learning', action: () => navigate('/app.html?activity=ros-learning') },
      { label: 'API Reference', route: '/docs.html', action: () => navigate('/docs.html') },
      { divider: true },
      { label: 'Keyboard Shortcuts', action: () => console.log('Keyboard Shortcuts') },
      { divider: true },
      { label: 'About NAVΛ OS', action: () => console.log('About NAVΛ OS') },
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
      safeWindowOpen(route, '_blank');
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
      safeLocalStorage.setItem('navlambda-active-activity', activity);
    } else if (absoluteRoute === '/app.html') {
      // No activity specified, clear it
      console.log(`[NAVIGATE] No activity in route, clearing localStorage`);
      safeLocalStorage.removeItem('navlambda-active-activity');
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

  const handleDockClick = (app: DockApp, event?: React.MouseEvent | React.KeyboardEvent) => {
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
    
    // LOCKED: NAVΛ RS1 always opens overlay - highest priority
    if (app.id === 'robotis-systemic') {
      console.log(`[DOCK CLICK] NAVΛ RS1 detected - LOCKED ROUTE to overlay`);
      handleOpenNavARS1();
      return;
    }
    
    // Handle onClick for apps that use overlays
    if (app.onClick) {
      console.log(`[DOCK CLICK] onClick handler exists, calling it...`);
      app.onClick();
      return;
    }
    
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

  // Removed handleDownloadInterface - replaced with SDKDownloadButton

  // Close dropdowns when clicking outside
  useEffect(() => {
    const handleClickOutside = (event: MouseEvent) => {
      if (activeMenu && menuRefs.current[activeMenu]) {
        const menuElement = menuRefs.current[activeMenu];
        if (menuElement && !menuElement.contains(event.target as Node)) {
          setActiveMenu(null);
        }
      }
      // Close NAVA menu
      const navaMenuElement = document.querySelector('.nava-menu-container');
      if (showNavaMenu && navaMenuElement && !navaMenuElement.contains(event.target as Node)) {
        setShowNavaMenu(false);
        setHoveredSubmenu(null);
      }
    };

    document.addEventListener('mousedown', handleClickOutside);
    return () => document.removeEventListener('mousedown', handleClickOutside);
  }, [activeMenu, showNavaMenu]);

  return (
    <div className="os-desktop">
      {/* Top Menu Bar */}
      <div className="desktop-top-bar">
        <div className="desktop-menu">
          {/* NAVA Logo Menu */}
          <div className="nava-menu-container" ref={(el) => (menuRefs.current['nava'] = el)}>
            <span
              className={`menu-item nava-logo-menu ${showNavaMenu ? 'active' : ''}`}
              onClick={() => setShowNavaMenu(!showNavaMenu)}
            >
              <span className="nava-logo-icon">⋋</span>
            </span>
            {showNavaMenu && (
              <div className="menu-dropdown nava-menu-dropdown">
                {navaMenuItems.map((item, index) => (
                  <React.Fragment key={index}>
                    {item.divider ? (
                      <div className="menu-divider" />
                    ) : (
                      <div
                        className="menu-dropdown-item menu-item-with-submenu"
                        onMouseEnter={() => item.submenu && setHoveredSubmenu(`nava-${index}`)}
                        onMouseLeave={() => !item.submenu && setHoveredSubmenu(null)}
                        onClick={() => {
                          if (!item.submenu) {
                            if (item.route) {
                              navigate(item.route);
                            } else if (item.action) {
                              item.action();
                            }
                            setShowNavaMenu(false);
                          }
                        }}
                      >
                        <span>{item.label}</span>
                        <span style={{ display: 'flex', alignItems: 'center', gap: '8px' }}>
                          {item.shortcut && (
                            <span className="menu-shortcut">{item.shortcut}</span>
                          )}
                          {item.submenu && (
                            <span className="menu-arrow">▶</span>
                          )}
                        </span>
                        {item.submenu && hoveredSubmenu === `nava-${index}` && (
                          <div 
                            className="menu-submenu"
                            onMouseEnter={() => setHoveredSubmenu(`nava-${index}`)}
                            onMouseLeave={() => setHoveredSubmenu(null)}
                          >
                            <div className="submenu-title">Applications</div>
                            {item.submenu.map((subItem, subIndex) => (
                              <div
                                key={subIndex}
                                className="menu-dropdown-item"
                                onClick={(e) => {
                                  e.stopPropagation();
                                  if (subItem.route) {
                                    navigate(subItem.route);
                                  } else if (subItem.action) {
                                    subItem.action();
                                  }
                                  setShowNavaMenu(false);
                                  setHoveredSubmenu(null);
                                }}
                              >
                                {subItem.label}
                              </div>
                            ))}
                          </div>
                        )}
                      </div>
                    )}
                  </React.Fragment>
                ))}
              </div>
            )}
          </div>
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
            <div className="logo-main">NAV<span className="lambda-in-text">Λ</span> OS</div>
            <div className="logo-subtitle">THE FUTURE OF AI DESKTOP COMPUTING</div>
          </div>
        </div>
        <div style={{ display: 'flex', gap: '12px', alignItems: 'center' }}>
          <button 
            className="media-centre-btn" 
            onClick={() => setShowMediaCentre(true)}
            style={{
              padding: '8px 16px',
              background: 'rgba(59, 130, 246, 0.2)',
              border: '1px solid rgba(59, 130, 246, 0.4)',
              borderRadius: '8px',
              color: '#60a5fa',
              fontSize: '13px',
              fontWeight: '500',
              cursor: 'pointer',
              transition: 'all 0.2s',
              display: 'flex',
              alignItems: 'center',
              gap: '6px',
            }}
            onMouseEnter={(e) => {
              e.currentTarget.style.background = 'rgba(59, 130, 246, 0.3)';
              e.currentTarget.style.transform = 'translateY(-1px)';
            }}
            onMouseLeave={(e) => {
              e.currentTarget.style.background = 'rgba(59, 130, 246, 0.2)';
              e.currentTarget.style.transform = 'translateY(0)';
            }}
          >
            <span>🎛️</span>
            <span>Media Centre</span>
          </button>
          <button 
            className="branding-controls-btn" 
            onClick={() => setShowBrandingControls(!showBrandingControls)}
            title="Branding Controls"
            style={{
              padding: '8px 16px',
              background: 'rgba(0, 255, 0, 0.2)',
              border: '1px solid rgba(0, 255, 0, 0.4)',
              borderRadius: '8px',
              color: '#00ff00',
              fontSize: '13px',
              fontWeight: '500',
              cursor: 'pointer',
              transition: 'all 0.2s',
              display: 'flex',
              alignItems: 'center',
              gap: '6px',
              textShadow: '0 0 8px rgba(0, 255, 0, 0.6), 0 0 16px rgba(0, 255, 0, 0.4)',
            }}
            onMouseEnter={(e) => {
              e.currentTarget.style.background = 'rgba(0, 255, 0, 0.3)';
              e.currentTarget.style.transform = 'translateY(-1px)';
              e.currentTarget.style.textShadow = '0 0 12px rgba(0, 255, 0, 0.8), 0 0 24px rgba(0, 255, 0, 0.6)';
            }}
            onMouseLeave={(e) => {
              e.currentTarget.style.background = 'rgba(0, 255, 0, 0.2)';
              e.currentTarget.style.transform = 'translateY(0)';
              e.currentTarget.style.textShadow = '0 0 8px rgba(0, 255, 0, 0.6), 0 0 16px rgba(0, 255, 0, 0.4)';
            }}
          >
            <span>🎨</span>
            <span>Branding</span>
          </button>
          <IDEDownloadButton />
        </div>
      </div>

      {/* Main Desktop Content */}
      <div className="desktop-content">
        {/* Left Sidebar - Finder Style */}
        {!leftSidebarCollapsed && (
          <FinderSidebar
            onItemClick={(item) => {
              console.log('[FINDER SIDEBAR] Item clicked:', item);
              handleSidebarClick(item.name);
            }}
            onCollapse={() => {
              setLeftSidebarCollapsed(true);
              localStorage.setItem('nava-left-sidebar-collapsed', 'true');
            }}
          />
        )}
        {leftSidebarCollapsed && (
          <div className="desktop-sidebar-left-collapsed">
            <button 
              className="sidebar-expand-btn"
              onClick={() => {
                setLeftSidebarCollapsed(false);
                localStorage.setItem('nava-left-sidebar-collapsed', 'false');
              }}
              title="Show Sidebar"
            >
              ◀
            </button>
          </div>
        )}
        {/* Old Sidebar - Hidden but kept for reference */}
        <div className="desktop-sidebar-left" style={{ display: 'none' }}>
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
          
          {/* The NAVΛ Navigation Institute Branding */}
          <div className="desktop-branding">
            <div className="branding-logo">⋋</div>
            <div className="branding-text">
              <h1 className="branding-title">The NAV<span className="lambda-in-text">Λ</span></h1>
              <p className="branding-subtitle">NAVIGATION INSTITUTE</p>
            </div>
          </div>

          {/* Media apps are now in Media Centre - click Media Centre button in top right to access */}

          {/* Apps are now in App Centre - click App Centre icon (📱) in dock to access */}

          {/* Blueprint Canvas/Grid Area - Green Arch Visualization */}
          <div className="blueprint-canvas">
            <div className="canvas-grid">
              {/* Grid background with faint NAVA logo */}
              <div className="grid-background">
                <div className="grid-logo">⋋</div>
              </div>
              {/* Grid lines and coordinate system */}
              <svg className="grid-svg" viewBox="0 0 800 600" preserveAspectRatio="xMidYMid meet">
                <defs>
                  <pattern id="grid" width="40" height="40" patternUnits="userSpaceOnUse">
                    <path d="M 40 0 L 0 0 0 40" fill="none" stroke="rgba(59, 130, 246, 0.2)" strokeWidth="1"/>
                  </pattern>
                  <filter id="glow">
                    <feGaussianBlur stdDeviation="4" result="coloredBlur"/>
                    <feMerge>
                      <feMergeNode in="coloredBlur"/>
                      <feMergeNode in="SourceGraphic"/>
                    </feMerge>
                  </filter>
                </defs>
                <rect width="100%" height="100%" fill="url(#grid)" />
                {/* Coordinate axes */}
                <line x1="50" y1="550" x2="750" y2="550" stroke="rgba(59, 130, 246, 0.5)" strokeWidth="2" />
                <line x1="50" y1="550" x2="50" y2="50" stroke="rgba(59, 130, 246, 0.5)" strokeWidth="2" />
                {/* Green Arch Path - Prominent visualization (shows above dock) */}
                <g>
                  <path 
                    d="M 100 500 Q 350 300 600 500" 
                    fill="none" 
                    stroke="#00ff00" 
                    strokeWidth="4" 
                    filter="url(#glow)"
                    style={{ 
                      stroke: '#00ff00',
                      strokeWidth: '4px',
                    }}
                  />
                  <circle cx="100" cy="500" r="6" fill="#00ff00" filter="url(#glow)" />
                  <circle cx="350" cy="300" r="6" fill="#3b82f6" filter="url(#glow)" />
                  <circle cx="600" cy="500" r="6" fill="#00ff00" filter="url(#glow)" />
                </g>
              </svg>
            </div>
          </div>

          {/* Apps are now in App Centre - click App Centre icon in dock to access */}

          {/* Media apps are now in Media Centre - click Media Centre button in top right to access */}

          {/* Desktop Branding Elements */}
          <DesktopBranding />
        </div>

        {/* Right Sidebar - Widgets */}
        {!rightSidebarCollapsed && (
        <div className="desktop-sidebar-right">
            <WorldWidgets onCollapse={() => {
              setRightSidebarCollapsed(true);
              localStorage.setItem('nava-right-sidebar-collapsed', 'true');
            }} />
        </div>
        )}
        {rightSidebarCollapsed && (
          <div className="desktop-sidebar-right-collapsed">
            <button 
              className="widgets-expand-btn"
              onClick={() => {
                setRightSidebarCollapsed(false);
                localStorage.setItem('nava-right-sidebar-collapsed', 'false');
              }}
              title="Show Widgets"
            >
              ▶
            </button>
          </div>
        )}
      </div>

      {/* Bottom Container - Separated sections */}
      <div className="desktop-bottom-container" style={{
        position: 'fixed',
        bottom: 0,
        left: 0,
        right: 0,
        zIndex: 1000,
        display: 'flex',
        flexDirection: 'column',
        gap: 0,
        pointerEvents: 'none'
      }}>
        {/* Dock Section - Circular icons */}
        <div className="desktop-dock-wrapper" style={{
          background: 'rgba(10, 15, 26, 0.85)',
          backdropFilter: 'blur(20px)',
          borderTop: '1px solid rgba(59, 130, 246, 0.3)',
          padding: '8px 0',
          pointerEvents: 'auto'
        }}>
          <div 
            className="desktop-dock" 
            style={{ 
            display: 'flex', 
            justifyContent: 'center',
            alignItems: 'center',
            gap: '8px',
            visibility: 'visible', 
            opacity: 1,
            padding: '4px 16px'
            }}
            onDragOver={(e) => {
              e.preventDefault();
              try {
                const data = e.dataTransfer.getData('application/json');
                if (data) {
                  const appData = JSON.parse(data);
                  // Check if dragging over trash bin
                  const target = e.target as HTMLElement;
                  const trashIcon = target.closest('[data-dock-id="trash"]');
                  if (trashIcon && appData.type === 'app' && appData.id) {
                    e.dataTransfer.dropEffect = 'move';
                  } else if (appData.type === 'app' && appData.id) {
                    e.dataTransfer.dropEffect = 'copy';
                  }
                }
              } catch {}
            }}
            onDrop={(e) => {
              e.preventDefault();
              try {
                const data = e.dataTransfer.getData('application/json');
                if (data) {
                  const appData = JSON.parse(data);
                  
                  // Check if dropped on trash bin
                  const target = e.target as HTMLElement;
                  const trashIcon = target.closest('[data-dock-id="trash"]');
                  
                  if (trashIcon && appData.type === 'app' && appData.id) {
                    // Move to trash
                    import('../../utils/appStorage').then(({ getAllApps, deleteApp }) => {
                      const apps = getAllApps();
                      const app = apps.find(a => a.id === appData.id);
                      if (app) {
                        import('../../utils/trashStorage').then(({ addToTrash }) => {
                          addToTrash({
                            originalId: app.id,
                            name: app.name,
                            icon: app.icon,
                            color: app.color,
                            description: app.description,
                            route: app.route,
                            category: app.category,
                            type: 'app',
                            originalData: app,
                          });
                          deleteApp(app.id);
                          setTrashHasItems(true);
                        });
                      }
                    });
                  } else if (appData.type === 'app' && appData.id) {
                    // Add to dock
                    import('../../utils/appStorage').then(({ addToDock }) => {
                      addToDock(appData.id);
                      console.log(`[DOCK] Added ${appData.id} to dock`);
                      // Refresh to show updated dock
                      setTimeout(() => window.location.reload(), 500);
                    });
                  }
                }
              } catch (error) {
                console.error('[DOCK] Error handling drop:', error);
              }
            }}
          >
            {dockApps.map((app, index) => {
              console.log(`[OS DESKTOP] Rendering dock icon ${index + 1}/${dockApps.length}: ${app.name}`);
              return (
                <div
                  key={app.id}
                  data-dock-id={app.id}
                  className={`dock-icon ${activeDockApp === app.id ? 'active' : ''} ${hoveredDockApp === app.id ? 'hovered' : ''} ${app.id === 'trash' && trashHasItems ? 'trash-full' : ''} ${app.id === 'downloads' && downloadsCount > 0 ? 'downloads-full' : ''}`}
                  title={app.name}
                  style={{ 
                    display: 'flex', 
                    visibility: 'visible', 
                    opacity: 1, 
                    cursor: 'pointer',
                    position: 'relative'
                  }}
                  onClick={(e) => {
                    // Only handle click if context menu is not showing
                    if (!contextMenu || contextMenu.appId !== app.id) {
                      console.log(`[DOCK] Click detected on: ${app.name}`);
                      handleDockClick(app, e);
                    }
                  }}
                  onContextMenu={(e) => {
                    e.preventDefault();
                    e.stopPropagation();
                    const rect = (e.currentTarget as HTMLElement).getBoundingClientRect();
                    setContextMenu({
                      appId: app.id,
                      appName: app.name,
                      position: {
                        x: rect.left + rect.width / 2,
                        y: rect.top,
                      },
                    });
                  }}
                  onMouseDown={(e) => {
                    // Handle double-tap (double-click) for context menu on touch devices
                    if (e.detail === 2) {
                      e.preventDefault();
                      const rect = (e.currentTarget as HTMLElement).getBoundingClientRect();
                      setContextMenu({
                        appId: app.id,
                        appName: app.name,
                        position: {
                          x: rect.left + rect.width / 2,
                          y: rect.top,
                        },
                      });
                    }
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
                    <span className="dock-icon-emoji" style={{
                      opacity: app.id === 'trash' && trashHasItems ? 1 : (app.id === 'trash' ? 0.5 : 1)
                    }}>{app.icon}</span>
                    {app.id === 'trash' && trashHasItems && (
                      <div className="trash-indicator" style={{
                        position: 'absolute',
                        bottom: '4px',
                        right: '4px',
                        width: '12px',
                        height: '12px',
                        background: 'rgba(16, 185, 129, 0.9)',
                        border: '2px solid rgba(15, 23, 42, 0.9)',
                        borderRadius: '50%',
                        boxShadow: '0 0 8px rgba(16, 185, 129, 0.6)',
                        zIndex: 10,
                      }} />
                    )}
                  </div>
                  {activeDockApp === app.id && <div className="dock-active-indicator" />}
                  <div className="dock-tooltip">{app.name}</div>
                </div>
              );
            })}
          </div>
        </div>

        {/* Dock Context Menu */}
        {contextMenu && (
          <DockContextMenu
            appId={contextMenu.appId}
            appName={contextMenu.appName}
            position={contextMenu.position}
            preferences={getAppPreferences(contextMenu.appId)}
            onClose={() => setContextMenu(null)}
            onRemoveFromDock={() => {
              removeFromDock(contextMenu.appId);
              setContextMenu(null);
              // Refresh the page or update dock apps
              window.location.reload();
            }}
            onOpenAtLogin={(enabled) => {
              const prefs = getAppPreferences(contextMenu.appId);
              saveDockPreferences(contextMenu.appId, {
                ...prefs,
                openAtLogin: enabled,
              });
            }}
            onShowInFinder={() => {
              // Show app in finder/file explorer
              const app = dockApps.find(a => a.id === contextMenu.appId);
              if (app) {
                // Navigate to explorer and highlight the app
                navigate('/app.html?activity=explorer');
                // Dispatch event to highlight app in explorer
                const event = new CustomEvent('nava:highlight-app', { detail: { appId: contextMenu.appId } });
                window.dispatchEvent(event);
              }
              setContextMenu(null);
            }}
            onAssignToDesktop={(desktop) => {
              const prefs = getAppPreferences(contextMenu.appId);
              saveDockPreferences(contextMenu.appId, {
                ...prefs,
                assignToDesktop: desktop,
              });
            }}
            onShowRecents={() => {
              // Show recent files/activity for this app
              const app = dockApps.find(a => a.id === contextMenu.appId);
              if (app) {
                // Could show a recent files panel or navigate to recent activity
                console.log(`[Dock] Show recents for: ${app.name}`);
                // For now, just open the app
                handleDockClick(app);
              }
              setContextMenu(null);
            }}
            onOpen={() => {
              const app = dockApps.find(a => a.id === contextMenu.appId);
              if (app) {
                handleDockClick(app);
              }
              setContextMenu(null);
            }}
          />
        )}

        {/* Status Bar Section - Separated below dock */}
        <div className="desktop-status-bar" style={{
          background: 'rgba(6, 10, 18, 0.95)',
          backdropFilter: 'blur(10px)',
          borderTop: '1px solid rgba(59, 130, 246, 0.2)',
          height: '32px',
          display: 'flex',
          alignItems: 'center',
          justifyContent: 'space-between',
          padding: '0 16px',
          fontSize: '12px',
          pointerEvents: 'auto'
        }}>
          <div className="status-bar-left" style={{ display: 'flex', gap: '12px', alignItems: 'center' }}>
            <span className="status-icon">💡</span>
            <span className="status-icon">⚙️</span>
          </div>
          <div className="status-bar-center" style={{ display: 'flex', gap: '16px', alignItems: 'center' }}>
            <span className="status-time">{new Date().toLocaleTimeString('en-US', { hour: '2-digit', minute: '2-digit' })}</span>
            <span className="status-date">{new Date().toLocaleDateString('en-GB', { day: '2-digit', month: '2-digit', year: 'numeric' })}</span>
          </div>
          <div className="status-bar-right" style={{ display: 'flex', gap: '12px', alignItems: 'center' }}>
            <span className="status-language">EN</span>
            <span className="status-icon">⚡</span>
          </div>
        </div>
      </div>

      {/* LOCKED: NAVΛ RS1 Overlay - Always renders when showRobotisOverlay is true */}
      {showRobotisOverlay && (
        <div 
          style={{
            position: 'fixed',
            top: 0,
            left: 0,
            right: 0,
            bottom: 0,
            zIndex: 99999,
            background: robotisTheme === 'dark' ? '#000000' : '#ffffff',
            display: 'flex',
            flexDirection: 'column',
            transition: 'background-color 0.3s ease',
          }}
          // Backend check is now handled by useEffect hook
        >
          {/* Header with NAVΛ Logo and Theme Toggle */}
          <div
            style={{
              display: 'flex',
              alignItems: 'center',
              justifyContent: 'space-between',
              padding: '12px 20px',
              background: robotisTheme === 'dark' 
                ? 'rgba(59, 130, 246, 0.2)' 
                : 'rgba(59, 130, 246, 0.1)',
              borderBottom: robotisTheme === 'dark'
                ? '1px solid rgba(59, 130, 246, 0.3)'
                : '1px solid rgba(59, 130, 246, 0.2)',
            }}
          >
            <div style={{ display: 'flex', alignItems: 'center', gap: '16px' }}>
              {/* NAVΛ Logo - Clickable to return home */}
              <div
                onClick={() => {
                  setShowRobotisOverlay(false);
                  setRobotisError(null);
                  setRobotisLoading(true);
                  navigate('/workspace.html');
                }}
                style={{
                  display: 'flex',
                  alignItems: 'center',
                  gap: '6px',
                  cursor: 'pointer',
                  padding: '4px 8px',
                  borderRadius: '6px',
                  transition: 'all 0.3s ease',
                }}
                onMouseEnter={(e) => {
                  e.currentTarget.style.background = robotisTheme === 'dark' 
                    ? 'rgba(0, 255, 0, 0.15)' 
                    : 'rgba(0, 255, 0, 0.1)';
                }}
                onMouseLeave={(e) => {
                  e.currentTarget.style.background = 'transparent';
                }}
                title="Back to Workspace"
              >
                <span style={{
                  fontSize: '18px',
                  fontWeight: '700',
                  color: '#00ff00',
                  textShadow: '0 0 10px rgba(0, 255, 0, 0.5)',
                  transition: 'all 0.3s ease',
                }}>λ</span>
                <span style={{
                  fontSize: '20px',
                  fontWeight: '700',
                  color: '#00ff00',
                  textShadow: '0 0 10px rgba(0, 255, 0, 0.5)',
                  transition: 'all 0.3s ease',
                }}>Λ</span>
              </div>
              <span style={{ 
                color: robotisTheme === 'dark' ? '#ffffff' : '#1e293b', 
                fontWeight: '600' 
              }}>
                🔷 NAVΛ RS1
              </span>
            </div>
            <div style={{ display: 'flex', gap: '8px', alignItems: 'center' }}>
              {/* Theme Toggle */}
              <button
                onClick={() => {
                  const newTheme = robotisTheme === 'dark' ? 'light' : 'dark';
                  setRobotisTheme(newTheme);
                  try {
                    localStorage.setItem('robotis-overlay-theme', newTheme);
                  } catch (e) {
                    console.warn('Failed to save theme preference:', e);
                  }
                }}
                style={{
                  background: robotisTheme === 'dark'
                    ? 'rgba(255, 255, 255, 0.1)'
                    : 'rgba(0, 0, 0, 0.05)',
                  border: robotisTheme === 'dark'
                    ? '1px solid rgba(255, 255, 255, 0.2)'
                    : '1px solid rgba(0, 0, 0, 0.1)',
                  borderRadius: '6px',
                  padding: '6px 12px',
                  color: robotisTheme === 'dark' ? '#ffffff' : '#1e293b',
                  cursor: 'pointer',
                  fontSize: '12px',
                  display: 'flex',
                  alignItems: 'center',
                  gap: '6px',
                }}
                title={`Switch to ${robotisTheme === 'dark' ? 'light' : 'dark'} theme`}
              >
                {robotisTheme === 'dark' ? '☀️' : '🌙'} {robotisTheme === 'dark' ? 'Light' : 'Dark'}
              </button>
              {/* Close Button */}
              <button
                onClick={() => {
                  setShowRobotisOverlay(false);
                  setRobotisError(null);
                  setRobotisLoading(true);
                }}
                style={{
                  background: 'rgba(220, 38, 38, 0.3)',
                  border: '1px solid rgba(220, 38, 38, 0.5)',
                  borderRadius: '6px',
                  padding: '6px 12px',
                  color: '#ffffff',
                  cursor: 'pointer',
                  fontSize: '12px',
                }}
              >
                ✕ Close
              </button>
            </div>
          </div>
          
          {/* Error Display or Iframe */}
          {robotisError ? (
            <div style={{
              flex: 1,
              display: 'flex',
              flexDirection: 'column',
              alignItems: 'center',
              justifyContent: 'center',
              padding: '40px',
              color: robotisTheme === 'dark' ? '#ffffff' : '#1e293b',
              textAlign: 'center',
            }}>
              <div style={{
                background: robotisTheme === 'dark' ? 'rgba(220, 38, 38, 0.2)' : 'rgba(220, 38, 38, 0.1)',
                border: `1px solid ${robotisTheme === 'dark' ? 'rgba(220, 38, 38, 0.5)' : 'rgba(220, 38, 38, 0.3)'}`,
                borderRadius: '12px',
                padding: '32px',
                maxWidth: '600px',
              }}>
                <h2 style={{ margin: '0 0 16px 0', fontSize: '20px', fontWeight: '600' }}>
                  🔷 NAVΛ RS1 Backend Not Running
                </h2>
                <p style={{ margin: '0 0 24px 0', fontSize: '14px', lineHeight: '1.6', opacity: 0.9 }}>
                  {robotisError}
                </p>
                {robotisRetryCount > 0 && (
                  <p style={{ margin: '0 0 16px 0', fontSize: '12px', opacity: 0.7, color: '#00ff00' }}>
                    🔄 Auto-checking backend... (Attempt {robotisRetryCount})
                    {robotisRetryCount > 30 && (
                      <span style={{ display: 'block', marginTop: '4px', fontSize: '11px', opacity: 0.6 }}>
                        Backend is starting automatically. This may take a minute...
                      </span>
                    )}
                  </p>
                )}
                <div style={{ marginBottom: '24px', fontSize: '13px', opacity: 0.8 }}>
                  <p style={{ margin: '0 0 12px 0', fontWeight: '600' }}>To start the backend:</p>
                  <div style={{
                    background: robotisTheme === 'dark' ? 'rgba(0, 0, 0, 0.3)' : 'rgba(0, 0, 0, 0.05)',
                    padding: '16px',
                    borderRadius: '8px',
                    fontFamily: 'monospace',
                    fontSize: '12px',
                    textAlign: 'left',
                    marginBottom: '16px',
                  }}>
                    <div style={{ marginBottom: '8px' }}>1. Open terminal</div>
                    <div style={{ marginBottom: '8px' }}>2. Run: <code style={{ color: '#00ff00' }}>./start-robotis-system.sh</code></div>
                    <div>3. Wait for services to start (ports 3000 and 8080)</div>
                  </div>
                </div>
                <div style={{ display: 'flex', gap: '12px', justifyContent: 'center', flexWrap: 'wrap' }}>
                  <button
                    onClick={async () => {
                      // Retry connection - restart the check process
                      setRobotisError(null);
                      setRobotisLoading(true);
                      setRobotisRetryCount(0);
                      
                      // Clear existing interval
                      if (robotisCheckIntervalRef.current) {
                        clearInterval(robotisCheckIntervalRef.current);
                        robotisCheckIntervalRef.current = null;
                      }

                      // Start persistent checking again
                      let retryCount = 0;
                      const maxRetriesBeforeError = 30;
                      let hasShownError = false;
                      
                      robotisCheckIntervalRef.current = setInterval(async () => {
                        retryCount++;
                        setRobotisRetryCount(retryCount);

                        const isRunning = await checkRobotisBackend();
                        if (isRunning) {
                          setRobotisLoading(false);
                          setRobotisError(null);
                          hasShownError = false;
                          // Reload iframe
                          const iframe = document.querySelector('iframe[title="ROBOTIS-SYSTEMIC Platform"]') as HTMLIFrameElement;
                          if (iframe) {
                            iframe.src = iframe.src;
                          }
                          if (robotisCheckIntervalRef.current) {
                            clearInterval(robotisCheckIntervalRef.current);
                            robotisCheckIntervalRef.current = null;
                          }
                        } else if (retryCount >= maxRetriesBeforeError && !hasShownError) {
                          setRobotisError('The NAVΛ RS1 backend server is not running on http://localhost:3000. The system is automatically trying to start it. Please wait...');
                          setRobotisLoading(true);
                          hasShownError = true;
                        }
                        // Continue checking indefinitely
                      }, 2000);
                    }}
                    style={{
                      background: 'linear-gradient(135deg, #00ff00 0%, #00cc00 100%)',
                      border: '1px solid rgba(0, 255, 0, 0.3)',
                      borderRadius: '8px',
                      color: '#ffffff',
                      padding: '12px 24px',
                      fontSize: '14px',
                      fontWeight: '600',
                      cursor: 'pointer',
                      display: 'flex',
                      alignItems: 'center',
                      gap: '8px',
                      boxShadow: '0 4px 12px rgba(0, 255, 0, 0.3)',
                      transition: 'all 0.2s ease',
                    }}
                    onMouseEnter={(e) => {
                      e.currentTarget.style.transform = 'translateY(-2px)';
                      e.currentTarget.style.boxShadow = '0 6px 16px rgba(0, 255, 0, 0.4)';
                    }}
                    onMouseLeave={(e) => {
                      e.currentTarget.style.transform = 'translateY(0)';
                      e.currentTarget.style.boxShadow = '0 4px 12px rgba(0, 255, 0, 0.3)';
                    }}
                  >
                    🔄 Retry Connection
                  </button>
                  <button
                    onClick={() => {
                      // Open terminal instruction
                      alert('To start the backend manually:\n\n1. Open Terminal\n2. Navigate to: /Users/frankvanlaarhoven/Desktop/NAVA Studio IDE\n3. Run: ./start-robotis-system.sh\n\nOr the system will try to start it automatically.');
                    }}
                    style={{
                      background: 'rgba(59, 130, 246, 0.1)',
                      border: '1px solid rgba(59, 130, 246, 0.3)',
                      borderRadius: '8px',
                      color: '#3b82f6',
                      padding: '12px 24px',
                      fontSize: '14px',
                      fontWeight: '600',
                      cursor: 'pointer',
                      transition: 'all 0.2s ease',
                    }}
                    onMouseEnter={(e) => {
                      e.currentTarget.style.background = 'rgba(59, 130, 246, 0.2)';
                      e.currentTarget.style.borderColor = 'rgba(59, 130, 246, 0.5)';
                    }}
                    onMouseLeave={(e) => {
                      e.currentTarget.style.background = 'rgba(59, 130, 246, 0.1)';
                      e.currentTarget.style.borderColor = 'rgba(59, 130, 246, 0.3)';
                    }}
                  >
                    📖 Manual Start Instructions
                  </button>
                </div>
              </div>
            </div>
          ) : (
            <iframe
              src={`http://localhost:3000?theme=${robotisTheme}`}
              style={{
                flex: 1,
                width: '100%',
                height: '100%',
                border: 'none',
                background: robotisTheme === 'dark' ? '#000000' : '#ffffff',
                display: robotisLoading ? 'none' : 'block',
              }}
                      title="NAVΛ RS1 Platform"
              allow="xr-spatial-tracking; pointer-lock; fullscreen; autoplay; camera; microphone"
              sandbox="allow-same-origin allow-scripts allow-forms allow-popups allow-popups-to-escape-sandbox allow-pointer-lock allow-presentation"
              onLoad={() => {
                // Only clear error if iframe loaded successfully
                if (!robotisError) {
                  setRobotisLoading(false);
                }
              }}
              onError={() => {
                // Don't set error here - let the health check handle it
                // This prevents false errors when backend is starting up
              }}
            />
          )}
          {robotisLoading && !robotisError && (
            <div style={{
              position: 'absolute',
              top: '50%',
              left: '50%',
              transform: 'translate(-50%, -50%)',
              color: robotisTheme === 'dark' ? '#ffffff' : '#1e293b',
              fontSize: '16px',
            }}>
              Loading NAVΛ RS1...
            </div>
          )}
        </div>
      )}
      
      {/* Media Centre Overlay */}
      {showMediaCentre && (
        <MediaCentre onClose={() => setShowMediaCentre(false)} />
      )}

      {/* Trash Bin Window */}
      {showTrashBin && (
        <TrashBin onClose={() => setShowTrashBin(false)} />
      )}

      {showDownloadsStack && (
        <DownloadsStack onClose={() => setShowDownloadsStack(false)} />
      )}

      {/* Branding Controls Panel */}
      {showBrandingControls && (
        <BrandingControlsPanel 
          onClose={() => setShowBrandingControls(false)}
          onUpdate={() => {
            // Force re-render of DesktopBranding component
            setShowBrandingControls(false);
            setTimeout(() => setShowBrandingControls(true), 100);
          }}
        />
      )}
      
      {/* CI Indicator Widget - Removed */}
    </div>
  );
};

export default OSDesktop;

