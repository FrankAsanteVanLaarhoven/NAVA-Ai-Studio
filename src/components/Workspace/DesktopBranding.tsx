/**
 * Desktop Branding Elements
 * 
 * Draggable branding components for the NAVΛ OS Desktop:
 * - NAVΛ Navigation Institute Logo
 * - Career Happens Text
 * - Display Screen (16:9 format for advertising)
 */

import React, { useState, useEffect, useRef } from 'react';
import './DesktopBranding.css';

interface DraggableElement {
  id: string;
  x: number;
  y: number;
  visible: boolean;
}

interface DisplaySettings {
  videoUrl: string;
  autoplay: boolean;
  loop: boolean;
  muted: boolean;
  showControls: boolean;
}

interface PictureInPictureSettings {
  enabled: boolean;
  videoUrl: string;
  x: number;
  y: number;
  width: number;
  height: number;
  autoplay: boolean;
  loop: boolean;
  muted: boolean;
  showControls: boolean;
}

export const DesktopBranding: React.FC = () => {
  const [displaySettings, setDisplaySettings] = useState<DisplaySettings>(() => {
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
  const [isVideoPlaying, setIsVideoPlaying] = useState(false);
  const [showStandbyGraph, setShowStandbyGraph] = useState<boolean>(() => {
    try {
      const saved = localStorage.getItem('nava-display-standby');
      return saved !== 'false'; // Default to true (show graph)
    } catch {
      return true; // Default to showing graph
    }
  });

  const loadElements = (): { [key: string]: DraggableElement } => {
    try {
      const saved = localStorage.getItem('nava-desktop-branding');
      if (saved) {
        const parsed = JSON.parse(saved);
        // Ensure all elements exist
        return {
          logo: parsed.logo || { id: 'logo', x: 50, y: 80, visible: true },
          career: parsed.career || { id: 'career', x: Math.max(50, window.innerWidth - 400), y: Math.max(80, window.innerHeight - 200), visible: true },
          display: parsed.display || { id: 'display', x: Math.max(50, window.innerWidth / 2 - 400), y: Math.max(80, window.innerHeight / 2 - 225), visible: true },
        };
      }
    } catch {}
    // Default positions
    const width = typeof window !== 'undefined' ? window.innerWidth : 1920;
    const height = typeof window !== 'undefined' ? window.innerHeight : 1080;
    return {
      logo: { id: 'logo', x: 50, y: 80, visible: true },
      career: { id: 'career', x: Math.max(50, width - 400), y: Math.max(80, height - 200), visible: true },
      display: { id: 'display', x: Math.max(50, width / 2 - 400), y: Math.max(80, height / 2 - 225), visible: true },
    };
  };

  const [elements, setElements] = useState<{ [key: string]: DraggableElement }>(loadElements);
  const [dragging, setDragging] = useState<string | null>(null);
  const [dragOffset, setDragOffset] = useState({ x: 0, y: 0 });
  const [displayExpanded, setDisplayExpanded] = useState<boolean>(() => {
    try {
      const saved = localStorage.getItem('nava-display-expanded');
      return saved === 'true';
    } catch {
      return false;
    }
  });
  const [pipSettings, setPipSettings] = useState<PictureInPictureSettings>(() => {
    try {
      const saved = localStorage.getItem('nava-pip-settings');
      if (saved) {
        const parsed = JSON.parse(saved);
        // Ensure all properties exist
        return {
          enabled: parsed.enabled || false,
          videoUrl: parsed.videoUrl || 'https://www.youtube.com/embed/Eu5mYMavctM',
          x: parsed.x !== undefined ? parsed.x : (typeof window !== 'undefined' ? window.innerWidth - 420 : 1500),
          y: parsed.y !== undefined ? parsed.y : 100,
          width: parsed.width || 400,
          height: parsed.height || 225,
          autoplay: parsed.autoplay !== undefined ? parsed.autoplay : true,
          loop: parsed.loop !== undefined ? parsed.loop : true,
          muted: parsed.muted !== undefined ? parsed.muted : false,
          showControls: parsed.showControls !== undefined ? parsed.showControls : true,
        };
      }
    } catch {}
    const width = typeof window !== 'undefined' ? window.innerWidth : 1920;
    return {
      enabled: false,
      videoUrl: 'https://www.youtube.com/embed/Eu5mYMavctM',
      x: width - 420,
      y: 100,
      width: 400,
      height: 225,
      autoplay: true,
      loop: true,
      muted: false,
      showControls: true,
    };
  });
  const [pipDragging, setPipDragging] = useState(false);
  const [pipDragOffset, setPipDragOffset] = useState({ x: 0, y: 0 });

  // Save positions to localStorage
  useEffect(() => {
    localStorage.setItem('nava-desktop-branding', JSON.stringify(elements));
  }, [elements]);

  // Save PiP settings to localStorage
  useEffect(() => {
    localStorage.setItem('nava-pip-settings', JSON.stringify(pipSettings));
  }, [pipSettings]);

  // Save display settings to localStorage
  useEffect(() => {
    localStorage.setItem('nava-display-settings', JSON.stringify(displaySettings));
  }, [displaySettings]);

  // Listen for display settings updates
  useEffect(() => {
    const handleDisplayUpdate = () => {
      try {
        const saved = localStorage.getItem('nava-display-settings');
        if (saved) {
          const newSettings = JSON.parse(saved);
          setDisplaySettings(newSettings);
          // If video URL changes, reset to standby
          if (newSettings.videoUrl !== displaySettings.videoUrl) {
            setShowStandbyGraph(true);
            setIsVideoPlaying(false);
            localStorage.setItem('nava-display-standby', 'true');
          }
        }
        // Check standby state
        const standbySaved = localStorage.getItem('nava-display-standby');
        if (standbySaved === 'false') {
          setShowStandbyGraph(false);
        } else {
          setShowStandbyGraph(true);
        }
        const pipSaved = localStorage.getItem('nava-pip-settings');
        if (pipSaved) {
          const parsed = JSON.parse(pipSaved);
          setPipSettings(prev => ({
            ...prev,
            ...parsed,
            // Preserve position if not in saved data
            x: parsed.x !== undefined ? parsed.x : prev.x,
            y: parsed.y !== undefined ? parsed.y : prev.y,
          }));
        }
      } catch {}
    };

    window.addEventListener('nava:display-update', handleDisplayUpdate);
    return () => window.removeEventListener('nava:display-update', handleDisplayUpdate);
  }, [displaySettings.videoUrl]);

  // Listen for storage changes and custom events to refresh
  useEffect(() => {
    const handleStorageChange = () => {
      setElements(loadElements());
    };

    const handleBrandingUpdate = () => {
      setElements(loadElements());
    };

    window.addEventListener('storage', handleStorageChange);
    window.addEventListener('nava:branding-update', handleBrandingUpdate);

    return () => {
      window.removeEventListener('storage', handleStorageChange);
      window.removeEventListener('nava:branding-update', handleBrandingUpdate);
    };
  }, []);

  // Handle window resize
  useEffect(() => {
    const handleResize = () => {
      setElements(prev => {
        const updated = { ...prev };
        // Keep elements within bounds on resize
        Object.keys(updated).forEach(key => {
          if (updated[key].x > window.innerWidth - 100) {
            updated[key].x = window.innerWidth - 100;
          }
          if (updated[key].y > window.innerHeight - 100) {
            updated[key].y = window.innerHeight - 100;
          }
        });
        return updated;
      });
    };

    window.addEventListener('resize', handleResize);
    return () => window.removeEventListener('resize', handleResize);
  }, []);

  const handleMouseDown = (id: string, e: React.MouseEvent) => {
    // Don't start dragging if clicking on buttons or controls
    if ((e.target as HTMLElement).closest('button') || (e.target as HTMLElement).closest('.display-expand-btn')) {
      return;
    }
    e.preventDefault();
    e.stopPropagation();
    const element = elements[id];
    if (!element) return;
    
    setDragging(id);
    setDragOffset({
      x: e.clientX - element.x,
      y: e.clientY - element.y,
    });
  };

  const handlePipMouseDown = (e: React.MouseEvent) => {
    // Don't start dragging if clicking on buttons or controls
    if ((e.target as HTMLElement).closest('button')) {
      return;
    }
    e.preventDefault();
    e.stopPropagation();
    
    setPipDragging(true);
    setPipDragOffset({
      x: e.clientX - pipSettings.x,
      y: e.clientY - pipSettings.y,
    });
  };

  useEffect(() => {
    if (!dragging) return;

    const handleMouseMove = (e: MouseEvent) => {
      setElements(prev => ({
        ...prev,
        [dragging]: {
          ...prev[dragging],
          x: e.clientX - dragOffset.x,
          y: e.clientY - dragOffset.y,
        },
      }));
    };

    const handleMouseUp = () => {
      setDragging(null);
    };

    document.addEventListener('mousemove', handleMouseMove);
    document.addEventListener('mouseup', handleMouseUp);

    return () => {
      document.removeEventListener('mousemove', handleMouseMove);
      document.removeEventListener('mouseup', handleMouseUp);
    };
  }, [dragging, dragOffset]);

  return (
    <>
      {/* NAVΛ Navigation Institute Logo */}
      {elements.logo.visible && (
        <div
          className="desktop-branding-element nava-logo-element"
          style={{
            left: `${elements.logo.x}px`,
            top: `${elements.logo.y}px`,
            cursor: dragging === 'logo' ? 'grabbing' : 'grab',
          }}
          onMouseDown={(e) => handleMouseDown('logo', e)}
        >
          <div className="nava-logo-container">
            <span className="lambda-symbol">λ</span>
            <div className="nava-logo-text">
              <div className="nava-logo-main">
                <span className="gradient-text-start">The NAV<span className="lambda-in-text">Λ</span></span>
              </div>
              <div className="nava-logo-subtitle">NAVIGATION INSTITUTE</div>
            </div>
          </div>
        </div>
      )}

      {/* Career Happens Text */}
      {elements.career.visible && (
        <div
          className="desktop-branding-element career-text-element"
          style={{
            left: `${elements.career.x}px`,
            top: `${elements.career.y}px`,
            cursor: dragging === 'career' ? 'grabbing' : 'grab',
          }}
          onMouseDown={(e) => handleMouseDown('career', e)}
        >
          <div className="career-text-container">
            <div className="career-line">Where Your</div>
            <div className="career-line career-gradient">
              Navigation Calculus
            </div>
            <div className="career-line">Career Happens</div>
          </div>
        </div>
      )}

      {/* Display Screen (16:9) */}
      {elements.display.visible && (
        <>
          {displayExpanded && (
            <div 
              className="display-expand-backdrop"
              onMouseDown={(e) => {
                // Only close if clicking directly on backdrop, not on the screen
                if (e.target === e.currentTarget) {
                  setDisplayExpanded(false);
                  localStorage.setItem('nava-display-expanded', 'false');
                }
              }}
            />
          )}
          <div
            className={`desktop-branding-element display-screen-element ${displayExpanded ? 'expanded' : ''}`}
            style={{
              left: `${elements.display.x}px`,
              top: `${elements.display.y}px`,
              width: displayExpanded ? '90vw' : '800px',
              height: displayExpanded ? '50.625vw' : '450px',
              maxWidth: displayExpanded ? '1600px' : '800px',
              maxHeight: displayExpanded ? '900px' : '450px',
              cursor: dragging === 'display' ? 'grabbing' : 'grab',
              zIndex: displayExpanded ? 1001 : 5,
            }}
          onMouseDown={(e) => handleMouseDown('display', e)}
          >
          <div className="display-screen-container">
            {/* Expand/Collapse Button */}
            <button
              className="display-expand-btn"
              onClick={(e) => {
                e.stopPropagation();
                const newExpanded = !displayExpanded;
                setDisplayExpanded(newExpanded);
                localStorage.setItem('nava-display-expanded', String(newExpanded));
              }}
              title={displayExpanded ? 'Collapse' : 'Expand'}
            >
              {displayExpanded ? '✕' : '⛶'}
            </button>
            {/* Picture-in-Picture Toggle Button */}
            <button
              className="display-pip-btn"
              onClick={(e) => {
                e.stopPropagation();
                setPipSettings(prev => ({
                  ...prev,
                  enabled: !prev.enabled,
                }));
              }}
              title={pipSettings.enabled ? 'Disable Picture-in-Picture' : 'Enable Picture-in-Picture'}
            >
              {pipSettings.enabled ? '⊟' : '⊞'}
            </button>
            {/* Play Video Button - Shows when in standby */}
            {showStandbyGraph && displaySettings.videoUrl && (
              <button
                className="display-play-btn"
                onClick={(e) => {
                  e.stopPropagation();
                  setShowStandbyGraph(false);
                  setIsVideoPlaying(true);
                  localStorage.setItem('nava-display-standby', 'false');
                  // Force iframe reload to start video
                  setTimeout(() => {
                    const event = new CustomEvent('nava:play-video');
                    window.dispatchEvent(event);
                  }, 100);
                }}
                title="Play Video"
              >
                ▶
              </button>
            )}
            <div className="display-screen-content">
              {displaySettings.videoUrl && !showStandbyGraph ? (
                <>
                  <YouTubeEmbed 
                    videoId={extractYouTubeId(displaySettings.videoUrl)}
                    autoplay={displaySettings.autoplay}
                    loop={displaySettings.loop}
                    muted={displaySettings.muted}
                    controls={displaySettings.showControls}
                    onVideoEnd={() => {
                      // Always return to standby graph after video ends
                      setShowStandbyGraph(true);
                      setIsVideoPlaying(false);
                      localStorage.setItem('nava-display-standby', 'true');
                    }}
                    onVideoPlay={() => {
                      setIsVideoPlaying(true);
                      setShowStandbyGraph(false);
                      localStorage.setItem('nava-display-standby', 'false');
                    }}
                  />
                </>
              ) : (
                <>
                  <div className="display-grid-overlay"></div>
                  <div className="display-graph">
                    {/* Graph visualization */}
                    <svg className="display-svg" viewBox="0 0 800 450" preserveAspectRatio="xMidYMid meet">
                      <defs>
                        <pattern id="grid-pattern" width="40" height="40" patternUnits="userSpaceOnUse">
                          <path d="M 40 0 L 0 0 0 40" fill="none" stroke="rgba(59, 130, 246, 0.2)" strokeWidth="1"/>
                        </pattern>
                      </defs>
                      <rect width="100%" height="100%" fill="url(#grid-pattern)" />
                      
                      {/* S-curve path */}
                      <path
                        d="M 100 350 Q 200 250 400 300 T 700 200"
                        fill="none"
                        stroke="#10b981"
                        strokeWidth="3"
                        className="graph-curve"
                      />
                      
                      {/* Nodes */}
                      <circle cx="100" cy="350" r="8" fill="#10b981" />
                      <circle cx="400" cy="300" r="8" fill="#3b82f6" />
                      <circle cx="700" cy="200" r="8" fill="#10b981" />
                      
                      {/* L-shape path */}
                      <line x1="200" y1="150" x2="400" y2="150" stroke="#3b82f6" strokeWidth="2" />
                      <line x1="400" y1="150" x2="400" y2="50" stroke="#3b82f6" strokeWidth="2" />
                      <circle cx="400" cy="50" r="8" fill="#3b82f6" />
                      
                      {/* A shape overlay */}
                      <path
                        d="M 350 300 L 400 150 L 450 300 M 370 250 L 430 250"
                        fill="none"
                        stroke="rgba(59, 130, 246, 0.3)"
                        strokeWidth="2"
                      />
                    </svg>
                  </div>
                </>
              )}
            </div>
          </div>
        </div>
        </>
      )}

      {/* Picture-in-Picture Display */}
      {pipSettings.enabled && (
        <div
          className="desktop-branding-element pip-screen-element"
          style={{
            left: `${pipSettings.x}px`,
            top: `${pipSettings.y}px`,
            width: `${pipSettings.width}px`,
            height: `${pipSettings.height}px`,
            cursor: pipDragging ? 'grabbing' : 'grab',
            zIndex: 1002,
          }}
          onMouseDown={handlePipMouseDown}
        >
          <div className="pip-screen-container">
            <button
              className="pip-close-btn"
              onClick={(e) => {
                e.stopPropagation();
                setPipSettings(prev => ({ ...prev, enabled: false }));
              }}
              title="Close Picture-in-Picture"
            >
              ✕
            </button>
            <div className="pip-screen-content">
              <YouTubeEmbed 
                videoId={extractYouTubeId(pipSettings.videoUrl)}
                autoplay={pipSettings.autoplay}
                loop={pipSettings.loop}
                muted={pipSettings.muted}
                controls={pipSettings.showControls}
              />
            </div>
          </div>
        </div>
      )}
    </>
  );
};

// Extract YouTube video ID from URL
const extractYouTubeId = (url: string): string => {
  // Handle embed URLs directly
  if (url.includes('/embed/')) {
    const match = url.match(/\/embed\/([^?#&]+)/);
    return match ? match[1] : '';
  }
  // Handle watch URLs and other formats
  const regExp = /^.*(youtu.be\/|v\/|u\/\w\/|embed\/|watch\?v=|&v=)([^#&?]*).*/;
  const match = url.match(regExp);
  return (match && match[2].length === 11) ? match[2] : '';
};

// YouTube Embed Component
interface YouTubeEmbedProps {
  videoId: string;
  autoplay?: boolean;
  loop?: boolean;
  muted?: boolean;
  controls?: boolean;
  onVideoEnd?: () => void;
  onVideoPlay?: () => void;
}

const YouTubeEmbed: React.FC<YouTubeEmbedProps> = ({ 
  videoId, 
  autoplay = true, 
  loop = true, 
  muted = false,
  controls = true,
  onVideoEnd,
  onVideoPlay
}) => {
  const iframeRef = React.useRef<HTMLIFrameElement>(null);
  const [iframeKey, setIframeKey] = React.useState(0);

  React.useEffect(() => {
    const iframe = iframeRef.current;
    if (!iframe) return;

    const handleMessage = (event: MessageEvent) => {
      // Only process messages from YouTube
      if (event.origin !== 'https://www.youtube.com') return;

      try {
        const data = typeof event.data === 'string' ? JSON.parse(event.data) : event.data;
        
        // YouTube iframe API events
        if (data.event === 'onStateChange') {
          // 0 = ended, 1 = playing, 2 = paused
          if (data.info === 0 && onVideoEnd) {
            // Video ended - always return to standby
            onVideoEnd();
          } else if (data.info === 1 && onVideoPlay) {
            // Video playing
            onVideoPlay();
          }
        }
      } catch (e) {
        // Not a JSON message, ignore
      }
    };

    // Listen for play video event - reload iframe to start video
    const handlePlayVideo = () => {
      setIframeKey(prev => prev + 1); // Force iframe reload
      if (onVideoPlay) {
        setTimeout(() => onVideoPlay(), 500);
      }
    };

    window.addEventListener('message', handleMessage);
    window.addEventListener('nava:play-video', handlePlayVideo);
    
    return () => {
      window.removeEventListener('message', handleMessage);
      window.removeEventListener('nava:play-video', handlePlayVideo);
    };
  }, [onVideoEnd, onVideoPlay, loop]);

  const embedUrl = `https://www.youtube.com/embed/${videoId}?${new URLSearchParams({
    autoplay: autoplay ? '1' : '0',
    loop: loop ? '1' : '0',
    playlist: loop ? videoId : '',
    mute: muted ? '1' : '0',
    controls: controls ? '1' : '0',
    rel: '0',
    modestbranding: '1',
    playsinline: '1',
    iv_load_policy: '3', // Hide annotations
    showinfo: '0', // Hide video info
    fs: '0', // Disable fullscreen button
    cc_load_policy: '0', // Hide captions by default
    enablejsapi: '1', // Enable JS API for event handling
  }).toString()}`;

  // Determine title based on video ID
  const getVideoTitle = (id: string): string => {
    if (id === 'Eu5mYMavctM') {
      return 'Introducing Figure 03';
    } else if (id === 'W1Ftle-w8HQ') {
      return 'Tesla Optimus Gen 3: Elon Musk Reveals a New Humanoid Robot';
    } else if (id === '6c8M0_EYa1c') {
      return 'These Unitree Robots Are So Advanced... It\'s UNREAL! (Full Lineup Review)';
    }
    return 'NAVΛ Display Screen';
  };

  return (
    <iframe
      key={iframeKey}
      ref={iframeRef}
      className="youtube-embed"
      src={embedUrl}
      allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share"
      allowFullScreen
      frameBorder="0"
      title={getVideoTitle(videoId)}
      referrerPolicy="strict-origin-when-cross-origin"
    />
  );
};

