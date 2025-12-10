/**
 * NAVA Media Centre - Media Storage & Management System
 * 
 * Production-ready CRUD operations for media app management
 * Similar to Apple Control Center
 */

export interface MediaItem {
  id: string;
  name: string;
  icon: string;
  color?: string;
  description?: string;
  route?: string | null;
  onClick?: () => void;
  category?: string;
  enabled: boolean;
  createdAt: number;
  updatedAt: number;
}

const STORAGE_KEY = 'nava-media-centre-apps';

/**
 * Get all media apps from storage
 */
export function getAllMediaApps(): MediaItem[] {
  try {
    const stored = localStorage.getItem(STORAGE_KEY);
    if (!stored) {
      // Initialize with default media apps
      return getDefaultMediaApps();
    }
    return JSON.parse(stored);
  } catch (error) {
    console.error('[MediaStorage] Error reading apps:', error);
    return getDefaultMediaApps();
  }
}

/**
 * Save media apps to storage
 */
export function saveMediaApps(apps: MediaItem[]): void {
  try {
    localStorage.setItem(STORAGE_KEY, JSON.stringify(apps));
  } catch (error) {
    console.error('[MediaStorage] Error saving apps:', error);
  }
}

/**
 * Create a new media app
 */
export function createMediaApp(app: Omit<MediaItem, 'id' | 'createdAt' | 'updatedAt' | 'enabled'>): MediaItem {
  const apps = getAllMediaApps();
  const newApp: MediaItem = {
    ...app,
    id: `media-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`,
    enabled: true,
    createdAt: Date.now(),
    updatedAt: Date.now(),
  };
  apps.push(newApp);
  saveMediaApps(apps);
  return newApp;
}

/**
 * Update an existing media app
 */
export function updateMediaApp(appId: string, updates: Partial<Omit<MediaItem, 'id' | 'createdAt'>>): MediaItem | null {
  const apps = getAllMediaApps();
  const index = apps.findIndex(app => app.id === appId);
  if (index === -1) return null;
  
  apps[index] = {
    ...apps[index],
    ...updates,
    updatedAt: Date.now(),
  };
  saveMediaApps(apps);
  return apps[index];
}

/**
 * Delete a media app
 */
export function deleteMediaApp(appId: string): boolean {
  const apps = getAllMediaApps();
  const index = apps.findIndex(app => app.id === appId);
  if (index === -1) return false;
  
  apps.splice(index, 1);
  saveMediaApps(apps);
  return true;
}

/**
 * Rename a media app
 */
export function renameMediaApp(appId: string, newName: string): boolean {
  const app = updateMediaApp(appId, { name: newName });
  return app !== null;
}

/**
 * Toggle media app enabled state
 */
export function toggleMediaApp(appId: string): void {
  const apps = getAllMediaApps();
  const app = apps.find(a => a.id === appId);
  if (app) {
    updateMediaApp(appId, { enabled: !app.enabled });
  }
}

/**
 * Get default media apps
 */
function getDefaultMediaApps(): MediaItem[] {
  const now = Date.now();
  return [
    {
      id: 'record',
      name: 'Record',
      icon: '🔴',
      color: '#ef4444',
      description: 'Screen Recording',
      route: '/app.html?activity=simulation',
      category: 'Recording',
      enabled: true,
      createdAt: now,
      updatedAt: now,
      onClick: () => {
        // Trigger screen recording
        const event = new CustomEvent('nava:start-recording');
        window.dispatchEvent(event);
      },
    },
    {
      id: 'camera',
      name: 'Camera',
      icon: '📷',
      color: '#3b82f6',
      description: 'Camera Access',
      route: '/app.html?activity=simulation',
      category: 'Media',
      enabled: true,
      createdAt: now,
      updatedAt: now,
      onClick: () => {
        // Trigger camera access
        navigator.mediaDevices.getUserMedia({ video: true })
          .then(stream => {
            const event = new CustomEvent('nava:camera-opened', { detail: { stream } });
            window.dispatchEvent(event);
          })
          .catch(err => console.error('Camera access error:', err));
      },
    },
    {
      id: 'fullscreen',
      name: 'Fullscreen',
      icon: '⛶',
      color: '#10b981',
      description: 'Toggle Fullscreen',
      category: 'Display',
      enabled: true,
      createdAt: now,
      updatedAt: now,
      onClick: () => {
        if (!document.fullscreenElement) {
          document.documentElement.requestFullscreen();
        } else {
          document.exitFullscreen();
        }
      },
    },
    {
      id: 'blueprint',
      name: 'Blueprint',
      icon: '📐',
      color: '#3b82f6',
      description: 'Blueprint Editor',
      route: '/app.html?activity=explorer',
      category: 'Design',
      enabled: true,
      createdAt: now,
      updatedAt: now,
    },
    {
      id: 'live-ad',
      name: 'Live Ad',
      icon: '📺',
      color: '#f59e0b',
      description: 'Live Advertisement Manager',
      route: '/app.html',
      category: 'Advertising',
      enabled: true,
      createdAt: now,
      updatedAt: now,
    },
    {
      id: 'youtube',
      name: 'YouTube',
      icon: '▶️',
      color: '#ff0000',
      description: 'YouTube Integration',
      route: 'https://youtube.com',
      category: 'Streaming',
      enabled: true,
      createdAt: now,
      updatedAt: now,
      onClick: () => {
        window.open('https://youtube.com', '_blank');
      },
    },
    {
      id: 'twitch',
      name: 'Twitch',
      icon: '🎮',
      color: '#9146ff',
      description: 'Twitch Streaming',
      route: 'https://twitch.tv',
      category: 'Streaming',
      enabled: true,
      createdAt: now,
      updatedAt: now,
      onClick: () => {
        window.open('https://twitch.tv', '_blank');
      },
    },
    {
      id: 'video-editor',
      name: 'Video Editor',
      icon: '🎬',
      color: '#8b5cf6',
      description: 'Video Editing Suite',
      route: '/app.html',
      category: 'Editing',
      enabled: true,
      createdAt: now,
      updatedAt: now,
    },
    {
      id: 'image-editor',
      name: 'Image Editor',
      icon: '🖼️',
      color: '#ec4899',
      description: 'Image Editing Tools',
      route: '/app.html',
      category: 'Editing',
      enabled: true,
      createdAt: now,
      updatedAt: now,
    },
    {
      id: 'web-builder',
      name: 'Web Builder',
      icon: '🌐',
      color: '#06b6d4',
      description: 'Web Design Studio',
      route: '/app.html',
      category: 'Design',
      enabled: true,
      createdAt: now,
      updatedAt: now,
    },
    {
      id: 'slides',
      name: 'Slides',
      icon: '📊',
      color: '#f97316',
      description: 'Presentation Creator',
      route: '/app.html',
      category: 'Productivity',
      enabled: true,
      createdAt: now,
      updatedAt: now,
    },
  ];
}

/**
 * Get media apps by category
 */
export function getMediaAppsByCategory(category?: string): MediaItem[] {
  const apps = getAllMediaApps();
  if (!category) return apps;
  return apps.filter(app => app.category === category);
}

/**
 * Get all categories
 */
export function getMediaCategories(): string[] {
  const apps = getAllMediaApps();
  const categories = new Set<string>();
  apps.forEach(app => {
    if (app.category) {
      categories.add(app.category);
    }
  });
  return Array.from(categories).sort();
}

