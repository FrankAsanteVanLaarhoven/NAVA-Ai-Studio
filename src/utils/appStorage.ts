/**
 * NAVA App Centre - App Storage & Management System
 * 
 * Production-ready CRUD operations for app management
 * Similar to Apple Finder's app organization
 */

export interface AppItem {
  id: string;
  name: string;
  icon: string;
  color?: string;
  description?: string;
  route?: string | null;
  onClick?: () => void;
  category?: string;
  inDock: boolean;
  createdAt: number;
  updatedAt: number;
}

const STORAGE_KEY = 'nava-app-centre-apps';
const DOCK_APPS_KEY = 'nava-dock-apps';

/**
 * Get all apps from storage
 */
export function getAllApps(): AppItem[] {
  try {
    const stored = localStorage.getItem(STORAGE_KEY);
    if (!stored) {
      // Initialize with default apps
      return getDefaultApps();
    }
    return JSON.parse(stored);
  } catch (error) {
    console.error('[AppStorage] Error reading apps:', error);
    return getDefaultApps();
  }
}

/**
 * Get apps that are in the dock
 */
export function getDockApps(): string[] {
  try {
    const stored = localStorage.getItem(DOCK_APPS_KEY);
    if (!stored) return [];
    return JSON.parse(stored);
  } catch (error) {
    console.error('[AppStorage] Error reading dock apps:', error);
    return [];
  }
}

/**
 * Save apps to storage
 */
export function saveApps(apps: AppItem[]): void {
  try {
    localStorage.setItem(STORAGE_KEY, JSON.stringify(apps));
  } catch (error) {
    console.error('[AppStorage] Error saving apps:', error);
  }
}

/**
 * Save dock apps list
 */
export function saveDockApps(appIds: string[]): void {
  try {
    localStorage.setItem(DOCK_APPS_KEY, JSON.stringify(appIds));
  } catch (error) {
    console.error('[AppStorage] Error saving dock apps:', error);
  }
}

/**
 * Create a new app
 */
export function createApp(app: Omit<AppItem, 'id' | 'createdAt' | 'updatedAt' | 'inDock'>): AppItem {
  const apps = getAllApps();
  const newApp: AppItem = {
    ...app,
    id: `app-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`,
    inDock: false,
    createdAt: Date.now(),
    updatedAt: Date.now(),
  };
  apps.push(newApp);
  saveApps(apps);
  return newApp;
}

/**
 * Update an existing app
 */
export function updateApp(appId: string, updates: Partial<Omit<AppItem, 'id' | 'createdAt'>>): AppItem | null {
  const apps = getAllApps();
  const index = apps.findIndex(app => app.id === appId);
  if (index === -1) return null;
  
  apps[index] = {
    ...apps[index],
    ...updates,
    updatedAt: Date.now(),
  };
  saveApps(apps);
  return apps[index];
}

/**
 * Delete an app
 */
export function deleteApp(appId: string): boolean {
  const apps = getAllApps();
  const index = apps.findIndex(app => app.id === appId);
  if (index === -1) return false;
  
  apps.splice(index, 1);
  saveApps(apps);
  
  // Also remove from dock if present
  const dockApps = getDockApps();
  const dockIndex = dockApps.indexOf(appId);
  if (dockIndex !== -1) {
    dockApps.splice(dockIndex, 1);
    saveDockApps(dockApps);
  }
  
  return true;
}

/**
 * Rename an app
 */
export function renameApp(appId: string, newName: string): boolean {
  const app = updateApp(appId, { name: newName });
  return app !== null;
}

/**
 * Add app to dock
 */
export function addToDock(appId: string): void {
  const dockApps = getDockApps();
  if (!dockApps.includes(appId)) {
    dockApps.push(appId);
    saveDockApps(dockApps);
  }
  updateApp(appId, { inDock: true });
}

/**
 * Remove app from dock
 */
export function removeFromDock(appId: string): void {
  const dockApps = getDockApps();
  const index = dockApps.indexOf(appId);
  if (index !== -1) {
    dockApps.splice(index, 1);
    saveDockApps(dockApps);
  }
  updateApp(appId, { inDock: false });
}

/**
 * Get default apps (ROBOTIS & Univarm apps)
 */
function getDefaultApps(): AppItem[] {
  const now = Date.now();
  return [
    {
      id: 'robotis-systemic',
      name: 'NAVΛ RS1',
      icon: '🔷',
      color: '#3b82f6',
      description: 'Enterprise Robot Control Platform',
      route: null,
      category: 'Robotics',
      inDock: true,
      createdAt: now,
      updatedAt: now,
      onClick: () => {
        // This will be handled by App Centre's handleAppClick
        // which dispatches the custom event
        const event = new CustomEvent('nava:open-robotis-overlay');
        window.dispatchEvent(event);
      },
    },
    {
      id: 'univarm-starter',
      name: 'Univarm Starter',
      icon: '⚡',
      color: '#facc15',
      description: 'Multi-Language Code Generation',
      route: '/app.html?activity=univarm-starter',
      category: 'Development',
      inDock: true,
      createdAt: now,
      updatedAt: now,
    },
    {
      id: 'univarm-advanced',
      name: 'Univarm Advanced',
      icon: '🦀',
      color: '#f97316',
      description: 'Real-Time Path Planning & SSE',
      route: '/app.html?activity=univarm-advanced',
      category: 'Robotics',
      inDock: true,
      createdAt: now,
      updatedAt: now,
    },
  ];
}

/**
 * Get apps by category
 */
export function getAppsByCategory(category?: string): AppItem[] {
  const apps = getAllApps();
  if (!category) return apps;
  return apps.filter(app => app.category === category);
}

/**
 * Get all categories
 */
export function getCategories(): string[] {
  const apps = getAllApps();
  const categories = new Set<string>();
  apps.forEach(app => {
    if (app.category) {
      categories.add(app.category);
    }
  });
  return Array.from(categories).sort();
}

