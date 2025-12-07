/**
 * NAVΛ Studio App Registry
 * 
 * Central registry for all applications and their manifests
 */

export interface AppManifest {
  id: string;
  name: string;
  version: string;
  description?: string;
  icon?: string;
  category?: string;
  author?: string;
  keywords?: string[];
  capabilities?: string[];
  routes?: Array<{ path: string; component: string }>;
  actions?: Array<{
    id: string;
    title: string;
    prefix?: string;
    description?: string;
    shortcut?: string;
  }>;
  dock?: {
    enabled: boolean;
    icon: string;
    name: string;
    description?: string;
  };
  [key: string]: any;
}

// Global registry
const appRegistry: Map<string, AppManifest> = new Map();

/**
 * Register an app manifest
 */
export function register(manifest: AppManifest): void {
  if (!manifest.id) {
    console.error('[Registry] Cannot register app without id:', manifest);
    return;
  }
  
  if (appRegistry.has(manifest.id)) {
    console.warn(`[Registry] App '${manifest.id}' already registered. Overwriting.`);
  }
  
  appRegistry.set(manifest.id, manifest);
  console.log(`[Registry] Registered app: ${manifest.id} (${manifest.name})`);
}

/**
 * Unregister an app
 */
export function unregister(appId: string): boolean {
  const result = appRegistry.delete(appId);
  if (result) {
    console.log(`[Registry] Unregistered app: ${appId}`);
  }
  return result;
}

/**
 * Get a specific app manifest
 */
export function getApp(appId: string): AppManifest | undefined {
  return appRegistry.get(appId);
}

/**
 * Get all registered apps
 */
export function getAllApps(): AppManifest[] {
  return Array.from(appRegistry.values());
}

/**
 * Get apps by category
 */
export function getAppsByCategory(category: string): AppManifest[] {
  return getAllApps().filter(app => app.category === category);
}

/**
 * Get apps with specific capability
 */
export function getAppsByCapability(capability: string): AppManifest[] {
  return getAllApps().filter(app => 
    app.capabilities?.includes(capability)
  );
}

/**
 * Get all app actions (for command palette)
 */
export function getAllActions(): Array<{
  appId: string;
  action: AppManifest['actions'][0];
}> {
  const actions: Array<{ appId: string; action: any }> = [];
  
  for (const [appId, manifest] of appRegistry.entries()) {
    if (manifest.actions) {
      manifest.actions.forEach(action => {
        actions.push({ appId, action });
      });
    }
  }
  
  return actions;
}

/**
 * Check if an app is registered
 */
export function hasApp(appId: string): boolean {
  return appRegistry.has(appId);
}

/**
 * Get registry statistics
 */
export function getStats() {
  return {
    totalApps: appRegistry.size,
    categories: new Set(getAllApps().map(app => app.category).filter(Boolean)).size,
    totalActions: getAllActions().length,
  };
}

// Log registry stats when in development
if (import.meta.env.DEV) {
  // Wait a bit for apps to register
  setTimeout(() => {
    const stats = getStats();
    console.log('[Registry] Stats:', stats);
    console.log('[Registry] Registered apps:', Array.from(appRegistry.keys()));
  }, 1000);
}

export default {
  register,
  unregister,
  getApp,
  getAllApps,
  getAppsByCategory,
  getAppsByCapability,
  getAllActions,
  hasApp,
  getStats,
};

