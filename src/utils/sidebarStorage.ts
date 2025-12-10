/**
 * NAVA Sidebar Storage & Preferences
 * 
 * Manages sidebar state, view modes, and customization
 */

export type ViewMode = 'icons' | 'list' | 'columns' | 'gallery';

export interface SidebarSection {
  id: string;
  title: string;
  items: SidebarItem[];
  collapsed: boolean;
  viewMode: ViewMode;
  order: number;
}

export interface SidebarItem {
  id: string;
  name: string;
  icon: string;
  route?: string;
  type: 'folder' | 'file' | 'link' | 'app';
  children?: SidebarItem[];
}

const STORAGE_KEY = 'nava-sidebar-preferences';

export interface SidebarPreferences {
  sections: SidebarSection[];
  sidebarWidth: number;
  showIcons: boolean;
  showToolbar: boolean;
}

/**
 * Get sidebar preferences from storage
 */
export function getSidebarPreferences(): SidebarPreferences {
  try {
    const stored = localStorage.getItem(STORAGE_KEY);
    if (!stored) {
      return getDefaultPreferences();
    }
    return JSON.parse(stored);
  } catch (error) {
    console.error('[SidebarStorage] Error reading preferences:', error);
    return getDefaultPreferences();
  }
}

/**
 * Save sidebar preferences to storage
 */
export function saveSidebarPreferences(prefs: SidebarPreferences): void {
  try {
    localStorage.setItem(STORAGE_KEY, JSON.stringify(prefs));
  } catch (error) {
    console.error('[SidebarStorage] Error saving preferences:', error);
  }
}

/**
 * Toggle section collapse state
 */
export function toggleSection(sectionId: string): void {
  const prefs = getSidebarPreferences();
  const section = prefs.sections.find(s => s.id === sectionId);
  if (section) {
    section.collapsed = !section.collapsed;
    saveSidebarPreferences(prefs);
  }
}

/**
 * Set section view mode
 */
export function setSectionViewMode(sectionId: string, viewMode: ViewMode): void {
  const prefs = getSidebarPreferences();
  const section = prefs.sections.find(s => s.id === sectionId);
  if (section) {
    section.viewMode = viewMode;
    saveSidebarPreferences(prefs);
  }
}

/**
 * Set sidebar width
 */
export function setSidebarWidth(width: number): void {
  const prefs = getSidebarPreferences();
  prefs.sidebarWidth = width;
  saveSidebarPreferences(prefs);
}

/**
 * Get default sidebar preferences
 */
function getDefaultPreferences(): SidebarPreferences {
  return {
    sections: [
      {
        id: 'favorites',
        title: 'Favorites',
        collapsed: false,
        viewMode: 'list',
        order: 0,
        items: [
          { id: 'iCloud', name: 'iCloud', icon: '☁️', type: 'folder' },
          { id: 'shared', name: 'Shared', icon: '👥', type: 'folder' },
        ],
      },
      {
        id: 'community-projects',
        title: 'Community Projects',
        collapsed: false,
        viewMode: 'list',
        order: 1,
        items: [
          { id: 'community', name: 'Community Projects', icon: '🌐', type: 'folder', route: '/app.html' },
          { id: 'tutorials', name: 'ROS 2 Tutorials', icon: '📚', type: 'folder', route: '/app.html?activity=ros-learning' },
          { id: 'demos', name: 'Navigation Demos', icon: '🕐', type: 'folder', route: '/app.html' },
        ],
      },
      {
        id: 'my-rosjects',
        title: 'My Rosjects',
        collapsed: false,
        viewMode: 'list',
        order: 2,
        items: [
          { id: 'nav-bot', name: 'Navigation Bot', icon: '🤖', type: 'folder', route: '/app.html' },
          { id: 'slam', name: 'SLAM Mapping', icon: '🗺️', type: 'folder', route: '/app.html' },
          { id: 'path-planning', name: 'Path Planning', icon: '🧭', type: 'folder', route: '/app.html' },
          { id: 'sensor-fusion', name: 'Sensor Fusion', icon: '📡', type: 'folder', route: '/app.html' },
        ],
      },
      {
        id: 'favorites-files',
        title: '⭐ Favorites',
        collapsed: false,
        viewMode: 'list',
        order: 3,
        items: [
          { id: 'nav-guide', name: 'Navigation Guide', icon: '📄', type: 'file', route: '/app.html?activity=explorer' },
          { id: 'ros2-cheat', name: 'ROS2 Cheat Sheet', icon: '📄', type: 'file', route: '/app.html?activity=explorer' },
          { id: 'calc-ref', name: 'Calculus Reference', icon: '📐', type: 'file', route: '/app.html?activity=explorer' },
        ],
      },
      {
        id: 'achievements',
        title: '🏆 Achievements',
        collapsed: false,
        viewMode: 'list',
        order: 4,
        items: [
          { id: 'ros2-master', name: 'ROS 2 Master', icon: '🏅', type: 'file', route: '/app.html' },
          { id: 'nav-expert', name: 'Navigation Expert', icon: '🧭', type: 'file', route: '/app.html' },
          { id: 'code-ninja', name: 'Code Ninja', icon: '🥷', type: 'file', route: '/app.html' },
        ],
      },
      {
        id: 'ros-learning',
        title: 'ROS 2 Learning Center',
        collapsed: false,
        viewMode: 'list',
        order: 5,
        items: [
          { id: 'courses', name: 'Interactive Courses', icon: '📚', type: 'folder', route: '/app.html?activity=ros-learning' },
          { id: 'simulator', name: 'Robot Simulator', icon: '🤖', type: 'folder', route: '/app.html?activity=simulation' },
          { id: 'workspace', name: 'ROS Workspace', icon: '⚙️', type: 'folder', route: '/app.html?activity=explorer' },
        ],
      },
      {
        id: 'portfolio',
        title: 'My Portfolio',
        collapsed: false,
        viewMode: 'list',
        order: 6,
        items: [
          { id: 'projects', name: 'Projects', icon: '📁', type: 'folder', route: '/app.html?activity=explorer' },
          { id: 'certs', name: 'Certifications', icon: '🏆', type: 'folder', route: '/app.html' },
        ],
      },
    ],
    sidebarWidth: 280,
    showIcons: true,
    showToolbar: true,
  };
}

