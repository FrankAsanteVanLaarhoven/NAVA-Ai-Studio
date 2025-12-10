/**
 * Browser Compatibility Utilities
 * 
 * Ensures cross-browser compatibility and graceful degradation
 */

export interface BrowserInfo {
  name: string;
  version: number;
  isSupported: boolean;
  features: {
    localStorage: boolean;
    sessionStorage: boolean;
    fetch: boolean;
    webSocket: boolean;
    iframe: boolean;
    backdropFilter: boolean;
    cssGrid: boolean;
    flexbox: boolean;
  };
}

/**
 * Detect browser and check compatibility
 */
export function detectBrowser(): BrowserInfo {
  const userAgent = navigator.userAgent;
  let name = 'Unknown';
  let version = 0;

  // Detect browser
  if (userAgent.indexOf('Chrome') > -1 && userAgent.indexOf('Edg') === -1) {
    name = 'Chrome';
    const match = userAgent.match(/Chrome\/(\d+)/);
    version = match ? parseInt(match[1]) : 0;
  } else if (userAgent.indexOf('Firefox') > -1) {
    name = 'Firefox';
    const match = userAgent.match(/Firefox\/(\d+)/);
    version = match ? parseInt(match[1]) : 0;
  } else if (userAgent.indexOf('Safari') > -1 && userAgent.indexOf('Chrome') === -1) {
    name = 'Safari';
    const match = userAgent.match(/Version\/(\d+)/);
    version = match ? parseInt(match[1]) : 0;
  } else if (userAgent.indexOf('Edg') > -1) {
    name = 'Edge';
    const match = userAgent.match(/Edg\/(\d+)/);
    version = match ? parseInt(match[1]) : 0;
  } else if (userAgent.indexOf('Opera') > -1 || userAgent.indexOf('OPR') > -1) {
    name = 'Opera';
    const match = userAgent.match(/(?:Opera|OPR)\/(\d+)/);
    version = match ? parseInt(match[1]) : 0;
  }

  // Check feature support
  const features = {
    localStorage: checkLocalStorage(),
    sessionStorage: checkSessionStorage(),
    fetch: typeof fetch !== 'undefined',
    webSocket: typeof WebSocket !== 'undefined',
    iframe: true, // Always supported
    backdropFilter: checkBackdropFilter(),
    cssGrid: checkCSSGrid(),
    flexbox: checkFlexbox(),
  };

  // Determine if browser is supported
  const isSupported = 
    (name === 'Chrome' && version >= 90) ||
    (name === 'Firefox' && version >= 88) ||
    (name === 'Safari' && version >= 14) ||
    (name === 'Edge' && version >= 90) ||
    (name === 'Opera' && version >= 76);

  return {
    name,
    version,
    isSupported,
    features,
  };
}

/**
 * Check localStorage support
 */
function checkLocalStorage(): boolean {
  try {
    const test = '__localStorage_test__';
    localStorage.setItem(test, test);
    localStorage.removeItem(test);
    return true;
  } catch {
    return false;
  }
}

/**
 * Check sessionStorage support
 */
function checkSessionStorage(): boolean {
  try {
    const test = '__sessionStorage_test__';
    sessionStorage.setItem(test, test);
    sessionStorage.removeItem(test);
    return true;
  } catch {
    return false;
  }
}

/**
 * Check backdrop-filter CSS support
 */
function checkBackdropFilter(): boolean {
  if (typeof document === 'undefined') return false;
  const el = document.createElement('div');
  el.style.backdropFilter = 'blur(10px)';
  return el.style.backdropFilter !== '';
}

/**
 * Check CSS Grid support
 */
function checkCSSGrid(): boolean {
  if (typeof document === 'undefined') return false;
  const el = document.createElement('div');
  el.style.display = 'grid';
  return el.style.display === 'grid';
}

/**
 * Check Flexbox support
 */
function checkFlexbox(): boolean {
  if (typeof document === 'undefined') return false;
  const el = document.createElement('div');
  el.style.display = 'flex';
  return el.style.display === 'flex';
}

/**
 * Safe localStorage wrapper with fallback
 */
export const safeLocalStorage = {
  getItem: (key: string): string | null => {
    try {
      return localStorage.getItem(key);
    } catch {
      return null;
    }
  },
  setItem: (key: string, value: string): boolean => {
    try {
      localStorage.setItem(key, value);
      return true;
    } catch {
      return false;
    }
  },
  removeItem: (key: string): boolean => {
    try {
      localStorage.removeItem(key);
      return true;
    } catch {
      return false;
    }
  },
  clear: (): boolean => {
    try {
      localStorage.clear();
      return true;
    } catch {
      return false;
    }
  },
};

/**
 * Safe window.open wrapper
 */
export function safeWindowOpen(url: string, target: string = '_blank'): Window | null {
  try {
    return window.open(url, target);
  } catch (error) {
    console.warn('window.open blocked or failed:', error);
    // Fallback: try to navigate in current window
    try {
      window.location.href = url;
      return null;
    } catch {
      return null;
    }
  }
}

/**
 * Safe fetch wrapper with timeout
 */
export async function safeFetch(
  url: string,
  options: RequestInit = {},
  timeout: number = 10000
): Promise<Response> {
  const controller = new AbortController();
  const timeoutId = setTimeout(() => controller.abort(), timeout);

  try {
    const response = await fetch(url, {
      ...options,
      signal: controller.signal,
    });
    clearTimeout(timeoutId);
    return response;
  } catch (error) {
    clearTimeout(timeoutId);
    throw error;
  }
}

/**
 * Check if running in iframe
 */
export function isInIframe(): boolean {
  try {
    return window.self !== window.top;
  } catch {
    return true; // Assume iframe if we can't check
  }
}

/**
 * Get viewport dimensions
 */
export function getViewportSize(): { width: number; height: number } {
  return {
    width: window.innerWidth || document.documentElement.clientWidth || 0,
    height: window.innerHeight || document.documentElement.clientHeight || 0,
  };
}

/**
 * Show browser compatibility warning if needed
 */
export function checkBrowserCompatibility(): void {
  const browser = detectBrowser();

  if (!browser.isSupported) {
    console.warn(`[Browser Compatibility] ${browser.name} ${browser.version} may not be fully supported. Please use a modern browser.`);
  }

  // Check critical features
  if (!browser.features.localStorage) {
    console.warn('[Browser Compatibility] localStorage not available. Some features may not work.');
  }

  if (!browser.features.fetch) {
    console.warn('[Browser Compatibility] Fetch API not available. Network requests may fail.');
  }

  // Log feature support
  console.log('[Browser Compatibility]', {
    browser: `${browser.name} ${browser.version}`,
    supported: browser.isSupported,
    features: browser.features,
  });
}

/**
 * Initialize browser compatibility checks
 */
export function initializeBrowserCompatibility(): void {
  // Run checks on load
  if (typeof window !== 'undefined') {
    checkBrowserCompatibility();

    // Warn if in iframe (may have restrictions)
    if (isInIframe()) {
      console.info('[Browser Compatibility] Running in iframe. Some features may be restricted.');
    }
  }
}

