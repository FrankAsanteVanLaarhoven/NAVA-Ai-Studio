/**
 * Error Monitor Service
 * Automatically detects and handles 500 errors and other critical issues
 */

interface ErrorReport {
  id: string;
  timestamp: string;
  type: '500' | 'network' | 'syntax' | 'runtime' | 'unknown';
  url?: string;
  message: string;
  stack?: string;
  userAgent: string;
  resolved: boolean;
}

class ErrorMonitorService {
  private errors: ErrorReport[] = [];
  private readonly STORAGE_KEY = 'nava_error_reports';
  private listeners: Array<(error: ErrorReport) => void> = [];

  constructor() {
    this.initializeErrorHandling();
    this.loadStoredErrors();
  }

  /**
   * Initialize global error handlers
   */
  private initializeErrorHandling(): void {
    // Catch unhandled promise rejections
    window.addEventListener('unhandledrejection', (event) => {
      this.handleError({
        type: 'runtime',
        message: event.reason?.message || String(event.reason),
        stack: event.reason?.stack,
      });
      
      // Auto-fix common issues
      this.autoFixError(event.reason);
    });

    // Catch general errors (including ReferenceError)
    window.addEventListener('error', (event) => {
      this.handleError({
        type: 'runtime',
        message: event.message,
        stack: event.error?.stack,
        url: event.filename,
      });
      
      // Auto-fix common issues
      this.autoFixError(event.error);
    });

    // Monitor fetch requests for 500 errors
    this.interceptFetch();
  }

  /**
   * Auto-fix common errors
   */
  private autoFixError(error: any): void {
    const errorMessage = error?.message || String(error);
    
    // Fix undefined variable errors
    if (errorMessage.includes('is not defined')) {
      const match = errorMessage.match(/(\w+) is not defined/);
      if (match) {
        const varName = match[1];
        console.warn(`[Auto-Debug] Detected undefined variable: ${varName}`);
        console.warn(`[Auto-Debug] This usually means the variable needs to be declared or imported.`);
        
        // Try to reload if it's a critical error
        if (errorMessage.includes('showProjectManager') || 
            errorMessage.includes('setShowProjectManager')) {
          console.warn('[Auto-Debug] Detected showProjectManager error - page may need refresh');
          // Don't auto-reload, just log - let the code fix handle it
        }
      }
    }
    
    // Fix module import errors
    if (errorMessage.includes('Failed to resolve import') || 
        errorMessage.includes('Cannot find module')) {
      console.warn('[Auto-Debug] Module import error detected');
      console.warn('[Auto-Debug] This usually means a file is missing or path is incorrect');
    }
  }

  /**
   * Intercept fetch requests to detect 500 errors
   */
  private interceptFetch(): void {
    const originalFetch = window.fetch;
    window.fetch = async (...args) => {
      try {
        const response = await originalFetch(...args);
        
        // Check for 500 errors
        if (response.status === 500) {
          const url = typeof args[0] === 'string' ? args[0] : args[0].url;
          this.handleError({
            type: '500',
            message: `Server Error (500): ${response.statusText}`,
            url,
          });
        }

        // Check for other critical errors
        if (response.status >= 500 && response.status < 600) {
          const url = typeof args[0] === 'string' ? args[0] : args[0].url;
          this.handleError({
            type: '500',
            message: `Server Error (${response.status}): ${response.statusText}`,
            url,
          });
        }

        return response;
      } catch (error: any) {
        // Network errors
        if (error.message?.includes('Failed to fetch') || error.message?.includes('NetworkError')) {
          this.handleError({
            type: 'network',
            message: error.message || 'Network request failed',
            url: typeof args[0] === 'string' ? args[0] : args[0]?.url,
          });
        }
        throw error;
      }
    };
  }

  /**
   * Handle an error
   */
  private handleError(error: Partial<ErrorReport>): void {
    const errorReport: ErrorReport = {
      id: `error_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`,
      timestamp: new Date().toISOString(),
      type: error.type || 'unknown',
      url: error.url,
      message: error.message || 'Unknown error',
      stack: error.stack,
      userAgent: navigator.userAgent,
      resolved: false,
    };

    this.errors.push(errorReport);
    this.saveErrors();
    this.notifyListeners(errorReport);

    // Auto-resolve common issues
    this.attemptAutoResolve(errorReport);

    // Log to console in development
    if (import.meta.env.DEV) {
      console.error('[Error Monitor]', errorReport);
    }
  }

  /**
   * Attempt to auto-resolve common errors
   */
  private attemptAutoResolve(error: ErrorReport): void {
    // Auto-reload on 500 errors from Vite dev server
    if (error.type === '500' && error.url?.includes('/@vite/')) {
      console.warn('[Error Monitor] Vite dev server error detected. Attempting auto-reload...');
      setTimeout(() => {
        if (confirm('A development server error was detected. Reload the page?')) {
          window.location.reload();
        }
      }, 1000);
    }

    // Clear cache on repeated 500 errors
    if (this.getRecentErrors('500').length > 5) {
      console.warn('[Error Monitor] Multiple 500 errors detected. Consider clearing cache.');
    }
  }

  /**
   * Get recent errors
   */
  getRecentErrors(type?: ErrorReport['type'], limit = 10): ErrorReport[] {
    let filtered = this.errors.filter(e => !e.resolved);
    if (type) {
      filtered = filtered.filter(e => e.type === type);
    }
    return filtered.slice(-limit).reverse();
  }

  /**
   * Mark error as resolved
   */
  resolveError(errorId: string): void {
    const error = this.errors.find(e => e.id === errorId);
    if (error) {
      error.resolved = true;
      this.saveErrors();
    }
  }

  /**
   * Clear all errors
   */
  clearErrors(): void {
    this.errors = [];
    this.saveErrors();
  }

  /**
   * Subscribe to error events
   */
  onError(callback: (error: ErrorReport) => void): () => void {
    this.listeners.push(callback);
    return () => {
      this.listeners = this.listeners.filter(l => l !== callback);
    };
  }

  /**
   * Notify all listeners
   */
  private notifyListeners(error: ErrorReport): void {
    this.listeners.forEach(listener => {
      try {
        listener(error);
      } catch (e) {
        console.error('[Error Monitor] Listener error:', e);
      }
    });
  }

  /**
   * Save errors to localStorage
   */
  private saveErrors(): void {
    try {
      localStorage.setItem(this.STORAGE_KEY, JSON.stringify(this.errors.slice(-100))); // Keep last 100
    } catch (e) {
      console.error('[Error Monitor] Failed to save errors:', e);
    }
  }

  /**
   * Load errors from localStorage
   */
  private loadStoredErrors(): void {
    try {
      const stored = localStorage.getItem(this.STORAGE_KEY);
      if (stored) {
        this.errors = JSON.parse(stored);
      }
    } catch (e) {
      console.error('[Error Monitor] Failed to load errors:', e);
    }
  }

  /**
   * Get error statistics
   */
  getStats(): {
    total: number;
    unresolved: number;
    byType: Record<string, number>;
  } {
    const unresolved = this.errors.filter(e => !e.resolved);
    const byType: Record<string, number> = {};
    
    unresolved.forEach(error => {
      byType[error.type] = (byType[error.type] || 0) + 1;
    });

    return {
      total: this.errors.length,
      unresolved: unresolved.length,
      byType,
    };
  }
}

export const errorMonitorService = new ErrorMonitorService();

