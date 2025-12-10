/**
 * Backend Auto-Start Service
 * 
 * Automatically checks and starts the NAVΛ RS1 backend when needed
 */

class BackendAutoStartService {
  private checkInterval: NodeJS.Timeout | null = null;
  private isChecking = false;
  private lastCheckTime = 0;
  private readonly CHECK_INTERVAL = 10000; // Check every 10 seconds
  private readonly BACKEND_URL = 'http://localhost:3000';
  private readonly START_SCRIPT = './start-robotis-system.sh';

  /**
   * Check if backend is running
   */
  async checkBackend(): Promise<boolean> {
    try {
      const controller = new AbortController();
      const timeoutId = setTimeout(() => controller.abort(), 2000);

      try {
        await fetch(this.BACKEND_URL, {
          method: 'HEAD',
          mode: 'no-cors',
          cache: 'no-cache',
          signal: controller.signal,
        });
        clearTimeout(timeoutId);
        return true;
      } catch (error: any) {
        clearTimeout(timeoutId);
        if (error.name === 'AbortError' || error.message?.includes('Failed to fetch')) {
          return false;
        }
        return false;
      }
    } catch {
      return false;
    }
  }

  /**
   * Start backend automatically (via API endpoint or notification)
   */
  async startBackend(): Promise<boolean> {
    try {
      // Try to use a backend API endpoint if available
      // Otherwise, show notification to user
      const response = await fetch('/api/start-backend', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
      });

      if (response.ok) {
        return true;
      }
    } catch (error) {
      console.warn('[Backend Auto-Start] API endpoint not available, using fallback');
    }

    // Fallback: Store intent to start backend
    localStorage.setItem('nava-backend-start-requested', Date.now().toString());
    
    // Dispatch event for UI to handle
    window.dispatchEvent(new CustomEvent('nava:start-backend-requested'));
    
    return false;
  }

  /**
   * Start automatic health checking
   */
  startAutoCheck(onBackendStatusChange?: (isRunning: boolean) => void): void {
    if (this.checkInterval) {
      return; // Already checking
    }

    // Check immediately
    this.performCheck(onBackendStatusChange);

    // Then check periodically
    this.checkInterval = setInterval(() => {
      this.performCheck(onBackendStatusChange);
    }, this.CHECK_INTERVAL);
  }

  /**
   * Stop automatic health checking
   */
  stopAutoCheck(): void {
    if (this.checkInterval) {
      clearInterval(this.checkInterval);
      this.checkInterval = null;
    }
  }

  private async performCheck(onBackendStatusChange?: (isRunning: boolean) => void): Promise<void> {
    // Prevent concurrent checks
    if (this.isChecking) return;
    
    // Throttle checks (don't check more than once per 5 seconds)
    const now = Date.now();
    if (now - this.lastCheckTime < 5000) return;
    
    this.isChecking = true;
    this.lastCheckTime = now;

    try {
      const isRunning = await this.checkBackend();
      
      if (onBackendStatusChange) {
        onBackendStatusChange(isRunning);
      }

      // If backend is not running and we haven't tried to start it recently
      if (!isRunning) {
        const lastStartAttempt = localStorage.getItem('nava-backend-last-start-attempt');
        const now = Date.now();
        
        // Only try to start if we haven't tried in the last 5 minutes
        if (!lastStartAttempt || now - parseInt(lastStartAttempt) > 5 * 60 * 1000) {
          console.log('[Backend Auto-Start] Backend not running, attempting to start...');
          localStorage.setItem('nava-backend-last-start-attempt', now.toString());
          
          // Try to start backend
          await this.startBackend();
        }
      } else {
        // Backend is running, clear any error states
        localStorage.removeItem('nava-backend-last-start-attempt');
        localStorage.removeItem('nava-backend-start-requested');
      }
    } catch (error) {
      console.error('[Backend Auto-Start] Check failed:', error);
    } finally {
      this.isChecking = false;
    }
  }

  /**
   * Get current backend status
   */
  async getStatus(): Promise<{ isRunning: boolean; lastCheck: number }> {
    const isRunning = await this.checkBackend();
    return {
      isRunning,
      lastCheck: this.lastCheckTime,
    };
  }
}

export const backendAutoStartService = new BackendAutoStartService();

