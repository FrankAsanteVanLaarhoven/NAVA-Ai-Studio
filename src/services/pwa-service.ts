/**
 * PWA Service
 * 
 * Handles Progressive Web App functionality including:
 * - Service Worker registration
 * - Offline support
 * - Install prompts
 * - Update notifications
 */

class PWAService {
  private registration: ServiceWorkerRegistration | null = null;
  private deferredPrompt: any = null;

  async initialize(): Promise<void> {
    // Enable PWA in both dev and prod for full SDK functionality
    // Service worker is now enabled in dev mode for testing
    if ('serviceWorker' in navigator) {
      try {
        this.registration = await navigator.serviceWorker.register('/service-worker.js');
        console.log('✅ NAVΛ Studio: Service Worker registered');

        // Check for updates
        this.registration.addEventListener('updatefound', () => {
          const newWorker = this.registration?.installing;
          if (newWorker) {
            newWorker.addEventListener('statechange', () => {
              if (newWorker.state === 'installed' && navigator.serviceWorker.controller) {
                // New service worker available
                this.notifyUpdate();
              }
            });
          }
        });
      } catch (error) {
        console.error('NAVΛ Studio: Service Worker registration failed:', error);
      }
    }

    // Listen for install prompt
    window.addEventListener('beforeinstallprompt', (e) => {
      e.preventDefault();
      this.deferredPrompt = e;
      console.log('📱 NAVΛ Studio: Install prompt available');
    });
  }

  async showInstallPrompt(): Promise<boolean> {
    if (!this.deferredPrompt) {
      return false;
    }

    this.deferredPrompt.prompt();
    const { outcome } = await this.deferredPrompt.userChoice;
    console.log(`User ${outcome} the install prompt`);

    this.deferredPrompt = null;
    return outcome === 'accepted';
  }

  canInstall(): boolean {
    return this.deferredPrompt !== null;
  }

  private notifyUpdate(): void {
    const notification = confirm(
      'NAVΛ Studio update available! Reload to get the latest version?'
    );

    if (notification && this.registration?.waiting) {
      this.registration.waiting.postMessage({ type: 'SKIP_WAITING' });
      window.location.reload();
    }
  }

  async checkOnlineStatus(): Promise<boolean> {
    return navigator.onLine;
  }

  async cacheFile(url: string, content: string): Promise<void> {
    if ('caches' in window) {
      const cache = await caches.open('navlambda-user-files');
      const response = new Response(content);
      await cache.put(url, response);
    }
  }

  async getCachedFile(url: string): Promise<string | null> {
    if ('caches' in window) {
      const cache = await caches.open('navlambda-user-files');
      const response = await cache.match(url);
      return response ? await response.text() : null;
    }
    return null;
  }
}

export const pwaService = new PWAService();

