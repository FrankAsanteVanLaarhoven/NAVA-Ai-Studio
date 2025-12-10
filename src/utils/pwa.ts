// PWA Utilities for NAVΛ Studio

export const registerServiceWorker = async (): Promise<ServiceWorkerRegistration | null> => {
  if ('serviceWorker' in navigator) {
    try {
      // Try service-worker.js first, fallback to sw.js
      let registration: ServiceWorkerRegistration | null = null;
      try {
        registration = await navigator.serviceWorker.register('/service-worker.js', {
          scope: '/',
          updateViaCache: 'none'
        });
        console.log('[PWA] Service Worker registered: service-worker.js');
      } catch (e) {
        try {
          registration = await navigator.serviceWorker.register('/sw.js', {
            scope: '/',
            updateViaCache: 'none'
          });
          console.log('[PWA] Service Worker registered: sw.js');
        } catch (e2) {
          console.warn('[PWA] Service Worker registration failed:', e2);
          return null;
        }
      }
      
      if (!registration) {
        return null;
      }

      console.log('[PWA] Service Worker registered:', registration.scope);

      // Check for updates
      registration.addEventListener('updatefound', () => {
        const newWorker = registration.installing;
        if (newWorker) {
          newWorker.addEventListener('statechange', () => {
            if (newWorker.state === 'installed' && navigator.serviceWorker.controller) {
              // New service worker available
              console.log('[PWA] New service worker available');
              // You can show a notification to the user here
            }
          });
        }
      });

      return registration;
    } catch (error) {
      console.error('[PWA] Service Worker registration failed:', error);
      return null;
    }
  } else {
    console.warn('[PWA] Service Workers are not supported');
    return null;
  }
};

export const unregisterServiceWorker = async (): Promise<boolean> => {
  if ('serviceWorker' in navigator) {
    try {
      const registration = await navigator.serviceWorker.getRegistration();
      if (registration) {
        const unregistered = await registration.unregister();
        console.log('[PWA] Service Worker unregistered:', unregistered);
        return unregistered;
      }
    } catch (error) {
      console.error('[PWA] Service Worker unregistration failed:', error);
    }
  }
  return false;
};

export const isPWAInstalled = (): boolean => {
  // Check if running as standalone (installed PWA)
  if (window.matchMedia('(display-mode: standalone)').matches) {
    return true;
  }

  // Check for iOS standalone mode
  if ((window.navigator as any).standalone === true) {
    return true;
  }

  // Check for Android
  if (document.referrer.includes('android-app://')) {
    return true;
  }

  // Check localStorage flag
  return localStorage.getItem('nava-pwa-installed') === 'true';
};

export const isOnline = (): boolean => {
  return navigator.onLine;
};

export const getCacheSize = async (): Promise<number> => {
  if ('storage' in navigator && 'estimate' in navigator.storage) {
    try {
      const estimate = await navigator.storage.estimate();
      return estimate.usage || 0;
    } catch (error) {
      console.error('[PWA] Failed to get cache size:', error);
    }
  }
  return 0;
};

export const clearAllCaches = async (): Promise<void> => {
  if ('caches' in window) {
    try {
      const cacheNames = await caches.keys();
      await Promise.all(cacheNames.map(name => caches.delete(name)));
      console.log('[PWA] All caches cleared');
    } catch (error) {
      console.error('[PWA] Failed to clear caches:', error);
    }
  }
};

export const requestNotificationPermission = async (): Promise<NotificationPermission> => {
  if ('Notification' in window) {
    if (Notification.permission === 'default') {
      return await Notification.requestPermission();
    }
    return Notification.permission;
  }
  return 'denied';
};

export const showNotification = (title: string, options?: NotificationOptions): void => {
  if ('Notification' in window && Notification.permission === 'granted') {
    new Notification(title, {
      icon: '/icons/icon-192x192.png',
      badge: '/icons/icon-72x72.png',
      ...options,
    });
  }
};

export const requestPersistentStorage = async (): Promise<boolean> => {
  if ('storage' in navigator && 'persist' in navigator.storage) {
    try {
      const isPersisted = await navigator.storage.persist();
      console.log('[PWA] Persistent storage:', isPersisted);
      return isPersisted;
    } catch (error) {
      console.error('[PWA] Failed to request persistent storage:', error);
      return false;
    }
  }
  return false;
};

// Background Sync
export const registerBackgroundSync = async (tag: string): Promise<void> => {
  if ('serviceWorker' in navigator && 'sync' in (ServiceWorkerRegistration.prototype as any)) {
    try {
      const registration = await navigator.serviceWorker.ready;
      await (registration as any).sync.register(tag);
      console.log('[PWA] Background sync registered:', tag);
    } catch (error) {
      console.error('[PWA] Background sync registration failed:', error);
    }
  }
};

// Periodic Background Sync (if supported)
export const registerPeriodicSync = async (tag: string, minInterval: number): Promise<void> => {
  if ('serviceWorker' in navigator && 'periodicSync' in (ServiceWorkerRegistration.prototype as any)) {
    try {
      const registration = await navigator.serviceWorker.ready;
      await (registration as any).periodicSync.register(tag, {
        minInterval,
      });
      console.log('[PWA] Periodic sync registered:', tag);
    } catch (error) {
      console.error('[PWA] Periodic sync registration failed:', error);
    }
  }
};

