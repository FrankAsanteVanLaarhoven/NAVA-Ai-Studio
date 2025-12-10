// NAVΛ Studio Enterprise Service Worker
// Advanced PWA features: Offline support, Background Sync, Push Notifications, Cache Management

const CACHE_VERSION = 'navlambda-studio-v2.0.0';
const CACHE_NAME = `navlambda-studio-${CACHE_VERSION}`;
const RUNTIME_CACHE = 'navlambda-runtime-v1';
const IMAGE_CACHE = 'navlambda-images-v1';
const API_CACHE = 'navlambda-api-v1';

// Resources to cache on install
const PRECACHE_URLS = [
  '/',
  '/workspace.html',
  '/app.html',
  '/index.html',
  '/download.html',
  '/favicon.svg',
  '/favicon.ico',
  '/manifest.json',
  '/icons/icon-192x192.png',
  '/icons/icon-512x512.png',
  // SDK resources
  '/sdk/README.md',
  '/sdk/installers/NAVΛ-SDK-latest.dmg',
  '/NAVΛ-Studio-IDE.dmg',
];

// Install event - cache critical resources
self.addEventListener('install', (event) => {
  console.log('[SW] Installing service worker...', CACHE_NAME);
  
  event.waitUntil(
    caches.open(CACHE_NAME)
      .then((cache) => {
        console.log('[SW] Caching app shell');
        return cache.addAll(PRECACHE_URLS.map(url => new Request(url, { cache: 'reload' })));
      })
      .then(() => {
        console.log('[SW] App shell cached');
        return self.skipWaiting();
      })
      .catch((error) => {
        console.error('[SW] Install failed:', error);
      })
  );
});

// Activate event - clean up old caches
self.addEventListener('activate', (event) => {
  console.log('[SW] Activating service worker...');
  
  event.waitUntil(
    caches.keys()
      .then((cacheNames) => {
        return Promise.all(
          cacheNames.map((cacheName) => {
            if (cacheName !== CACHE_NAME && 
                cacheName !== RUNTIME_CACHE && 
                cacheName !== IMAGE_CACHE && 
                cacheName !== API_CACHE) {
              console.log('[SW] Deleting old cache:', cacheName);
              return caches.delete(cacheName);
            }
          })
        );
      })
      .then(() => {
        console.log('[SW] Service worker activated');
        // Claim clients after a short delay to ensure we're fully active
        return new Promise(resolve => {
          setTimeout(() => {
            self.clients.claim().catch(err => {
              console.warn('[SW] Could not claim clients:', err);
            });
            resolve();
          }, 200);
        });
      })
  );
});

// Fetch event - intelligent caching strategy
self.addEventListener('fetch', (event) => {
  const { request } = event;
  const url = new URL(request.url);

  // Skip non-GET requests
  if (request.method !== 'GET') {
    return;
  }

  // Skip chrome-extension and other non-http(s) requests
  if (!url.protocol.startsWith('http')) {
    return;
  }

  // Skip Vite dev server requests in development
  if (url.hostname === 'localhost' && 
      (url.pathname.startsWith('/@') || 
       url.pathname.includes('node_modules') ||
       url.pathname.includes('__vite'))) {
    return;
  }

  // Different strategies for different resource types
  if (url.pathname.startsWith('/api/')) {
    // API requests - Network first, cache fallback
    event.respondWith(networkFirstStrategy(request));
  } else if (request.destination === 'image') {
    // Images - Cache first, network fallback
    event.respondWith(cacheFirstStrategy(request, IMAGE_CACHE));
  } else if (request.destination === 'document') {
    // HTML pages - Network first with cache fallback
    event.respondWith(networkFirstStrategy(request));
  } else {
    // Static assets - Stale while revalidate
    event.respondWith(staleWhileRevalidateStrategy(request));
  }
});

// Network First Strategy - for dynamic content
async function networkFirstStrategy(request) {
  try {
    const networkResponse = await fetch(request);
    
    if (networkResponse.ok) {
      const cache = await caches.open(RUNTIME_CACHE);
      cache.put(request, networkResponse.clone());
    }
    
    return networkResponse;
  } catch (error) {
    console.log('[SW] Network failed, trying cache:', request.url);
    const cachedResponse = await caches.match(request);
    
    if (cachedResponse) {
      return cachedResponse;
    }
    
    // Return offline page for navigation requests
    if (request.mode === 'navigate') {
      return caches.match('/workspace.html') || 
             new Response('Offline - Please check your connection', {
               status: 503,
               headers: { 'Content-Type': 'text/html' }
             });
    }
    
    throw error;
  }
}

// Cache First Strategy - for static assets
async function cacheFirstStrategy(request, cacheName = CACHE_NAME) {
  const cachedResponse = await caches.match(request);
  
  if (cachedResponse) {
    return cachedResponse;
  }
  
  try {
    const networkResponse = await fetch(request);
    
    if (networkResponse.ok) {
      const cache = await caches.open(cacheName);
      cache.put(request, networkResponse.clone());
    }
    
    return networkResponse;
  } catch (error) {
    console.error('[SW] Fetch failed:', error);
    throw error;
  }
}

// Stale While Revalidate Strategy - for static assets
async function staleWhileRevalidateStrategy(request) {
  const cache = await caches.open(RUNTIME_CACHE);
  const cachedResponse = await caches.match(request);
  
  const fetchPromise = fetch(request).then((networkResponse) => {
    if (networkResponse.ok) {
      cache.put(request, networkResponse.clone());
    }
    return networkResponse;
  }).catch(() => {
    // Ignore network errors
  });
  
  return cachedResponse || fetchPromise;
}

// Background Sync - for offline actions
self.addEventListener('sync', (event) => {
  console.log('[SW] Background sync:', event.tag);
  
  if (event.tag === 'sync-files') {
    event.waitUntil(syncFiles());
  } else if (event.tag === 'sync-settings') {
    event.waitUntil(syncSettings());
  }
});

async function syncFiles() {
  // Sync files when back online
  try {
    const cache = await caches.open(RUNTIME_CACHE);
    const requests = await cache.keys();
    
    // Re-validate cached files
    for (const request of requests) {
      try {
        const response = await fetch(request);
        if (response.ok) {
          await cache.put(request, response);
        }
      } catch (error) {
        console.warn('[SW] Failed to sync file:', request.url);
      }
    }
  } catch (error) {
    console.error('[SW] Sync files failed:', error);
  }
}

async function syncSettings() {
  // Sync user settings when back online
  console.log('[SW] Syncing settings...');
  // Implementation depends on your settings storage
}

// Push Notifications
self.addEventListener('push', (event) => {
  console.log('[SW] Push notification received');
  
  const options = {
    body: event.data ? event.data.text() : 'New notification from NAVΛ Studio',
    icon: '/icons/icon-192x192.png',
    badge: '/icons/icon-72x72.png',
    vibrate: [200, 100, 200],
    tag: 'navlambda-notification',
    requireInteraction: false,
    actions: [
      {
        action: 'open',
        title: 'Open NAVΛ Studio'
      },
      {
        action: 'close',
        title: 'Close'
      }
    ]
  };
  
  event.waitUntil(
    self.registration.showNotification('NAVΛ Studio', options)
  );
});

// Notification Click
self.addEventListener('notificationclick', (event) => {
  console.log('[SW] Notification clicked:', event.action);
  
  event.notification.close();
  
  if (event.action === 'open' || !event.action) {
    event.waitUntil(
      clients.openWindow('/workspace.html')
    );
  }
});

// Message handling
self.addEventListener('message', (event) => {
  console.log('[SW] Message received:', event.data);
  
  if (event.data && event.data.type === 'SKIP_WAITING') {
    self.skipWaiting();
  } else if (event.data && event.data.type === 'CACHE_URLS') {
    // Cache additional URLs on demand
    event.waitUntil(
      caches.open(RUNTIME_CACHE).then((cache) => {
        return cache.addAll(event.data.urls);
      })
    );
  } else if (event.data && event.data.type === 'CLEAR_CACHE') {
    // Clear specific cache
    event.waitUntil(
      caches.delete(event.data.cacheName).then((success) => {
        event.ports[0].postMessage({ success });
      })
    );
  }
});

// Periodic Background Sync (if supported)
self.addEventListener('periodicsync', (event) => {
  if (event.tag === 'content-sync') {
    event.waitUntil(syncContent());
  }
});

async function syncContent() {
  console.log('[SW] Periodic sync triggered');
  // Sync content periodically
}
