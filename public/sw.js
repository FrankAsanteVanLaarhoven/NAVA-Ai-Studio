// NAVΛ Studio IDE - Service Worker
// Van Laarhoven Navigation Calculus - Offline PWA
// Version 2.0 - Full Offline Capabilities

const CACHE_VERSION = 'navlambda-v2.0';
const RUNTIME_CACHE = 'navlambda-runtime';
const STATIC_CACHE = 'navlambda-static-v2.0';

// Complete list of files to cache for offline usage
const STATIC_FILES = [
  '/',
  '/index.html',
  '/app.html',
  '/workspace.html',
  '/download.html',
  '/manifest.json',
  '/favicon.svg',
  '/favicon.ico',
  '/icon-192x192.png',
  '/icon-512x512.png',
];

// Dynamic cache patterns
const CACHE_PATTERNS = {
  fonts: /\.(woff2?|ttf|otf|eot)$/,
  images: /\.(png|jpg|jpeg|svg|gif|webp|ico)$/,
  scripts: /\.(js|mjs|jsx|ts|tsx)$/,
  styles: /\.(css|scss|sass)$/,
  wasm: /\.(wasm)$/,
  data: /\.(json|xml|yaml|yml)$/,
};

// Install event - cache static assets
self.addEventListener('install', (event) => {
  console.log('⋋ NAVΛ Service Worker: Installing...');
  
  event.waitUntil(
    caches.open(STATIC_CACHE)
      .then((cache) => {
        console.log('⋋ NAVΛ Service Worker: Caching static files');
        return cache.addAll(STATIC_FILES);
      })
      .then(() => {
        console.log('⋋ NAVΛ Service Worker: Static files cached');
        return self.skipWaiting();
      })
      .catch((error) => {
        console.error('⋋ NAVΛ Service Worker: Cache error', error);
      })
  );
});

// Activate event - clean up old caches
self.addEventListener('activate', (event) => {
  console.log('⋋ NAVΛ Service Worker: Activating...');
  
  event.waitUntil(
    caches.keys()
      .then((cacheNames) => {
        return Promise.all(
          cacheNames
            .filter((cacheName) => {
              return cacheName.startsWith('navlambda-') && 
                     cacheName !== STATIC_CACHE && 
                     cacheName !== RUNTIME_CACHE;
            })
            .map((cacheName) => {
              console.log('⋋ NAVΛ Service Worker: Deleting old cache:', cacheName);
              return caches.delete(cacheName);
            })
        );
      })
      .then(() => {
        console.log('⋋ NAVΛ Service Worker: Activated');
        return self.clients.claim();
      })
  );
});

// Fetch event - serve from cache, fallback to network
self.addEventListener('fetch', (event) => {
  const { request } = event;
  const url = new URL(request.url);

  // Skip cross-origin requests
  if (url.origin !== location.origin) {
    return;
  }

  // Network-first for HTML
  if (request.mode === 'navigate') {
    event.respondWith(
      fetch(request)
        .then((response) => {
          const responseToCache = response.clone();
          caches.open(RUNTIME_CACHE).then((cache) => {
            cache.put(request, responseToCache);
          });
          return response;
        })
        .catch(() => {
          return caches.match(request)
            .then((cachedResponse) => {
              return cachedResponse || caches.match('/index.html');
            });
        })
    );
    return;
  }

  // Cache-first for static assets
  event.respondWith(
    caches.match(request)
      .then((cachedResponse) => {
        if (cachedResponse) {
          return cachedResponse;
        }

        return fetch(request)
          .then((response) => {
            // Only cache successful responses
            if (!response || response.status !== 200 || response.type === 'error') {
              return response;
            }

            // Check if response should be cached
            const shouldCache = Object.values(CACHE_PATTERNS).some((pattern) => {
              return pattern.test(url.pathname);
            });

            if (shouldCache) {
              const responseToCache = response.clone();
              caches.open(RUNTIME_CACHE).then((cache) => {
                cache.put(request, responseToCache);
              });
            }

            return response;
          })
          .catch(() => {
            // Return offline page for navigation requests
            if (request.mode === 'navigate') {
              return caches.match('/index.html');
            }
            return new Response('⋋ NAVΛ: Offline', {
              status: 503,
              statusText: 'Service Unavailable',
              headers: new Headers({
                'Content-Type': 'text/plain',
              }),
            });
          });
      })
  );
});

// Background sync for offline actions
self.addEventListener('sync', (event) => {
  console.log('⋋ NAVΛ Service Worker: Background sync', event.tag);
  
  if (event.tag === 'sync-navigation-data') {
    event.waitUntil(syncNavigationData());
  }
});

// Push notifications
self.addEventListener('push', (event) => {
  const options = {
    body: event.data ? event.data.text() : '⋋ NAVΛ Studio IDE',
    icon: '/icon-192x192.png',
    badge: '/icon-192x192.png',
    vibrate: [200, 100, 200],
    data: {
      dateOfArrival: Date.now(),
      primaryKey: 1,
    },
  };

  event.waitUntil(
    self.registration.showNotification('⋋ NAVΛ Studio', options)
  );
});

// Notification click
self.addEventListener('notificationclick', (event) => {
  console.log('⋋ NAVΛ Service Worker: Notification clicked');
  event.notification.close();

  event.waitUntil(
    clients.openWindow('/')
  );
});

// Message event - communicate with main app
self.addEventListener('message', (event) => {
  console.log('⋋ NAVΛ Service Worker: Message received', event.data);

  if (event.data && event.data.type === 'SKIP_WAITING') {
    self.skipWaiting();
  }

  if (event.data && event.data.type === 'CACHE_URLS') {
    const urlsToCache = event.data.payload;
    event.waitUntil(
      caches.open(RUNTIME_CACHE)
        .then((cache) => cache.addAll(urlsToCache))
    );
  }

  if (event.data && event.data.type === 'CLEAR_CACHE') {
    event.waitUntil(
      caches.keys().then((cacheNames) => {
        return Promise.all(
          cacheNames.map((cacheName) => caches.delete(cacheName))
        );
      })
    );
  }
});

// Helper function to sync navigation data
async function syncNavigationData() {
  try {
    // Get pending navigation data from IndexedDB
    const db = await openNavigationDB();
    const pendingData = await getPendingNavigationData(db);

    // Sync with server
    for (const data of pendingData) {
      await fetch('/api/navigation/sync', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(data),
      });
    }

    console.log('⋋ NAVΛ Service Worker: Navigation data synced');
  } catch (error) {
    console.error('⋋ NAVΛ Service Worker: Sync error', error);
    throw error;
  }
}

// Helper function to open IndexedDB
function openNavigationDB() {
  return new Promise((resolve, reject) => {
    const request = indexedDB.open('NavLambdaDB', 1);
    request.onsuccess = () => resolve(request.result);
    request.onerror = () => reject(request.error);
    request.onupgradeneeded = (event) => {
      const db = event.target.result;
      if (!db.objectStoreNames.contains('navigation')) {
        db.createObjectStore('navigation', { keyPath: 'id', autoIncrement: true });
      }
    };
  });
}

// Helper function to get pending data
function getPendingNavigationData(db) {
  return new Promise((resolve, reject) => {
    const transaction = db.transaction(['navigation'], 'readonly');
    const store = transaction.objectStore('navigation');
    const request = store.getAll();
    request.onsuccess = () => resolve(request.result);
    request.onerror = () => reject(request.error);
  });
}

console.log('⋋ NAVΛ Studio IDE - Service Worker Loaded');
console.log('Van Laarhoven Navigation Calculus - Offline Enabled');

