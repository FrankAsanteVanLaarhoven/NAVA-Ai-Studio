// Unregister Service Worker Script
// Run this in browser console to clear existing service workers

if ('serviceWorker' in navigator) {
  navigator.serviceWorker.getRegistrations().then((registrations) => {
    for (const registration of registrations) {
      registration.unregister().then((success) => {
        if (success) {
          console.log('✅ Service Worker unregistered successfully');
        } else {
          console.log('⚠️ Service Worker unregistration failed');
        }
      });
    }
  });
  
  // Clear all caches
  if ('caches' in window) {
    caches.keys().then((cacheNames) => {
      return Promise.all(
        cacheNames.map((cacheName) => {
          console.log('🗑️ Deleting cache:', cacheName);
          return caches.delete(cacheName);
        })
      );
    }).then(() => {
      console.log('✅ All caches cleared');
      console.log('🔄 Please reload the page');
    });
  }
} else {
  console.log('ℹ️ Service Workers not supported');
}

