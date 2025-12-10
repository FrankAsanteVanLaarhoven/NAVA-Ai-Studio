# NAVΛ Studio PWA Setup Guide

## Overview

NAVΛ Studio IDE is now a fully-featured Progressive Web App (PWA) with enterprise-grade features that can be installed as a native app on any desktop or mobile device.

## Features Implemented

### ✅ Core PWA Features
- **Web App Manifest** - Complete manifest with all required icons, shortcuts, and metadata
- **Service Worker** - Advanced caching strategies for offline support
- **Install Prompt** - Smart install prompts for all platforms
- **Offline Support** - Full offline functionality with intelligent caching

### ✅ Enterprise Features
- **Background Sync** - Sync files and settings when back online
- **Push Notifications** - Support for push notifications (when permission granted)
- **File Handling** - Open files directly from the OS file manager
- **Protocol Handlers** - Custom protocol support (`web+navlambda://`)
- **Share Target** - Receive shared files and content
- **App Shortcuts** - Quick actions from the app icon
- **Persistent Storage** - Request persistent storage for better performance

### ✅ Platform Support
- **Desktop**: Windows, macOS, Linux (Chrome, Edge, Firefox)
- **Mobile**: iOS Safari, Android Chrome
- **Standalone Mode**: Runs like a native app

## Installation

### For Users

1. **Automatic Prompt**: The app will show an install prompt when available
2. **Manual Installation**:
   - **Chrome/Edge**: Click the install icon in the address bar or use menu > "Install NAVΛ Studio"
   - **Firefox**: Menu > "Install" or "Add to Home Screen"
   - **Safari (iOS)**: Share button > "Add to Home Screen"
   - **Safari (macOS)**: File > "Add to Dock"

### For Developers

1. **Generate Icons**: Run the icon generation script:
   ```bash
   ./scripts/generate-pwa-icons.sh public/favicon.svg
   ```
   Or manually create icons in `public/icons/` (see sizes in `public/icons/README.md`)

2. **Build for Production**:
   ```bash
   npm run build
   ```

3. **Test PWA Features**:
   - Use Chrome DevTools > Application > Service Workers
   - Test offline mode by disabling network in DevTools
   - Test install prompt in production build

## Service Worker Features

### Caching Strategies
- **Network First**: For API requests and dynamic content
- **Cache First**: For images and static assets
- **Stale While Revalidate**: For static resources

### Background Features
- **Background Sync**: Sync files when connection restored
- **Periodic Sync**: Periodic content updates (if supported)
- **Push Notifications**: Receive notifications from the app

## File Structure

```
public/
├── manifest.json          # PWA manifest
├── service-worker.js      # Service worker with advanced features
├── icons/                 # PWA icons (multiple sizes)
│   ├── icon-72x72.png
│   ├── icon-96x96.png
│   ├── icon-128x128.png
│   ├── icon-144x144.png
│   ├── icon-152x152.png
│   ├── icon-192x192.png
│   ├── icon-384x384.png
│   └── icon-512x512.png
└── screenshots/          # App screenshots for stores
    ├── ide-view.png
    ├── terminal-view.png
    └── workspace-view.png

src/
├── components/
│   └── PWA/
│       ├── PWAInstallPrompt.tsx  # Install prompt component
│       └── PWAInstallPrompt.css
└── utils/
    └── pwa.ts            # PWA utility functions
```

## API Reference

### PWA Utilities (`src/utils/pwa.ts`)

```typescript
// Register service worker
registerServiceWorker(): Promise<ServiceWorkerRegistration | null>

// Check if PWA is installed
isPWAInstalled(): boolean

// Check online status
isOnline(): boolean

// Get cache size
getCacheSize(): Promise<number>

// Clear all caches
clearAllCaches(): Promise<void>

// Request notification permission
requestNotificationPermission(): Promise<NotificationPermission>

// Show notification
showNotification(title: string, options?: NotificationOptions): void

// Request persistent storage
requestPersistentStorage(): Promise<boolean>

// Register background sync
registerBackgroundSync(tag: string): Promise<void>
```

## Testing

### Test Checklist
- [ ] Service worker registers successfully
- [ ] App can be installed on target platforms
- [ ] Offline mode works correctly
- [ ] Files are cached properly
- [ ] Install prompt appears when appropriate
- [ ] App shortcuts work
- [ ] File handling works (opening files from OS)
- [ ] Share target works (receiving shared content)
- [ ] Push notifications work (if implemented)

### Testing Tools
- Chrome DevTools > Application tab
- Lighthouse PWA audit
- PWA Builder (https://www.pwabuilder.com)

## Troubleshooting

### Service Worker Not Registering
- Check browser console for errors
- Ensure HTTPS (or localhost for development)
- Check service-worker.js is accessible

### Install Prompt Not Showing
- Must be in production build
- User must meet installability criteria
- Check manifest.json is valid

### Icons Not Showing
- Ensure all icon files exist in `public/icons/`
- Check manifest.json icon paths are correct
- Verify icon sizes match manifest

## Next Steps

1. **Generate Icons**: Create all required icon sizes
2. **Add Screenshots**: Add app screenshots for app stores
3. **Test on All Platforms**: Verify installation works on all target platforms
4. **Optimize Caching**: Fine-tune cache strategies for your use case
5. **Add Push Notifications**: Implement push notification backend if needed

## Resources

- [MDN PWA Guide](https://developer.mozilla.org/en-US/docs/Web/Progressive_web_apps)
- [Web.dev PWA](https://web.dev/progressive-web-apps/)
- [PWA Builder](https://www.pwabuilder.com)
- [Service Worker API](https://developer.mozilla.org/en-US/docs/Web/API/Service_Worker_API)

