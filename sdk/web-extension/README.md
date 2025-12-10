# NAVΛ Studio SDK - Web Extension

Enterprise-grade browser extension for NAVΛ Studio SDK, supporting Chrome, Firefox, Edge, and Safari.

## Features

- ✅ **Cross-Browser Support**: Chrome, Firefox, Edge, Safari
- ✅ **Manifest V3**: Latest extension standard
- ✅ **Content Scripts**: Inject SDK into web pages
- ✅ **Background Service Worker**: Persistent background processing
- ✅ **Popup Interface**: Quick access to SDK features
- ✅ **Options Page**: Comprehensive settings management
- ✅ **Context Menus**: Right-click integration
- ✅ **Keyboard Shortcuts**: Quick access commands
- ✅ **Storage API**: Persistent data storage
- ✅ **Notifications**: User notifications support
- ✅ **OAuth2**: Authentication support
- ✅ **Web Accessible Resources**: SDK assets accessible to web pages

## Installation

### Development

```bash
cd sdk/web-extension
npm install
npm run dev  # Loads extension in browser for testing
```

### Build for Production

```bash
npm run build  # Builds for all browsers
npm run build:chrome  # Chrome only
npm run build:firefox  # Firefox only
npm run build:edge  # Edge only
```

## Structure

```
web-extension/
├── manifest.json       # Extension manifest (Manifest V3)
├── background.js       # Service worker (background script)
├── content.js         # Content script (runs in web pages)
├── content.css         # Content script styles
├── popup.html          # Popup interface
├── popup.js            # Popup logic
├── popup.css           # Popup styles
├── options.html        # Settings page
├── options.js          # Settings logic
├── options.css         # Settings styles
├── injected.js         # Injected script (page context)
├── assets/             # Icons and images
└── package.json        # Build configuration
```

## Usage

1. **Load Extension**: Load the `web-extension` folder as an unpacked extension in your browser
2. **Use Popup**: Click the extension icon to open the popup
3. **Inject SDK**: Click "Inject SDK" to inject NAVΛ SDK into the current page
4. **Calculate Navigation**: Use "Calculate Navigation" to perform navigation calculations
5. **Settings**: Right-click extension icon > Options to access settings

## Permissions

- `storage`: Store SDK settings and data
- `activeTab`: Access current tab
- `scripting`: Inject scripts into pages
- `tabs`: Manage browser tabs
- `contextMenus`: Add right-click menu items
- `notifications`: Show notifications
- `background`: Run background tasks

## Browser Support

| Browser | Version | Status |
|---------|---------|--------|
| Chrome  | 109+    | ✅ Full Support |
| Firefox | 109+    | ✅ Full Support |
| Edge    | 109+    | ✅ Full Support |
| Safari  | 16+     | ⚠️ Limited (Manifest V2) |

## Development

### Testing

```bash
# Run in Chrome
npm run dev -- --target=chromium

# Run in Firefox
npm run dev -- --target=firefox-desktop
```

### Linting

```bash
npm run lint
```

## Publishing

1. Build extension: `npm run build`
2. Test thoroughly
3. Submit to browser stores:
   - Chrome Web Store
   - Firefox Add-ons
   - Microsoft Edge Add-ons
   - Safari App Extensions (requires macOS)

## License

MIT OR Apache-2.0

