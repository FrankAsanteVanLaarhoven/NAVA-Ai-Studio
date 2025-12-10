import React from 'react';
import ReactDOM from 'react-dom/client';
import App from './App';
import { initializeBrowserCompatibility } from './utils/browser-compatibility';
import { registerServiceWorker } from './utils/pwa';
import './styles/global.css';

// Configure Monaco Editor Web Workers
// This must be done before any Monaco Editor instances are created
if (typeof window !== 'undefined') {
  // @ts-ignore - MonacoEnvironment is a global that Monaco Editor expects
  window.MonacoEnvironment = {
    getWorkerUrl: function (_moduleId: string, label: string) {
      // Use CDN for Monaco workers (most reliable and works with Vite)
      const baseUrl = 'https://cdn.jsdelivr.net/npm/monaco-editor@0.47.0/esm/vs';
      
      if (label === 'json') {
        return `${baseUrl}/language/json/json.worker.js`;
      }
      if (label === 'css' || label === 'scss' || label === 'less') {
        return `${baseUrl}/language/css/css.worker.js`;
      }
      if (label === 'html' || label === 'handlebars' || label === 'razor') {
        return `${baseUrl}/language/html/html.worker.js`;
      }
      if (label === 'typescript' || label === 'javascript') {
        return `${baseUrl}/language/typescript/ts.worker.js`;
      }
      // Default editor worker for all other languages
      return `${baseUrl}/editor/editor.worker.js`;
    },
  };
  
  console.log('[Monaco] Web workers configured using CDN');
}

// Initialize browser compatibility checks
initializeBrowserCompatibility();

// Register Service Worker for PWA (enabled in both dev and prod for full SDK functionality)
registerServiceWorker().catch(console.error);

// Suppress harmless browser extension and telemetry errors
if (typeof window !== 'undefined') {
  // Filter out Visual Studio telemetry errors (from browser extensions)
  const originalError = console.error;
  console.error = (...args: any[]) => {
    const message = args.join(' ');
    // Suppress Visual Studio telemetry errors
    if (message.includes('dc.services.visualstudio.com') || 
        message.includes('ERR_INTERNET_DISCONNECTED') ||
        message.includes('visualstudio')) {
      return; // Suppress these errors
    }
    originalError.apply(console, args);
  };

  // Also suppress unhandled promise rejections from browser extensions
  window.addEventListener('unhandledrejection', (event) => {
    const reason = event.reason?.message || String(event.reason);
    if (reason.includes('visualstudio') || 
        reason.includes('message channel closed') ||
        reason.includes('MetaMask')) {
      event.preventDefault(); // Suppress these
    }
  });
}

// Check for required browser features
if (typeof window !== 'undefined') {
  // Check for required APIs
  if (typeof document === 'undefined') {
    console.error('[Browser Compatibility] Document object not available');
  }

  // Polyfill for older browsers if needed
  if (!window.requestAnimationFrame) {
    window.requestAnimationFrame = (callback: FrameRequestCallback) => {
      return setTimeout(callback, 1000 / 60);
    };
  }

  if (!window.cancelAnimationFrame) {
    window.cancelAnimationFrame = (id: number) => {
      clearTimeout(id);
    };
  }
}

ReactDOM.createRoot(document.getElementById('root')!).render(
  <React.StrictMode>
    <App />
  </React.StrictMode>
);
