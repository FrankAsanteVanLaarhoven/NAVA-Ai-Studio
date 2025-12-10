import React, { useState, useEffect } from 'react';
import { Download, X, Check } from 'lucide-react';
import './PWAInstallPrompt.css';

interface BeforeInstallPromptEvent extends Event {
  prompt: () => Promise<void>;
  userChoice: Promise<{ outcome: 'accepted' | 'dismissed' }>;
}

export const PWAInstallPrompt: React.FC = () => {
  const [deferredPrompt, setDeferredPrompt] = useState<BeforeInstallPromptEvent | null>(null);
  const [showPrompt, setShowPrompt] = useState(false);
  const [isInstalled, setIsInstalled] = useState(false);
  const [isStandalone, setIsStandalone] = useState(false);

  useEffect(() => {
    // Check if app is already installed
    const checkInstalled = () => {
      // Check if running as standalone (installed PWA)
      if (window.matchMedia('(display-mode: standalone)').matches ||
          (window.navigator as any).standalone ||
          document.referrer.includes('android-app://')) {
        setIsStandalone(true);
        setIsInstalled(true);
        return;
      }

      // Check if already installed via other means
      const installed = localStorage.getItem('nava-pwa-installed') === 'true';
      setIsInstalled(installed);
    };

    checkInstalled();

    // Listen for beforeinstallprompt event
    const handleBeforeInstallPrompt = (e: Event) => {
      e.preventDefault();
      setDeferredPrompt(e as BeforeInstallPromptEvent);
      
      // Show prompt after a delay if not dismissed before
      const dismissed = localStorage.getItem('nava-pwa-install-dismissed');
      if (!dismissed) {
        setTimeout(() => {
          setShowPrompt(true);
        }, 3000); // Show after 3 seconds
      }
    };

    window.addEventListener('beforeinstallprompt', handleBeforeInstallPrompt);

    // Listen for app installed event
    window.addEventListener('appinstalled', () => {
      console.log('PWA installed successfully');
      setIsInstalled(true);
      setShowPrompt(false);
      setDeferredPrompt(null);
      localStorage.setItem('nava-pwa-installed', 'true');
      localStorage.removeItem('nava-pwa-install-dismissed');
    });

    return () => {
      window.removeEventListener('beforeinstallprompt', handleBeforeInstallPrompt);
    };
  }, []);

  const handleInstallClick = async () => {
    if (!deferredPrompt) {
      // Fallback: show instructions
      showInstallInstructions();
      return;
    }

    try {
      // Show the install prompt
      await deferredPrompt.prompt();
      
      // Wait for user response
      const { outcome } = await deferredPrompt.userChoice;
      
      if (outcome === 'accepted') {
        console.log('User accepted the install prompt');
        setIsInstalled(true);
        setShowPrompt(false);
      } else {
        console.log('User dismissed the install prompt');
        localStorage.setItem('nava-pwa-install-dismissed', Date.now().toString());
      }
      
      setDeferredPrompt(null);
    } catch (error) {
      console.error('Error showing install prompt:', error);
      showInstallInstructions();
    }
  };

  const handleDismiss = () => {
    setShowPrompt(false);
    localStorage.setItem('nava-pwa-install-dismissed', Date.now().toString());
  };

  const showInstallInstructions = () => {
    const isIOS = /iPad|iPhone|iPod/.test(navigator.userAgent);
    const isAndroid = /Android/.test(navigator.userAgent);
    const isChrome = /Chrome/.test(navigator.userAgent);
    const isEdge = /Edg/.test(navigator.userAgent);
    const isSafari = /Safari/.test(navigator.userAgent) && !isChrome;

    let instructions = '';

    if (isIOS) {
      instructions = `
1. Tap the Share button (square with arrow)
2. Scroll down and tap "Add to Home Screen"
3. Tap "Add" to confirm
      `;
    } else if (isAndroid) {
      instructions = `
1. Tap the menu (three dots) in your browser
2. Tap "Install app" or "Add to Home screen"
3. Tap "Install" to confirm
      `;
    } else if (isChrome || isEdge) {
      instructions = `
1. Look for the install icon in your browser's address bar
2. Click it and select "Install"
3. Or use the menu (three dots) > "Install NAVΛ Studio"
      `;
    } else if (isSafari) {
      instructions = `
1. Go to File > Add to Dock (macOS)
2. Or use Safari menu > "Add to Home Screen" (iOS)
      `;
    } else {
      instructions = `
1. Look for an install option in your browser's menu
2. Or use your browser's "Add to Home Screen" feature
      `;
    }

    alert(`Install NAVΛ Studio:\n\n${instructions}`);
  };

  // Don't show if already installed or running as standalone
  if (isInstalled || isStandalone || !showPrompt) {
    return null;
  }

  return (
    <div className="pwa-install-prompt">
      <div className="pwa-install-content">
        <div className="pwa-install-icon">
          <span className="pwa-logo">λ</span>
          <span className="pwa-logo-capital">Λ</span>
        </div>
        <div className="pwa-install-text">
          <h3>Install NAVΛ Studio</h3>
          <p>Install as an app for a better experience with offline support</p>
        </div>
        <div className="pwa-install-actions">
          <button
            className="pwa-install-btn"
            onClick={handleInstallClick}
          >
            <Download size={18} />
            Install
          </button>
          <button
            className="pwa-dismiss-btn"
            onClick={handleDismiss}
            title="Dismiss"
          >
            <X size={18} />
          </button>
        </div>
      </div>
    </div>
  );
};

