// NAVΛ Studio SDK - Content Script
// Runs in the context of web pages

import { NavigationField } from '../web/dist/index.esm.js';

// Initialize NAVΛ SDK
let navField = null;
let isActive = false;

// Load SDK
async function initSDK() {
  try {
    navField = new NavigationField();
    console.log('[NAVΛ SDK] Content script initialized');
    return true;
  } catch (error) {
    console.error('[NAVΛ SDK] Failed to initialize:', error);
    return false;
  }
}

// Message handler from background or popup
chrome.runtime.onMessage.addListener((message, sender, sendResponse) => {
  if (message.action === 'toggle-navigation') {
    isActive = !isActive;
    toggleNavigationUI(isActive);
    sendResponse({ active: isActive });
  } else if (message.action === 'calculate') {
    if (navField && message.text) {
      // Perform navigation calculation
      const result = calculateNavigation(message.text);
      sendResponse({ result });
    }
  } else if (message.type === 'STORAGE_CHANGED') {
    // Handle storage changes
    handleStorageChange(message.changes);
  }
  return true; // Keep channel open for async
});

// Toggle navigation UI overlay
function toggleNavigationUI(active) {
  const existing = document.getElementById('nava-sdk-overlay');
  
  if (active && !existing) {
    // Create overlay
    const overlay = document.createElement('div');
    overlay.id = 'nava-sdk-overlay';
    overlay.innerHTML = `
      <div class="nava-sdk-panel">
        <div class="nava-sdk-header">
          <span class="nava-logo">λΛ</span>
          <span>NAVΛ SDK</span>
          <button class="nava-close">×</button>
        </div>
        <div class="nava-sdk-content">
          <p>Navigation Calculus Active</p>
          <div class="nava-controls">
            <button class="nava-btn" id="nava-calculate">Calculate Path</button>
            <button class="nava-btn" id="nava-visualize">Visualize</button>
          </div>
        </div>
      </div>
    `;
    document.body.appendChild(overlay);
    
    // Add event listeners
    overlay.querySelector('.nava-close').addEventListener('click', () => {
      isActive = false;
      overlay.remove();
    });
    
    overlay.querySelector('#nava-calculate').addEventListener('click', () => {
      calculateCurrentPage();
    });
  } else if (!active && existing) {
    existing.remove();
  }
}

// Calculate navigation for current page
function calculateCurrentPage() {
  if (!navField) return;
  
  // Extract navigation-relevant data from page
  const links = Array.from(document.querySelectorAll('a[href]')).map(a => ({
    url: a.href,
    text: a.textContent,
  }));
  
  // Perform calculation
  console.log('[NAVΛ SDK] Calculating navigation for', links.length, 'links');
  
  // Send result to background
  chrome.runtime.sendMessage({
    type: 'NAVIGATION_RESULT',
    data: { links, timestamp: Date.now() },
  });
}

// Calculate navigation from text
function calculateNavigation(text) {
  if (!navField) return null;
  
  // Simple text-based navigation calculation
  // In real implementation, this would use the full SDK
  return {
    input: text,
    result: 'Navigation path calculated',
    timestamp: Date.now(),
  };
}

// Handle storage changes
function handleStorageChange(changes) {
  console.log('[NAVΛ SDK] Storage changed:', changes);
  
  // Update UI based on storage changes
  if (changes.autoInject) {
    // Handle auto-inject setting change
  }
}

// Initialize on load
if (document.readyState === 'loading') {
  document.addEventListener('DOMContentLoaded', initSDK);
} else {
  initSDK();
}

// Inject SDK badge indicator
function injectBadge() {
  const badge = document.createElement('div');
  badge.id = 'nava-sdk-badge';
  badge.innerHTML = '<span class="nava-logo-small">λΛ</span>';
  badge.style.cssText = `
    position: fixed;
    bottom: 20px;
    right: 20px;
    width: 48px;
    height: 48px;
    background: linear-gradient(135deg, #3b82f6 0%, #00ff00 100%);
    border-radius: 50%;
    display: flex;
    align-items: center;
    justify-content: center;
    cursor: pointer;
    z-index: 999999;
    box-shadow: 0 4px 12px rgba(0, 255, 0, 0.4);
    transition: transform 0.2s;
  `;
  
  badge.addEventListener('click', () => {
    isActive = !isActive;
    toggleNavigationUI(isActive);
  });
  
  document.body.appendChild(badge);
}

// Inject badge after page load
setTimeout(injectBadge, 1000);

