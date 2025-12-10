// NAVΛ Studio SDK - Popup Script

import { NavigationField } from '../web/dist/index.esm.js';

// Initialize SDK
let navField = null;

async function init() {
  try {
    navField = new NavigationField();
    console.log('[NAVΛ SDK] Popup initialized');
    
    // Load version
    const manifest = chrome.runtime.getManifest();
    document.getElementById('version').textContent = manifest.version;
    
    // Check SDK status
    chrome.storage.local.get(['sdkStatus'], (result) => {
      const status = result.sdkStatus || 'Ready';
      document.getElementById('status').textContent = status;
    });
  } catch (error) {
    console.error('[NAVΛ SDK] Initialization failed:', error);
    document.getElementById('status').textContent = 'Error';
  }
}

// Button handlers
document.getElementById('calculate-btn').addEventListener('click', async () => {
  chrome.tabs.query({ active: true, currentWindow: true }, async (tabs) => {
    const tab = tabs[0];
    
    // Send message to content script
    chrome.tabs.sendMessage(tab.id, {
      action: 'calculate',
      text: 'Current page navigation',
    }, (response) => {
      if (response) {
        console.log('[NAVΛ SDK] Calculation result:', response);
        updateStatus('Calculated');
      }
    });
  });
});

document.getElementById('visualize-btn').addEventListener('click', () => {
  chrome.tabs.create({
    url: chrome.runtime.getURL('visualize.html'),
  });
});

document.getElementById('inject-btn').addEventListener('click', () => {
  chrome.tabs.query({ active: true, currentWindow: true }, (tabs) => {
    chrome.scripting.executeScript({
      target: { tabId: tabs[0].id },
      files: ['injected.js'],
    }, () => {
      updateStatus('Injected');
    });
  });
});

document.getElementById('settings-btn').addEventListener('click', () => {
  chrome.runtime.openOptionsPage();
});

function updateStatus(status) {
  document.getElementById('status').textContent = status;
  chrome.storage.local.set({ sdkStatus: status });
}

// Initialize on load
init();

