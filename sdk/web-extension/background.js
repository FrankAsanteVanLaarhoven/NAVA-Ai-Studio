// NAVΛ Studio SDK - Background Service Worker
// Enterprise-grade background processing for web extension

import { NavigationField } from '../web/dist/index.esm.js';

// Extension installation and startup
chrome.runtime.onInstalled.addListener((details) => {
  console.log('[NAVΛ SDK] Extension installed:', details.reason);
  
  if (details.reason === 'install') {
    // First time installation
    chrome.storage.local.set({
      installed: true,
      installDate: Date.now(),
      version: chrome.runtime.getManifest().version,
    });
    
    // Open welcome page
    chrome.tabs.create({
      url: chrome.runtime.getURL('welcome.html'),
    });
  } else if (details.reason === 'update') {
    // Extension updated
    chrome.storage.local.set({
      lastUpdate: Date.now(),
      previousVersion: details.previousVersion,
    });
  }
  
  // Create context menu items
  chrome.contextMenus.create({
    id: 'nava-sdk-open',
    title: 'Open NAVΛ SDK',
    contexts: ['page', 'selection'],
  });
  
  chrome.contextMenus.create({
    id: 'nava-sdk-calculate',
    title: 'Calculate Navigation Path',
    contexts: ['selection'],
  });
});

// Context menu click handler
chrome.contextMenus.onClicked.addListener((info, tab) => {
  if (info.menuItemId === 'nava-sdk-open') {
    chrome.tabs.create({
      url: chrome.runtime.getURL('popup.html'),
    });
  } else if (info.menuItemId === 'nava-sdk-calculate') {
    // Send message to content script
    chrome.tabs.sendMessage(tab.id, {
      action: 'calculate',
      text: info.selectionText,
    });
  }
});

// Command handler
chrome.commands.onCommand.addListener((command) => {
  if (command === 'open-sdk') {
    chrome.tabs.create({
      url: chrome.runtime.getURL('popup.html'),
    });
  } else if (command === 'toggle-navigation') {
    chrome.tabs.query({ active: true, currentWindow: true }, (tabs) => {
      chrome.tabs.sendMessage(tabs[0].id, {
        action: 'toggle-navigation',
      });
    });
  }
});

// Message handler
chrome.runtime.onMessage.addListener((message, sender, sendResponse) => {
  if (message.type === 'NAVIGATION_CALCULATE') {
    // Use NAVΛ SDK for navigation calculation
    const nav = new NavigationField();
    // Process navigation request
    sendResponse({ success: true, result: 'Navigation calculated' });
    return true; // Keep channel open for async response
  }
  
  if (message.type === 'STORAGE_GET') {
    chrome.storage.local.get(message.keys, (result) => {
      sendResponse(result);
    });
    return true;
  }
  
  if (message.type === 'STORAGE_SET') {
    chrome.storage.local.set(message.data, () => {
      sendResponse({ success: true });
    });
    return true;
  }
});

// Tab update handler
chrome.tabs.onUpdated.addListener((tabId, changeInfo, tab) => {
  if (changeInfo.status === 'complete' && tab.url) {
    // Inject SDK if needed
    chrome.storage.local.get(['autoInject'], (result) => {
      if (result.autoInject) {
        chrome.scripting.executeScript({
          target: { tabId },
          files: ['injected.js'],
        });
      }
    });
  }
});

// Storage change handler
chrome.storage.onChanged.addListener((changes, areaName) => {
  console.log('[NAVΛ SDK] Storage changed:', changes, areaName);
  
  // Sync changes across all tabs if needed
  chrome.tabs.query({}, (tabs) => {
    tabs.forEach((tab) => {
      chrome.tabs.sendMessage(tab.id, {
        type: 'STORAGE_CHANGED',
        changes,
        areaName,
      }).catch(() => {
        // Ignore errors for tabs that don't have content script
      });
    });
  });
});

// Periodic background tasks
setInterval(() => {
  // Sync data, check for updates, etc.
  console.log('[NAVΛ SDK] Background task running');
}, 60000); // Every minute

// Keep service worker alive
chrome.runtime.onConnect.addListener((port) => {
  port.onDisconnect.addListener(() => {
    console.log('[NAVΛ SDK] Port disconnected');
  });
});

