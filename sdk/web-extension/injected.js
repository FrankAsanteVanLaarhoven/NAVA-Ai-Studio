// NAVΛ Studio SDK - Injected Script
// Runs in page context (not isolated world)

(function() {
  'use strict';
  
  // Create SDK namespace
  window.NAVASDK = {
    version: '1.0.0',
    initialized: false,
    
    init: function() {
      if (this.initialized) return;
      
      console.log('[NAVΛ SDK] Injected script loaded');
      
      // Create SDK indicator
      const indicator = document.createElement('div');
      indicator.id = 'nava-sdk-indicator';
      indicator.innerHTML = '<span style="color: #00ff00; font-weight: 700;">λΛ</span>';
      indicator.style.cssText = `
        position: fixed;
        top: 10px;
        right: 10px;
        width: 40px;
        height: 40px;
        background: rgba(0, 255, 0, 0.1);
        border: 2px solid rgba(0, 255, 0, 0.3);
        border-radius: 50%;
        display: flex;
        align-items: center;
        justify-content: center;
        z-index: 999999;
        cursor: pointer;
        font-size: 18px;
      `;
      
      indicator.addEventListener('click', () => {
        window.postMessage({
          type: 'NAVA_SDK_TOGGLE',
          source: 'nava-sdk-injected'
        }, '*');
      });
      
      document.body.appendChild(indicator);
      this.initialized = true;
    }
  };
  
  // Initialize when DOM is ready
  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', () => NAVASDK.init());
  } else {
    NAVASDK.init();
  }
  
  // Listen for messages from content script
  window.addEventListener('message', (event) => {
    if (event.data && event.data.type === 'NAVA_SDK_MESSAGE') {
      console.log('[NAVΛ SDK] Message received:', event.data);
    }
  });
})();

