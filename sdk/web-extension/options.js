// NAVΛ Studio SDK - Options Page Script

// Load saved settings
chrome.storage.local.get([
  'autoInject',
  'showBadge',
  'enableNotifications',
  'calculationMethod',
  'maxIterations'
], (result) => {
  document.getElementById('auto-inject').checked = result.autoInject || false;
  document.getElementById('show-badge').checked = result.showBadge !== false;
  document.getElementById('enable-notifications').checked = result.enableNotifications || false;
  document.getElementById('calculation-method').value = result.calculationMethod || 'euclidean';
  document.getElementById('max-iterations').value = result.maxIterations || 100;
});

// Save settings
document.getElementById('save-btn').addEventListener('click', () => {
  const settings = {
    autoInject: document.getElementById('auto-inject').checked,
    showBadge: document.getElementById('show-badge').checked,
    enableNotifications: document.getElementById('enable-notifications').checked,
    calculationMethod: document.getElementById('calculation-method').value,
    maxIterations: parseInt(document.getElementById('max-iterations').value),
  };
  
  chrome.storage.local.set(settings, () => {
    // Show success message
    const btn = document.getElementById('save-btn');
    const originalText = btn.textContent;
    btn.textContent = 'Saved!';
    btn.style.background = 'linear-gradient(135deg, #00ff00 0%, #3b82f6 100%)';
    
    setTimeout(() => {
      btn.textContent = originalText;
      btn.style.background = '';
    }, 2000);
  });
});

// Clear storage
document.getElementById('clear-storage').addEventListener('click', () => {
  if (confirm('Are you sure you want to clear all storage?')) {
    chrome.storage.local.clear(() => {
      alert('Storage cleared');
      location.reload();
    });
  }
});

// Export settings
document.getElementById('export-settings').addEventListener('click', () => {
  chrome.storage.local.get(null, (items) => {
    const dataStr = JSON.stringify(items, null, 2);
    const blob = new Blob([dataStr], { type: 'application/json' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = 'nava-sdk-settings.json';
    a.click();
    URL.revokeObjectURL(url);
  });
});

// Import settings
document.getElementById('import-settings').addEventListener('click', () => {
  const input = document.createElement('input');
  input.type = 'file';
  input.accept = 'application/json';
  input.onchange = (e) => {
    const file = e.target.files[0];
    const reader = new FileReader();
    reader.onload = (event) => {
      try {
        const settings = JSON.parse(event.target.result);
        chrome.storage.local.set(settings, () => {
          alert('Settings imported');
          location.reload();
        });
      } catch (error) {
        alert('Invalid settings file');
      }
    };
    reader.readAsText(file);
  };
  input.click();
});

