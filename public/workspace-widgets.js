// Workspace Widgets - Compact Weather and World Clock
// For NAVΛ Studio workspace.html

// Initialize when DOM is loaded
document.addEventListener('DOMContentLoaded', function() {
    initializeWorkspaceWidgets();
});

// Main initialization function
function initializeWorkspaceWidgets() {
    // Initialize world clock
    initializeWorldClock();
    
    // Initialize weather widget
    initializeWeatherWidget();
    
    // Initialize language selector
    initializeLanguageSelector();
    
    // Start updating
    startWidgetUpdates();
}

// World Clock Functions
let clockInterval;

function initializeWorldClock() {
    updateWorldClock();
}

function updateWorldClock() {
    const now = new Date();
    
    // Time with seconds
    const hours = String(now.getHours()).padStart(2, '0');
    const minutes = String(now.getMinutes()).padStart(2, '0');
    const seconds = String(now.getSeconds()).padStart(2, '0');
    const timeStr = `${hours}:${minutes}:${seconds}`;
    
    // Date
    const options = { weekday: 'long', year: 'numeric', month: 'long', day: 'numeric' };
    const dateStr = now.toLocaleDateString('en-US', options);
    
    // Timezone
    const timezone = Intl.DateTimeFormat().resolvedOptions().timeZone;
    const offset = -now.getTimezoneOffset();
    const offsetHours = Math.floor(Math.abs(offset) / 60);
    const offsetMinutes = Math.abs(offset) % 60;
    const offsetSign = offset >= 0 ? '+' : '-';
    const timezoneStr = `GMT${offsetSign}${offsetHours.toString().padStart(2, '0')}:${offsetMinutes.toString().padStart(2, '0')}`;
    
    // Update DOM elements if they exist
    const clockTimeElement = document.getElementById('clockTime');
    const clockLocationElement = document.getElementById('clockLocation');
    const clockTimezoneElement = document.getElementById('clockTimezone');
    
    if (clockTimeElement) clockTimeElement.textContent = timeStr;
    if (clockLocationElement) clockLocationElement.textContent = 'New York, US'; // Default location
    if (clockTimezoneElement) clockTimezoneElement.textContent = timezoneStr;
    
    // Also update bottom time display
    const bottomTimeElement = document.getElementById('time');
    const bottomDateElement = document.getElementById('date');
    
    if (bottomTimeElement) bottomTimeElement.textContent = `${hours}:${minutes}`;
    if (bottomDateElement) bottomDateElement.textContent = `${now.getMonth() + 1}/${now.getDate()}/${now.getFullYear()}`;
}

function startClockUpdates() {
    if (clockInterval) clearInterval(clockInterval);
    clockInterval = setInterval(updateWorldClock, 1000);
    updateWorldClock(); // Initial update
}

// Weather Widget Functions
function initializeWeatherWidget() {
    updateWeatherWidget();
}

function updateWeatherWidget() {
    // Mock weather data
    const weatherData = {
        temperature: 22,
        condition: 'Partly Cloudy',
        humidity: 65,
        windSpeed: 12,
        icon: '⛅'
    };
    
    // Update DOM elements if they exist
    const weatherTempElement = document.getElementById('weatherTempCompact');
    const weatherConditionElement = document.getElementById('weatherConditionCompact');
    const weatherIconElement = document.getElementById('weatherIconCompact');
    const weatherHumidityElement = document.getElementById('weatherHumidityCompact');
    const weatherWindElement = document.getElementById('weatherWindCompact');
    
    if (weatherTempElement) weatherTempElement.textContent = `${weatherData.temperature}°C`;
    if (weatherConditionElement) weatherConditionElement.textContent = weatherData.condition;
    if (weatherIconElement) weatherIconElement.textContent = weatherData.icon;
    if (weatherHumidityElement) weatherHumidityElement.textContent = `${weatherData.humidity}%`;
    if (weatherWindElement) weatherWindElement.textContent = `${weatherData.windSpeed} km/h`;
}

// Language Selector Functions
function initializeLanguageSelector() {
    // Set initial language
    const currentLang = localStorage.getItem('navl_language') || 'en';
    updateLanguageButton(currentLang);
}

function updateLanguageButton(langCode) {
    const languageBtn = document.getElementById('languageBtn');
    if (!languageBtn) return;
    
    const languages = {
        'en': { flag: '🇬🇧', name: 'EN' },
        'es': { flag: '🇪🇸', name: 'ES' },
        'fr': { flag: '🇫🇷', name: 'FR' },
        'de': { flag: '🇩🇪', name: 'DE' },
        'zh': { flag: '🇨🇳', name: 'ZH' },
        'ja': { flag: '🇯🇵', name: 'JA' },
        'ar': { flag: '🇸🇦', name: 'AR' },
        'pt': { flag: '🇵🇹', name: 'PT' },
        'ru': { flag: '🇷🇺', name: 'RU' }
    };
    
    const lang = languages[langCode] || languages['en'];
    languageBtn.textContent = `${lang.flag} ${lang.name}`;
}

// Show language modal
function showLanguageModal() {
    const modal = document.getElementById('languageModal');
    if (modal) {
        modal.classList.add('active');
    }
}

// Close language modal
function closeLanguageModal() {
    const modal = document.getElementById('languageModal');
    if (modal) {
        modal.classList.remove('active');
    }
}

// Select language
function selectLanguage(langCode) {
    localStorage.setItem('navl_language', langCode);
    updateLanguageButton(langCode);
    closeLanguageModal();
    
    // Show confirmation
    console.log(`Language changed to: ${langCode}`);
}

// Start all widget updates
function startWidgetUpdates() {
    startClockUpdates();
    
    // Update weather every 30 minutes
    updateWeatherWidget();
    setInterval(updateWeatherWidget, 30 * 60 * 1000);
}

// Make functions globally available
window.showLanguageModal = showLanguageModal;
window.closeLanguageModal = closeLanguageModal;
window.selectLanguage = selectLanguage;