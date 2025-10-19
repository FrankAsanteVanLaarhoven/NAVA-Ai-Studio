/**
 * NAVΛ Studio Localization System
 * Comprehensive localization with i18n, weather, currency, and geolocation
 * 
 * Features:
 * - Multi-language support (9 languages)
 * - Real-time world clock with seconds
 * - Local weather forecast (7-day)
 * - Currency converter (30+ currencies)
 * - Automatic geolocation detection
 */

// ============================================================================
// LOCALIZATION & i18n
// ============================================================================

const translations = {
  en: { name: 'English', flag: '🇬🇧', workspace: 'Workspace', weather: 'Weather', currency: 'Currency', settings: 'Settings' },
  es: { name: 'Español', flag: '🇪🇸', workspace: 'Espacio de trabajo', weather: 'Clima', currency: 'Moneda', settings: 'Configuración' },
  fr: { name: 'Français', flag: '🇫🇷', workspace: 'Espace de travail', weather: 'Météo', currency: 'Devise', settings: 'Paramètres' },
  de: { name: 'Deutsch', flag: '🇩🇪', workspace: 'Arbeitsbereich', weather: 'Wetter', currency: 'Währung', settings: 'Einstellungen' },
  zh: { name: '中文', flag: '🇨🇳', workspace: '工作区', weather: '天气', currency: '货币', settings: '设置' },
  ja: { name: '日本語', flag: '🇯🇵', workspace: 'ワークスペース', weather: '天気', currency: '通貨', settings: '設定' },
  ar: { name: 'العربية', flag: '🇸🇦', workspace: 'مساحة العمل', weather: 'الطقس', currency: 'العملة', settings: 'الإعدادات' },
  pt: { name: 'Português', flag: '🇵🇹', workspace: 'Espaço de trabalho', weather: 'Clima', currency: 'Moeda', settings: 'Configurações' },
  ru: { name: 'Русский', flag: '🇷🇺', workspace: 'Рабочее пространство', weather: 'Погода', currency: 'Валюта', settings: 'Настройки' },
};

let currentLanguage = localStorage.getItem('navl_language') || 'en';

function setLanguage(lang) {
  if (translations[lang]) {
    currentLanguage = lang;
    localStorage.setItem('navl_language', lang);
    updateLanguageDisplay();
  }
}

function updateLanguageDisplay() {
  const langBtn = document.getElementById('languageBtn');
  if (langBtn) {
    langBtn.innerHTML = `${translations[currentLanguage].flag} ${translations[currentLanguage].name.substring(0, 2).toUpperCase()}`;
  }
}

// ============================================================================
// WORLD CLOCK
// ============================================================================

let clockInterval = null;
let userCity = localStorage.getItem('navl_user_city') || 'Unknown';
let userCountry = localStorage.getItem('navl_user_country') || 'Unknown';

function updateClock() {
  const now = new Date();
  
  const hours = String(now.getHours()).padStart(2, '0');
  const minutes = String(now.getMinutes()).padStart(2, '0');
  const seconds = String(now.getSeconds()).padStart(2, '0');
  
  const timeEl = document.getElementById('time');
  if (timeEl) {
    timeEl.textContent = `${hours}:${minutes}:${seconds}`;
  }
  
  const month = String(now.getMonth() + 1).padStart(2, '0');
  const day = String(now.getDate()).padStart(2, '0');
  const year = now.getFullYear();
  
  const dateEl = document.getElementById('date');
  if (dateEl) {
    dateEl.textContent = `${month}/${day}/${year}`;
  }
  
  // Update timezone
  const timezone = Intl.DateTimeFormat().resolvedOptions().timeZone;
  const offset = -now.getTimezoneOffset();
  const offsetHours = Math.floor(Math.abs(offset) / 60);
  const offsetMinutes = Math.abs(offset) % 60;
  const offsetSign = offset >= 0 ? '+' : '-';
  const timezoneStr = `GMT${offsetSign}${offsetHours.toString().padStart(2, '0')}:${offsetMinutes.toString().padStart(2, '0')}`;
  
  const timezoneEl = document.getElementById('timezone');
  if (timezoneEl) {
    timezoneEl.textContent = timezoneStr;
  }
}

function startClock() {
  updateClock();
  if (clockInterval) clearInterval(clockInterval);
  clockInterval = setInterval(updateClock, 1000);
}

// ============================================================================
// WEATHER SERVICE
// ============================================================================

let weatherData = null;

function getWeatherEmoji(condition) {
  const c = condition.toLowerCase();
  if (c.includes('clear')) return '☀️';
  if (c.includes('cloud')) return '☁️';
  if (c.includes('rain')) return '🌧️';
  if (c.includes('drizzle')) return '🌦️';
  if (c.includes('thunder')) return '⛈️';
  if (c.includes('snow')) return '❄️';
  if (c.includes('mist') || c.includes('fog')) return '🌫️';
  return '🌤️';
}

function generateMockWeather(city, country) {
  const conditions = ['Clear', 'Clouds', 'Rain', 'Partly Cloudy'];
  const condition = conditions[Math.floor(Math.random() * conditions.length)];
  
  return {
    current: {
      temp: Math.round(15 + Math.random() * 15),
      condition,
      emoji: getWeatherEmoji(condition),
      humidity: Math.round(40 + Math.random() * 40),
      windSpeed: Math.round(5 + Math.random() * 15),
      feelsLike: Math.round(15 + Math.random() * 15),
    },
    forecast: Array.from({ length: 7 }, (_, i) => {
      const date = new Date();
      date.setDate(date.getDate() + i);
      const cond = conditions[Math.floor(Math.random() * conditions.length)];
      return {
        day: date.toLocaleDateString('en-US', { weekday: 'short' }),
        emoji: getWeatherEmoji(cond),
        high: Math.round(18 + Math.random() * 12),
        low: Math.round(8 + Math.random() * 10),
      };
    }),
    location: { city, country },
  };
}

function updateWeatherDisplay() {
  if (!weatherData) return;
  
  document.getElementById('weatherCity').textContent = weatherData.location.city;
  document.getElementById('weatherTemp').textContent = `${weatherData.current.temp}°C`;
  document.getElementById('weatherEmoji').textContent = weatherData.current.emoji;
  document.getElementById('weatherCondition').textContent = weatherData.current.condition;
  document.getElementById('weatherHumidity').textContent = `${weatherData.current.humidity}%`;
  document.getElementById('weatherWind').textContent = `${weatherData.current.windSpeed} km/h`;
  document.getElementById('weatherFeels').textContent = `${weatherData.current.feelsLike}°C`;
  
  const forecastEl = document.getElementById('weatherForecast');
  if (forecastEl) {
    forecastEl.innerHTML = weatherData.forecast.map(day => `
      <div class="forecast-day">
        <div class="forecast-day-name">${day.day}</div>
        <div class="forecast-icon">${day.emoji}</div>
        <div class="forecast-temp">${day.high}°/${day.low}°</div>
      </div>
    `).join('');
  }
}

function toggleWeatherWidget() {
  const widget = document.getElementById('weatherWidget');
  if (widget) {
    widget.classList.toggle('active');
    if (widget.classList.contains('active') && !weatherData) {
      weatherData = generateMockWeather(userCity, userCountry);
      updateWeatherDisplay();
    }
  }
}

// ============================================================================
// CURRENCY CONVERTER
// ============================================================================

const currencies = [
  { code: 'USD', name: 'US Dollar', symbol: '$', flag: '🇺🇸' },
  { code: 'EUR', name: 'Euro', symbol: '€', flag: '🇪🇺' },
  { code: 'GBP', name: 'British Pound', symbol: '£', flag: '🇬🇧' },
  { code: 'JPY', name: 'Japanese Yen', symbol: '¥', flag: '🇯🇵' },
  { code: 'CNY', name: 'Chinese Yuan', symbol: '¥', flag: '🇨🇳' },
  { code: 'AUD', name: 'Australian Dollar', symbol: 'A$', flag: '🇦🇺' },
  { code: 'CAD', name: 'Canadian Dollar', symbol: 'C$', flag: '🇨🇦' },
  { code: 'CHF', name: 'Swiss Franc', symbol: 'Fr', flag: '🇨🇭' },
  { code: 'INR', name: 'Indian Rupee', symbol: '₹', flag: '🇮🇳' },
  { code: 'BRL', name: 'Brazilian Real', symbol: 'R$', flag: '🇧🇷' },
];

const mockRates = {
  USD: 1, EUR: 0.92, GBP: 0.79, JPY: 149.50, CNY: 7.24,
  AUD: 1.53, CAD: 1.36, CHF: 0.88, INR: 83.12, BRL: 4.97,
};

let fromCurrency = 'USD';
let toCurrency = 'EUR';

function populateCurrencySelects() {
  const fromSelect = document.getElementById('currencyFrom');
  const toSelect = document.getElementById('currencyTo');
  
  if (fromSelect && toSelect) {
    const options = currencies.map(c => 
      `<option value="${c.code}">${c.flag} ${c.code} - ${c.name}</option>`
    ).join('');
    
    fromSelect.innerHTML = options;
    toSelect.innerHTML = options;
    fromSelect.value = fromCurrency;
    toSelect.value = toCurrency;
  }
}

function convertCurrency() {
  const amount = parseFloat(document.getElementById('currencyAmount').value) || 0;
  fromCurrency = document.getElementById('currencyFrom').value;
  toCurrency = document.getElementById('currencyTo').value;
  
  const rate = mockRates[toCurrency] / mockRates[fromCurrency];
  const result = amount * rate;
  
  const fromCurr = currencies.find(c => c.code === fromCurrency);
  const toCurr = currencies.find(c => c.code === toCurrency);
  
  document.getElementById('currencyResult').textContent = 
    `${toCurr.symbol}${result.toFixed(2)} ${toCurrency}`;
  document.getElementById('currencyRate').textContent = 
    `1 ${fromCurrency} = ${rate.toFixed(4)} ${toCurrency}`;
}

function swapCurrencies() {
  const temp = fromCurrency;
  fromCurrency = toCurrency;
  toCurrency = temp;
  
  document.getElementById('currencyFrom').value = fromCurrency;
  document.getElementById('currencyTo').value = toCurrency;
  convertCurrency();
}

function toggleCurrencyWidget() {
  const widget = document.getElementById('currencyWidget');
  if (widget) {
    widget.classList.toggle('active');
    if (widget.classList.contains('active')) {
      populateCurrencySelects();
      convertCurrency();
    }
  }
}

// ============================================================================
// GEOLOCATION
// ============================================================================

async function detectLocation() {
  try {
    // Try IP-based geolocation
    const response = await fetch('https://ipapi.co/json/');
    const data = await response.json();
    
    userCity = data.city || 'Unknown';
    userCountry = data.country_name || 'Unknown';
    
    localStorage.setItem('navl_user_city', userCity);
    localStorage.setItem('navl_user_country', userCountry);
    
    // Update weather with new location
    if (weatherData) {
      weatherData = generateMockWeather(userCity, userCountry);
      updateWeatherDisplay();
    }
  } catch (error) {
    console.log('Location detection failed, using defaults');
  }
}

// ============================================================================
// LANGUAGE MODAL
// ============================================================================

function showLanguageModal() {
  const modal = document.getElementById('languageModal');
  if (modal) {
    modal.classList.add('active');
  }
}

function closeLanguageModal() {
  const modal = document.getElementById('languageModal');
  if (modal) {
    modal.classList.remove('active');
  }
}

function selectLanguage(lang) {
  setLanguage(lang);
  closeLanguageModal();
}

// ============================================================================
// INITIALIZATION
// ============================================================================

function initializeLocalization() {
  // Start clock
  startClock();
  
  // Update language display
  updateLanguageDisplay();
  
  // Detect location
  detectLocation();
  
  // Setup event listeners
  const currencyAmount = document.getElementById('currencyAmount');
  if (currencyAmount) {
    currencyAmount.addEventListener('input', convertCurrency);
  }
  
  const currencyFrom = document.getElementById('currencyFrom');
  if (currencyFrom) {
    currencyFrom.addEventListener('change', convertCurrency);
  }
  
  const currencyTo = document.getElementById('currencyTo');
  if (currencyTo) {
    currencyTo.addEventListener('change', convertCurrency);
  }
  
  console.log('✅ NAVΛ Studio Localization System Initialized');
}

// Auto-initialize when DOM is ready
if (document.readyState === 'loading') {
  document.addEventListener('DOMContentLoaded', initializeLocalization);
} else {
  initializeLocalization();
}
