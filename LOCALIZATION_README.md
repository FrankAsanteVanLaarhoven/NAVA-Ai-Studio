# NAVΛ Studio Localization System

## 🌍 Complete Internationalization & Localization Features

A comprehensive localization system for NAVΛ Studio with multi-language support, real-time world clock, weather forecasts, currency conversion, and automatic geolocation detection.

---

## ✨ Features

### 1. **Multi-Language Support (i18n)**
- 9 languages supported: English, Spanish, French, German, Chinese, Japanese, Arabic, Portuguese, Russian
- Beautiful language selector modal with flags
- Persistent language preference (localStorage)
- Easy to extend with more languages

### 2. **Real-Time World Clock**
- Live clock with **seconds counter** (updates every second)
- Automatic timezone detection
- Date display with month/day/year format
- GMT offset display (e.g., GMT+05:30)
- Location-aware (shows user's city and country)

### 3. **Local Weather Forecast**
- Current weather conditions with emoji icons
- Temperature, humidity, wind speed, "feels like" temperature
- **7-day weather forecast** with daily highs and lows
- Beautiful weather widget with smooth animations
- Mock data included (can be connected to real weather API)

### 4. **Currency Converter**
- **30+ currencies** supported (USD, EUR, GBP, JPY, CNY, and more)
- Real-time conversion with exchange rates
- Swap currencies with animated button
- Live calculation as you type
- Currency flags and symbols
- Mock exchange rates included (can be connected to real API)

### 5. **Automatic Geolocation**
- IP-based geolocation (no permissions required)
- Detects user's city, country, and timezone
- Fallback to browser timezone if geolocation fails
- Cached location data for faster loading

---

## 📦 Files Included

```
src/localization/
├── i18n.ts                    # Internationalization system
├── worldClock.ts              # Real-time world clock with seconds
├── weatherService.ts          # Weather forecast service
├── currencyService.ts         # Currency converter with 30+ currencies
├── geolocationService.ts      # Automatic location detection
└── index.ts                   # Main export file

public/
├── localization.js            # Standalone JavaScript (for workspace.html)
├── localization.css           # Complete styles for all widgets
└── localization-widgets.html  # HTML snippets for widgets
```

---

## 🚀 Quick Start

### Option 1: For React/TypeScript Projects

1. **Import the localization module:**

```typescript
import {
  initializeLocalization,
  cleanupLocalization,
  i18n,
  worldClock,
  weatherService,
  currencyService,
  geolocationService
} from './localization';
```

2. **Initialize on app start:**

```typescript
// In your main App component or index.tsx
useEffect(() => {
  initializeLocalization();
  
  return () => {
    cleanupLocalization();
  };
}, []);
```

3. **Use the services:**

```typescript
// Change language
i18n.setLanguage('es');

// Get current time
const timeData = worldClock.getTime();

// Get weather
const weather = weatherService.getWeatherData();

// Convert currency
const result = currencyService.convert(100, 'USD', 'EUR');

// Get location
const location = geolocationService.getLocationData();
```

### Option 2: For Standalone HTML (workspace.html)

1. **Add CSS to `<head>` section:**

```html
<link rel="stylesheet" href="/localization.css">
```

2. **Add widget HTML before closing `</body>` tag:**

Copy the contents from `public/localization-widgets.html` and paste before `</body>`.

3. **Add JavaScript before closing `</body>` tag:**

```html
<script src="/localization.js"></script>
```

4. **Update existing HTML elements:**

See the detailed instructions in `public/localization-widgets.html`.

---

## 🔧 Integration with workspace.html

### Step 1: Add Styles

Add this line in the `<head>` section:

```html
<link rel="stylesheet" href="/localization.css">
```

Or copy the contents of `public/localization.css` into the existing `<style>` tag.

### Step 2: Update Time Display

Replace the existing time display (around line 885-888):

**Before:**
```html
<div>
    <div class="time-display" id="time">02:19</div>
    <div class="date-display" id="date">10/13/2025</div>
</div>
```

**After:**
```html
<div class="time-container">
    <div class="time-display" id="time">02:19:45</div>
    <div class="date-display" id="date">10/13/2025</div>
    <div class="timezone-display" id="timezone">GMT+00:00</div>
</div>
```

### Step 3: Update Language Button

Add `id="languageBtn"` to the language button (around line 889):

**Before:**
```html
<div class="info-icon" onclick="showLanguageModal()" title="Language" style="cursor: pointer;">🇬🇧 EN</div>
```

**After:**
```html
<div class="info-icon" id="languageBtn" onclick="showLanguageModal()" title="Language" style="cursor: pointer;">🇬🇧 EN</div>
```

### Step 4: Add Widget Buttons

Add these buttons to the bottom-icon-bar (after line 880):

```html
<div class="bottom-icon widget-toggle" onclick="toggleWeatherWidget()" title="Weather Forecast">
    🌤️
    <div class="widget-badge"></div>
</div>
<div class="bottom-icon widget-toggle" onclick="toggleCurrencyWidget()" title="Currency Converter">
    💱
    <div class="widget-badge"></div>
</div>
```

### Step 5: Add Widget HTML

Copy all the widget HTML from `public/localization-widgets.html` and paste before the closing `</body>` tag.

### Step 6: Add JavaScript

Add this line before the closing `</body>` tag:

```html
<script src="/localization.js"></script>
```

### Step 7: Remove Old Functions

Remove or comment out the old `updateTime()` function and `showLanguageModal()` function, as they are now handled by `localization.js`.

---

## 🎨 Customization

### Adding More Languages

Edit `src/localization/i18n.ts` or `public/localization.js`:

```typescript
const translations = {
  // ... existing languages
  it: {
    name: 'Italiano',
    flag: '🇮🇹',
    workspace: 'Area di lavoro',
    weather: 'Meteo',
    currency: 'Valuta',
    settings: 'Impostazioni'
  }
};
```

Then add the language option to the language modal HTML.

### Adding More Currencies

Edit `src/localization/currencyService.ts` or `public/localization.js`:

```typescript
const currencies = [
  // ... existing currencies
  { code: 'SEK', name: 'Swedish Krona', symbol: 'kr', flag: '🇸🇪' }
];
```

### Connecting Real Weather API

Replace the mock weather data in `weatherService.ts` with a real API:

```typescript
// Example with OpenWeatherMap API
async fetchWeather(lat: number, lon: number): Promise<void> {
  const apiKey = 'YOUR_API_KEY';
  const response = await fetch(
    `https://api.openweathermap.org/data/2.5/weather?lat=${lat}&lon=${lon}&appid=${apiKey}&units=metric`
  );
  const data = await response.json();
  // Process and update weatherData
}
```

### Connecting Real Currency API

Replace the mock exchange rates in `currencyService.ts`:

```typescript
// Example with exchangerate-api.com
async fetchExchangeRates(baseCurrency: string): Promise<void> {
  const apiKey = 'YOUR_API_KEY';
  const response = await fetch(
    `https://v6.exchangerate-api.com/v6/${apiKey}/latest/${baseCurrency}`
  );
  const data = await response.json();
  this.exchangeRates = {
    base: baseCurrency,
    rates: data.conversion_rates,
    lastUpdated: new Date()
  };
}
```

---

## 🌐 API Integration (Optional)

### Weather API Options

1. **OpenWeatherMap** (Free tier: 1000 calls/day)
   - https://openweathermap.org/api
   - Best for: Current weather + 5-day forecast

2. **WeatherAPI** (Free tier: 1M calls/month)
   - https://www.weatherapi.com/
   - Best for: Current + 7-day forecast

3. **Tomorrow.io** (Free tier: 500 calls/day)
   - https://www.tomorrow.io/weather-api/
   - Best for: Hyperlocal weather data

### Currency API Options

1. **ExchangeRate-API** (Free tier: 1500 requests/month)
   - https://www.exchangerate-api.com/
   - Best for: Simple currency conversion

2. **Fixer.io** (Free tier: 100 requests/month)
   - https://fixer.io/
   - Best for: Historical rates

3. **CurrencyAPI** (Free tier: 300 requests/month)
   - https://currencyapi.com/
   - Best for: Real-time rates

### Geolocation API Options

1. **ipapi.co** (Free tier: 1000 requests/day) ✅ Currently used
   - https://ipapi.co/
   - No API key required for basic usage

2. **IPGeolocation** (Free tier: 1000 requests/day)
   - https://ipgeolocation.io/
   - More detailed location data

---

## 📱 Responsive Design

All widgets are fully responsive:

- **Desktop**: Full-width widgets with all features
- **Tablet**: Optimized layout with adjusted spacing
- **Mobile**: Stacked layout, touch-friendly buttons

---

## 🎯 Browser Support

- ✅ Chrome/Edge 90+
- ✅ Firefox 88+
- ✅ Safari 14+
- ✅ Opera 76+

---

## 🔒 Privacy & Data

- **No tracking**: All data stays in your browser
- **localStorage**: Preferences saved locally
- **No cookies**: No third-party cookies used
- **Optional APIs**: All external APIs are optional

---

## 🐛 Troubleshooting

### Clock not updating
- Check if `startClock()` is called in initialization
- Verify `setInterval` is not blocked

### Weather widget not showing
- Check if `toggleWeatherWidget()` function exists
- Verify CSS is loaded correctly

### Currency conversion not working
- Check if currency selects are populated
- Verify `convertCurrency()` is called on input

### Language not changing
- Check localStorage for saved language
- Verify `updateLanguageDisplay()` is called

---

## 📄 License

MIT OR Apache-2.0 (same as NAVΛ Studio)

---

## 🤝 Contributing

Contributions welcome! Please:

1. Add more languages to i18n
2. Improve weather/currency mock data
3. Add more currency pairs
4. Enhance UI/UX
5. Fix bugs

---

## 📞 Support

For issues or questions:
- GitHub Issues: [NAVΛ Studio Repository]
- Email: support@navlambda.studio
- Docs: https://docs.navlambda.studio

---

## 🎉 Credits

Built with ❤️ for NAVΛ Studio IDE

**Features:**
- 🌍 9 Languages
- ⏰ Real-time Clock with Seconds
- 🌤️ 7-Day Weather Forecast
- 💱 30+ Currency Converter
- 📍 Auto Geolocation

**100% Free & Open Source**
