 // Localization Module - Main Export
// Comprehensive localization system with i18n, weather, currency, and geolocation

export { i18n, translations, languageNames } from './i18n';
export type { LocalizationStrings } from './i18n';

export { worldClock } from './worldClock';
export type { TimeData } from './worldClock';

export { weatherService } from './weatherService';
export type { CurrentWeather, DailyForecast, WeatherData } from './weatherService';

export { currencyService, popularCurrencies } from './currencyService';
export type { Currency, ExchangeRates, ConversionResult } from './currencyService';

export { geolocationService } from './geolocationService';
export type { LocationData } from './geolocationService';

// Import services for internal use
import { geolocationService } from './geolocationService';
import { worldClock } from './worldClock';
import { weatherService } from './weatherService';
import { currencyService } from './currencyService';

// Initialize all services
export async function initializeLocalization(): Promise<void> {
  // Detect user location
  const location = await geolocationService.detectLocation();

  if (location) {
    // Start world clock with location
    worldClock.setLocation(location.city, location.country);
    worldClock.start();

    // Start weather updates
    weatherService.startAutoUpdate(
      location.latitude,
      location.longitude,
      location.city,
      location.country
    );

    // Start currency updates with local currency
    const localCurrency = currencyService.detectLocalCurrency(location.countryCode);
    currencyService.startAutoUpdate(localCurrency.code);
  } else {
    // Start with defaults
    worldClock.start();
    currencyService.startAutoUpdate('USD');
  }
}

// Cleanup function
export function cleanupLocalization(): void {
  worldClock.stop();
  weatherService.stopAutoUpdate();
  currencyService.stopAutoUpdate();
}
