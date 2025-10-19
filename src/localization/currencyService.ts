// Currency Converter Service with Real-time Exchange Rates
// Supports multiple currencies with live conversion

export interface Currency {
  code: string;
  name: string;
  symbol: string;
  flag: string;
}

export interface ExchangeRates {
  base: string;
  rates: Record<string, number>;
  lastUpdated: Date;
}

export interface ConversionResult {
  from: Currency;
  to: Currency;
  amount: number;
  result: number;
  rate: number;
  timestamp: Date;
}

export const popularCurrencies: Currency[] = [
  { code: 'USD', name: 'US Dollar', symbol: '$', flag: '🇺🇸' },
  { code: 'EUR', name: 'Euro', symbol: '€', flag: '🇪🇺' },
  { code: 'GBP', name: 'British Pound', symbol: '£', flag: '🇬🇧' },
  { code: 'JPY', name: 'Japanese Yen', symbol: '¥', flag: '🇯🇵' },
  { code: 'CNY', name: 'Chinese Yuan', symbol: '¥', flag: '🇨🇳' },
  { code: 'AUD', name: 'Australian Dollar', symbol: 'A$', flag: '🇦🇺' },
  { code: 'CAD', name: 'Canadian Dollar', symbol: 'C$', flag: '🇨🇦' },
  { code: 'CHF', name: 'Swiss Franc', symbol: 'Fr', flag: '🇨🇭' },
  { code: 'INR', name: 'Indian Rupee', symbol: '₹', flag: '🇮🇳' },
  { code: 'RUB', name: 'Russian Ruble', symbol: '₽', flag: '🇷🇺' },
  { code: 'BRL', name: 'Brazilian Real', symbol: 'R$', flag: '🇧🇷' },
  { code: 'ZAR', name: 'South African Rand', symbol: 'R', flag: '🇿🇦' },
  { code: 'KRW', name: 'South Korean Won', symbol: '₩', flag: '🇰🇷' },
  { code: 'MXN', name: 'Mexican Peso', symbol: '$', flag: '🇲🇽' },
  { code: 'SGD', name: 'Singapore Dollar', symbol: 'S$', flag: '🇸🇬' },
  { code: 'HKD', name: 'Hong Kong Dollar', symbol: 'HK$', flag: '🇭🇰' },
  { code: 'NOK', name: 'Norwegian Krone', symbol: 'kr', flag: '🇳🇴' },
  { code: 'SEK', name: 'Swedish Krona', symbol: 'kr', flag: '🇸🇪' },
  { code: 'DKK', name: 'Danish Krone', symbol: 'kr', flag: '🇩🇰' },
  { code: 'PLN', name: 'Polish Zloty', symbol: 'zł', flag: '🇵🇱' },
  { code: 'THB', name: 'Thai Baht', symbol: '฿', flag: '🇹🇭' },
  { code: 'IDR', name: 'Indonesian Rupiah', symbol: 'Rp', flag: '🇮🇩' },
  { code: 'MYR', name: 'Malaysian Ringgit', symbol: 'RM', flag: '🇲🇾' },
  { code: 'PHP', name: 'Philippine Peso', symbol: '₱', flag: '🇵🇭' },
  { code: 'TRY', name: 'Turkish Lira', symbol: '₺', flag: '🇹🇷' },
  { code: 'AED', name: 'UAE Dirham', symbol: 'د.إ', flag: '🇦🇪' },
  { code: 'SAR', name: 'Saudi Riyal', symbol: '﷼', flag: '🇸🇦' },
  { code: 'ILS', name: 'Israeli Shekel', symbol: '₪', flag: '🇮🇱' },
  { code: 'NZD', name: 'New Zealand Dollar', symbol: 'NZ$', flag: '🇳🇿' },
  { code: 'ARS', name: 'Argentine Peso', symbol: '$', flag: '🇦🇷' },
];

class CurrencyService {
  private apiKey: string = ''; // Users can add their own API key (e.g., from exchangerate-api.com)
  private exchangeRates: ExchangeRates | null = null;
  private listeners: Array<(rates: ExchangeRates | null) => void> = [];
  private updateInterval: number | null = null;

  constructor() {
    this.loadPreferences();
  }

  private loadPreferences(): void {
    const savedApiKey = localStorage.getItem('navl_currency_api_key');
    if (savedApiKey) this.apiKey = savedApiKey;
    
    // Load cached rates
    const cachedRates = localStorage.getItem('navl_exchange_rates');
    if (cachedRates) {
      try {
        const parsed = JSON.parse(cachedRates);
        this.exchangeRates = {
          ...parsed,
          lastUpdated: new Date(parsed.lastUpdated),
        };
      } catch (e) {
        console.error('Failed to parse cached rates:', e);
      }
    }
  }

  setApiKey(key: string): void {
    this.apiKey = key;
    localStorage.setItem('navl_currency_api_key', key);
  }

  async fetchExchangeRates(baseCurrency: string = 'USD'): Promise<void> {
    if (!this.apiKey) {
      // Use mock data if no API key
      this.exchangeRates = this.getMockExchangeRates(baseCurrency);
      this.cacheRates();
      this.notifyListeners();
      return;
    }

    try {
      // Using exchangerate-api.com format (free tier available)
      const response = await fetch(
        `https://v6.exchangerate-api.com/v6/${this.apiKey}/latest/${baseCurrency}`
      );
      const data = await response.json();

      if (data.result === 'success') {
        this.exchangeRates = {
          base: baseCurrency,
          rates: data.conversion_rates,
          lastUpdated: new Date(),
        };
        this.cacheRates();
        this.notifyListeners();
      } else {
        throw new Error('API request failed');
      }
    } catch (error) {
      console.error('Currency fetch error:', error);
      // Fallback to mock data
      this.exchangeRates = this.getMockExchangeRates(baseCurrency);
      this.cacheRates();
      this.notifyListeners();
    }
  }

  private getMockExchangeRates(base: string): ExchangeRates {
    // Mock exchange rates (approximate values for demonstration)
    const mockRates: Record<string, Record<string, number>> = {
      USD: {
        USD: 1, EUR: 0.92, GBP: 0.79, JPY: 149.50, CNY: 7.24,
        AUD: 1.53, CAD: 1.36, CHF: 0.88, INR: 83.12, RUB: 92.50,
        BRL: 4.97, ZAR: 18.75, KRW: 1320.50, MXN: 17.15, SGD: 1.35,
        HKD: 7.83, NOK: 10.65, SEK: 10.45, DKK: 6.87, PLN: 3.98,
        THB: 35.50, IDR: 15650, MYR: 4.72, PHP: 56.25, TRY: 32.15,
        AED: 3.67, SAR: 3.75, ILS: 3.65, NZD: 1.65, ARS: 850.50,
      },
    };

    // If base is not USD, convert from USD rates
    if (base !== 'USD' && mockRates.USD[base]) {
      const baseRate = mockRates.USD[base];
      const convertedRates: Record<string, number> = {};
      
      Object.entries(mockRates.USD).forEach(([currency, rate]) => {
        convertedRates[currency] = rate / baseRate;
      });
      
      return {
        base,
        rates: convertedRates,
        lastUpdated: new Date(),
      };
    }

    return {
      base,
      rates: mockRates[base] || mockRates.USD,
      lastUpdated: new Date(),
    };
  }

  private cacheRates(): void {
    if (this.exchangeRates) {
      localStorage.setItem('navl_exchange_rates', JSON.stringify(this.exchangeRates));
    }
  }

  convert(amount: number, fromCode: string, toCode: string): ConversionResult | null {
    if (!this.exchangeRates) return null;

    const fromCurrency = popularCurrencies.find(c => c.code === fromCode);
    const toCurrency = popularCurrencies.find(c => c.code === toCode);

    if (!fromCurrency || !toCurrency) return null;

    let rate: number;
    
    if (this.exchangeRates.base === fromCode) {
      rate = this.exchangeRates.rates[toCode];
    } else if (this.exchangeRates.base === toCode) {
      rate = 1 / this.exchangeRates.rates[fromCode];
    } else {
      // Convert through base currency
      const fromRate = this.exchangeRates.rates[fromCode];
      const toRate = this.exchangeRates.rates[toCode];
      rate = toRate / fromRate;
    }

    const result = amount * rate;

    return {
      from: fromCurrency,
      to: toCurrency,
      amount,
      result,
      rate,
      timestamp: new Date(),
    };
  }

  getExchangeRates(): ExchangeRates | null {
    return this.exchangeRates;
  }

  getCurrencyByCode(code: string): Currency | undefined {
    return popularCurrencies.find(c => c.code === code);
  }

  startAutoUpdate(baseCurrency: string = 'USD'): void {
    // Update immediately
    this.fetchExchangeRates(baseCurrency);
    
    // Update every hour
    if (this.updateInterval) clearInterval(this.updateInterval);
    this.updateInterval = window.setInterval(() => {
      this.fetchExchangeRates(baseCurrency);
    }, 60 * 60 * 1000);
  }

  stopAutoUpdate(): void {
    if (this.updateInterval) {
      clearInterval(this.updateInterval);
      this.updateInterval = null;
    }
  }

  onChange(callback: (rates: ExchangeRates | null) => void): void {
    this.listeners.push(callback);
  }

  private notifyListeners(): void {
    this.listeners.forEach(callback => callback(this.exchangeRates));
  }

  // Get user's local currency based on location
  detectLocalCurrency(countryCode: string): Currency {
    const currencyMap: Record<string, string> = {
      US: 'USD', GB: 'GBP', EU: 'EUR', JP: 'JPY', CN: 'CNY',
      AU: 'AUD', CA: 'CAD', CH: 'CHF', IN: 'INR', RU: 'RUB',
      BR: 'BRL', ZA: 'ZAR', KR: 'KRW', MX: 'MXN', SG: 'SGD',
      HK: 'HKD', NO: 'NOK', SE: 'SEK', DK: 'DKK', PL: 'PLN',
      TH: 'THB', ID: 'IDR', MY: 'MYR', PH: 'PHP', TR: 'TRY',
      AE: 'AED', SA: 'SAR', IL: 'ILS', NZ: 'NZD', AR: 'ARS',
    };

    const currencyCode = currencyMap[countryCode] || 'USD';
    return this.getCurrencyByCode(currencyCode) || popularCurrencies[0];
  }
}

export const currencyService = new CurrencyService();
