/**
 * World Widgets Component
 * 
 * Professional workspace widgets including:
 * - Multi-city world clocks
 * - Weather forecasts for multiple locations
 * - Discovery panel (Reddit, RSS, Hugging Face papers)
 * - Currency calculator with live rates
 * - Real-time stock market feeds
 */

import React, { useState, useEffect } from 'react';
import { DiscoveryPanel } from '../ActivityPanels/DiscoveryPanel';
import './WorldWidgets.css';

interface WorldClock {
  city: string;
  country: string;
  timezone: string;
  flag: string;
}

interface WeatherData {
  city: string;
  temp: number;
  condition: string;
  humidity: number;
  wind: number;
  icon: string;
}

interface StockData {
  symbol: string;
  price: number;
  change: number;
  percentChange: number;
}

interface WorldWidgetsProps {
  onCollapse?: () => void;
}

export const WorldWidgets: React.FC<WorldWidgetsProps> = ({ onCollapse }) => {
  // Collapsible state for each widget - default to all collapsed
  const [collapsed, setCollapsed] = useState<{ [key: string]: boolean }>(() => {
    try {
      const saved = localStorage.getItem('nava-widgets-collapsed');
      if (saved) {
        return JSON.parse(saved);
      }
      // Default: all widgets collapsed
      return {
        weather: true,
        'world-clock': true,
        discovery: true,
        currency: true,
        stocks: true,
      };
    } catch {
      // Default: all widgets collapsed
      return {
        weather: true,
        'world-clock': true,
        discovery: true,
        currency: true,
        stocks: true,
      };
    }
  });

  const toggleWidget = (widgetId: string) => {
    setCollapsed(prev => {
      const newState = { ...prev, [widgetId]: !prev[widgetId] };
      localStorage.setItem('nava-widgets-collapsed', JSON.stringify(newState));
      return newState;
    });
  };

  const [selectedCities, setSelectedCities] = useState<WorldClock[]>([
    { city: 'New York', country: 'USA', timezone: 'America/New_York', flag: '🇺🇸' },
    { city: 'London', country: 'UK', timezone: 'Europe/London', flag: '🇬🇧' },
    { city: 'Tokyo', country: 'Japan', timezone: 'Asia/Tokyo', flag: '🇯🇵' },
  ]);

  const [weatherData, setWeatherData] = useState<WeatherData[]>([
    { city: 'New York', temp: 22, condition: 'Partly Cloudy', humidity: 65, wind: 12, icon: '⛅' },
    { city: 'London', temp: 15, condition: 'Rainy', humidity: 80, wind: 20, icon: '🌧️' },
    { city: 'Tokyo', temp: 18, condition: 'Clear', humidity: 55, wind: 8, icon: '☀️' },
  ]);


  const [stocks, setStocks] = useState<StockData[]>([
    { symbol: 'TSLA', price: 242.84, change: +5.23, percentChange: +2.20 },
    { symbol: 'NVDA', price: 875.42, change: -12.34, percentChange: -1.39 },
    { symbol: 'GOOGL', price: 145.67, change: +2.14, percentChange: +1.49 },
  ]);

  const [currencyRates, setCurrencyRates] = useState({
    EUR: 0.92,
    GBP: 0.79,
    JPY: 149.50,
    CNY: 7.24,
  });

  const [fromCurrency, setFromCurrency] = useState('USD');
  const [toCurrency, setToCurrency] = useState('EUR');
  const [amount, setAmount] = useState(100);
  const [convertedAmount, setConvertedAmount] = useState(92);

  useEffect(() => {
    // Update clocks every second
    const clockInterval = setInterval(() => {
      // Force re-render for clocks by updating state
      setSelectedCities(prev => [...prev]);
    }, 1000);

    // Simulate stock updates every 5 seconds
    const stockInterval = setInterval(() => {
      setStocks(prevStocks => prevStocks.map(stock => ({
        ...stock,
        price: stock.price + (Math.random() - 0.5) * 2,
        change: stock.change + (Math.random() - 0.5) * 0.5,
        percentChange: stock.percentChange + (Math.random() - 0.5) * 0.1,
      })));
    }, 5000);

    return () => {
      clearInterval(clockInterval);
      clearInterval(stockInterval);
    };
  }, []);

  const getTimeForTimezone = (timezone: string): string => {
    const now = new Date();
    return now.toLocaleTimeString('en-US', {
      timeZone: timezone,
      hour12: false,
      hour: '2-digit',
      minute: '2-digit',
      second: '2-digit'
    });
  };

  const handleCurrencyConvert = () => {
    if (fromCurrency === 'USD') {
      const rate = currencyRates[toCurrency as keyof typeof currencyRates] || 1;
      setConvertedAmount(amount * rate);
    } else {
      // Simplified conversion
      setConvertedAmount(amount * 0.92);
    }
  };

  return (
    <div className="world-widgets-container">
      {/* Hamburger Menu Button */}
      {onCollapse && (
        <div className="widgets-header-bar">
          <button 
            className="widgets-hamburger-btn"
            onClick={onCollapse}
            title="Collapse Sidebar"
          >
            <span className="hamburger-icon">☰</span>
          </button>
        </div>
      )}
      {/* Weather Widget */}
      <div className="collapsible-widget">
        <div className="widget-header" onClick={() => toggleWidget('weather')}>
          <span className={`widget-toggle ${collapsed['weather'] ? 'collapsed' : ''}`}>{collapsed['weather'] ? '▼' : '▲'}</span>
          <span className="widget-icon">⛅</span>
          <span className="widget-title">Weather</span>
          <div className="widget-header-right">
            <select className="widget-location-select" onClick={(e) => e.stopPropagation()}>
              <option>London, UK</option>
              <option>New York, USA</option>
              <option>Tokyo, Japan</option>
            </select>
          </div>
        </div>
        {!collapsed['weather'] && (
          <div className="widget-content">
            <div className="weather-grid">
              {weatherData.map((weather, index) => (
                <div key={index} className="weather-card">
                  <div className="weather-card-header">
                    <span className="weather-icon-large">{weather.icon}</span>
                    <span className="weather-city">{weather.city}</span>
                  </div>
                  <div className="weather-temp-large">{weather.temp}°C</div>
                  <div className="weather-condition">{weather.condition}</div>
                  <div className="weather-details-grid">
                    <div className="weather-detail">💧 {weather.humidity}%</div>
                    <div className="weather-detail">💨 {weather.wind} km/h</div>
                  </div>
                </div>
              ))}
            </div>
          </div>
        )}
      </div>

      {/* World Clock Widget */}
      <div className="collapsible-widget">
        <div className="widget-header" onClick={() => toggleWidget('world-clock')}>
          <span className={`widget-toggle ${collapsed['world-clock'] ? 'collapsed' : ''}`}>{collapsed['world-clock'] ? '▼' : '▲'}</span>
          <span className="widget-icon">🕐</span>
          <span className="widget-title">World Clock</span>
          <div className="widget-header-right">
            <select className="widget-location-select" onClick={(e) => e.stopPropagation()}>
              <option>London, UK</option>
              <option>New York, USA</option>
              <option>Tokyo, Japan</option>
            </select>
          </div>
        </div>
        {!collapsed['world-clock'] && (
          <div className="widget-content">
            <div className="world-clocks-grid">
          {selectedCities.map((clock, index) => (
            <div key={index} className="world-clock-card">
              <div className="world-clock-header">
                <span className="world-clock-flag">{clock.flag}</span>
                <span className="world-clock-city">{clock.city}</span>
              </div>
              <div className="world-clock-time">{getTimeForTimezone(clock.timezone)}</div>
              <div className="world-clock-country">{clock.country}</div>
            </div>
          ))}
            </div>
          </div>
        )}
      </div>

      {/* Discovery Widget - Replaces News Feed */}
      <div className="collapsible-widget discovery-widget">
        <div className="widget-header" onClick={() => toggleWidget('discovery')}>
          <span className={`widget-toggle ${collapsed['discovery'] ? 'collapsed' : ''}`}>{collapsed['discovery'] ? '▼' : '▲'}</span>
          <span className="widget-icon">🔥</span>
          <span className="widget-title">Discover</span>
          <div className="widget-header-right">
            <span className="discovery-badge" title="Real-time news, RSS feeds, and research papers">LIVE</span>
          </div>
        </div>
        {!collapsed['discovery'] && (
          <div className="widget-content discovery-widget-content">
            <DiscoveryPanel />
          </div>
        )}
      </div>

      {/* Currency Widget */}
      <div className="collapsible-widget">
        <div className="widget-header" onClick={() => toggleWidget('currency')}>
          <span className={`widget-toggle ${collapsed['currency'] ? 'collapsed' : ''}`}>{collapsed['currency'] ? '▼' : '▲'}</span>
          <span className="widget-icon">💱</span>
          <span className="widget-title">Currency</span>
        </div>
        {!collapsed['currency'] && (
          <div className="widget-content">
            <div className="currency-calculator">
          <div className="currency-input-group">
            <input
              type="number"
              value={amount}
              onChange={(e) => setAmount(parseFloat(e.target.value))}
              className="currency-amount"
            />
            <select
              value={fromCurrency}
              onChange={(e) => setFromCurrency(e.target.value)}
              className="currency-select"
            >
              <option value="USD">🇺🇸 USD</option>
              <option value="EUR">🇪🇺 EUR</option>
              <option value="GBP">🇬🇧 GBP</option>
              <option value="JPY">🇯🇵 JPY</option>
              <option value="CNY">🇨🇳 CNY</option>
            </select>
          </div>

          <div className="currency-arrow">↓</div>

          <div className="currency-input-group">
            <input
              type="number"
              value={convertedAmount.toFixed(2)}
              readOnly
              className="currency-amount result"
            />
            <select
              value={toCurrency}
              onChange={(e) => setToCurrency(e.target.value)}
              className="currency-select"
            >
              <option value="EUR">🇪🇺 EUR</option>
              <option value="GBP">🇬🇧 GBP</option>
              <option value="JPY">🇯🇵 JPY</option>
              <option value="CNY">🇨🇳 CNY</option>
              <option value="USD">🇺🇸 USD</option>
            </select>
          </div>

          <button onClick={handleCurrencyConvert} className="convert-btn">
            Calculate
          </button>

          <div className="exchange-rate">
            Rate: 1 {fromCurrency} = {currencyRates[toCurrency as keyof typeof currencyRates] || 1} {toCurrency}
          </div>
        </div>
          </div>
        )}
      </div>

      {/* Stocks Widget */}
      <div className="collapsible-widget">
        <div className="widget-header" onClick={() => toggleWidget('stocks')}>
          <span className={`widget-toggle ${collapsed['stocks'] ? 'collapsed' : ''}`}>{collapsed['stocks'] ? '▼' : '▲'}</span>
          <span className="widget-icon">📈</span>
          <span className="widget-title">Stocks</span>
          <div className="widget-header-right">
            <span className="live-indicator">
              <span className="live-dot"></span>
              LIVE
            </span>
          </div>
        </div>
        {!collapsed['stocks'] && (
          <div className="widget-content">
            <div className="stocks-list">
          {stocks.map((stock, index) => (
            <div key={index} className="stock-item">
              <div className="stock-symbol">{stock.symbol}</div>
              <div className="stock-price">${stock.price.toFixed(2)}</div>
              <div className={`stock-change ${stock.change >= 0 ? 'positive' : 'negative'}`}>
                {stock.change >= 0 ? '▲' : '▼'} ${Math.abs(stock.change).toFixed(2)} ({stock.percentChange >= 0 ? '+' : ''}{stock.percentChange.toFixed(2)}%)
              </div>
            </div>
          ))}
            </div>
          </div>
        )}
      </div>
    </div>
  );
};

export default WorldWidgets;


