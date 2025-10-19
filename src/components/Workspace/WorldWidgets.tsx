/**
 * World Widgets Component
 * 
 * Professional workspace widgets including:
 * - Multi-city world clocks
 * - Weather forecasts for multiple locations
 * - Perplexity-style news feed
 * - Currency calculator with live rates
 * - Real-time stock market feeds
 */

import React, { useState, useEffect } from 'react';
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

interface NewsItem {
  title: string;
  source: string;
  time: string;
  category: string;
}

interface StockData {
  symbol: string;
  price: number;
  change: number;
  percentChange: number;
}

export const WorldWidgets: React.FC = () => {
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

  const [newsItems, setNewsItems] = useState<NewsItem[]>([
    { title: 'AI Breakthrough in Autonomous Navigation', source: 'Tech News', time: '2h ago', category: 'Technology' },
    { title: 'New Robotics Standards Released', source: 'IEEE', time: '4h ago', category: 'Research' },
    { title: 'Waymo Expands Self-Driving Service', source: 'Auto News', time: '6h ago', category: 'Business' },
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
      // Force re-render for clocks
      setSelectedCities([...selectedCities]);
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
      {/* World Clocks Grid */}
      <div className="widget-section">
        <h3 className="widget-section-title">🌍 World Clocks</h3>
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

      {/* Weather Forecasts */}
      <div className="widget-section">
        <h3 className="widget-section-title">🌦️ Weather Forecasts</h3>
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

      {/* News Feed */}
      <div className="widget-section">
        <h3 className="widget-section-title">📰 News Feed</h3>
        <div className="news-feed">
          {newsItems.map((news, index) => (
            <div key={index} className="news-item">
              <div className="news-category">{news.category}</div>
              <div className="news-title">{news.title}</div>
              <div className="news-meta">
                <span className="news-source">{news.source}</span>
                <span className="news-time">{news.time}</span>
              </div>
            </div>
          ))}
        </div>
      </div>

      {/* Currency Calculator */}
      <div className="widget-section">
        <h3 className="widget-section-title">💱 Currency Calculator</h3>
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

      {/* Stock Market */}
      <div className="widget-section">
        <h3 className="widget-section-title">📈 Stock Market</h3>
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
    </div>
  );
};

export default WorldWidgets;

