// Weather Service with Local Forecast
// Provides current weather and weekly forecast based on location

export interface CurrentWeather {
  temperature: number;
  feelsLike: number;
  condition: string;
  description: string;
  humidity: number;
  windSpeed: number;
  windDirection: string;
  pressure: number;
  visibility: number;
  icon: string;
  emoji: string;
}

export interface DailyForecast {
  date: string;
  dayOfWeek: string;
  tempHigh: number;
  tempLow: number;
  condition: string;
  description: string;
  precipitation: number;
  humidity: number;
  windSpeed: number;
  icon: string;
  emoji: string;
}

export interface WeatherData {
  current: CurrentWeather;
  forecast: DailyForecast[];
  location: {
    city: string;
    country: string;
    lat: number;
    lon: number;
  };
  lastUpdated: Date;
}

class WeatherService {
  private apiKey: string = ''; // Users can add their own OpenWeatherMap API key
  private weatherData: WeatherData | null = null;
  private listeners: Array<(data: WeatherData | null) => void> = [];
  private updateInterval: number | null = null;
  private useMetric: boolean = true;

  constructor() {
    this.loadPreferences();
  }

  private loadPreferences(): void {
    const savedApiKey = localStorage.getItem('navl_weather_api_key');
    if (savedApiKey) this.apiKey = savedApiKey;
    
    const savedUnit = localStorage.getItem('navl_weather_unit');
    this.useMetric = savedUnit !== 'imperial';
  }

  setApiKey(key: string): void {
    this.apiKey = key;
    localStorage.setItem('navl_weather_api_key', key);
  }

  setUnit(metric: boolean): void {
    this.useMetric = metric;
    localStorage.setItem('navl_weather_unit', metric ? 'metric' : 'imperial');
    this.notifyListeners();
  }

  private getWeatherEmoji(condition: string): string {
    const conditionLower = condition.toLowerCase();
    if (conditionLower.includes('clear')) return '☀️';
    if (conditionLower.includes('cloud')) return '☁️';
    if (conditionLower.includes('rain')) return '🌧️';
    if (conditionLower.includes('drizzle')) return '🌦️';
    if (conditionLower.includes('thunder')) return '⛈️';
    if (conditionLower.includes('snow')) return '❄️';
    if (conditionLower.includes('mist') || conditionLower.includes('fog')) return '🌫️';
    if (conditionLower.includes('wind')) return '💨';
    return '🌤️';
  }

  async fetchWeather(lat: number, lon: number, city: string, country: string): Promise<void> {
    if (!this.apiKey) {
      // Use mock data if no API key
      this.weatherData = this.getMockWeatherData(city, country, lat, lon);
      this.notifyListeners();
      return;
    }

    try {
      const units = this.useMetric ? 'metric' : 'imperial';
      
      // Fetch current weather
      const currentResponse = await fetch(
        `https://api.openweathermap.org/data/2.5/weather?lat=${lat}&lon=${lon}&units=${units}&appid=${this.apiKey}`
      );
      const currentData = await currentResponse.json();

      // Fetch 7-day forecast
      const forecastResponse = await fetch(
        `https://api.openweathermap.org/data/2.5/forecast/daily?lat=${lat}&lon=${lon}&cnt=7&units=${units}&appid=${this.apiKey}`
      );
      const forecastData = await forecastResponse.json();

      this.weatherData = this.parseWeatherData(currentData, forecastData, city, country, lat, lon);
      this.notifyListeners();
    } catch (error) {
      console.error('Weather fetch error:', error);
      this.weatherData = this.getMockWeatherData(city, country, lat, lon);
      this.notifyListeners();
    }
  }

  private parseWeatherData(current: any, forecast: any, city: string, country: string, lat: number, lon: number): WeatherData {
    const currentWeather: CurrentWeather = {
      temperature: Math.round(current.main.temp),
      feelsLike: Math.round(current.main.feels_like),
      condition: current.weather[0].main,
      description: current.weather[0].description,
      humidity: current.main.humidity,
      windSpeed: Math.round(current.wind.speed),
      windDirection: this.getWindDirection(current.wind.deg),
      pressure: current.main.pressure,
      visibility: Math.round(current.visibility / 1000),
      icon: current.weather[0].icon,
      emoji: this.getWeatherEmoji(current.weather[0].main),
    };

    const dailyForecast: DailyForecast[] = forecast.list.map((day: any) => ({
      date: new Date(day.dt * 1000).toLocaleDateString(),
      dayOfWeek: new Date(day.dt * 1000).toLocaleDateString('en-US', { weekday: 'short' }),
      tempHigh: Math.round(day.temp.max),
      tempLow: Math.round(day.temp.min),
      condition: day.weather[0].main,
      description: day.weather[0].description,
      precipitation: Math.round((day.pop || 0) * 100),
      humidity: day.humidity,
      windSpeed: Math.round(day.speed),
      icon: day.weather[0].icon,
      emoji: this.getWeatherEmoji(day.weather[0].main),
    }));

    return {
      current: currentWeather,
      forecast: dailyForecast,
      location: { city, country, lat, lon },
      lastUpdated: new Date(),
    };
  }

  private getMockWeatherData(city: string, country: string, lat: number, lon: number): WeatherData {
    const conditions = ['Clear', 'Clouds', 'Rain', 'Snow', 'Mist'];
    const randomCondition = conditions[Math.floor(Math.random() * conditions.length)];
    
    return {
      current: {
        temperature: Math.round(15 + Math.random() * 15),
        feelsLike: Math.round(15 + Math.random() * 15),
        condition: randomCondition,
        description: randomCondition.toLowerCase(),
        humidity: Math.round(40 + Math.random() * 40),
        windSpeed: Math.round(5 + Math.random() * 15),
        windDirection: 'NW',
        pressure: 1013,
        visibility: 10,
        icon: '01d',
        emoji: this.getWeatherEmoji(randomCondition),
      },
      forecast: Array.from({ length: 7 }, (_, i) => {
        const date = new Date();
        date.setDate(date.getDate() + i);
        const condition = conditions[Math.floor(Math.random() * conditions.length)];
        
        return {
          date: date.toLocaleDateString(),
          dayOfWeek: date.toLocaleDateString('en-US', { weekday: 'short' }),
          tempHigh: Math.round(18 + Math.random() * 12),
          tempLow: Math.round(8 + Math.random() * 10),
          condition,
          description: condition.toLowerCase(),
          precipitation: Math.round(Math.random() * 60),
          humidity: Math.round(40 + Math.random() * 40),
          windSpeed: Math.round(5 + Math.random() * 15),
          icon: '01d',
          emoji: this.getWeatherEmoji(condition),
        };
      }),
      location: { city, country, lat, lon },
      lastUpdated: new Date(),
    };
  }

  private getWindDirection(degrees: number): string {
    const directions = ['N', 'NNE', 'NE', 'ENE', 'E', 'ESE', 'SE', 'SSE', 'S', 'SSW', 'SW', 'WSW', 'W', 'WNW', 'NW', 'NNW'];
    const index = Math.round(degrees / 22.5) % 16;
    return directions[index];
  }

  getWeatherData(): WeatherData | null {
    return this.weatherData;
  }

  startAutoUpdate(lat: number, lon: number, city: string, country: string): void {
    // Update immediately
    this.fetchWeather(lat, lon, city, country);
    
    // Update every 30 minutes
    if (this.updateInterval) clearInterval(this.updateInterval);
    this.updateInterval = window.setInterval(() => {
      this.fetchWeather(lat, lon, city, country);
    }, 30 * 60 * 1000);
  }

  stopAutoUpdate(): void {
    if (this.updateInterval) {
      clearInterval(this.updateInterval);
      this.updateInterval = null;
    }
  }

  onChange(callback: (data: WeatherData | null) => void): void {
    this.listeners.push(callback);
  }

  private notifyListeners(): void {
    this.listeners.forEach(callback => callback(this.weatherData));
  }

  getTemperatureUnit(): string {
    return this.useMetric ? '°C' : '°F';
  }

  getSpeedUnit(): string {
    return this.useMetric ? 'km/h' : 'mph';
  }
}

export const weatherService = new WeatherService();
