// Geolocation Service
// Automatically detects user's location for weather, time, and currency

export interface LocationData {
  latitude: number;
  longitude: number;
  city: string;
  country: string;
  countryCode: string;
  region: string;
  timezone: string;
  accuracy?: number;
}

class GeolocationService {
  private locationData: LocationData | null = null;
  private listeners: Array<(data: LocationData | null) => void> = [];

  constructor() {
    this.loadCachedLocation();
  }

  private loadCachedLocation(): void {
    const cached = localStorage.getItem('navl_location_data');
    if (cached) {
      try {
        this.locationData = JSON.parse(cached);
      } catch (e) {
        console.error('Failed to parse cached location:', e);
      }
    }
  }

  private cacheLocation(): void {
    if (this.locationData) {
      localStorage.setItem('navl_location_data', JSON.stringify(this.locationData));
    }
  }

  async detectLocation(): Promise<LocationData | null> {
    try {
      // First, try to get coordinates from browser geolocation
      const position = await this.getBrowserLocation();
      
      if (position) {
        // Use reverse geocoding to get city and country
        const locationInfo = await this.reverseGeocode(position.latitude, position.longitude);
        
        this.locationData = {
          latitude: position.latitude,
          longitude: position.longitude,
          city: locationInfo.city,
          country: locationInfo.country,
          countryCode: locationInfo.countryCode,
          region: locationInfo.region,
          timezone: Intl.DateTimeFormat().resolvedOptions().timeZone,
          accuracy: position.accuracy,
        };
      } else {
        // Fallback to IP-based geolocation
        this.locationData = await this.getLocationByIP();
      }

      this.cacheLocation();
      this.notifyListeners();
      return this.locationData;
    } catch (error) {
      console.error('Location detection error:', error);
      // Use fallback location
      this.locationData = this.getFallbackLocation();
      this.notifyListeners();
      return this.locationData;
    }
  }

  private getBrowserLocation(): Promise<{ latitude: number; longitude: number; accuracy: number } | null> {
    return new Promise((resolve) => {
      if (!navigator.geolocation) {
        resolve(null);
        return;
      }

      navigator.geolocation.getCurrentPosition(
        (position) => {
          resolve({
            latitude: position.coords.latitude,
            longitude: position.coords.longitude,
            accuracy: position.coords.accuracy,
          });
        },
        (error) => {
          console.warn('Browser geolocation failed:', error);
          resolve(null);
        },
        {
          timeout: 10000,
          maximumAge: 300000, // 5 minutes
          enableHighAccuracy: false,
        }
      );
    });
  }

  private async reverseGeocode(lat: number, lon: number): Promise<{
    city: string;
    country: string;
    countryCode: string;
    region: string;
  }> {
    try {
      // Using OpenStreetMap Nominatim (free, no API key required)
      const response = await fetch(
        `https://nominatim.openstreetmap.org/reverse?format=json&lat=${lat}&lon=${lon}&zoom=10&addressdetails=1`,
        {
          headers: {
            'User-Agent': 'NAVL-Studio-IDE/1.0',
          },
        }
      );
      const data = await response.json();

      return {
        city: data.address.city || data.address.town || data.address.village || 'Unknown',
        country: data.address.country || 'Unknown',
        countryCode: data.address.country_code?.toUpperCase() || 'US',
        region: data.address.state || data.address.region || '',
      };
    } catch (error) {
      console.error('Reverse geocoding failed:', error);
      return {
        city: 'Unknown',
        country: 'Unknown',
        countryCode: 'US',
        region: '',
      };
    }
  }

  private async getLocationByIP(): Promise<LocationData> {
    try {
      // Using ipapi.co (free tier, no API key required for basic usage)
      const response = await fetch('https://ipapi.co/json/');
      const data = await response.json();

      return {
        latitude: data.latitude,
        longitude: data.longitude,
        city: data.city || 'Unknown',
        country: data.country_name || 'Unknown',
        countryCode: data.country_code || 'US',
        region: data.region || '',
        timezone: data.timezone || Intl.DateTimeFormat().resolvedOptions().timeZone,
      };
    } catch (error) {
      console.error('IP geolocation failed:', error);
      return this.getFallbackLocation();
    }
  }

  private getFallbackLocation(): LocationData {
    // Default to a generic location based on browser timezone
    const timezone = Intl.DateTimeFormat().resolvedOptions().timeZone;
    
    // Simple timezone to location mapping
    const timezoneMap: Record<string, Partial<LocationData>> = {
      'America/New_York': { city: 'New York', country: 'United States', countryCode: 'US', latitude: 40.7128, longitude: -74.0060 },
      'America/Los_Angeles': { city: 'Los Angeles', country: 'United States', countryCode: 'US', latitude: 34.0522, longitude: -118.2437 },
      'America/Chicago': { city: 'Chicago', country: 'United States', countryCode: 'US', latitude: 41.8781, longitude: -87.6298 },
      'Europe/London': { city: 'London', country: 'United Kingdom', countryCode: 'GB', latitude: 51.5074, longitude: -0.1278 },
      'Europe/Paris': { city: 'Paris', country: 'France', countryCode: 'FR', latitude: 48.8566, longitude: 2.3522 },
      'Europe/Berlin': { city: 'Berlin', country: 'Germany', countryCode: 'DE', latitude: 52.5200, longitude: 13.4050 },
      'Asia/Tokyo': { city: 'Tokyo', country: 'Japan', countryCode: 'JP', latitude: 35.6762, longitude: 139.6503 },
      'Asia/Shanghai': { city: 'Shanghai', country: 'China', countryCode: 'CN', latitude: 31.2304, longitude: 121.4737 },
      'Asia/Dubai': { city: 'Dubai', country: 'United Arab Emirates', countryCode: 'AE', latitude: 25.2048, longitude: 55.2708 },
      'Australia/Sydney': { city: 'Sydney', country: 'Australia', countryCode: 'AU', latitude: -33.8688, longitude: 151.2093 },
    };

    const fallback = timezoneMap[timezone] || {
      city: 'Unknown',
      country: 'Unknown',
      countryCode: 'US',
      latitude: 0,
      longitude: 0,
    };

    return {
      latitude: fallback.latitude!,
      longitude: fallback.longitude!,
      city: fallback.city!,
      country: fallback.country!,
      countryCode: fallback.countryCode!,
      region: '',
      timezone,
    };
  }

  getLocationData(): LocationData | null {
    return this.locationData;
  }

  setManualLocation(data: LocationData): void {
    this.locationData = data;
    this.cacheLocation();
    this.notifyListeners();
  }

  onChange(callback: (data: LocationData | null) => void): void {
    this.listeners.push(callback);
  }

  private notifyListeners(): void {
    this.listeners.forEach(callback => callback(this.locationData));
  }

  // Request permission for browser geolocation
  async requestPermission(): Promise<boolean> {
    if (!navigator.geolocation) return false;

    try {
      const position = await this.getBrowserLocation();
      return position !== null;
    } catch {
      return false;
    }
  }
}

export const geolocationService = new GeolocationService();
