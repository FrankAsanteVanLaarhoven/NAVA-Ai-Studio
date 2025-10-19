// World Clock Service with Real-time Updates
// Displays local time with seconds counter

export interface TimeData {
  hours: string;
  minutes: string;
  seconds: string;
  period: string; // AM/PM
  date: string;
  dayOfWeek: string;
  timezone: string;
  timezoneOffset: string;
  city: string;
  country: string;
}

class WorldClockService {
  private updateInterval: number | null = null;
  private listeners: Array<(data: TimeData) => void> = [];
  private use24Hour: boolean = false;
  private userCity: string = '';
  private userCountry: string = '';

  constructor() {
    this.loadPreferences();
  }

  private loadPreferences(): void {
    const saved24Hour = localStorage.getItem('navl_use24hour');
    this.use24Hour = saved24Hour === 'true';
  }

  set24HourFormat(use24Hour: boolean): void {
    this.use24Hour = use24Hour;
    localStorage.setItem('navl_use24hour', use24Hour.toString());
    this.notifyListeners();
  }

  setLocation(city: string, country: string): void {
    this.userCity = city;
    this.userCountry = country;
    localStorage.setItem('navl_user_city', city);
    localStorage.setItem('navl_user_country', country);
    this.notifyListeners();
  }

  getTimeData(): TimeData {
    const now = new Date();
    
    let hours = now.getHours();
    let period = '';
    
    if (!this.use24Hour) {
      period = hours >= 12 ? 'PM' : 'AM';
      hours = hours % 12 || 12;
    }
    
    const minutes = now.getMinutes();
    const seconds = now.getSeconds();
    
    const daysOfWeek = ['Sunday', 'Monday', 'Tuesday', 'Wednesday', 'Thursday', 'Friday', 'Saturday'];
    const months = ['January', 'February', 'March', 'April', 'May', 'June', 
                    'July', 'August', 'September', 'October', 'November', 'December'];
    
    const dayOfWeek = daysOfWeek[now.getDay()];
    const month = months[now.getMonth()];
    const day = now.getDate();
    const year = now.getFullYear();
    
    const timezone = Intl.DateTimeFormat().resolvedOptions().timeZone;
    const offset = -now.getTimezoneOffset();
    const offsetHours = Math.floor(Math.abs(offset) / 60);
    const offsetMinutes = Math.abs(offset) % 60;
    const offsetSign = offset >= 0 ? '+' : '-';
    const timezoneOffset = `GMT${offsetSign}${offsetHours.toString().padStart(2, '0')}:${offsetMinutes.toString().padStart(2, '0')}`;
    
    return {
      hours: hours.toString().padStart(2, '0'),
      minutes: minutes.toString().padStart(2, '0'),
      seconds: seconds.toString().padStart(2, '0'),
      period: this.use24Hour ? '' : period,
      date: `${month} ${day}, ${year}`,
      dayOfWeek,
      timezone,
      timezoneOffset,
      city: this.userCity || 'Unknown',
      country: this.userCountry || 'Unknown',
    };
  }

  start(): void {
    if (this.updateInterval) return;
    
    // Update immediately
    this.notifyListeners();
    
    // Update every second
    this.updateInterval = window.setInterval(() => {
      this.notifyListeners();
    }, 1000);
  }

  stop(): void {
    if (this.updateInterval) {
      clearInterval(this.updateInterval);
      this.updateInterval = null;
    }
  }

  onChange(callback: (data: TimeData) => void): void {
    this.listeners.push(callback);
  }

  private notifyListeners(): void {
    const data = this.getTimeData();
    this.listeners.forEach(callback => callback(data));
  }
}

export const worldClock = new WorldClockService();
