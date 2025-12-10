/**
 * Timeline Service
 * Tracks real-time IDE events and activities
 */

export interface TimelineEvent {
  id: string;
  time: string;
  timestamp: number;
  action: string;
  file?: string;
  details?: string;
  type: 'file' | 'server' | 'config' | 'build' | 'git' | 'other';
}

class TimelineService {
  private events: TimelineEvent[] = [];
  private readonly STORAGE_KEY = 'nava_timeline_events';
  private readonly MAX_EVENTS = 100;
  private listeners: Array<(event: TimelineEvent) => void> = [];

  constructor() {
    this.loadEvents();
  }

  /**
   * Add a new timeline event
   */
  addEvent(action: string, file?: string, type: TimelineEvent['type'] = 'other', details?: string): TimelineEvent {
    const now = new Date();
    const event: TimelineEvent = {
      id: `event_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`,
      time: this.formatTime(now),
      timestamp: now.getTime(),
      action,
      file,
      type,
      details,
    };

    this.events.unshift(event); // Add to beginning
    if (this.events.length > this.MAX_EVENTS) {
      this.events = this.events.slice(0, this.MAX_EVENTS);
    }

    this.saveEvents();
    this.notifyListeners(event);
    return event;
  }

  /**
   * Get all events
   */
  getEvents(limit?: number): TimelineEvent[] {
    return limit ? this.events.slice(0, limit) : this.events;
  }

  /**
   * Clear all events
   */
  clearEvents(): void {
    this.events = [];
    this.saveEvents();
  }

  /**
   * Subscribe to new events
   */
  onEvent(callback: (event: TimelineEvent) => void): () => void {
    this.listeners.push(callback);
    return () => {
      this.listeners = this.listeners.filter(l => l !== callback);
    };
  }

  /**
   * Notify all listeners
   */
  private notifyListeners(event: TimelineEvent): void {
    this.listeners.forEach(listener => {
      try {
        listener(event);
      } catch (e) {
        console.error('[Timeline Service] Listener error:', e);
      }
    });
  }

  /**
   * Format time as "HH:MM AM/PM"
   */
  private formatTime(date: Date): string {
    const hours = date.getHours();
    const minutes = date.getMinutes();
    const ampm = hours >= 12 ? 'PM' : 'AM';
    const displayHours = hours % 12 || 12;
    const displayMinutes = minutes.toString().padStart(2, '0');
    return `${displayHours}:${displayMinutes} ${ampm}`;
  }

  /**
   * Save events to localStorage
   */
  private saveEvents(): void {
    try {
      localStorage.setItem(this.STORAGE_KEY, JSON.stringify(this.events));
    } catch (e) {
      console.error('[Timeline Service] Failed to save events:', e);
    }
  }

  /**
   * Load events from localStorage
   */
  private loadEvents(): void {
    try {
      const stored = localStorage.getItem(this.STORAGE_KEY);
      if (stored) {
        this.events = JSON.parse(stored);
      }
    } catch (e) {
      console.error('[Timeline Service] Failed to load events:', e);
    }
  }
}

export const timelineService = new TimelineService();

