import React, { useState, useEffect } from 'react';
import { Clock, Trash2 } from 'lucide-react';
import { timelineService, type TimelineEvent } from '../../services/timeline-service';
import './Timeline.css';

interface TimelineProps {
  hideHeader?: boolean;
}

export const Timeline: React.FC<TimelineProps> = ({ hideHeader = false }) => {
  const [events, setEvents] = useState<TimelineEvent[]>([]);

  useEffect(() => {
    // Load initial events
    setEvents(timelineService.getEvents(50));

    // Subscribe to new events
    const unsubscribe = timelineService.onEvent((event) => {
      setEvents(prev => [event, ...prev].slice(0, 50));
    });

    return unsubscribe;
  }, []);

  const handleClear = () => {
    if (confirm('Clear all timeline events?')) {
      timelineService.clearEvents();
      setEvents([]);
    }
  };

  const handleEventClick = (event: TimelineEvent) => {
    if (event.file) {
      // Emit event to open file
      const customEvent = new CustomEvent('nava:open-file', {
        detail: { path: event.file },
      });
      window.dispatchEvent(customEvent);
    }
  };

  return (
    <div className="timeline-panel">
      {!hideHeader && (
        <div className="timeline-header">
          <Clock size={16} />
          <h3>TIMELINE</h3>
          {events.length > 0 && (
            <button
              className="timeline-clear-btn"
              onClick={handleClear}
              title="Clear timeline"
            >
              <Trash2 size={14} />
            </button>
          )}
        </div>
      )}
      <div className="timeline-list">
        {events.length > 0 ? (
          events.map((event) => (
            <div 
              key={event.id} 
              className={`timeline-event ${event.type}`}
              onClick={() => handleEventClick(event)}
              title={event.details || event.action}
            >
              <span className="timeline-time">{event.time}</span>
              <span className="timeline-action">{event.action}</span>
              {event.file && (
                <span 
                  className="timeline-file"
                  onClick={(e) => {
                    e.stopPropagation();
                    handleEventClick(event);
                  }}
                >
                  {event.file}
                </span>
              )}
            </div>
          ))
        ) : (
          <div className="timeline-empty">
            <p>No events yet</p>
            <p className="timeline-hint">Activity will appear here</p>
          </div>
        )}
      </div>
    </div>
  );
};

