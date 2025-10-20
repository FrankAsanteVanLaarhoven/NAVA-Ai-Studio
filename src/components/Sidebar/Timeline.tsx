import React from 'react';
import { Clock } from 'lucide-react';
import './Timeline.css';

interface TimelineEvent {
  time: string;
  action: string;
  file?: string;
}

interface TimelineProps {
  hideHeader?: boolean;
}

const TIMELINE_EVENTS: TimelineEvent[] = [
  { time: '8:05 PM', action: 'File opened', file: 'consciousness-integration.vnc' },
  { time: '8:04 PM', action: 'Server restarted', file: 'vite.config.ts' },
  { time: '8:03 PM', action: 'Configuration updated', file: 'index.html' },
  { time: '8:02 PM', action: 'Port changed to 3000' },
  { time: '8:00 PM', action: 'Server started' },
];

export const Timeline: React.FC<TimelineProps> = ({ hideHeader = false }) => {
  return (
    <div className="timeline-panel">
      {!hideHeader && (
        <div className="timeline-header">
          <Clock size={16} />
          <h3>TIMELINE</h3>
        </div>
      )}
      <div className="timeline-list">
        {TIMELINE_EVENTS.map((event, index) => (
          <div key={index} className="timeline-event">
            <span className="timeline-time">{event.time}</span>
            <span className="timeline-action">{event.action}</span>
            {event.file && <span className="timeline-file">{event.file}</span>}
          </div>
        ))}
      </div>
    </div>
  );
};

