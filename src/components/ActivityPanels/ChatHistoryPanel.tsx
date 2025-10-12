import React from 'react';
import { MessageSquare, Clock, Trash2, Pin, Search } from 'lucide-react';
import './ActivityPanels.css';

export const ChatHistoryPanel: React.FC = () => {
  const conversations = [
    {
      id: 1,
      title: 'Building NAVΛ Studio IDE',
      preview: 'Implementing resizable columns and activity bar...',
      timestamp: '2 hours ago',
      pinned: true,
    },
    {
      id: 2,
      title: 'VNC Syntax Highlighting',
      preview: 'How to implement custom Monaco language support...',
      timestamp: 'Yesterday',
      pinned: false,
    },
    {
      id: 3,
      title: 'WebAssembly Integration',
      preview: 'Setting up WASM preview engine with Three.js...',
      timestamp: '2 days ago',
      pinned: false,
    },
    {
      id: 4,
      title: 'Navigation Calculus Help',
      preview: 'Understanding Van Laarhoven lambda operators...',
      timestamp: '3 days ago',
      pinned: false,
    },
  ];

  return (
    <div className="activity-panel chat-history-panel">
      <div className="panel-header">
        <div className="panel-title">
          <MessageSquare size={16} />
          <span>Chat History</span>
        </div>
      </div>

      <div className="panel-content">
        {/* Search */}
        <div className="search-box">
          <Search size={16} />
          <input type="text" placeholder="Search conversations..." />
        </div>

        {/* New Chat */}
        <button className="new-chat-btn">
          <MessageSquare size={16} />
          <span>New Chat</span>
        </button>

        {/* Pinned Conversations */}
        {conversations.filter(c => c.pinned).length > 0 && (
          <div className="conversations-section">
            <div className="section-header">
              <Pin size={14} />
              <span>Pinned</span>
            </div>
            <div className="conversations-list">
              {conversations.filter(c => c.pinned).map((conv) => (
                <div key={conv.id} className="conversation-item pinned">
                  <div className="conversation-icon">
                    <MessageSquare size={16} />
                  </div>
                  <div className="conversation-info">
                    <div className="conversation-title">{conv.title}</div>
                    <div className="conversation-preview">{conv.preview}</div>
                    <div className="conversation-time">
                      <Clock size={12} />
                      <span>{conv.timestamp}</span>
                    </div>
                  </div>
                  <div className="conversation-actions">
                    <button className="icon-btn" title="Unpin">
                      <Pin size={14} />
                    </button>
                    <button className="icon-btn" title="Delete">
                      <Trash2 size={14} />
                    </button>
                  </div>
                </div>
              ))}
            </div>
          </div>
        )}

        {/* Recent Conversations */}
        <div className="conversations-section">
          <div className="section-header">
            <Clock size={14} />
            <span>Recent</span>
          </div>
          <div className="conversations-list">
            {conversations.filter(c => !c.pinned).map((conv) => (
              <div key={conv.id} className="conversation-item">
                <div className="conversation-icon">
                  <MessageSquare size={16} />
                </div>
                <div className="conversation-info">
                  <div className="conversation-title">{conv.title}</div>
                  <div className="conversation-preview">{conv.preview}</div>
                  <div className="conversation-time">
                    <Clock size={12} />
                    <span>{conv.timestamp}</span>
                  </div>
                </div>
                <div className="conversation-actions">
                  <button className="icon-btn" title="Pin">
                    <Pin size={14} />
                  </button>
                  <button className="icon-btn" title="Delete">
                    <Trash2 size={14} />
                  </button>
                </div>
              </div>
            ))}
          </div>
        </div>
      </div>
    </div>
  );
};

