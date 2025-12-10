import React, { useState } from 'react';
import { 
  FolderTree, 
  Search, 
  GitBranch, 
  Play, 
  Database,
  Box,
  CreditCard,
  Github,
  MessageSquare,
  ExternalLink,
  Settings,
  User,
  Download,
  Trash2,
  PinIcon,
  Plus,
  LogIn,
  MoreVertical,
  Bot,
  Gamepad2,
  Zap,
  Navigation,
  Route,
  TrendingUp
} from 'lucide-react';
import './ActivityBar.css';

export type ActivityType = 
  | 'explorer' 
  | 'search' 
  | 'source-control' 
  | 'debug' 
  | 'remote-explorer'
  | 'extensions' 
  | 'containers'
  | 'stripe'
  | 'github'
  | 'chat-history'
  | 'ros-learning'
  | 'mcp-toolkit'
  | 'simulation'
  | 'profile'
  | 'settings'
  | 'univarm-starter'
  | 'univarm-advanced'
  | 'workspace'
  | 'app-centre'
  | 'discovery';

interface ActivityBarProps {
  activeActivity: ActivityType | null;
  onActivityChange: (activity: ActivityType) => void;
  onNewWindow: () => void;
  onExportChat: () => void;
  onDeleteChat: () => void;
  onSignIn: () => void;
  onAddContext: () => void;
  onCustomInstructions: () => void;
}

export const ActivityBar: React.FC<ActivityBarProps> = ({
  activeActivity,
  onActivityChange,
  onNewWindow,
  onExportChat,
  onDeleteChat,
  onSignIn,
  onAddContext,
  onCustomInstructions,
}) => {
  const [showChatMenu, setShowChatMenu] = useState(false);
  const [showProfileMenu, setShowProfileMenu] = useState(false);

  const activities = [
    { id: 'explorer' as ActivityType, icon: FolderTree, label: 'Explorer (Ctrl+Shift+E)', emoji: null },
    { id: 'search' as ActivityType, icon: Search, label: 'Search (Ctrl+Shift+F)', emoji: null },
    { id: 'source-control' as ActivityType, icon: GitBranch, label: 'Source Control (Ctrl+Shift+G)', emoji: null },
    { id: 'debug' as ActivityType, icon: Play, label: 'Run and Debug (Ctrl+Shift+D)', emoji: null },
    { id: 'discovery' as ActivityType, icon: TrendingUp, label: 'Discover 🔥 (News & Papers)', emoji: null },
    { id: 'simulation' as ActivityType, icon: Gamepad2, label: 'NAVΛ SIM 🤖 (Rust-Powered)', emoji: null },
    { id: 'univarm-starter' as ActivityType, icon: null, label: 'Univarm Starter ⚡ (Path Optimizer & Code Generator)', emoji: '⚡' },
    { id: 'univarm-advanced' as ActivityType, icon: null, label: 'Univarm Advanced 🦀 (Production Path Planning)', emoji: '🦀' },
    { id: 'remote-explorer' as ActivityType, icon: Database, label: 'Remote Explorer', emoji: null },
    { id: 'extensions' as ActivityType, icon: Box, label: 'Extensions (Ctrl+Shift+X)', emoji: null },
    { id: 'containers' as ActivityType, icon: Database, label: 'Containers', emoji: null },
    { id: 'stripe' as ActivityType, icon: CreditCard, label: 'Stripe Integration', emoji: null },
    { id: 'github' as ActivityType, icon: Github, label: 'GitHub', emoji: null },
    { id: 'chat-history' as ActivityType, icon: MessageSquare, label: 'Chat History', emoji: null },
    { id: 'ros-learning' as ActivityType, icon: Bot, label: 'ROS Learning Center 🤖 (Free Courses)', emoji: null },
    { id: 'mcp-toolkit' as ActivityType, icon: Bot, label: 'MCP Toolkit 🤖 (AI Agent Management)', emoji: null },
  ];

  return (
    <div className="activity-bar">
      {/* Main Activities */}
      <div className="activity-bar-main">
        {activities.map((activity) => {
          const IconComponent = activity.icon;
          return (
            <button
              key={activity.id}
              className={`activity-item ${activeActivity === activity.id ? 'active' : ''}`}
              onClick={() => onActivityChange(activity.id)}
              title={activity.label}
            >
              {activity.emoji ? (
                <span style={{ fontSize: '24px', lineHeight: 1, display: 'flex', alignItems: 'center', justifyContent: 'center' }}>
                  {activity.emoji}
                </span>
              ) : IconComponent ? (
                <IconComponent size={24} />
              ) : null}
            </button>
          );
        })}
      </div>

      {/* Bottom Actions */}
      <div className="activity-bar-bottom">
        {/* Chat Options Menu */}
        <div className="activity-item-menu">
          <button
            className="activity-item"
            onClick={() => setShowChatMenu(!showChatMenu)}
            title="Chat Options"
          >
            <MoreVertical size={24} />
          </button>
          {showChatMenu && (
            <div className="activity-menu">
              <button onClick={onNewWindow}>
                <ExternalLink size={16} />
                <span>Open New Window</span>
              </button>
              <button onClick={onExportChat}>
                <Download size={16} />
                <span>Export Conversation</span>
              </button>
              <button onClick={onDeleteChat}>
                <Trash2 size={16} />
                <span>Delete Chat</span>
              </button>
              <button onClick={onCustomInstructions}>
                <Settings size={16} />
                <span>Custom Instructions (0/100)</span>
              </button>
              <button onClick={onAddContext}>
                <Plus size={16} />
                <span>Add Context</span>
              </button>
              <div className="menu-divider" />
              <button onClick={onAddContext}>
                <PinIcon size={16} />
                <span>Pinned Contexts</span>
              </button>
            </div>
          )}
        </div>

        {/* Profile Menu */}
        <div className="activity-item-menu">
          <button
            className="activity-item"
            onClick={() => setShowProfileMenu(!showProfileMenu)}
            title="Account & Profile"
          >
            <User size={24} />
          </button>
          {showProfileMenu && (
            <div className="activity-menu">
              <button onClick={onSignIn}>
                <LogIn size={16} />
                <span>Sign In</span>
              </button>
              <button onClick={() => onActivityChange('profile')}>
                <User size={16} />
                <span>Open Profile</span>
              </button>
              <div className="menu-divider" />
              <button onClick={() => onActivityChange('settings')}>
                <Settings size={16} />
                <span>Settings</span>
              </button>
            </div>
          )}
        </div>
      </div>
    </div>
  );
};

