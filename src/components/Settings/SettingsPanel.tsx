import React, { useState } from 'react';
import { 
  Settings, 
  MessageSquare, 
  ArrowRight, 
  Box,
  Wrench,
  FileText,
  Database,
  Globe,
  Zap,
  BookOpen,
  User,
  Lock,
  Bell,
  Palette
} from 'lucide-react';
import './SettingsPanel.css';

interface SettingsPanelProps {
  isOpen: boolean;
  onClose: () => void;
}

type SettingsTab = 'general' | 'chat' | 'tab' | 'models' | 'tools' | 'rules' | 'indexing' | 'network' | 'beta';

export const SettingsPanel: React.FC<SettingsPanelProps> = ({ isOpen, onClose }) => {
  const [activeTab, setActiveTab] = useState<SettingsTab>('general');
  const [privacyMode, setPrivacyMode] = useState(true);
  const [systemNotifications, setSystemNotifications] = useState(true);
  const [menuBarIcon, setMenuBarIcon] = useState(true);
  const [completionSound, setCompletionSound] = useState(false);
  const [defaultLayout, setDefaultLayout] = useState('editor');

  if (!isOpen) return null;

  const tabs = [
    { id: 'general' as SettingsTab, icon: Settings, label: 'General' },
    { id: 'chat' as SettingsTab, icon: MessageSquare, label: 'Chat' },
    { id: 'tab' as SettingsTab, icon: ArrowRight, label: 'Tab' },
    { id: 'models' as SettingsTab, icon: Box, label: 'Models' },
    { id: 'tools' as SettingsTab, icon: Wrench, label: 'Tools & MCP' },
    { id: 'rules' as SettingsTab, icon: FileText, label: 'Rules & Memories' },
    { id: 'indexing' as SettingsTab, icon: Database, label: 'Indexing & Docs' },
    { id: 'network' as SettingsTab, icon: Globe, label: 'Network' },
    { id: 'beta' as SettingsTab, icon: Zap, label: 'Beta' },
  ];

  return (
    <div className="settings-overlay" onClick={onClose}>
      <div className="settings-panel" onClick={(e) => e.stopPropagation()}>
        {/* Sidebar */}
        <div className="settings-sidebar">
          <div className="settings-profile">
            <div className="profile-avatar">C</div>
            <div className="profile-info">
              <div className="profile-email">coupesville@gm...</div>
              <div className="profile-plan">Pro+ Plan</div>
            </div>
          </div>

          <div className="settings-search">
            <input type="text" placeholder="Search settings ⌘F" />
          </div>

          <div className="settings-tabs">
            {tabs.map((tab) => (
              <button
                key={tab.id}
                className={`settings-tab ${activeTab === tab.id ? 'active' : ''}`}
                onClick={() => setActiveTab(tab.id)}
              >
                <tab.icon size={18} />
                <span>{tab.label}</span>
              </button>
            ))}
          </div>

          <div className="settings-docs">
            <BookOpen size={16} />
            <span>Docs</span>
            <span className="external-icon">↗</span>
          </div>
        </div>

        {/* Content */}
        <div className="settings-content">
          <h2 className="settings-title">General</h2>

          {/* Manage Account */}
          <div className="settings-section">
            <div className="setting-item">
              <div className="setting-label">
                <h3>Manage Account</h3>
                <p>Manage your account and billing</p>
              </div>
              <button className="setting-btn">Open ↗</button>
            </div>
          </div>

          {/* Preferences */}
          <div className="settings-section">
            <h3 className="section-title">Preferences</h3>

            <div className="setting-item">
              <div className="setting-label">
                <h3>Editor Settings</h3>
                <p>Configure font, formatting, minimap and more</p>
              </div>
              <button className="setting-btn">Open</button>
            </div>

            <div className="setting-item">
              <div className="setting-label">
                <h3>Keyboard Shortcuts</h3>
                <p>Configure keyboard shortcuts</p>
              </div>
              <button className="setting-btn">Open</button>
            </div>

            <div className="setting-item">
              <div className="setting-label">
                <h3>Import Settings from VS Code</h3>
                <p>Import settings, extensions, and keybindings from VS Code</p>
              </div>
              <button className="setting-btn">Import</button>
            </div>

            <div className="setting-item">
              <div className="setting-label">
                <h3>Default Layout</h3>
                <p>The default layout for new windows</p>
              </div>
              <select 
                className="setting-select" 
                value={defaultLayout}
                onChange={(e) => setDefaultLayout(e.target.value)}
              >
                <option value="editor">Editor</option>
                <option value="chat">Chat</option>
                <option value="split">Split View</option>
              </select>
            </div>

            <div className="setting-item">
              <div className="setting-label">
                <h3>Reset "Don't Ask Again" Dialogs</h3>
                <p>See warnings and tips that you've hidden</p>
              </div>
              <button className="setting-btn">Show</button>
            </div>
          </div>

          {/* Privacy */}
          <div className="settings-section">
            <h3 className="section-title">Privacy</h3>

            <div className="setting-item privacy-item">
              <div className="setting-label">
                <div className="privacy-header">
                  <Lock size={16} />
                  <h3>Privacy Mode (Legacy)</h3>
                </div>
                <p>Your data will not be trained on or used to improve the product. We will not store your code.</p>
              </div>
              <select 
                className="setting-select"
                value={privacyMode ? 'legacy' : 'off'}
                onChange={(e) => setPrivacyMode(e.target.value === 'legacy')}
              >
                <option value="legacy">Privacy Mode (Legacy)</option>
                <option value="off">Off</option>
              </select>
            </div>

            {privacyMode && (
              <div className="privacy-notice">
                <p>Privacy Mode (Legacy) is enabled. Background Agent and some features not available.</p>
                <button className="link-btn">Switch to Privacy Mode</button>
              </div>
            )}
          </div>

          {/* Notifications */}
          <div className="settings-section">
            <h3 className="section-title">Notifications</h3>

            <div className="setting-item">
              <div className="setting-label">
                <h3>System Notifications</h3>
                <p>Show system notifications when Agent completes or needs attention</p>
              </div>
              <label className="toggle-switch">
                <input
                  type="checkbox"
                  checked={systemNotifications}
                  onChange={(e) => setSystemNotifications(e.target.checked)}
                />
                <span className="toggle-slider"></span>
              </label>
            </div>

            <div className="setting-item">
              <div className="setting-label">
                <h3>Menu Bar Icon</h3>
                <p>Show Cursor in menu bar</p>
              </div>
              <label className="toggle-switch">
                <input
                  type="checkbox"
                  checked={menuBarIcon}
                  onChange={(e) => setMenuBarIcon(e.target.checked)}
                />
                <span className="toggle-slider"></span>
              </label>
            </div>

            <div className="setting-item">
              <div className="setting-label">
                <h3>Completion Sound</h3>
                <p>Play a sound when Agent finishes responding</p>
              </div>
              <label className="toggle-switch">
                <input
                  type="checkbox"
                  checked={completionSound}
                  onChange={(e) => setCompletionSound(e.target.checked)}
                />
                <span className="toggle-slider"></span>
              </label>
            </div>
          </div>

          {/* Accessibility */}
          <div className="settings-section">
            <h3 className="section-title">Accessibility</h3>

            <div className="setting-item">
              <div className="setting-label">
                <h3>Voice Assistant</h3>
                <p>Enable text-to-speech and voice commands for accessibility</p>
              </div>
              <button className="setting-btn">Configure</button>
            </div>

            <div className="setting-item">
              <div className="setting-label">
                <h3>Keyboard Navigation</h3>
                <p>Enhanced keyboard shortcuts for full IDE control</p>
              </div>
              <label className="toggle-switch">
                <input type="checkbox" defaultChecked />
                <span className="toggle-slider"></span>
              </label>
            </div>

            <div className="setting-item">
              <div className="setting-label">
                <h3>High Contrast Mode</h3>
                <p>Increase visual contrast for better readability</p>
              </div>
              <label className="toggle-switch">
                <input type="checkbox" />
                <span className="toggle-slider"></span>
              </label>
            </div>

            <div className="setting-item">
              <div className="setting-label">
                <h3>Screen Reader Support</h3>
                <p>Optimize for screen readers and assistive technologies</p>
              </div>
              <label className="toggle-switch">
                <input type="checkbox" defaultChecked />
                <span className="toggle-slider"></span>
              </label>
            </div>
          </div>

          {/* Log Out */}
          <div className="settings-section">
            <button className="logout-btn">Log Out</button>
          </div>
        </div>
      </div>
    </div>
  );
};

