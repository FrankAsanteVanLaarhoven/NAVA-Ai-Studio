import React, { useState, useEffect } from 'react';
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
  Palette,
  Plus,
  Trash2,
  Edit,
  Key,
  Check,
  X
} from 'lucide-react';
import './SettingsPanel.css';
import { 
  modelConfigService, 
  type AIModel, 
  type CustomModel, 
  type ModelProvider,
  type ProviderConfig 
} from '../../services/model-config-service';
import { 
  mcpConnectionsService, 
  type MCPConnection, 
  type MCPServiceType,
  type MCPServiceDefinition 
} from '../../services/mcp-connections-service';
import { ResizablePanel } from '../Common/ResizablePanel';
import { authService, type User as UserType } from '../../services/auth-service';
import { LoginModal } from '../Auth/LoginModal';

type SettingsTab = 'general' | 'chat' | 'tab' | 'models' | 'tools' | 'rules' | 'indexing' | 'network' | 'beta';

export const SettingsPanel: React.FC<SettingsPanelProps> = ({ isOpen, onClose }) => {
  const [activeTab, setActiveTab] = useState<SettingsTab>('general');
  const [privacyMode, setPrivacyMode] = useState(true);
  const [systemNotifications, setSystemNotifications] = useState(true);
  const [menuBarIcon, setMenuBarIcon] = useState(true);
  const [completionSound, setCompletionSound] = useState(false);
  const [defaultLayout, setDefaultLayout] = useState('editor');
  const [currentUser, setCurrentUser] = useState<UserType | null>(authService.getCurrentUser());
  const [showLoginModal, setShowLoginModal] = useState(false);
  const [showEditProfile, setShowEditProfile] = useState(false);
  const [editingName, setEditingName] = useState('');
  const [editingEmail, setEditingEmail] = useState('');
  
  // Models tab state
  const [models, setModels] = useState<AIModel[]>([]);
  const [defaultModel, setDefaultModel] = useState<string>('');
  const [providerConfigs, setProviderConfigs] = useState<Record<ModelProvider, ProviderConfig>>({} as Record<ModelProvider, ProviderConfig>);
  const [editingModel, setEditingModel] = useState<CustomModel | null>(null);
  const [showAddModel, setShowAddModel] = useState(false);
  const [editingProvider, setEditingProvider] = useState<ModelProvider | null>(null);
  
  // MCP Connections state
  const [mcpConnections, setMcpConnections] = useState<MCPConnection[]>([]);
  const [mcpServices, setMcpServices] = useState<MCPServiceDefinition[]>([]);
  const [connectingService, setConnectingService] = useState<MCPServiceType | null>(null);
  const [apiKeyInput, setApiKeyInput] = useState<{ service: MCPServiceType; value: string } | null>(null);
  const [sidebarWidth, setSidebarWidth] = useState(() => {
    const saved = localStorage.getItem('settings-sidebar-width');
    return saved ? parseInt(saved, 10) : 280;
  });

  // Load models and user on mount
  useEffect(() => {
    if (isOpen) {
      setCurrentUser(authService.getCurrentUser());
      setModels(modelConfigService.getModels());
      setDefaultModel(modelConfigService.getDefaultModel());
      
      // Load provider configs
      const providers: ModelProvider[] = ['gemini', 'openrouter', 'openai', 'anthropic', 'custom'];
      const configs = {} as Record<ModelProvider, ProviderConfig>;
      providers.forEach(provider => {
        configs[provider] = modelConfigService.getProviderConfig(provider);
      });
      setProviderConfigs(configs);
      
      // Load MCP connections
      setMcpConnections(mcpConnectionsService.getConnections());
      setMcpServices(mcpConnectionsService.getServiceDefinitions());
    }
  }, [isOpen]);

  const handleLogin = () => {
    setShowLoginModal(true);
  };

  const handleLoginSuccess = () => {
    setCurrentUser(authService.getCurrentUser());
    setShowLoginModal(false);
  };

  const handleLogout = () => {
    if (confirm('Are you sure you want to log out?')) {
      authService.logout();
      setCurrentUser(null);
    }
  };

  const handleEditProfile = () => {
    if (currentUser) {
      setEditingName(currentUser.name);
      setEditingEmail(currentUser.email);
      setShowEditProfile(true);
    }
  };

  const handleSaveProfile = async () => {
    if (!currentUser) return;
    
    try {
      const updated = await authService.updateUserProfile({
        name: editingName,
        email: editingEmail,
      });
      setCurrentUser(updated);
      setShowEditProfile(false);
    } catch (error: any) {
      alert(`Failed to update profile: ${error.message}`);
    }
  };

  const handleSwitchUser = async (userId: string) => {
    try {
      const user = await authService.switchUser(userId);
      setCurrentUser(user);
    } catch (error: any) {
      alert(`Failed to switch user: ${error.message}`);
    }
  };

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
      <div className="settings-panel" onClick={(e) => e.stopPropagation()} style={{ display: 'flex' }}>
        {/* Sidebar with Resizable Divider */}
        <ResizablePanel
          side="left"
          defaultWidth={sidebarWidth}
          minWidth={200}
          maxWidth={500}
          storageKey="settings-sidebar-width"
          onResize={(size) => {
            setSidebarWidth(size);
            localStorage.setItem('settings-sidebar-width', size.toString());
          }}
        >
          <div className="settings-sidebar">
          <div className="settings-profile">
            <div className="profile-avatar">
              {currentUser ? (currentUser.avatar || currentUser.name.charAt(0).toUpperCase()) : '?'}
            </div>
            <div className="profile-info">
              {currentUser ? (
                <>
                  <div className="profile-email" title={currentUser.email}>
                    {currentUser.email.length > 20 ? `${currentUser.email.substring(0, 20)}...` : currentUser.email}
                  </div>
                  <div className="profile-plan">
                    {currentUser.plan === 'free' ? 'Free Plan' : 
                     currentUser.plan === 'pro' ? 'Pro Plan' :
                     currentUser.plan === 'pro-plus' ? 'Pro+ Plan' :
                     'Enterprise Plan'}
                  </div>
                </>
              ) : (
                <>
                  <div className="profile-email">Not signed in</div>
                  <div className="profile-plan">Guest Mode</div>
                </>
              )}
            </div>
            {currentUser && (
              <button
                className="profile-edit-btn"
                onClick={handleEditProfile}
                title="Edit Profile"
              >
                <Edit size={14} />
              </button>
            )}
          </div>
          
          {!currentUser && (
            <div style={{ padding: '0 20px 16px' }}>
              <button
                className="login-btn"
                onClick={handleLogin}
                style={{ width: '100%' }}
              >
                Sign In
              </button>
            </div>
          )}
          
          {currentUser && (
            <div style={{ padding: '0 20px 16px' }}>
              <button
                className="logout-btn"
                onClick={handleLogout}
                style={{ width: '100%' }}
              >
                Log Out
              </button>
            </div>
          )}

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
        </ResizablePanel>

        {/* Content Area */}
        <div className="settings-content">
          {activeTab === 'general' && (
            <>
              <h2 className="settings-title">General</h2>

          {/* Manage Account */}
          <div className="settings-section">
            <div className="setting-item">
              <div className="setting-label">
                <h3>Manage Account</h3>
                <p>Manage your account and billing</p>
              </div>
              {currentUser ? (
                <button className="setting-btn" onClick={handleEditProfile}>
                  Edit Profile
                </button>
              ) : (
                <button className="setting-btn" onClick={handleLogin}>
                  Sign In
                </button>
              )}
            </div>
            
            {currentUser && (
              <>
                <div className="setting-item">
                  <div className="setting-label">
                    <h3>Account Details</h3>
                    <p>View and edit your account information</p>
                  </div>
                  <div style={{ display: 'flex', flexDirection: 'column', gap: '8px', alignItems: 'flex-end' }}>
                    <div style={{ fontSize: '12px', color: '#858585' }}>
                      Name: <strong style={{ color: '#cccccc' }}>{currentUser.name}</strong>
                    </div>
                    <div style={{ fontSize: '12px', color: '#858585' }}>
                      Email: <strong style={{ color: '#cccccc' }}>{currentUser.email}</strong>
                    </div>
                    <div style={{ fontSize: '12px', color: '#858585' }}>
                      Plan: <strong style={{ color: '#cccccc' }}>{currentUser.plan}</strong>
                    </div>
                  </div>
                </div>
                
                {/* Switch User */}
                {authService.getStoredUsers().length > 1 && (
                  <div className="setting-item">
                    <div className="setting-label">
                      <h3>Switch User</h3>
                      <p>Switch to a different account</p>
                    </div>
                    <select
                      className="setting-select"
                      value={currentUser.id}
                      onChange={(e) => handleSwitchUser(e.target.value)}
                    >
                      {authService.getStoredUsers().map(user => (
                        <option key={user.id} value={user.id}>
                          {user.email} ({user.plan})
                        </option>
                      ))}
                    </select>
                  </div>
                )}
              </>
            )}
          </div>

          {/* Preferences */}
          <div className="settings-section">
            <h3 className="section-title">Preferences</h3>

            <div className="setting-item">
              <div className="setting-label">
                <h3>Editor Settings</h3>
                <p>Configure font, formatting, minimap and more</p>
              </div>
              <button 
                className="setting-btn"
                onClick={() => {
                  setActiveTab('chat');
                  // Could also open a dedicated editor settings modal
                }}
              >
                Open
              </button>
            </div>

            <div className="setting-item">
              <div className="setting-label">
                <h3>Keyboard Shortcuts</h3>
                <p>Configure keyboard shortcuts</p>
              </div>
              <button 
                className="setting-btn"
                onClick={() => {
                  // Open keyboard shortcuts help
                  const event = new CustomEvent('nava:show-shortcuts');
                  window.dispatchEvent(event);
                }}
              >
                Open
              </button>
            </div>

            <div className="setting-item">
              <div className="setting-label">
                <h3>Import Settings from VS Code</h3>
                <p>Import settings, extensions, and keybindings from VS Code</p>
              </div>
              <button 
                className="setting-btn"
                onClick={async () => {
                  const input = document.createElement('input');
                  input.type = 'file';
                  input.accept = '.json';
                  input.onchange = async (e) => {
                    const file = (e.target as HTMLInputElement).files?.[0];
                    if (file) {
                      try {
                        const text = await file.text();
                        const settings = JSON.parse(text);
                        // Import VS Code settings
                        if (settings.editor) {
                          localStorage.setItem('nava_editor_settings', JSON.stringify(settings.editor));
                        }
                        if (settings.keybindings) {
                          localStorage.setItem('nava_keybindings', JSON.stringify(settings.keybindings));
                        }
                        alert('Settings imported successfully!');
                      } catch (error) {
                        alert('Failed to import settings. Please check the file format.');
                      }
                    }
                  };
                  input.click();
                }}
              >
                Import
              </button>
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
              <button 
                className="setting-btn"
                onClick={() => {
                  // Clear all "don't ask again" flags
                  const keys = Object.keys(localStorage);
                  keys.forEach(key => {
                    if (key.startsWith('nava_dont_ask_')) {
                      localStorage.removeItem(key);
                    }
                  });
                  alert('All "Don\'t Ask Again" dialogs have been reset.');
                }}
              >
                Show
              </button>
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
          {currentUser && (
            <div className="settings-section">
              <button className="logout-btn" onClick={handleLogout}>
                Log Out
              </button>
            </div>
          )}
            </>
          )}

          {activeTab === 'models' && (
            <>
              <h2 className="settings-title">Models</h2>

              {/* Default Model */}
              <div className="settings-section">
                <h3 className="section-title">Default Model</h3>
                <div className="setting-item">
                  <div className="setting-label">
                    <h3>Default AI Model</h3>
                    <p>Select the default model for AI Assistant</p>
                  </div>
                  <select
                    className="setting-select"
                    value={defaultModel}
                    onChange={(e) => {
                      const newDefault = e.target.value;
                      setDefaultModel(newDefault);
                      modelConfigService.setDefaultModel(newDefault);
                    }}
                  >
                    {models.filter(m => m.enabled).map(model => (
                      <option key={model.id} value={model.id}>
                        {model.name}
                      </option>
                    ))}
                  </select>
                </div>
              </div>

              {/* Provider API Keys */}
              <div className="settings-section">
                <h3 className="section-title">Provider API Keys</h3>
                <p className="section-description">Configure API keys for different providers. Keys are stored securely and can also be set via .env file.</p>

                {(['gemini', 'openrouter', 'openai', 'anthropic'] as ModelProvider[]).map(provider => {
                  const config = providerConfigs[provider];
                  const isEditing = editingProvider === provider;
                  const envKey = provider === 'gemini' 
                    ? import.meta.env.VITE_GEMINI_API_KEY 
                    : provider === 'openrouter' 
                    ? import.meta.env.VITE_OPENROUTER_API_KEY 
                    : '';

                  return (
                    <div key={provider} className="setting-item">
                      <div className="setting-label">
                        <div style={{ display: 'flex', alignItems: 'center', gap: '8px', marginBottom: '4px' }}>
                          <Key size={16} style={{ color: '#858585' }} />
                          <h3 style={{ textTransform: 'capitalize' }}>{provider === 'openrouter' ? 'OpenRouter' : provider === 'openai' ? 'OpenAI' : provider === 'anthropic' ? 'Anthropic' : 'Google Gemini'}</h3>
                          {envKey && (
                            <span style={{ 
                              fontSize: '11px', 
                              color: '#4285f4', 
                              background: 'rgba(66, 133, 244, 0.1)', 
                              padding: '2px 6px', 
                              borderRadius: '4px' 
                            }}>
                              From .env
                            </span>
                          )}
                        </div>
                        <p>
                          {provider === 'gemini' && 'Get FREE API key from Google AI Studio'}
                          {provider === 'openrouter' && 'Access 100+ models via OpenRouter'}
                          {provider === 'openai' && 'Direct OpenAI API access'}
                          {provider === 'anthropic' && 'Direct Anthropic Claude API access'}
                        </p>
                      </div>
                      <div style={{ display: 'flex', gap: '8px', alignItems: 'center' }}>
                        {isEditing ? (
                          <>
                            <input
                              type="password"
                              className="api-key-input"
                              placeholder={envKey ? "Using .env key" : "Enter API key..."}
                              defaultValue={config?.apiKey || ''}
                              disabled={!!envKey}
                              onKeyDown={(e) => {
                                if (e.key === 'Enter') {
                                  const input = e.target as HTMLInputElement;
                                  if (!envKey) {
                                    modelConfigService.setProviderApiKey(provider, input.value);
                                    setProviderConfigs({
                                      ...providerConfigs,
                                      [provider]: { ...config, apiKey: input.value },
                                    });
                                  }
                                  setEditingProvider(null);
                                } else if (e.key === 'Escape') {
                                  setEditingProvider(null);
                                }
                              }}
                              autoFocus
                            />
                            <button
                              className="icon-btn"
                              onClick={() => {
                                const input = document.querySelector('.api-key-input') as HTMLInputElement;
                                if (input && !envKey) {
                                  modelConfigService.setProviderApiKey(provider, input.value);
                                  setProviderConfigs({
                                    ...providerConfigs,
                                    [provider]: { ...config, apiKey: input.value },
                                  });
                                }
                                setEditingProvider(null);
                              }}
                            >
                              <Check size={16} />
                            </button>
                            <button
                              className="icon-btn"
                              onClick={() => setEditingProvider(null)}
                            >
                              <X size={16} />
                            </button>
                          </>
                        ) : (
                          <>
                            <div style={{ 
                              padding: '8px 12px', 
                              background: config?.apiKey || envKey ? 'rgba(0, 255, 0, 0.1)' : 'rgba(255, 255, 255, 0.05)',
                              border: `1px solid ${config?.apiKey || envKey ? 'rgba(0, 255, 0, 0.3)' : '#3c3c3c'}`,
                              borderRadius: '6px',
                              fontSize: '12px',
                              color: config?.apiKey || envKey ? '#00ff00' : '#858585',
                              minWidth: '120px',
                              textAlign: 'center'
                            }}>
                              {envKey ? '✓ Using .env' : config?.apiKey ? '✓ Configured' : 'Not set'}
                            </div>
                            <button
                              className="setting-btn"
                              onClick={() => setEditingProvider(provider)}
                              disabled={!!envKey}
                              title={envKey ? 'API key is loaded from .env file' : 'Edit API key'}
                            >
                              <Edit size={14} style={{ marginRight: '4px' }} />
                              {envKey ? 'View' : 'Edit'}
                            </button>
                          </>
                        )}
                      </div>
                    </div>
                  );
                })}
              </div>

              {/* Available Models */}
              <div className="settings-section">
                <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginBottom: '20px' }}>
                  <h3 className="section-title">Available Models</h3>
                  <button
                    className="setting-btn"
                    onClick={() => setShowAddModel(true)}
                    style={{ display: 'flex', alignItems: 'center', gap: '6px' }}
                  >
                    <Plus size={14} />
                    Add Custom Model
                  </button>
                </div>

                {models.map(model => (
                  <div key={model.id} className="setting-item">
                    <div className="setting-label">
                      <div style={{ display: 'flex', alignItems: 'center', gap: '8px', marginBottom: '4px' }}>
                        <h3>{model.name}</h3>
                        {model.isCustom && (
                          <span style={{ 
                            fontSize: '10px', 
                            color: '#858585', 
                            background: '#2a2a2a', 
                            padding: '2px 6px', 
                            borderRadius: '4px' 
                          }}>
                            Custom
                          </span>
                        )}
                      </div>
                      <p>{model.description}</p>
                      <div style={{ 
                        display: 'flex', 
                        gap: '12px', 
                        marginTop: '8px', 
                        fontSize: '11px', 
                        color: '#858585' 
                      }}>
                        <span>Provider: {model.provider}</span>
                        <span>Context: {model.contextLength.toLocaleString()} tokens</span>
                      </div>
                    </div>
                    <div style={{ display: 'flex', gap: '8px', alignItems: 'center' }}>
                      <label className="toggle-switch">
                        <input
                          type="checkbox"
                          checked={model.enabled}
                          onChange={(e) => {
                            const updated = modelConfigService.updateModel(model.id, { enabled: e.target.checked });
                            if (updated) {
                              setModels(modelConfigService.getModels());
                            }
                          }}
                        />
                        <span className="toggle-slider"></span>
                      </label>
                      {model.isCustom && (
                        <button
                          className="icon-btn"
                          onClick={() => {
                            if (confirm(`Delete custom model "${model.name}"?`)) {
                              modelConfigService.deleteModel(model.id);
                              setModels(modelConfigService.getModels());
                            }
                          }}
                          style={{ color: '#f44336' }}
                        >
                          <Trash2 size={16} />
                        </button>
                      )}
                    </div>
                  </div>
                ))}
              </div>

              {/* Add Custom Model Modal */}
              {showAddModel && (
                <div className="modal-overlay" onClick={() => setShowAddModel(false)}>
                  <div className="modal-content" onClick={(e) => e.stopPropagation()}>
                    <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginBottom: '20px' }}>
                      <h3>Add Custom Model</h3>
                      <button className="icon-btn" onClick={() => setShowAddModel(false)}>
                        <X size={20} />
                      </button>
                    </div>
                    <CustomModelForm
                      onSave={(model) => {
                        modelConfigService.addCustomModel(model);
                        setModels(modelConfigService.getModels());
                        setShowAddModel(false);
                      }}
                      onCancel={() => setShowAddModel(false)}
                    />
                  </div>
                </div>
              )}

              {/* MCP Service Connections */}
              <div className="settings-section">
                <h3 className="section-title">MCP Service Connections</h3>
                <p className="section-description">
                  Connect to external services (GitHub, Slack, Linear, etc.) to enable AI models with context from these platforms.
                </p>

                <div style={{ display: 'grid', gridTemplateColumns: 'repeat(auto-fill, minmax(300px, 1fr))', gap: '16px', marginTop: '20px' }}>
                  {mcpServices.map(service => {
                    const connection = mcpConnections.find(c => c.serviceType === service.type);
                    const isConnecting = connectingService === service.type;
                    const isApiKeyMode = apiKeyInput?.service === service.type;

                    return (
                      <div key={service.type} style={{
                        border: '1px solid #3c3c3c',
                        borderRadius: '8px',
                        padding: '16px',
                        background: connection?.connected ? 'rgba(0, 255, 0, 0.05)' : '#252526',
                      }}>
                        <div style={{ display: 'flex', alignItems: 'center', gap: '12px', marginBottom: '12px' }}>
                          <span style={{ fontSize: '24px' }}>{service.icon}</span>
                          <div style={{ flex: 1 }}>
                            <h4 style={{ margin: 0, fontSize: '14px', color: '#ffffff' }}>{service.name}</h4>
                            <p style={{ margin: '4px 0 0', fontSize: '11px', color: '#858585' }}>{service.description}</p>
                          </div>
                          {connection?.connected && (
                            <span style={{
                              fontSize: '10px',
                              color: '#00ff00',
                              background: 'rgba(0, 255, 0, 0.15)',
                              padding: '4px 8px',
                              borderRadius: '4px',
                            }}>
                              ✓ Connected
                            </span>
                          )}
                        </div>

                        {connection?.metadata.userName && (
                          <div style={{ fontSize: '11px', color: '#858585', marginBottom: '12px' }}>
                            Connected as: <strong>{connection.metadata.userName}</strong>
                          </div>
                        )}

                        {isApiKeyMode ? (
                          <div style={{ display: 'flex', flexDirection: 'column', gap: '8px' }}>
                            <input
                              type="password"
                              className="api-key-input"
                              placeholder={`Enter ${service.name} API key...`}
                              value={apiKeyInput.value}
                              onChange={(e) => setApiKeyInput({ ...apiKeyInput, value: e.target.value })}
                              autoFocus
                            />
                            <div style={{ display: 'flex', gap: '8px' }}>
                              <button
                                className="setting-btn"
                                style={{ flex: 1, fontSize: '12px', padding: '6px 12px' }}
                                onClick={async () => {
                                  try {
                                    await mcpConnectionsService.connectWithApiKey(service.type, apiKeyInput.value);
                                    setMcpConnections(mcpConnectionsService.getConnections());
                                    setApiKeyInput(null);
                                  } catch (error: any) {
                                    alert(`Connection failed: ${error.message}`);
                                  }
                                }}
                              >
                                Connect
                              </button>
                              <button
                                className="setting-btn"
                                style={{ fontSize: '12px', padding: '6px 12px' }}
                                onClick={() => setApiKeyInput(null)}
                              >
                                Cancel
                              </button>
                            </div>
                          </div>
                        ) : connection?.connected ? (
                          <div style={{ display: 'flex', gap: '8px' }}>
                            <button
                              className="setting-btn"
                              style={{ flex: 1, fontSize: '12px', padding: '6px 12px', background: '#f44336', borderColor: '#f44336', color: '#ffffff' }}
                              onClick={() => {
                                mcpConnectionsService.disconnectConnection(connection.id);
                                setMcpConnections(mcpConnectionsService.getConnections());
                              }}
                            >
                              Disconnect
                            </button>
                            <button
                              className="setting-btn"
                              style={{ fontSize: '12px', padding: '6px 12px' }}
                              onClick={async () => {
                                const isValid = await mcpConnectionsService.testConnection(connection);
                                alert(isValid ? 'Connection is valid!' : 'Connection test failed');
                              }}
                            >
                              Test
                            </button>
                          </div>
                        ) : (
                          <button
                            className="setting-btn"
                            style={{ width: '100%', fontSize: '12px', padding: '8px 12px' }}
                            onClick={async () => {
                              if (service.authType === 'oauth') {
                                setConnectingService(service.type);
                                try {
                                  await mcpConnectionsService.connectOAuth(service.type);
                                } catch (error: any) {
                                  alert(`OAuth connection failed: ${error.message}`);
                                  setConnectingService(null);
                                }
                              } else {
                                setApiKeyInput({ service: service.type, value: '' });
                              }
                            }}
                            disabled={isConnecting}
                          >
                            {isConnecting ? 'Connecting...' : service.authType === 'oauth' ? `Connect with ${service.name}` : `Connect with API Key`}
                          </button>
                        )}

                        {service.documentationUrl && (
                          <a
                            href={service.documentationUrl}
                            target="_blank"
                            rel="noopener noreferrer"
                            style={{
                              display: 'block',
                              marginTop: '8px',
                              fontSize: '11px',
                              color: '#4fc1ff',
                              textDecoration: 'none',
                            }}
                          >
                            View Documentation →
                          </a>
                        )}
                      </div>
                    );
                  })}
                </div>
              </div>
            </>
          )}

          {activeTab !== 'general' && activeTab !== 'models' && (
            <div style={{ padding: '40px', textAlign: 'center', color: '#858585' }}>
              <h2 className="settings-title">{tabs.find(t => t.id === activeTab)?.label || 'Settings'}</h2>
              <p>This section is coming soon...</p>
            </div>
          )}
        </div>
      </div>
      
      {/* Login Modal */}
      <LoginModal
        isOpen={showLoginModal}
        onClose={() => setShowLoginModal(false)}
        onLoginSuccess={handleLoginSuccess}
      />
      
      {/* Edit Profile Modal */}
      {showEditProfile && currentUser && (
        <div className="modal-overlay" onClick={() => setShowEditProfile(false)}>
          <div className="modal-content" onClick={(e) => e.stopPropagation()}>
            <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginBottom: '20px' }}>
              <h3>Edit Profile</h3>
              <button className="icon-btn" onClick={() => setShowEditProfile(false)}>
                <X size={20} />
              </button>
            </div>
            <div style={{ display: 'flex', flexDirection: 'column', gap: '16px' }}>
              <div>
                <label style={{ display: 'block', marginBottom: '6px', fontSize: '13px', color: '#cccccc' }}>
                  Name
                </label>
                <input
                  type="text"
                  value={editingName}
                  onChange={(e) => setEditingName(e.target.value)}
                  className="form-input"
                  placeholder="Your name"
                />
              </div>
              <div>
                <label style={{ display: 'block', marginBottom: '6px', fontSize: '13px', color: '#cccccc' }}>
                  Email
                </label>
                <input
                  type="email"
                  value={editingEmail}
                  onChange={(e) => setEditingEmail(e.target.value)}
                  className="form-input"
                  placeholder="your@email.com"
                />
              </div>
              <div style={{ display: 'flex', gap: '12px', justifyContent: 'flex-end', marginTop: '8px' }}>
                <button type="button" className="setting-btn" onClick={() => setShowEditProfile(false)}>
                  Cancel
                </button>
                <button type="button" className="setting-btn" style={{ background: '#007acc', borderColor: '#007acc', color: '#ffffff' }} onClick={handleSaveProfile}>
                  Save Changes
                </button>
              </div>
            </div>
          </div>
        </div>
      )}
    </div>
  );
};

// Custom Model Form Component
interface CustomModelFormProps {
  onSave: (model: Omit<CustomModel, 'id' | 'isCustom'>) => void;
  onCancel: () => void;
}

const CustomModelForm: React.FC<CustomModelFormProps> = ({ onSave, onCancel }) => {
  const [name, setName] = useState('');
  const [description, setDescription] = useState('');
  const [provider, setProvider] = useState<ModelProvider>('custom');
  const [apiEndpoint, setApiEndpoint] = useState('');
  const [contextLength, setContextLength] = useState(32000);

  const handleSubmit = (e: React.FormEvent) => {
    e.preventDefault();
    if (!name || !apiEndpoint) {
      alert('Please fill in all required fields');
      return;
    }
    onSave({
      name,
      description,
      provider,
      apiEndpoint,
      contextLength,
      enabled: true,
    });
  };

  return (
    <form onSubmit={handleSubmit} style={{ display: 'flex', flexDirection: 'column', gap: '16px' }}>
      <div>
        <label style={{ display: 'block', marginBottom: '6px', fontSize: '13px', color: '#cccccc' }}>
          Model Name *
        </label>
        <input
          type="text"
          value={name}
          onChange={(e) => setName(e.target.value)}
          className="form-input"
          placeholder="e.g., My Custom GPT-4"
          required
        />
      </div>
      <div>
        <label style={{ display: 'block', marginBottom: '6px', fontSize: '13px', color: '#cccccc' }}>
          Description
        </label>
        <input
          type="text"
          value={description}
          onChange={(e) => setDescription(e.target.value)}
          className="form-input"
          placeholder="Brief description of the model"
        />
      </div>
      <div>
        <label style={{ display: 'block', marginBottom: '6px', fontSize: '13px', color: '#cccccc' }}>
          Provider
        </label>
        <select
          value={provider}
          onChange={(e) => setProvider(e.target.value as ModelProvider)}
          className="form-input"
        >
          <option value="custom">Custom</option>
          <option value="openai">OpenAI</option>
          <option value="anthropic">Anthropic</option>
          <option value="gemini">Google Gemini</option>
        </select>
      </div>
      <div>
        <label style={{ display: 'block', marginBottom: '6px', fontSize: '13px', color: '#cccccc' }}>
          API Endpoint *
        </label>
        <input
          type="text"
          value={apiEndpoint}
          onChange={(e) => setApiEndpoint(e.target.value)}
          className="form-input"
          placeholder="https://api.example.com/v1/chat/completions"
          required
        />
      </div>
      <div>
        <label style={{ display: 'block', marginBottom: '6px', fontSize: '13px', color: '#cccccc' }}>
          Context Length (tokens)
        </label>
        <input
          type="number"
          value={contextLength}
          onChange={(e) => setContextLength(parseInt(e.target.value) || 32000)}
          className="form-input"
          min="1000"
          max="1000000"
        />
      </div>
      <div style={{ display: 'flex', gap: '12px', justifyContent: 'flex-end', marginTop: '8px' }}>
        <button type="button" className="setting-btn" onClick={onCancel}>
          Cancel
        </button>
        <button type="submit" className="setting-btn" style={{ background: '#007acc', borderColor: '#007acc', color: '#ffffff' }}>
          Add Model
        </button>
      </div>
    </form>
  );
};

