/**
 * NAVΛ AI Welcome Screen
 * Similar to AbacusAI's welcome interface with feature cards, quick start, and help
 */

import React, { useState } from 'react';
import {
  FileText,
  FolderOpen,
  Code,
  GitBranch,
  Link2,
  Star,
  Lightbulb,
  FileCode,
  Plug,
  Terminal,
  MessageSquare,
  Mic,
  Play,
  BookOpen,
  Settings,
  ExternalLink,
} from 'lucide-react';
import './WelcomeScreen.css';

interface WelcomeScreenProps {
  onNewFile?: () => void;
  onOpenFile?: () => void;
  onStartAgent?: () => void;
  onCloneRepo?: () => void;
  onConnect?: () => void;
  onClose?: () => void;
}

export const WelcomeScreen: React.FC<WelcomeScreenProps> = ({
  onNewFile,
  onOpenFile,
  onStartAgent,
  onCloneRepo,
  onConnect,
  onClose,
}) => {
  const [installCommand, setInstallCommand] = useState('npm install -g @nava-studio/cli');
  const [launchCommand, setLaunchCommand] = useState('nava start');

  const handleInstall = () => {
    // Copy to clipboard
    navigator.clipboard.writeText(installCommand);
    alert('Installation command copied to clipboard!');
  };

  const handleLaunch = () => {
    navigator.clipboard.writeText(launchCommand);
    alert('Launch command copied to clipboard!');
  };

  return (
    <div className="welcome-screen">
      {/* Header */}
      <div className="welcome-header">
        <h1 className="welcome-title">
          Code with <span className="highlight">NAVΛ AI</span>
        </h1>
        <p className="welcome-subtitle">Your intelligent coding assistant powered by Van Laarhoven Navigation Calculus</p>
      </div>

      {/* Feature Cards */}
      <div className="feature-cards">
        <div className="feature-card cli-card">
          <div className="feature-icon">
            <Terminal size={32} />
          </div>
          <h3 className="feature-title">NAVΛ CLI</h3>
          <p className="feature-description">Top-ranked CLI AI Coding Agent with VNC support</p>
        </div>

        <div className="feature-card chat-card">
          <div className="feature-icon">
            <MessageSquare size={32} />
          </div>
          <h3 className="feature-title">NAVΛ Chat Mode</h3>
          <p className="feature-description">Chat with Gemini 2.0, Claude 3.5 Sonnet, GPT-4o, and more</p>
        </div>

        <div className="feature-card listener-card">
          <div className="feature-icon">
            <Mic size={32} />
          </div>
          <h3 className="feature-title">Voice Assistant</h3>
          <p className="feature-description">Real-time answers and suggestions in meetings and tasks</p>
        </div>
      </div>

      {/* Installation Instructions */}
      <div className="installation-section">
        <p className="installation-text">
          Use{' '}
          <input
            type="text"
            className="command-input"
            value={installCommand}
            onChange={(e) => setInstallCommand(e.target.value)}
            onClick={(e) => e.currentTarget.select()}
          />{' '}
          to install NAVΛ Studio CLI in your terminal and run{' '}
          <input
            type="text"
            className="command-input"
            value={launchCommand}
            onChange={(e) => setLaunchCommand(e.target.value)}
            onClick={(e) => e.currentTarget.select()}
          />{' '}
          to launch it.
        </p>
        <div className="command-buttons">
          <button className="copy-btn" onClick={handleInstall}>
            Copy Install
          </button>
          <button className="copy-btn" onClick={handleLaunch}>
            Copy Launch
          </button>
        </div>
      </div>

      {/* Divider */}
      <div className="welcome-divider"></div>

      {/* Main Content Grid */}
      <div className="welcome-content-grid">
        {/* Start Column */}
        <div className="welcome-column">
          <h2 className="column-title">Start</h2>
          <div className="action-list">
            <button className="action-item" onClick={onNewFile}>
              <FileText size={18} />
              <span>New File...</span>
            </button>
            <button className="action-item" onClick={onOpenFile}>
              <FolderOpen size={18} />
              <span>Open...</span>
            </button>
            <button className="action-item" onClick={onStartAgent}>
              <Code size={18} />
              <span>Start Coding Agent</span>
            </button>
            <button className="action-item" onClick={onCloneRepo}>
              <GitBranch size={18} />
              <span>Clone Git Repository...</span>
            </button>
            <button className="action-item" onClick={onConnect}>
              <Link2 size={18} />
              <span>Connect to...</span>
            </button>
          </div>
        </div>

        {/* Walkthroughs Column */}
        <div className="welcome-column">
          <h2 className="column-title">Walkthroughs</h2>
          <div className="walkthrough-list">
            <a href="#features" className="walkthrough-item">
              <Star size={18} />
              <div className="walkthrough-content">
                <span className="walkthrough-title">NAVΛ AI Features</span>
                <span className="walkthrough-description">Get an overview of NAVΛ AI's features.</span>
              </div>
            </a>
            <a href="#advanced" className="walkthrough-item">
              <Star size={18} />
              <div className="walkthrough-content">
                <span className="walkthrough-title">NAVΛ AI Advanced Features</span>
                <span className="walkthrough-description">Get an overview of NAVΛ AI's advanced features.</span>
              </div>
            </a>
            <a href="#getting-started" className="walkthrough-item">
              <Star size={18} />
              <div className="walkthrough-content">
                <span className="walkthrough-title">Get started with NAVΛ AI</span>
                <span className="walkthrough-description">Customize your editor, learn the basics, and start coding.</span>
              </div>
            </a>
            <a href="#fundamentals" className="walkthrough-item">
              <Lightbulb size={18} />
              <div className="walkthrough-content">
                <span className="walkthrough-title">Learn the Fundamentals</span>
                <span className="walkthrough-description">Master Van Laarhoven Navigation Calculus (VNC).</span>
              </div>
            </a>
            <a href="#python" className="walkthrough-item">
              <FileCode size={18} />
              <div className="walkthrough-content">
                <span className="walkthrough-title">Get Started with Python Development</span>
                <span className="walkthrough-description updated">Updated</span>
              </div>
            </a>
          </div>
        </div>

        {/* Help Column */}
        <div className="welcome-column">
          <h2 className="column-title">Help</h2>
          <div className="help-list">
            <a href="#agent-rules" className="help-item">
              <Settings size={18} />
              <div className="help-content">
                <div className="help-header">
                  <span className="help-title">Agent Rules</span>
                  <ExternalLink size={14} />
                </div>
                <span className="help-description">Personalize your agent.</span>
              </div>
            </a>
            <a href="#mcp" className="help-item">
              <Plug size={18} />
              <div className="help-content">
                <div className="help-header">
                  <span className="help-title">MCP</span>
                  <ExternalLink size={14} />
                </div>
                <span className="help-description">Unlock agent superpowers.</span>
              </div>
            </a>
            <a href="#documentation" className="help-item">
              <BookOpen size={18} />
              <div className="help-content">
                <div className="help-header">
                  <span className="help-title">Documentation</span>
                  <ExternalLink size={14} />
                </div>
                <span className="help-description">Complete NAVΛ Studio IDE guide.</span>
              </div>
            </a>
          </div>
        </div>
      </div>

      {/* Close Button */}
      {onClose && (
        <button className="welcome-close-btn" onClick={onClose}>
          Get Started
        </button>
      )}
    </div>
  );
};

