/**
 * Login Modal Component
 * Handles user authentication and account creation
 */

import React, { useState } from 'react';
import { X, Mail, Lock, Github, Chrome, Building2 } from 'lucide-react';
import { authService, type LoginCredentials } from '../../services/auth-service';
import './LoginModal.css';

interface LoginModalProps {
  isOpen: boolean;
  onClose: () => void;
  onLoginSuccess: () => void;
}

export const LoginModal: React.FC<LoginModalProps> = ({
  isOpen,
  onClose,
  onLoginSuccess,
}) => {
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [showPassword, setShowPassword] = useState(false);

  if (!isOpen) return null;

  const handleEmailLogin = async (e: React.FormEvent) => {
    e.preventDefault();
    setError(null);
    setIsLoading(true);

    try {
      const credentials: LoginCredentials = {
        email,
        password: password || undefined,
        provider: 'email',
      };
      await authService.login(credentials);
      onLoginSuccess();
      onClose();
      // Reset form
      setEmail('');
      setPassword('');
    } catch (err: any) {
      setError(err.message || 'Login failed. Please try again.');
    } finally {
      setIsLoading(false);
    }
  };

  const handleOAuthLogin = async (provider: 'google' | 'github' | 'microsoft') => {
    setError(null);
    setIsLoading(true);

    try {
      // In production, this would redirect to OAuth provider
      // For now, we'll use a demo email
      const demoEmail = `${provider}@example.com`;
      const credentials: LoginCredentials = {
        email: demoEmail,
        provider,
      };
      await authService.login(credentials);
      onLoginSuccess();
      onClose();
    } catch (err: any) {
      setError(err.message || `${provider} login failed. Please try again.`);
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <div className="login-modal-overlay" onClick={onClose}>
      <div className="login-modal" onClick={(e) => e.stopPropagation()}>
        <div className="login-modal-header">
          <h2>Sign in to NAVΛ Studio</h2>
          <button className="login-modal-close" onClick={onClose}>
            <X size={20} />
          </button>
        </div>

        <div className="login-modal-content">
          {error && (
            <div className="login-error">
              {error}
            </div>
          )}

          {/* OAuth Providers */}
          <div className="oauth-providers">
            <button
              className="oauth-btn google"
              onClick={() => handleOAuthLogin('google')}
              disabled={isLoading}
            >
              <Chrome size={18} />
              <span>Continue with Google</span>
            </button>
            <button
              className="oauth-btn github"
              onClick={() => handleOAuthLogin('github')}
              disabled={isLoading}
            >
              <Github size={18} />
              <span>Continue with GitHub</span>
            </button>
            <button
              className="oauth-btn microsoft"
              onClick={() => handleOAuthLogin('microsoft')}
              disabled={isLoading}
            >
              <Building2 size={18} />
              <span>Continue with Microsoft</span>
            </button>
          </div>

          <div className="login-divider">
            <span>or</span>
          </div>

          {/* Email/Password Form */}
          <form onSubmit={handleEmailLogin} className="login-form">
            <div className="form-group">
              <label htmlFor="email">
                <Mail size={16} />
                Email
              </label>
              <input
                id="email"
                type="email"
                value={email}
                onChange={(e) => setEmail(e.target.value)}
                placeholder="you@example.com"
                required
                disabled={isLoading}
              />
            </div>

            <div className="form-group">
              <label htmlFor="password">
                <Lock size={16} />
                Password (Optional)
              </label>
              <div className="password-input-wrapper">
                <input
                  id="password"
                  type={showPassword ? 'text' : 'password'}
                  value={password}
                  onChange={(e) => setPassword(e.target.value)}
                  placeholder="Enter password (optional for demo)"
                  disabled={isLoading}
                />
                <button
                  type="button"
                  className="password-toggle"
                  onClick={() => setShowPassword(!showPassword)}
                >
                  {showPassword ? '👁️' : '👁️‍🗨️'}
                </button>
              </div>
              <p className="form-hint">
                For demo purposes, password is optional. Just enter your email to continue.
              </p>
            </div>

            <button
              type="submit"
              className="login-submit-btn"
              disabled={isLoading || !email}
            >
              {isLoading ? 'Signing in...' : 'Sign In'}
            </button>
          </form>

          <div className="login-footer">
            <p>
              By signing in, you agree to our Terms of Service and Privacy Policy.
            </p>
            <p className="demo-note">
              💡 Demo Mode: No real authentication required. Enter any email to continue.
            </p>
          </div>
        </div>
      </div>
    </div>
  );
};

