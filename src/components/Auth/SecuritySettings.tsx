import React, { useState } from 'react';
import { useAuth } from '../../contexts/AuthContext';
import './SecuritySettings.css';

const SecuritySettings: React.FC = () => {
  const {
    user,
    changePassword,
    enableBiometric,
    enableFaceId,
    enableTwoFactor,
  } = useAuth();

  const [currentPassword, setCurrentPassword] = useState('');
  const [newPassword, setNewPassword] = useState('');
  const [confirmPassword, setConfirmPassword] = useState('');
  const [twoFactorMethod, setTwoFactorMethod] = useState<'email' | 'sms'>('email');
  const [message, setMessage] = useState('');
  const [error, setError] = useState('');
  const [loading, setLoading] = useState(false);

  const handleChangePassword = async (e: React.FormEvent) => {
    e.preventDefault();
    setError('');
    setMessage('');
    setLoading(true);

    try {
      if (newPassword !== confirmPassword) {
        setError('Passwords do not match');
        return;
      }

      if (newPassword.length < 8) {
        setError('Password must be at least 8 characters');
        return;
      }

      const result = await changePassword(currentPassword, newPassword);
      if (result) {
        setMessage('Password changed successfully!');
        setCurrentPassword('');
        setNewPassword('');
        setConfirmPassword('');
      } else {
        setError('Failed to change password');
      }
    } catch (err) {
      setError('An error occurred');
    } finally {
      setLoading(false);
    }
  };

  const handleEnableBiometric = async () => {
    setError('');
    setMessage('');
    setLoading(true);

    try {
      const result = await enableBiometric();
      if (result) {
        setMessage('Biometric authentication enabled!');
      } else {
        setError('Failed to enable biometric authentication');
      }
    } catch (err) {
      setError('Biometric authentication not supported on this device');
    } finally {
      setLoading(false);
    }
  };

  const handleEnableFaceId = async () => {
    setError('');
    setMessage('');
    setLoading(true);

    try {
      const result = await enableFaceId();
      if (result) {
        setMessage('Face ID enabled!');
      } else {
        setError('Failed to enable Face ID');
      }
    } catch (err) {
      setError('Face ID not supported on this device');
    } finally {
      setLoading(false);
    }
  };

  const handleEnableTwoFactor = async () => {
    setError('');
    setMessage('');
    setLoading(true);

    try {
      const result = await enableTwoFactor(twoFactorMethod);
      if (result) {
        setMessage(`Two-factor authentication enabled via ${twoFactorMethod}!`);
      } else {
        setError('Failed to enable two-factor authentication');
      }
    } catch (err) {
      setError('An error occurred');
    } finally {
      setLoading(false);
    }
  };

  if (!user) {
    return (
      <div className="security-settings">
        <p>Please sign in to access security settings.</p>
      </div>
    );
  }

  return (
    <div className="security-settings">
      <div className="security-header">
        <h2>🔒 Security Settings</h2>
        <p>Manage your account security and authentication methods</p>
      </div>

      {message && <div className="success-message">{message}</div>}
      {error && <div className="error-message">{error}</div>}

      {/* Account Information */}
      <div className="security-section">
        <h3>Account Information</h3>
        <div className="info-grid">
          <div className="info-item">
            <span className="info-label">Username:</span>
            <span className="info-value">{user.username}</span>
          </div>
          <div className="info-item">
            <span className="info-label">Email:</span>
            <span className="info-value">{user.email || 'Not set'}</span>
          </div>
          <div className="info-item">
            <span className="info-label">Phone:</span>
            <span className="info-value">{user.phone || 'Not set'}</span>
          </div>
          <div className="info-item">
            <span className="info-label">Last Login:</span>
            <span className="info-value">
              {user.lastLogin ? new Date(user.lastLogin).toLocaleString() : 'N/A'}
            </span>
          </div>
        </div>
      </div>

      {/* Change Password */}
      <div className="security-section">
        <h3>Change Password</h3>
        <form onSubmit={handleChangePassword} className="security-form">
          <div className="form-group">
            <label htmlFor="current-password">Current Password</label>
            <input
              type="password"
              id="current-password"
              value={currentPassword}
              onChange={(e) => setCurrentPassword(e.target.value)}
              placeholder="Enter current password"
              required
            />
          </div>

          <div className="form-group">
            <label htmlFor="new-password">New Password</label>
            <input
              type="password"
              id="new-password"
              value={newPassword}
              onChange={(e) => setNewPassword(e.target.value)}
              placeholder="At least 8 characters"
              required
            />
          </div>

          <div className="form-group">
            <label htmlFor="confirm-password">Confirm New Password</label>
            <input
              type="password"
              id="confirm-password"
              value={confirmPassword}
              onChange={(e) => setConfirmPassword(e.target.value)}
              placeholder="Re-enter new password"
              required
            />
          </div>

          <button type="submit" className="btn-primary" disabled={loading}>
            {loading ? 'Changing...' : 'Change Password'}
          </button>
        </form>
      </div>

      {/* Authentication Methods */}
      <div className="security-section">
        <h3>Authentication Methods</h3>

        <div className="auth-method">
          <div className="auth-method-info">
            <div className="auth-method-icon">👆</div>
            <div>
              <h4>Biometric Authentication</h4>
              <p>Use fingerprint or other biometric data to sign in</p>
            </div>
          </div>
          <div className="auth-method-status">
            {user.biometricEnabled ? (
              <span className="status-badge enabled">Enabled</span>
            ) : (
              <button
                className="btn-secondary"
                onClick={handleEnableBiometric}
                disabled={loading}
              >
                Enable
              </button>
            )}
          </div>
        </div>

        <div className="auth-method">
          <div className="auth-method-info">
            <div className="auth-method-icon">😊</div>
            <div>
              <h4>Face ID</h4>
              <p>Use facial recognition to sign in</p>
            </div>
          </div>
          <div className="auth-method-status">
            {user.faceIdEnabled ? (
              <span className="status-badge enabled">Enabled</span>
            ) : (
              <button
                className="btn-secondary"
                onClick={handleEnableFaceId}
                disabled={loading}
              >
                Enable
              </button>
            )}
          </div>
        </div>

        <div className="auth-method">
          <div className="auth-method-info">
            <div className="auth-method-icon">🔐</div>
            <div>
              <h4>Two-Factor Authentication</h4>
              <p>Add an extra layer of security to your account</p>
            </div>
          </div>
          <div className="auth-method-status">
            {user.twoFactorEnabled ? (
              <span className="status-badge enabled">Enabled</span>
            ) : (
              <div className="two-factor-setup">
                <select
                  value={twoFactorMethod}
                  onChange={(e) => setTwoFactorMethod(e.target.value as 'email' | 'sms')}
                  className="method-select"
                >
                  <option value="email">Email</option>
                  <option value="sms">SMS</option>
                </select>
                <button
                  className="btn-secondary"
                  onClick={handleEnableTwoFactor}
                  disabled={loading}
                >
                  Enable
                </button>
              </div>
            )}
          </div>
        </div>
      </div>

      {/* Security Tips */}
      <div className="security-section security-tips">
        <h3>🛡️ Security Tips</h3>
        <ul>
          <li>Use a strong, unique password for your account</li>
          <li>Enable two-factor authentication for extra security</li>
          <li>Never share your password or PIN with anyone</li>
          <li>Keep your email and phone number up to date</li>
          <li>Review your account activity regularly</li>
          <li>Sign out when using shared devices</li>
        </ul>
      </div>
    </div>
  );
};

export default SecuritySettings;
