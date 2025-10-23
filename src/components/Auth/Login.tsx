import React, { useState } from 'react';
import { useAuth } from '../../contexts/AuthContext';
import './Login.css';

type AuthMode = 'signin' | 'signup' | 'pin' | 'biometric' | 'faceid' | 'forgot' | 'reset' | 'verify-2fa';

const Login: React.FC = () => {
  const {
    login,
    loginWithPin,
    loginWithBiometric,
    loginWithFaceId,
    loginAsGuest,
    signUp,
    verifyTwoFactor,
    requestPasswordReset,
    resetPassword,
    authStatus,
  } = useAuth();

  const [mode, setMode] = useState<AuthMode>('signin');
  const [username, setUsername] = useState('');
  const [password, setPassword] = useState('');
  const [email, setEmail] = useState('');
  const [phone, setPhone] = useState('');
  const [confirmPassword, setConfirmPassword] = useState('');
  const [pin, setPin] = useState('');
  const [twoFactorCode, setTwoFactorCode] = useState('');
  const [resetToken, setResetToken] = useState('');
  const [error, setError] = useState('');
  const [success, setSuccess] = useState('');
  const [loading, setLoading] = useState(false);

  const handleCredentialsLogin = async (e: React.FormEvent) => {
    e.preventDefault();
    setError('');
    setLoading(true);

    try {
      const result = await login(username, password);
      if (!result) {
        setError('Invalid username or password');
      } else {
        // Check if 2FA is required
        setMode('verify-2fa');
        setSuccess('Please enter your 2FA code');
      }
    } catch (err) {
      setError('Login failed. Please try again.');
    } finally {
      setLoading(false);
    }
  };

  const handlePinLogin = async (e: React.FormEvent) => {
    e.preventDefault();
    setError('');
    setLoading(true);

    try {
      const result = await loginWithPin(pin);
      if (!result) {
        setError('Invalid PIN');
      }
    } catch (err) {
      setError('PIN login failed. Please try again.');
    } finally {
      setLoading(false);
    }
  };

  const handleBiometricLogin = async () => {
    setError('');
    setLoading(true);

    try {
      const result = await loginWithBiometric();
      if (!result) {
        setError('Biometric authentication failed');
      }
    } catch (err) {
      setError('Biometric authentication not available');
    } finally {
      setLoading(false);
    }
  };

  const handleFaceIdLogin = async () => {
    setError('');
    setLoading(true);

    try {
      const result = await loginWithFaceId();
      if (!result) {
        setError('Face ID authentication failed');
      }
    } catch (err) {
      setError('Face ID not available');
    } finally {
      setLoading(false);
    }
  };

  const handleSignUp = async (e: React.FormEvent) => {
    e.preventDefault();
    setError('');
    setSuccess('');
    setLoading(true);

    try {
      if (password !== confirmPassword) {
        setError('Passwords do not match');
        return;
      }

      if (password.length < 8) {
        setError('Password must be at least 8 characters');
        return;
      }

      const result = await signUp({
        username,
        email,
        phone,
        password,
        confirmPassword,
      });

      if (result) {
        setSuccess('Verification code sent! Please check your email/phone.');
        setMode('verify-2fa');
      } else {
        setError('Sign up failed. Please try again.');
      }
    } catch (err) {
      setError('Sign up failed. Please try again.');
    } finally {
      setLoading(false);
    }
  };

  const handleVerify2FA = async (e: React.FormEvent) => {
    e.preventDefault();
    setError('');
    setLoading(true);

    try {
      const result = await verifyTwoFactor(twoFactorCode);
      if (!result) {
        setError('Invalid verification code');
      }
    } catch (err) {
      setError('Verification failed. Please try again.');
    } finally {
      setLoading(false);
    }
  };

  const handleForgotPassword = async (e: React.FormEvent) => {
    e.preventDefault();
    setError('');
    setSuccess('');
    setLoading(true);

    try {
      const result = await requestPasswordReset(email || phone);
      if (result) {
        setSuccess('Password reset link sent! Check your email/phone.');
        setMode('reset');
      } else {
        setError('Failed to send reset link');
      }
    } catch (err) {
      setError('Failed to send reset link');
    } finally {
      setLoading(false);
    }
  };

  const handleResetPassword = async (e: React.FormEvent) => {
    e.preventDefault();
    setError('');
    setSuccess('');
    setLoading(true);

    try {
      if (password !== confirmPassword) {
        setError('Passwords do not match');
        return;
      }

      const result = await resetPassword(resetToken, password);
      if (result) {
        setSuccess('Password reset successful! You can now sign in.');
        setMode('signin');
      } else {
        setError('Failed to reset password');
      }
    } catch (err) {
      setError('Failed to reset password');
    } finally {
      setLoading(false);
    }
  };

  const handleGuestLogin = () => {
    loginAsGuest();
  };

  const getStatusColor = () => {
    switch (authStatus) {
      case 'signed-in':
        return '#10b981'; // Green
      case 'standby':
        return '#ffffff'; // White
      case 'signed-out':
        return '#ef4444'; // Red
      default:
        return '#6b7280'; // Gray
    }
  };

  const getStatusText = () => {
    switch (authStatus) {
      case 'signed-in':
        return 'Signed In';
      case 'standby':
        return 'Standby';
      case 'signed-out':
        return 'Signed Out';
      default:
        return 'Unknown';
    }
  };

  return (
    <div className="login-container">
      <div className="login-card">
        {/* Status Indicator */}
        <div className="status-indicator">
          <div
            className="status-dot"
            style={{ backgroundColor: getStatusColor() }}
          />
          <span className="status-text">{getStatusText()}</span>
        </div>

        <div className="login-header">
          <h1>NAVΛ Studio</h1>
          <p>Secure Authentication</p>
        </div>

        {error && <div className="error-message">{error}</div>}
        {success && <div className="success-message">{success}</div>}

        {/* Sign In Mode */}
        {mode === 'signin' && (
          <form onSubmit={handleCredentialsLogin} className="login-form">
            <div className="form-group">
              <label htmlFor="username">Username</label>
              <input
                type="text"
                id="username"
                value={username}
                onChange={(e) => setUsername(e.target.value)}
                placeholder="Enter your username"
                required
              />
            </div>

            <div className="form-group">
              <label htmlFor="password">Password</label>
              <input
                type="password"
                id="password"
                value={password}
                onChange={(e) => setPassword(e.target.value)}
                placeholder="Enter your password"
                required
              />
            </div>

            <button type="submit" className="btn-primary" disabled={loading}>
              {loading ? 'Signing In...' : 'Sign In'}
            </button>

            <div className="auth-links">
              <button
                type="button"
                className="link-button"
                onClick={() => setMode('forgot')}
              >
                Forgot Password?
              </button>
              <button
                type="button"
                className="link-button"
                onClick={() => setMode('signup')}
              >
                Create Account
              </button>
            </div>

            <div className="divider">
              <span>OR</span>
            </div>

            <div className="alternative-auth">
              <button
                type="button"
                className="btn-secondary"
                onClick={() => setMode('pin')}
              >
                🔢 Sign In with PIN
              </button>

              <button
                type="button"
                className="btn-secondary"
                onClick={handleBiometricLogin}
                disabled={loading}
              >
                👆 Biometric Login
              </button>

              <button
                type="button"
                className="btn-secondary"
                onClick={handleFaceIdLogin}
                disabled={loading}
              >
                😊 Face ID Login
              </button>

              <button
                type="button"
                className="btn-guest"
                onClick={handleGuestLogin}
              >
                Continue as Guest
              </button>
            </div>
          </form>
        )}

        {/* Sign Up Mode */}
        {mode === 'signup' && (
          <form onSubmit={handleSignUp} className="login-form">
            <h2>Create Account</h2>

            <div className="form-group">
              <label htmlFor="signup-username">Username</label>
              <input
                type="text"
                id="signup-username"
                value={username}
                onChange={(e) => setUsername(e.target.value)}
                placeholder="Choose a username"
                required
              />
            </div>

            <div className="form-group">
              <label htmlFor="signup-email">Email</label>
              <input
                type="email"
                id="signup-email"
                value={email}
                onChange={(e) => setEmail(e.target.value)}
                placeholder="your.email@example.com"
                required
              />
            </div>

            <div className="form-group">
              <label htmlFor="signup-phone">Phone (Optional)</label>
              <input
                type="tel"
                id="signup-phone"
                value={phone}
                onChange={(e) => setPhone(e.target.value)}
                placeholder="+1 (555) 123-4567"
              />
            </div>

            <div className="form-group">
              <label htmlFor="signup-password">Password</label>
              <input
                type="password"
                id="signup-password"
                value={password}
                onChange={(e) => setPassword(e.target.value)}
                placeholder="At least 8 characters"
                required
              />
            </div>

            <div className="form-group">
              <label htmlFor="signup-confirm">Confirm Password</label>
              <input
                type="password"
                id="signup-confirm"
                value={confirmPassword}
                onChange={(e) => setConfirmPassword(e.target.value)}
                placeholder="Re-enter password"
                required
              />
            </div>

            <button type="submit" className="btn-primary" disabled={loading}>
              {loading ? 'Creating Account...' : 'Sign Up'}
            </button>

            <div className="auth-links">
              <button
                type="button"
                className="link-button"
                onClick={() => setMode('signin')}
              >
                Already have an account? Sign In
              </button>
            </div>
          </form>
        )}

        {/* PIN Mode */}
        {mode === 'pin' && (
          <form onSubmit={handlePinLogin} className="login-form">
            <h2>Enter PIN</h2>

            <div className="form-group">
              <label htmlFor="pin">4-Digit PIN</label>
              <input
                type="password"
                id="pin"
                value={pin}
                onChange={(e) => setPin(e.target.value)}
                placeholder="••••"
                maxLength={4}
                pattern="[0-9]{4}"
                required
              />
            </div>

            <button type="submit" className="btn-primary" disabled={loading}>
              {loading ? 'Verifying...' : 'Verify PIN'}
            </button>

            <div className="auth-links">
              <button
                type="button"
                className="link-button"
                onClick={() => setMode('signin')}
              >
                Back to Sign In
              </button>
            </div>
          </form>
        )}

        {/* 2FA Verification Mode */}
        {mode === 'verify-2fa' && (
          <form onSubmit={handleVerify2FA} className="login-form">
            <h2>Two-Factor Authentication</h2>
            <p className="info-text">
              Enter the 6-digit code sent to your email or phone
            </p>

            <div className="form-group">
              <label htmlFor="2fa-code">Verification Code</label>
              <input
                type="text"
                id="2fa-code"
                value={twoFactorCode}
                onChange={(e) => setTwoFactorCode(e.target.value)}
                placeholder="123456"
                maxLength={6}
                pattern="[0-9]{6}"
                required
              />
            </div>

            <button type="submit" className="btn-primary" disabled={loading}>
              {loading ? 'Verifying...' : 'Verify Code'}
            </button>

            <div className="auth-links">
              <button
                type="button"
                className="link-button"
                onClick={() => setMode('signin')}
              >
                Back to Sign In
              </button>
            </div>
          </form>
        )}

        {/* Forgot Password Mode */}
        {mode === 'forgot' && (
          <form onSubmit={handleForgotPassword} className="login-form">
            <h2>Forgot Password</h2>
            <p className="info-text">
              Enter your email or phone number to receive a reset link
            </p>

            <div className="form-group">
              <label htmlFor="forgot-email">Email or Phone</label>
              <input
                type="text"
                id="forgot-email"
                value={email || phone}
                onChange={(e) => {
                  const value = e.target.value;
                  if (value.includes('@')) {
                    setEmail(value);
                    setPhone('');
                  } else {
                    setPhone(value);
                    setEmail('');
                  }
                }}
                placeholder="email@example.com or +1234567890"
                required
              />
            </div>

            <button type="submit" className="btn-primary" disabled={loading}>
              {loading ? 'Sending...' : 'Send Reset Link'}
            </button>

            <div className="auth-links">
              <button
                type="button"
                className="link-button"
                onClick={() => setMode('signin')}
              >
                Back to Sign In
              </button>
            </div>
          </form>
        )}

        {/* Reset Password Mode */}
        {mode === 'reset' && (
          <form onSubmit={handleResetPassword} className="login-form">
            <h2>Reset Password</h2>

            <div className="form-group">
              <label htmlFor="reset-token">Reset Token</label>
              <input
                type="text"
                id="reset-token"
                value={resetToken}
                onChange={(e) => setResetToken(e.target.value)}
                placeholder="Enter token from email/SMS"
                required
              />
            </div>

            <div className="form-group">
              <label htmlFor="reset-password">New Password</label>
              <input
                type="password"
                id="reset-password"
                value={password}
                onChange={(e) => setPassword(e.target.value)}
                placeholder="At least 8 characters"
                required
              />
            </div>

            <div className="form-group">
              <label htmlFor="reset-confirm">Confirm New Password</label>
              <input
                type="password"
                id="reset-confirm"
                value={confirmPassword}
                onChange={(e) => setConfirmPassword(e.target.value)}
                placeholder="Re-enter password"
                required
              />
            </div>

            <button type="submit" className="btn-primary" disabled={loading}>
              {loading ? 'Resetting...' : 'Reset Password'}
            </button>

            <div className="auth-links">
              <button
                type="button"
                className="link-button"
                onClick={() => setMode('signin')}
              >
                Back to Sign In
              </button>
            </div>
          </form>
        )}
      </div>
    </div>
  );
};

export default Login;