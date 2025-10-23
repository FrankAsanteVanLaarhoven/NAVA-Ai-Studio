// Example: How to integrate the authentication system into your App.tsx

import React from 'react';
import { AuthProvider, useAuth } from './contexts/AuthContext';
import Login from './components/Auth/Login';
import AuthStatusIndicator from './components/Auth/AuthStatusIndicator';
import SecuritySettings from './components/Auth/SecuritySettings';

// Example: Protected component that requires authentication
function ProtectedApp() {
  const { user, authStatus, isAuthenticated, logout, setStandbyMode } = useAuth();

  // Show login screen if not authenticated
  if (!isAuthenticated) {
    return <Login />;
  }

  // Show standby overlay if in standby mode
  if (authStatus === 'standby') {
    return (
      <div className="standby-overlay">
        <div className="standby-content">
          <AuthStatusIndicator size="large" showText={true} />
          <h2>Session Locked</h2>
          <p>Click the status indicator to resume or sign out</p>
        </div>
      </div>
    );
  }

  return (
    <div className="app">
      {/* Header with status indicator */}
      <header className="app-header">
        <h1>NAVΛ Studio</h1>
        <div className="header-actions">
          <AuthStatusIndicator position="inline" size="medium" showText={true} />
          <button onClick={setStandbyMode}>Lock</button>
          <button onClick={logout}>Sign Out</button>
        </div>
      </header>

      {/* Main content */}
      <main className="app-main">
        <h2>Welcome, {user?.username}!</h2>
        
        {/* Show different content based on user type */}
        {user?.isGuest ? (
          <div className="guest-notice">
            <p>⚠️ You're in guest mode with limited access</p>
            <button onClick={logout}>Sign in for full access</button>
          </div>
        ) : (
          <div className="user-content">
            <p>Email: {user?.email}</p>
            <p>Permissions: {user?.permissions.join(', ')}</p>
            <p>Last Login: {user?.lastLogin?.toLocaleString()}</p>
            
            {/* Security status */}
            <div className="security-status">
              <h3>Security Features</h3>
              <ul>
                <li>Biometric: {user?.biometricEnabled ? '✅ Enabled' : '❌ Disabled'}</li>
                <li>Face ID: {user?.faceIdEnabled ? '✅ Enabled' : '❌ Disabled'}</li>
                <li>2FA: {user?.twoFactorEnabled ? '✅ Enabled' : '❌ Disabled'}</li>
              </ul>
            </div>
          </div>
        )}

        {/* Security Settings Section */}
        <section className="settings-section">
          <SecuritySettings />
        </section>
      </main>

      {/* Footer with status indicator */}
      <footer className="app-footer">
        <AuthStatusIndicator position="inline" size="small" showText={false} />
        <p>© 2024 NAVΛ Studio</p>
      </footer>
    </div>
  );
}

// Main App component wrapped with AuthProvider
function App() {
  return (
    <AuthProvider>
      <ProtectedApp />
    </AuthProvider>
  );
}

export default App;

// Example CSS for the standby overlay
/*
.standby-overlay {
  position: fixed;
  top: 0;
  left: 0;
  right: 0;
  bottom: 0;
  background: rgba(0, 0, 0, 0.9);
  backdrop-filter: blur(10px);
  display: flex;
  align-items: center;
  justify-content: center;
  z-index: 9999;
}

.standby-content {
  text-align: center;
  color: white;
  padding: 40px;
  background: rgba(255, 255, 255, 0.1);
  border-radius: 20px;
  border: 1px solid rgba(255, 255, 255, 0.2);
}

.standby-content h2 {
  margin: 20px 0 10px;
  font-size: 32px;
}

.standby-content p {
  color: rgba(255, 255, 255, 0.8);
  font-size: 16px;
}

.guest-notice {
  padding: 20px;
  background: #fef3c7;
  border: 2px solid #f59e0b;
  border-radius: 12px;
  margin: 20px 0;
}

.guest-notice p {
  margin: 0 0 10px;
  color: #92400e;
  font-weight: 600;
}

.security-status {
  margin: 20px 0;
  padding: 20px;
  background: #f9fafb;
  border-radius: 12px;
}

.security-status h3 {
  margin: 0 0 15px;
  color: #1f2937;
}

.security-status ul {
  list-style: none;
  padding: 0;
  margin: 0;
}

.security-status li {
  padding: 8px 0;
  color: #374151;
  font-size: 15px;
}
*/
