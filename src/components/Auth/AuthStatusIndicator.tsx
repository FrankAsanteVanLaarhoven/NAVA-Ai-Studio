import React from 'react';
import { useAuth } from '../../contexts/AuthContext';
import './AuthStatusIndicator.css';

interface AuthStatusIndicatorProps {
  showText?: boolean;
  size?: 'small' | 'medium' | 'large';
  position?: 'top-left' | 'top-right' | 'bottom-left' | 'bottom-right' | 'inline';
}

const AuthStatusIndicator: React.FC<AuthStatusIndicatorProps> = ({
  showText = true,
  size = 'medium',
  position = 'inline',
}) => {
  const { authStatus, user, logout, setStandbyMode } = useAuth();

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
        return user?.isGuest ? 'Guest Mode' : `Signed In as ${user?.username}`;
      case 'standby':
        return 'Standby Mode';
      case 'signed-out':
        return 'Signed Out';
      default:
        return 'Unknown';
    }
  };

  const handleStatusClick = () => {
    if (authStatus === 'signed-in') {
      const action = window.confirm(
        'Choose an action:\nOK = Standby Mode\nCancel = Sign Out'
      );
      if (action) {
        setStandbyMode();
      } else {
        logout();
      }
    }
  };

  return (
    <div className={`auth-status-indicator ${position} ${size}`}>
      <div
        className="status-dot-container"
        onClick={handleStatusClick}
        title={getStatusText()}
      >
        <div
          className="status-dot"
          style={{ backgroundColor: getStatusColor() }}
        />
        {authStatus === 'signed-in' && (
          <div className="status-pulse" style={{ borderColor: getStatusColor() }} />
        )}
      </div>
      {showText && (
        <div className="status-info">
          <span className="status-text">{getStatusText()}</span>
          {user && authStatus === 'signed-in' && (
            <div className="status-actions">
              <button
                className="status-action-btn"
                onClick={setStandbyMode}
                title="Standby Mode"
              >
                ⏸️
              </button>
              <button
                className="status-action-btn"
                onClick={logout}
                title="Sign Out"
              >
                🚪
              </button>
            </div>
          )}
        </div>
      )}
    </div>
  );
};

export default AuthStatusIndicator;
