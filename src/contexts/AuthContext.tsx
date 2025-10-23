import React, { createContext, useContext, useState, useEffect, ReactNode } from 'react';

export type AuthStatus = 'signed-out' | 'standby' | 'signed-in';

export interface User {
  id: string;
  username: string;
  email: string;
  phone?: string;
  isGuest: boolean;
  permissions: string[];
  biometricEnabled: boolean;
  faceIdEnabled: boolean;
  twoFactorEnabled: boolean;
  lastLogin?: Date;
}

export interface AuthContextType {
  user: User | null;
  authStatus: AuthStatus;
  isAuthenticated: boolean;
  login: (username: string, password: string) => Promise<boolean>;
  loginWithPin: (pin: string) => Promise<boolean>;
  loginWithBiometric: () => Promise<boolean>;
  loginWithFaceId: () => Promise<boolean>;
  loginAsGuest: () => void;
  logout: () => void;
  signUp: (userData: SignUpData) => Promise<boolean>;
  verifyTwoFactor: (code: string) => Promise<boolean>;
  requestPasswordReset: (emailOrPhone: string) => Promise<boolean>;
  resetPassword: (token: string, newPassword: string) => Promise<boolean>;
  changePassword: (oldPassword: string, newPassword: string) => Promise<boolean>;
  enableBiometric: () => Promise<boolean>;
  enableFaceId: () => Promise<boolean>;
  enableTwoFactor: (method: 'email' | 'sms') => Promise<boolean>;
  setStandbyMode: () => void;
}

export interface SignUpData {
  username: string;
  email: string;
  phone?: string;
  password: string;
  confirmPassword: string;
}

const AuthContext = createContext<AuthContextType | undefined>(undefined);

export const useAuth = () => {
  const context = useContext(AuthContext);
  if (!context) {
    throw new Error('useAuth must be used within an AuthProvider');
  }
  return context;
};

interface AuthProviderProps {
  children: ReactNode;
}

export const AuthProvider: React.FC<AuthProviderProps> = ({ children }) => {
  const [user, setUser] = useState<User | null>(null);
  const [authStatus, setAuthStatus] = useState<AuthStatus>('signed-out');
  const [pendingTwoFactor, setPendingTwoFactor] = useState(false);

  useEffect(() => {
    const storedUser = localStorage.getItem('user');
    const storedStatus = localStorage.getItem('authStatus') as AuthStatus;
    if (storedUser && storedStatus) {
      setUser(JSON.parse(storedUser));
      setAuthStatus(storedStatus);
    }
  }, []);

  const saveUserSession = (userData: User, status: AuthStatus) => {
    localStorage.setItem('user', JSON.stringify(userData));
    localStorage.setItem('authStatus', status);
    setUser(userData);
    setAuthStatus(status);
  };

  const login = async (username: string, password: string): Promise<boolean> => {
    try {
      // Simulate API call
      await new Promise(resolve => setTimeout(resolve, 1000));

      // Mock validation
      if (username === 'admin' && password === 'admin123') {
        const userData: User = {
          id: '1',
          username,
          email: 'admin@navlambda.com',
          isGuest: false,
          permissions: ['read', 'write', 'admin'],
          biometricEnabled: false,
          faceIdEnabled: false,
          twoFactorEnabled: true,
          lastLogin: new Date(),
        };

        if (userData.twoFactorEnabled) {
          setPendingTwoFactor(true);
          setUser(userData);
          return true;
        }

        saveUserSession(userData, 'signed-in');
        return true;
      }
      return false;
    } catch (error) {
      console.error('Login error:', error);
      return false;
    }
  };

  const loginWithPin = async (pin: string): Promise<boolean> => {
    try {
      await new Promise(resolve => setTimeout(resolve, 500));

      if (pin === '1234') {
        const userData: User = {
          id: '2',
          username: 'pin_user',
          email: 'user@navlambda.com',
          isGuest: false,
          permissions: ['read', 'write'],
          biometricEnabled: false,
          faceIdEnabled: false,
          twoFactorEnabled: false,
          lastLogin: new Date(),
        };

        saveUserSession(userData, 'signed-in');
        return true;
      }
      return false;
    } catch (error) {
      console.error('PIN login error:', error);
      return false;
    }
  };

  const loginWithBiometric = async (): Promise<boolean> => {
    try {
      // Check if Web Authentication API is available
      if (!window.PublicKeyCredential) {
        throw new Error('Biometric authentication not supported');
      }

      // Simulate biometric authentication
      await new Promise(resolve => setTimeout(resolve, 1500));

      const userData: User = {
        id: '3',
        username: 'biometric_user',
        email: 'biometric@navlambda.com',
        isGuest: false,
        permissions: ['read', 'write'],
        biometricEnabled: true,
        faceIdEnabled: false,
        twoFactorEnabled: false,
        lastLogin: new Date(),
      };

      saveUserSession(userData, 'signed-in');
      return true;
    } catch (error) {
      console.error('Biometric login error:', error);
      return false;
    }
  };

  const loginWithFaceId = async (): Promise<boolean> => {
    try {
      // Simulate Face ID authentication
      await new Promise(resolve => setTimeout(resolve, 2000));

      const userData: User = {
        id: '4',
        username: 'faceid_user',
        email: 'faceid@navlambda.com',
        isGuest: false,
        permissions: ['read', 'write'],
        biometricEnabled: false,
        faceIdEnabled: true,
        twoFactorEnabled: false,
        lastLogin: new Date(),
      };

      saveUserSession(userData, 'signed-in');
      return true;
    } catch (error) {
      console.error('Face ID login error:', error);
      return false;
    }
  };

  const loginAsGuest = () => {
    const guestUser: User = {
      id: 'guest',
      username: 'Guest',
      email: '',
      isGuest: true,
      permissions: ['read'],
      biometricEnabled: false,
      faceIdEnabled: false,
      twoFactorEnabled: false,
      lastLogin: new Date(),
    };

    saveUserSession(guestUser, 'signed-in');
  };

  const logout = () => {
    localStorage.removeItem('user');
    localStorage.removeItem('authStatus');
    setUser(null);
    setAuthStatus('signed-out');
    setPendingTwoFactor(false);
  };

  const signUp = async (userData: SignUpData): Promise<boolean> => {
    try {
      if (userData.password !== userData.confirmPassword) {
        throw new Error('Passwords do not match');
      }

      // Simulate API call
      await new Promise(resolve => setTimeout(resolve, 1500));

      // Send verification code
      const verificationSent = await sendVerificationCode(userData.email, userData.phone);

      return verificationSent;
    } catch (error) {
      console.error('Sign up error:', error);
      return false;
    }
  };

  const verifyTwoFactor = async (code: string): Promise<boolean> => {
    try {
      await new Promise(resolve => setTimeout(resolve, 1000));

      // Mock verification
      if (code === '123456' && user) {
        setPendingTwoFactor(false);
        saveUserSession(user, 'signed-in');
        return true;
      }
      return false;
    } catch (error) {
      console.error('2FA verification error:', error);
      return false;
    }
  };

  const sendVerificationCode = async (email: string, phone?: string): Promise<boolean> => {
    try {
      // Simulate sending verification code
      await new Promise(resolve => setTimeout(resolve, 1000));
      console.log(`Verification code sent to ${email}${phone ? ` and ${phone}` : ''}`);
      return true;
    } catch (error) {
      console.error('Send verification error:', error);
      return false;
    }
  };

  const requestPasswordReset = async (emailOrPhone: string): Promise<boolean> => {
    try {
      await new Promise(resolve => setTimeout(resolve, 1000));
      console.log(`Password reset link sent to ${emailOrPhone}`);
      return true;
    } catch (error) {
      console.error('Password reset request error:', error);
      return false;
    }
  };

  const resetPassword = async (token: string, newPassword: string): Promise<boolean> => {
    try {
      await new Promise(resolve => setTimeout(resolve, 1000));
      console.log('Password reset successful');
      return true;
    } catch (error) {
      console.error('Password reset error:', error);
      return false;
    }
  };

  const changePassword = async (oldPassword: string, newPassword: string): Promise<boolean> => {
    try {
      await new Promise(resolve => setTimeout(resolve, 1000));
      console.log('Password changed successfully');
      return true;
    } catch (error) {
      console.error('Change password error:', error);
      return false;
    }
  };

  const enableBiometric = async (): Promise<boolean> => {
    try {
      if (!window.PublicKeyCredential) {
        throw new Error('Biometric authentication not supported');
      }

      await new Promise(resolve => setTimeout(resolve, 1500));

      if (user) {
        const updatedUser = { ...user, biometricEnabled: true };
        saveUserSession(updatedUser, authStatus);
      }
      return true;
    } catch (error) {
      console.error('Enable biometric error:', error);
      return false;
    }
  };

  const enableFaceId = async (): Promise<boolean> => {
    try {
      await new Promise(resolve => setTimeout(resolve, 2000));

      if (user) {
        const updatedUser = { ...user, faceIdEnabled: true };
        saveUserSession(updatedUser, authStatus);
      }
      return true;
    } catch (error) {
      console.error('Enable Face ID error:', error);
      return false;
    }
  };

  const enableTwoFactor = async (method: 'email' | 'sms'): Promise<boolean> => {
    try {
      await new Promise(resolve => setTimeout(resolve, 1000));

      if (user) {
        const updatedUser = { ...user, twoFactorEnabled: true };
        saveUserSession(updatedUser, authStatus);
      }
      return true;
    } catch (error) {
      console.error('Enable 2FA error:', error);
      return false;
    }
  };

  const setStandbyMode = () => {
    if (user) {
      setAuthStatus('standby');
      localStorage.setItem('authStatus', 'standby');
    }
  };

  return (
    <AuthContext.Provider
      value={{
        user,
        authStatus,
        isAuthenticated: authStatus === 'signed-in',
        login,
        loginWithPin,
        loginWithBiometric,
        loginWithFaceId,
        loginAsGuest,
        logout,
        signUp,
        verifyTwoFactor: pendingTwoFactor ? verifyTwoFactor : async () => false,
        requestPasswordReset,
        resetPassword,
        changePassword,
        enableBiometric,
        enableFaceId,
        enableTwoFactor,
        setStandbyMode,
      }}
    >
      {children}
    </AuthContext.Provider>
  );
};