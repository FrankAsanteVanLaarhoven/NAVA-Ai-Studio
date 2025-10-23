// NAVΛ Studio Authentication Service
// Handles email verification, PIN management, and biometric authentication

export interface User {
  id: string;
  username: string;
  email: string;
  emailVerified: boolean;
  hasPIN: boolean;
  biometricEnabled: boolean;
  createdAt: string;
  lastLogin: string;
}

export interface AuthSession {
  user: User;
  token: string;
  expiresAt: string;
}

export interface VerificationCode {
  code: string;
  email: string;
  expiresAt: string;
}

class AuthService {
  private readonly STORAGE_KEYS = {
    USERS: 'navlambda_users',
    CURRENT_USER: 'navlambda_current_user',
    SESSION: 'navlambda_session',
    VERIFICATION_CODES: 'navlambda_verification_codes',
    PINS: 'navlambda_pins',
    BIOMETRIC_ENABLED: 'navlambda_biometric_enabled'
  };

  // ==================== USER MANAGEMENT ====================

  /**
   * Create a new user account
   */
  async createAccount(username: string, email: string, password: string): Promise<{ success: boolean; message: string; userId?: string }> {
    try {
      // Validate input
      if (!username || username.length < 3) {
        return { success: false, message: 'Username must be at least 3 characters' };
      }

      if (!this.isValidEmail(email)) {
        return { success: false, message: 'Invalid email address' };
      }

      if (!password || password.length < 8) {
        return { success: false, message: 'Password must be at least 8 characters' };
      }

      // Check if user already exists
      const users = this.getUsers();
      if (users.find(u => u.email === email)) {
        return { success: false, message: 'Email already registered' };
      }

      if (users.find(u => u.username === username)) {
        return { success: false, message: 'Username already taken' };
      }

      // Create user
      const userId = this.generateId();
      const user: User = {
        id: userId,
        username,
        email,
        emailVerified: false,
        hasPIN: false,
        biometricEnabled: false,
        createdAt: new Date().toISOString(),
        lastLogin: new Date().toISOString()
      };

      // Store user
      users.push(user);
      this.saveUsers(users);

      // Store password (hashed in production)
      this.storePassword(userId, password);

      // Send verification email
      await this.sendVerificationEmail(email);

      return { 
        success: true, 
        message: 'Account created! Please check your email for verification code.',
        userId 
      };
    } catch (error) {
      console.error('Create account error:', error);
      return { success: false, message: 'Failed to create account' };
    }
  }

  /**
   * Sign in with email and password
   */
  async signIn(email: string, password: string): Promise<{ success: boolean; message: string; user?: User; requiresVerification?: boolean }> {
    try {
      const users = this.getUsers();
      const user = users.find(u => u.email === email);

      if (!user) {
        return { success: false, message: 'Invalid email or password' };
      }

      // Verify password
      const storedPassword = this.getPassword(user.id);
      if (storedPassword !== password) {
        return { success: false, message: 'Invalid email or password' };
      }

      // Check email verification
      if (!user.emailVerified) {
        return { 
          success: false, 
          message: 'Please verify your email first',
          requiresVerification: true 
        };
      }

      // Update last login
      user.lastLogin = new Date().toISOString();
      this.updateUser(user);

      // Create session
      const session = this.createSession(user);
      this.saveSession(session);

      return { success: true, message: 'Signed in successfully', user };
    } catch (error) {
      console.error('Sign in error:', error);
      return { success: false, message: 'Failed to sign in' };
    }
  }

  /**
   * Sign in with PIN
   */
  async signInWithPIN(pin: string): Promise<{ success: boolean; message: string; user?: User }> {
    try {
      const users = this.getUsers();
      
      for (const user of users) {
        const storedPIN = this.getPIN(user.id);
        if (storedPIN === pin) {
          // Update last login
          user.lastLogin = new Date().toISOString();
          this.updateUser(user);

          // Create session
          const session = this.createSession(user);
          this.saveSession(session);

          return { success: true, message: 'Signed in with PIN', user };
        }
      }

      return { success: false, message: 'Invalid PIN' };
    } catch (error) {
      console.error('PIN sign in error:', error);
      return { success: false, message: 'Failed to sign in with PIN' };
    }
  }

  /**
   * Sign in with biometric authentication
   */
  async signInWithBiometric(): Promise<{ success: boolean; message: string; user?: User }> {
    try {
      // Check if biometric is available
      if (!this.isBiometricAvailable()) {
        return { success: false, message: 'Biometric authentication not available' };
      }

      // Authenticate with Web Authentication API
      const authenticated = await this.authenticateWithBiometric();
      
      if (!authenticated) {
        return { success: false, message: 'Biometric authentication failed' };
      }

      // Get user with biometric enabled
      const users = this.getUsers();
      const user = users.find(u => u.biometricEnabled);

      if (!user) {
        return { success: false, message: 'No user with biometric authentication enabled' };
      }

      // Update last login
      user.lastLogin = new Date().toISOString();
      this.updateUser(user);

      // Create session
      const session = this.createSession(user);
      this.saveSession(session);

      return { success: true, message: 'Signed in with biometric', user };
    } catch (error) {
      console.error('Biometric sign in error:', error);
      return { success: false, message: 'Biometric authentication failed' };
    }
  }

  /**
   * Sign out current user
   */
  signOut(): void {
    localStorage.removeItem(this.STORAGE_KEYS.SESSION);
    localStorage.removeItem(this.STORAGE_KEYS.CURRENT_USER);
  }

  // ==================== EMAIL VERIFICATION ====================

  /**
   * Send verification email with code
   */
  async sendVerificationEmail(email: string): Promise<void> {
    const code = this.generateVerificationCode();
    const verification: VerificationCode = {
      code,
      email,
      expiresAt: new Date(Date.now() + 15 * 60 * 1000).toISOString() // 15 minutes
    };

    // Store verification code
    const codes = this.getVerificationCodes();
    codes.push(verification);
    localStorage.setItem(this.STORAGE_KEYS.VERIFICATION_CODES, JSON.stringify(codes));

    // In production, send actual email
    console.log(`📧 Verification code for ${email}: ${code}`);
    
    // Show notification to user
    this.showNotification(`Verification code sent to ${email}. Code: ${code} (Check console in dev mode)`);
  }

  /**
   * Verify email with code
   */
  verifyEmail(email: string, code: string): { success: boolean; message: string } {
    const codes = this.getVerificationCodes();
    const verification = codes.find(v => v.email === email && v.code === code);

    if (!verification) {
      return { success: false, message: 'Invalid verification code' };
    }

    // Check expiration
    if (new Date(verification.expiresAt) < new Date()) {
      return { success: false, message: 'Verification code expired' };
    }

    // Mark email as verified
    const users = this.getUsers();
    const user = users.find(u => u.email === email);
    
    if (user) {
      user.emailVerified = true;
      this.updateUser(user);

      // Remove used verification code
      const updatedCodes = codes.filter(v => v !== verification);
      localStorage.setItem(this.STORAGE_KEYS.VERIFICATION_CODES, JSON.stringify(updatedCodes));

      return { success: true, message: 'Email verified successfully!' };
    }

    return { success: false, message: 'User not found' };
  }

  /**
   * Resend verification code
   */
  async resendVerificationCode(email: string): Promise<{ success: boolean; message: string }> {
    const users = this.getUsers();
    const user = users.find(u => u.email === email);

    if (!user) {
      return { success: false, message: 'User not found' };
    }

    if (user.emailVerified) {
      return { success: false, message: 'Email already verified' };
    }

    await this.sendVerificationEmail(email);
    return { success: true, message: 'Verification code sent!' };
  }

  // ==================== PIN MANAGEMENT ====================

  /**
   * Set up PIN for user
   */
  setupPIN(userId: string, pin: string): { success: boolean; message: string } {
    if (!pin || pin.length < 4 || pin.length > 8) {
      return { success: false, message: 'PIN must be 4-8 digits' };
    }

    if (!/^\d+$/.test(pin)) {
      return { success: false, message: 'PIN must contain only numbers' };
    }

    // Store PIN (encrypted in production)
    const pins = this.getPINs();
    pins[userId] = pin;
    localStorage.setItem(this.STORAGE_KEYS.PINS, JSON.stringify(pins));

    // Update user
    const users = this.getUsers();
    const user = users.find(u => u.id === userId);
    if (user) {
      user.hasPIN = true;
      this.updateUser(user);
    }

    return { success: true, message: 'PIN set up successfully!' };
  }

  /**
   * Change PIN
   */
  changePIN(userId: string, oldPIN: string, newPIN: string): { success: boolean; message: string } {
    const storedPIN = this.getPIN(userId);
    
    if (storedPIN !== oldPIN) {
      return { success: false, message: 'Current PIN is incorrect' };
    }

    return this.setupPIN(userId, newPIN);
  }

  /**
   * Remove PIN
   */
  removePIN(userId: string, pin: string): { success: boolean; message: string } {
    const storedPIN = this.getPIN(userId);
    
    if (storedPIN !== pin) {
      return { success: false, message: 'PIN is incorrect' };
    }

    const pins = this.getPINs();
    delete pins[userId];
    localStorage.setItem(this.STORAGE_KEYS.PINS, JSON.stringify(pins));

    // Update user
    const users = this.getUsers();
    const user = users.find(u => u.id === userId);
    if (user) {
      user.hasPIN = false;
      this.updateUser(user);
    }

    return { success: true, message: 'PIN removed successfully' };
  }

  // ==================== BIOMETRIC AUTHENTICATION ====================

  /**
   * Check if biometric authentication is available
   */
  isBiometricAvailable(): boolean {
    return window.PublicKeyCredential !== undefined && 
           navigator.credentials !== undefined;
  }

  /**
   * Get biometric type name
   */
  getBiometricType(): string {
    const platform = navigator.platform.toLowerCase();
    
    if (platform.includes('mac')) {
      return 'Touch ID / Face ID';
    } else if (platform.includes('win')) {
      return 'Windows Hello';
    } else if (platform.includes('linux')) {
      return 'Fingerprint';
    } else if (platform.includes('iphone') || platform.includes('ipad')) {
      return 'Face ID / Touch ID';
    } else if (platform.includes('android')) {
      return 'Fingerprint / Face Unlock';
    }
    
    return 'Biometric Authentication';
  }

  /**
   * Enable biometric authentication for user
   */
  async enableBiometric(userId: string): Promise<{ success: boolean; message: string }> {
    try {
      if (!this.isBiometricAvailable()) {
        return { success: false, message: 'Biometric authentication not available on this device' };
      }

      // Register biometric credential
      const registered = await this.registerBiometric(userId);
      
      if (!registered) {
        return { success: false, message: 'Failed to register biometric authentication' };
      }

      // Update user
      const users = this.getUsers();
      const user = users.find(u => u.id === userId);
      if (user) {
        user.biometricEnabled = true;
        this.updateUser(user);
      }

      return { success: true, message: `${this.getBiometricType()} enabled successfully!` };
    } catch (error) {
      console.error('Enable biometric error:', error);
      return { success: false, message: 'Failed to enable biometric authentication' };
    }
  }

  /**
   * Disable biometric authentication
   */
  disableBiometric(userId: string): { success: boolean; message: string } {
    const users = this.getUsers();
    const user = users.find(u => u.id === userId);
    
    if (user) {
      user.biometricEnabled = false;
      this.updateUser(user);
      return { success: true, message: 'Biometric authentication disabled' };
    }

    return { success: false, message: 'User not found' };
  }

  /**
   * Register biometric credential (Web Authentication API)
   */
  private async registerBiometric(userId: string): Promise<boolean> {
    try {
      // Create credential
      const credential = await navigator.credentials.create({
        publicKey: {
          challenge: new Uint8Array(32),
          rp: {
            name: 'NAVΛ Studio',
            id: window.location.hostname
          },
          user: {
            id: new TextEncoder().encode(userId),
            name: userId,
            displayName: 'NAVΛ Studio User'
          },
          pubKeyCredParams: [
            { type: 'public-key', alg: -7 },  // ES256
            { type: 'public-key', alg: -257 } // RS256
          ],
          authenticatorSelection: {
            authenticatorAttachment: 'platform',
            userVerification: 'required'
          },
          timeout: 60000
        }
      });

      // Store credential ID
      if (credential) {
        localStorage.setItem(`${this.STORAGE_KEYS.BIOMETRIC_ENABLED}_${userId}`, 'true');
        return true;
      }

      return false;
    } catch (error) {
      console.error('Register biometric error:', error);
      return false;
    }
  }

  /**
   * Authenticate with biometric
   */
  private async authenticateWithBiometric(): Promise<boolean> {
    try {
      const assertion = await navigator.credentials.get({
        publicKey: {
          challenge: new Uint8Array(32),
          timeout: 60000,
          userVerification: 'required'
        }
      });

      return assertion !== null;
    } catch (error) {
      console.error('Authenticate biometric error:', error);
      return false;
    }
  }

  // ==================== SESSION MANAGEMENT ====================

  /**
   * Get current session
   */
  getCurrentSession(): AuthSession | null {
    const sessionData = localStorage.getItem(this.STORAGE_KEYS.SESSION);
    if (!sessionData) return null;

    const session: AuthSession = JSON.parse(sessionData);
    
    // Check if session expired
    if (new Date(session.expiresAt) < new Date()) {
      this.signOut();
      return null;
    }

    return session;
  }

  /**
   * Get current user
   */
  getCurrentUser(): User | null {
    const session = this.getCurrentSession();
    return session?.user || null;
  }

  /**
   * Check if user is authenticated
   */
  isAuthenticated(): boolean {
    return this.getCurrentSession() !== null;
  }

  // ==================== HELPER METHODS ====================

  private getUsers(): User[] {
    const data = localStorage.getItem(this.STORAGE_KEYS.USERS);
    return data ? JSON.parse(data) : [];
  }

  private saveUsers(users: User[]): void {
    localStorage.setItem(this.STORAGE_KEYS.USERS, JSON.stringify(users));
  }

  private updateUser(user: User): void {
    const users = this.getUsers();
    const index = users.findIndex(u => u.id === user.id);
    if (index !== -1) {
      users[index] = user;
      this.saveUsers(users);
    }
  }

  private createSession(user: User): AuthSession {
    return {
      user,
      token: this.generateToken(),
      expiresAt: new Date(Date.now() + 24 * 60 * 60 * 1000).toISOString() // 24 hours
    };
  }

  private saveSession(session: AuthSession): void {
    localStorage.setItem(this.STORAGE_KEYS.SESSION, JSON.stringify(session));
    localStorage.setItem(this.STORAGE_KEYS.CURRENT_USER, JSON.stringify(session.user));
  }

  private storePassword(userId: string, password: string): void {
    // In production, use proper encryption
    localStorage.setItem(`navlambda_pwd_${userId}`, password);
  }

  private getPassword(userId: string): string | null {
    return localStorage.getItem(`navlambda_pwd_${userId}`);
  }

  private getPINs(): Record<string, string> {
    const data = localStorage.getItem(this.STORAGE_KEYS.PINS);
    return data ? JSON.parse(data) : {};
  }

  private getPIN(userId: string): string | null {
    const pins = this.getPINs();
    return pins[userId] || null;
  }

  private getVerificationCodes(): VerificationCode[] {
    const data = localStorage.getItem(this.STORAGE_KEYS.VERIFICATION_CODES);
    return data ? JSON.parse(data) : [];
  }

  private generateId(): string {
    return `user_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
  }

  private generateToken(): string {
    return `token_${Date.now()}_${Math.random().toString(36).substr(2, 16)}`;
  }

  private generateVerificationCode(): string {
    return Math.floor(100000 + Math.random() * 900000).toString();
  }

  private isValidEmail(email: string): boolean {
    return /^[^\s@]+@[^\s@]+\.[^\s@]+$/.test(email);
  }

  private showNotification(message: string): void {
    // In production, use proper notification system
    console.log('📢', message);
  }
}

// Export singleton instance
export const authService = new AuthService();
