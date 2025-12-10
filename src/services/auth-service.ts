/**
 * Authentication Service
 * Handles user login, logout, and session management
 */

export interface User {
  id: string;
  email: string;
  name: string;
  avatar?: string;
  plan: 'free' | 'pro' | 'pro-plus' | 'enterprise';
  createdAt: string;
  lastLoginAt?: string;
  preferences?: {
    theme?: 'dark' | 'light' | 'auto';
    defaultModel?: string;
    notifications?: boolean;
  };
}

export interface LoginCredentials {
  email: string;
  password?: string; // Optional for OAuth flows
  provider?: 'email' | 'google' | 'github' | 'microsoft';
}

class AuthService {
  private readonly STORAGE_KEY = 'nava_auth_user';
  private readonly SESSION_KEY = 'nava_auth_session';
  private currentUser: User | null = null;

  constructor() {
    // Load user from localStorage on initialization
    this.loadUserFromStorage();
  }

  /**
   * Login with email/password or OAuth provider
   */
  async login(credentials: LoginCredentials): Promise<User> {
    // Simulate API call - in production, this would call your backend
    return new Promise((resolve, reject) => {
      setTimeout(() => {
        // For demo purposes, accept any email
        // In production, validate against your backend
        if (!credentials.email) {
          reject(new Error('Email is required'));
          return;
        }

        // Check if user exists in localStorage
        const existingUsers = this.getStoredUsers();
        let user = existingUsers.find(u => u.email === credentials.email);

        if (!user) {
          // Create new user
          user = {
            id: this.generateUserId(),
            email: credentials.email,
            name: credentials.email.split('@')[0],
            plan: 'free',
            createdAt: new Date().toISOString(),
            lastLoginAt: new Date().toISOString(),
            preferences: {
              theme: 'dark',
              defaultModel: 'gemini-1.5-pro-latest',
              notifications: true,
            },
          };
          this.saveUser(user);
        } else {
          // Update last login
          user.lastLoginAt = new Date().toISOString();
          this.updateUser(user);
        }

        // Set as current user
        this.setCurrentUser(user);
        resolve(user);
      }, 500); // Simulate network delay
    });
  }

  /**
   * Logout current user
   */
  logout(): void {
    this.currentUser = null;
    localStorage.removeItem(this.SESSION_KEY);
    localStorage.removeItem(this.STORAGE_KEY);
  }

  /**
   * Get current logged-in user
   */
  getCurrentUser(): User | null {
    return this.currentUser;
  }

  /**
   * Check if user is logged in
   */
  isAuthenticated(): boolean {
    return this.currentUser !== null;
  }

  /**
   * Update user profile
   */
  async updateUserProfile(updates: Partial<User>): Promise<User> {
    if (!this.currentUser) {
      throw new Error('No user logged in');
    }

    const updatedUser = {
      ...this.currentUser,
      ...updates,
      id: this.currentUser.id, // Preserve ID
    };

    this.setCurrentUser(updatedUser);
    this.updateUser(updatedUser);
    return updatedUser;
  }

  /**
   * Switch to a different user account
   */
  async switchUser(userId: string): Promise<User> {
    const users = this.getStoredUsers();
    const user = users.find(u => u.id === userId);
    
    if (!user) {
      throw new Error('User not found');
    }

    user.lastLoginAt = new Date().toISOString();
    this.setCurrentUser(user);
    this.updateUser(user);
    return user;
  }

  /**
   * Get all stored users (for multi-user support)
   */
  getStoredUsers(): User[] {
    try {
      const stored = localStorage.getItem('nava_stored_users');
      return stored ? JSON.parse(stored) : [];
    } catch {
      return [];
    }
  }

  /**
   * Delete a user account
   */
  deleteUser(userId: string): void {
    const users = this.getStoredUsers().filter(u => u.id !== userId);
    localStorage.setItem('nava_stored_users', JSON.stringify(users));

    // If deleting current user, logout
    if (this.currentUser?.id === userId) {
      this.logout();
    }
  }

  /**
   * Private helper methods
   */
  private loadUserFromStorage(): void {
    try {
      const stored = localStorage.getItem(this.STORAGE_KEY);
      if (stored) {
        this.currentUser = JSON.parse(stored);
      }
    } catch {
      this.currentUser = null;
    }
  }

  private setCurrentUser(user: User): void {
    this.currentUser = user;
    localStorage.setItem(this.STORAGE_KEY, JSON.stringify(user));
    localStorage.setItem(this.SESSION_KEY, JSON.stringify({
      userId: user.id,
      loginTime: new Date().toISOString(),
    }));
  }

  private saveUser(user: User): void {
    const users = this.getStoredUsers();
    users.push(user);
    localStorage.setItem('nava_stored_users', JSON.stringify(users));
  }

  private updateUser(user: User): void {
    const users = this.getStoredUsers();
    const index = users.findIndex(u => u.id === user.id);
    if (index >= 0) {
      users[index] = user;
      localStorage.setItem('nava_stored_users', JSON.stringify(users));
    }
  }

  private generateUserId(): string {
    return `user_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
  }
}

export const authService = new AuthService();
