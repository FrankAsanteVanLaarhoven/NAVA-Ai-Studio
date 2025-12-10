export interface Breakpoint {
  id: string;
  filePath: string;
  lineNumber: number;
  enabled: boolean;
  condition?: string;
  hitCount?: number;
}

export interface DebugConfiguration {
  name: string;
  type: string;
  request: string;
  program?: string;
  args?: string[];
  env?: Record<string, string>;
  cwd?: string;
}

export interface DebugSession {
  id: string;
  configuration: DebugConfiguration;
  status: 'initializing' | 'running' | 'paused' | 'stopped';
  breakpoints: Breakpoint[];
  currentLine?: number;
  currentFile?: string;
  variables?: Record<string, any>;
  callStack?: Array<{ name: string; file: string; line: number }>;
}

class DebugService {
  private sessions: Map<string, DebugSession> = new Map();
  private breakpoints: Map<string, Breakpoint[]> = new Map();
  private currentSessionId: string | null = null;
  private defaultConfigurations: DebugConfiguration[] = [];

  constructor() {
    this.loadBreakpoints();
    this.loadConfigurations();
  }

  // Breakpoint Management
  addBreakpoint(filePath: string, lineNumber: number, condition?: string): Breakpoint {
    const id = `${filePath}:${lineNumber}`;
    const breakpoint: Breakpoint = {
      id,
      filePath,
      lineNumber,
      enabled: true,
      condition,
    };

    if (!this.breakpoints.has(filePath)) {
      this.breakpoints.set(filePath, []);
    }
    const fileBreakpoints = this.breakpoints.get(filePath)!;
    
    // Remove existing breakpoint at this line if any
    const existing = fileBreakpoints.findIndex(bp => bp.lineNumber === lineNumber);
    if (existing >= 0) {
      fileBreakpoints[existing] = breakpoint;
    } else {
      fileBreakpoints.push(breakpoint);
    }

    this.saveBreakpoints();
    this.notifyBreakpointsChanged();
    return breakpoint;
  }

  removeBreakpoint(filePath: string, lineNumber: number): void {
    const fileBreakpoints = this.breakpoints.get(filePath);
    if (fileBreakpoints) {
      const index = fileBreakpoints.findIndex(bp => bp.lineNumber === lineNumber);
      if (index >= 0) {
        fileBreakpoints.splice(index, 1);
        if (fileBreakpoints.length === 0) {
          this.breakpoints.delete(filePath);
        }
        this.saveBreakpoints();
        this.notifyBreakpointsChanged();
      }
    }
  }

  toggleBreakpoint(filePath: string, lineNumber: number): Breakpoint | null {
    const fileBreakpoints = this.breakpoints.get(filePath);
    if (fileBreakpoints) {
      const existing = fileBreakpoints.find(bp => bp.lineNumber === lineNumber);
      if (existing) {
        this.removeBreakpoint(filePath, lineNumber);
        return null;
      }
    }
    return this.addBreakpoint(filePath, lineNumber);
  }

  getBreakpoints(filePath?: string): Breakpoint[] {
    if (filePath) {
      return this.breakpoints.get(filePath) || [];
    }
    return Array.from(this.breakpoints.values()).flat();
  }

  enableAllBreakpoints(): void {
    this.breakpoints.forEach(breakpoints => {
      breakpoints.forEach(bp => bp.enabled = true);
    });
    this.saveBreakpoints();
    this.notifyBreakpointsChanged();
  }

  disableAllBreakpoints(): void {
    this.breakpoints.forEach(breakpoints => {
      breakpoints.forEach(bp => bp.enabled = false);
    });
    this.saveBreakpoints();
    this.notifyBreakpointsChanged();
  }

  removeAllBreakpoints(): void {
    this.breakpoints.clear();
    this.saveBreakpoints();
    this.notifyBreakpointsChanged();
  }

  // Debug Session Management
  startDebugging(configuration: DebugConfiguration): DebugSession {
    const sessionId = `session-${Date.now()}`;
    const session: DebugSession = {
      id: sessionId,
      configuration,
      status: 'initializing',
      breakpoints: this.getBreakpoints(),
    };

    this.sessions.set(sessionId, session);
    this.currentSessionId = sessionId;

    // Simulate debugger initialization
    setTimeout(() => {
      session.status = 'running';
      this.notifySessionChanged(session);
      console.log('[DebugService] Debug session started:', sessionId);
    }, 500);

    return session;
  }

  stopDebugging(sessionId?: string): void {
    const id = sessionId || this.currentSessionId;
    if (id) {
      const session = this.sessions.get(id);
      if (session) {
        session.status = 'stopped';
        this.notifySessionChanged(session);
        this.sessions.delete(id);
        if (this.currentSessionId === id) {
          this.currentSessionId = null;
        }
      }
    }
  }

  restartDebugging(sessionId?: string): DebugSession | null {
    const id = sessionId || this.currentSessionId;
    if (id) {
      const session = this.sessions.get(id);
      if (session) {
        this.stopDebugging(id);
        return this.startDebugging(session.configuration);
      }
    }
    return null;
  }

  pauseDebugging(sessionId?: string): void {
    const id = sessionId || this.currentSessionId;
    if (id) {
      const session = this.sessions.get(id);
      if (session && session.status === 'running') {
        session.status = 'paused';
        this.notifySessionChanged(session);
      }
    }
  }

  continueDebugging(sessionId?: string): void {
    const id = sessionId || this.currentSessionId;
    if (id) {
      const session = this.sessions.get(id);
      if (session && session.status === 'paused') {
        session.status = 'running';
        this.notifySessionChanged(session);
      }
    }
  }

  stepOver(sessionId?: string): void {
    const id = sessionId || this.currentSessionId;
    if (id) {
      const session = this.sessions.get(id);
      if (session && session.status === 'paused') {
        // Simulate step over
        if (session.currentLine !== undefined) {
          session.currentLine += 1;
        }
        this.notifySessionChanged(session);
      }
    }
  }

  stepInto(sessionId?: string): void {
    const id = sessionId || this.currentSessionId;
    if (id) {
      const session = this.sessions.get(id);
      if (session && session.status === 'paused') {
        // Simulate step into
        this.notifySessionChanged(session);
      }
    }
  }

  stepOut(sessionId?: string): void {
    const id = sessionId || this.currentSessionId;
    if (id) {
      const session = this.sessions.get(id);
      if (session && session.status === 'paused') {
        // Simulate step out
        this.notifySessionChanged(session);
      }
    }
  }

  getCurrentSession(): DebugSession | null {
    if (this.currentSessionId) {
      return this.sessions.get(this.currentSessionId) || null;
    }
    return null;
  }

  // Configuration Management
  addConfiguration(config: DebugConfiguration): void {
    this.defaultConfigurations.push(config);
    this.saveConfigurations();
  }

  getConfigurations(): DebugConfiguration[] {
    return this.defaultConfigurations;
  }

  // Persistence
  private saveBreakpoints(): void {
    const data = Array.from(this.breakpoints.entries()).map(([path, bps]) => ({
      path,
      breakpoints: bps,
    }));
    localStorage.setItem('nava-debug-breakpoints', JSON.stringify(data));
  }

  private loadBreakpoints(): void {
    const saved = localStorage.getItem('nava-debug-breakpoints');
    if (saved) {
      try {
        const data = JSON.parse(saved);
        data.forEach(({ path, breakpoints }: { path: string; breakpoints: Breakpoint[] }) => {
          this.breakpoints.set(path, breakpoints);
        });
      } catch (error) {
        console.error('[DebugService] Failed to load breakpoints:', error);
      }
    }
  }

  private saveConfigurations(): void {
    localStorage.setItem('nava-debug-configurations', JSON.stringify(this.defaultConfigurations));
  }

  private loadConfigurations(): void {
    const saved = localStorage.getItem('nava-debug-configurations');
    if (saved) {
      try {
        this.defaultConfigurations = JSON.parse(saved);
      } catch (error) {
        console.error('[DebugService] Failed to load configurations:', error);
      }
    } else {
      // Default configurations
      this.defaultConfigurations = [
        {
          name: 'Launch NAVΛ Program',
          type: 'navlambda',
          request: 'launch',
          program: '${workspaceFolder}/main.navλ',
        },
        {
          name: 'Launch Python Script',
          type: 'python',
          request: 'launch',
          program: '${workspaceFolder}/main.py',
        },
        {
          name: 'Launch Node.js',
          type: 'node',
          request: 'launch',
          program: '${workspaceFolder}/index.js',
        },
      ];
      this.saveConfigurations();
    }
  }

  private notifyBreakpointsChanged(): void {
    const event = new CustomEvent('nava:breakpoints-changed', {
      detail: { breakpoints: this.getBreakpoints() },
    });
    window.dispatchEvent(event);
  }

  private notifySessionChanged(session: DebugSession): void {
    const event = new CustomEvent('nava:debug-session-changed', {
      detail: { session },
    });
    window.dispatchEvent(event);
  }
}

export const debugService = new DebugService();

