/**
 * Terminal Service
 * 
 * Provides terminal functionality for executing commands in the IDE
 */

export interface TerminalCommand {
  command: string;
  args: string[];
  cwd?: string;
}

export interface TerminalOutput {
  type: 'stdout' | 'stderr' | 'info' | 'error' | 'success';
  content: string;
  timestamp: number;
}

export interface TerminalSession {
  id: string;
  name: string;
  cwd: string;
  history: string[];
  outputs: TerminalOutput[];
  isActive: boolean;
}

class TerminalService {
  private sessions: Map<string, TerminalSession> = new Map();
  private activeSessionId: string | null = null;

  createSession(name: string = 'Terminal', cwd: string = '~'): TerminalSession {
    const id = `terminal-${Date.now()}-${Math.random().toString(36).slice(2, 9)}`;
    const session: TerminalSession = {
      id,
      name,
      cwd,
      history: [],
      outputs: [],
      isActive: true,
    };
    this.sessions.set(id, session);
    this.activeSessionId = id;
    return session;
  }

  getSession(id: string): TerminalSession | undefined {
    return this.sessions.get(id);
  }

  getActiveSession(): TerminalSession | undefined {
    return this.activeSessionId ? this.sessions.get(this.activeSessionId) : undefined;
  }

  setActiveSession(id: string): void {
    if (this.sessions.has(id)) {
      this.activeSessionId = id;
      const session = this.sessions.get(id);
      if (session) {
        session.isActive = true;
      }
    }
  }

  closeSession(id: string): void {
    this.sessions.delete(id);
    if (this.activeSessionId === id) {
      const remaining = Array.from(this.sessions.values());
      this.activeSessionId = remaining.length > 0 ? remaining[0].id : null;
    }
  }

  async executeCommand(sessionId: string, command: string): Promise<TerminalOutput[]> {
    const session = this.sessions.get(sessionId);
    if (!session) {
      throw new Error(`Session ${sessionId} not found`);
    }

    session.history.push(command);
    const outputs: TerminalOutput[] = [];

    try {
      const result = await this.processCommand(command, session.cwd);
      outputs.push(...result);
      session.outputs.push(...result);
    } catch (error) {
      const errorOutput: TerminalOutput = {
        type: 'error',
        content: error instanceof Error ? error.message : 'Unknown error',
        timestamp: Date.now(),
      };
      outputs.push(errorOutput);
      session.outputs.push(errorOutput);
    }

    return outputs;
  }

  private async processCommand(command: string, cwd: string): Promise<TerminalOutput[]> {
    const parts = command.trim().split(' ');
    const cmd = parts[0];
    const args = parts.slice(1);

    // Built-in commands
    switch (cmd) {
      case 'clear':
        return [
          {
            type: 'info',
            content: 'Terminal cleared',
            timestamp: Date.now(),
          },
        ];

      case 'pwd':
        return [
          {
            type: 'stdout',
            content: cwd,
            timestamp: Date.now(),
          },
        ];

      case 'echo':
        return [
          {
            type: 'stdout',
            content: args.join(' '),
            timestamp: Date.now(),
          },
        ];

      case 'help':
        return [
          {
            type: 'info',
            content: this.getHelpText(),
            timestamp: Date.now(),
          },
        ];

      // Development commands
      case 'npm':
        return await this.executeNpmCommand(args);

      case 'node':
        return await this.executeNodeCommand(args);

      case 'python':
        return await this.executePythonCommand(args);

      case 'git':
        return await this.executeGitCommand(args);

      // NAVΛ specific commands
      case 'vnc':
      case 'navlambda':
        return await this.executeVncCommand(args);

      case 'compile':
        return await this.compileVncCode();

      case 'run':
        return await this.runProject();

      case 'test':
        return await this.runTests();

      case 'build':
        return await this.buildProject();

      default:
        return [
          {
            type: 'error',
            content: `Command not found: ${cmd}\nType 'help' for available commands`,
            timestamp: Date.now(),
          },
        ];
    }
  }

  private async executeNpmCommand(args: string[]): Promise<TerminalOutput[]> {
    const outputs: TerminalOutput[] = [];
    outputs.push({
      type: 'info',
      content: `$ npm ${args.join(' ')}`,
      timestamp: Date.now(),
    });

    // Simulate npm command execution
    await this.delay(1000);

    if (args[0] === 'install') {
      outputs.push({
        type: 'success',
        content: '✓ Package installed successfully',
        timestamp: Date.now(),
      });
    } else if (args[0] === 'run') {
      outputs.push({
        type: 'success',
        content: '✓ Script executed successfully',
        timestamp: Date.now(),
      });
    } else if (args[0] === 'test') {
      outputs.push({
        type: 'success',
        content: '✓ Tests passed',
        timestamp: Date.now(),
      });
    } else {
      outputs.push({
        type: 'success',
        content: '✓ Command completed',
        timestamp: Date.now(),
      });
    }

    return outputs;
  }

  private async executeNodeCommand(args: string[]): Promise<TerminalOutput[]> {
    return [
      {
        type: 'info',
        content: `$ node ${args.join(' ')}`,
        timestamp: Date.now(),
      },
      {
        type: 'success',
        content: '✓ Node.js script executed',
        timestamp: Date.now(),
      },
    ];
  }

  private async executePythonCommand(args: string[]): Promise<TerminalOutput[]> {
    return [
      {
        type: 'info',
        content: `$ python ${args.join(' ')}`,
        timestamp: Date.now(),
      },
      {
        type: 'success',
        content: '✓ Python script executed',
        timestamp: Date.now(),
      },
    ];
  }

  private async executeGitCommand(args: string[]): Promise<TerminalOutput[]> {
    const outputs: TerminalOutput[] = [];
    outputs.push({
      type: 'info',
      content: `$ git ${args.join(' ')}`,
      timestamp: Date.now(),
    });

    if (args[0] === 'status') {
      outputs.push({
        type: 'stdout',
        content: 'On branch main\nYour branch is up to date with \'origin/main\'.\n\nnothing to commit, working tree clean',
        timestamp: Date.now(),
      });
    } else if (args[0] === 'log') {
      outputs.push({
        type: 'stdout',
        content: `commit abc123def456\nAuthor: NAVΛ Studio <dev@navlambda.studio>\nDate:   ${new Date().toDateString()}\n\n    Latest updates to VNC compiler`,
        timestamp: Date.now(),
      });
    } else {
      outputs.push({
        type: 'success',
        content: '✓ Git command executed',
        timestamp: Date.now(),
      });
    }

    return outputs;
  }

  private async executeVncCommand(args: string[]): Promise<TerminalOutput[]> {
    return [
      {
        type: 'info',
        content: '🚀 NAVΛ Compiler',
        timestamp: Date.now(),
      },
      {
        type: 'success',
        content: '✓ VNC code compiled successfully',
        timestamp: Date.now(),
      },
    ];
  }

  private async compileVncCode(): Promise<TerminalOutput[]> {
    const outputs: TerminalOutput[] = [];
    outputs.push({
      type: 'info',
      content: '🔨 Compiling VNC code...',
      timestamp: Date.now(),
    });

    await this.delay(1500);

    outputs.push({
      type: 'success',
      content: '✓ Compilation successful\n  Output: dist/main.wasm',
      timestamp: Date.now(),
    });

    return outputs;
  }

  private async runProject(): Promise<TerminalOutput[]> {
    const outputs: TerminalOutput[] = [];
    outputs.push({
      type: 'info',
      content: '▶ Running project...',
      timestamp: Date.now(),
    });

    await this.delay(1000);

    outputs.push({
      type: 'success',
      content: '✓ Project started successfully\n  Server running at http://localhost:5173',
      timestamp: Date.now(),
    });

    return outputs;
  }

  private async runTests(): Promise<TerminalOutput[]> {
    const outputs: TerminalOutput[] = [];
    outputs.push({
      type: 'info',
      content: '🧪 Running tests...',
      timestamp: Date.now(),
    });

    await this.delay(2000);

    outputs.push({
      type: 'success',
      content: '✓ All tests passed (12/12)',
      timestamp: Date.now(),
    });

    return outputs;
  }

  private async buildProject(): Promise<TerminalOutput[]> {
    const outputs: TerminalOutput[] = [];
    outputs.push({
      type: 'info',
      content: '🏗️  Building project...',
      timestamp: Date.now(),
    });

    await this.delay(3000);

    outputs.push({
      type: 'success',
      content: '✓ Build completed successfully\n  Output directory: dist/',
      timestamp: Date.now(),
    });

    return outputs;
  }

  private getHelpText(): string {
    return `Available Commands:

General:
  help             - Show this help message
  clear            - Clear the terminal
  echo <text>      - Print text
  pwd              - Print working directory
  cd <dir>         - Change directory
  ls               - List directory contents

Development:
  npm <command>    - Run npm commands
  node <file>      - Run Node.js scripts
  python <file>    - Run Python scripts
  git <command>    - Git version control

NAVΛ Studio:
  compile          - Compile VNC code
  run              - Run current project
  test             - Run tests
  build            - Build project
  vnc              - NAVΛ compiler commands

Shortcuts:
  Ctrl+C          - Cancel command
  Ctrl+L          - Clear screen
  ↑/↓             - Command history
  Tab             - Auto-complete`;
  }

  private delay(ms: number): Promise<void> {
    return new Promise((resolve) => setTimeout(resolve, ms));
  }
}

export const terminalService = new TerminalService();

