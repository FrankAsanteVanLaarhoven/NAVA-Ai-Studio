import { useEffect, useRef, useState } from 'react';
import { Terminal as XTerm } from '@xterm/xterm';
import { FitAddon } from '@xterm/addon-fit';
import { WebLinksAddon } from '@xterm/addon-web-links';
import { SearchAddon } from '@xterm/addon-search';
import '@xterm/xterm/css/xterm.css';
import './Terminal.css';

interface TerminalProps {
  onClose?: () => void;
  initialDirectory?: string;
}

export const Terminal: React.FC<TerminalProps> = ({ onClose, initialDirectory = '~' }) => {
  const terminalRef = useRef<HTMLDivElement>(null);
  const xtermRef = useRef<XTerm | null>(null);
  const fitAddonRef = useRef<FitAddon | null>(null);
  const [terminalReady, setTerminalReady] = useState(false);
  const [currentDirectory, setCurrentDirectory] = useState(initialDirectory);
  const commandHistoryRef = useRef<string[]>([]);
  const historyIndexRef = useRef<number>(-1);
  const currentInputRef = useRef<string>('');

  useEffect(() => {
    if (!terminalRef.current || xtermRef.current) return;

    // Create terminal instance
    const term = new XTerm({
      cursorBlink: true,
      fontSize: 14,
      fontFamily: 'Menlo, Monaco, "Courier New", monospace',
      theme: {
        background: '#0a0f1e',
        foreground: '#e0e0e0',
        cursor: '#00ff00',
        cursorAccent: '#000000',
        selection: 'rgba(0, 255, 0, 0.3)',
        black: '#000000',
        red: '#ef4444',
        green: '#00ff00',
        yellow: '#fbbf24',
        blue: '#3b82f6',
        magenta: '#a855f7',
        cyan: '#06b6d4',
        white: '#ffffff',
        brightBlack: '#4b5563',
        brightRed: '#f87171',
        brightGreen: '#22c55e',
        brightYellow: '#fde047',
        brightBlue: '#60a5fa',
        brightMagenta: '#c084fc',
        brightCyan: '#22d3ee',
        brightWhite: '#f9fafb',
      },
      rows: 30,
      cols: 120,
      allowProposedApi: true,
    });

    // Add addons
    const fitAddon = new FitAddon();
    const webLinksAddon = new WebLinksAddon();
    const searchAddon = new SearchAddon();

    term.loadAddon(fitAddon);
    term.loadAddon(webLinksAddon);
    term.loadAddon(searchAddon);

    // Open terminal
    term.open(terminalRef.current);
    fitAddon.fit();

    xtermRef.current = term;
    fitAddonRef.current = fitAddon;

    // Welcome message
    term.writeln('\x1b[32m╔══════════════════════════════════════════════════════════════╗\x1b[0m');
    term.writeln('\x1b[32m║         NAVΛ Studio IDE - Integrated Terminal              ║\x1b[0m');
    term.writeln('\x1b[32m║    Van Laarhoven Navigation Calculus Development           ║\x1b[0m');
    term.writeln('\x1b[32m╚══════════════════════════════════════════════════════════════╝\x1b[0m');
    term.writeln('');
    term.writeln('\x1b[36m⋋ Welcome to the NAVΛ Terminal!\x1b[0m');
    term.writeln('\x1b[90mType \x1b[33mhelp\x1b[90m for general commands\x1b[0m');
    term.writeln('\x1b[90mType \x1b[33mnav-help\x1b[90m for NAVΛ navigation commands\x1b[0m');
    term.writeln('\x1b[90mType \x1b[33msymbols\x1b[90m to see all ⋋ navigation symbols\x1b[0m');
    term.writeln('');

    writePrompt(term, currentDirectory);

    // Handle input
    let currentInput = '';
    let cursorPosition = 0;

    term.onKey(({ key, domEvent }) => {
      const printable = !domEvent.altKey && !domEvent.ctrlKey && !domEvent.metaKey;

      if (domEvent.keyCode === 13) {
        // Enter
        term.write('\r\n');
        if (currentInput.trim()) {
          commandHistoryRef.current.push(currentInput);
          historyIndexRef.current = commandHistoryRef.current.length;
          executeCommand(term, currentInput, currentDirectory, setCurrentDirectory);
        } else {
          writePrompt(term, currentDirectory);
        }
        currentInput = '';
        cursorPosition = 0;
        currentInputRef.current = '';
      } else if (domEvent.keyCode === 8) {
        // Backspace
        if (cursorPosition > 0) {
          currentInput = currentInput.slice(0, cursorPosition - 1) + currentInput.slice(cursorPosition);
          cursorPosition--;
          term.write('\b \b');
          // Redraw rest of line
          if (cursorPosition < currentInput.length) {
            term.write(currentInput.slice(cursorPosition) + ' ');
            for (let i = 0; i <= currentInput.length - cursorPosition; i++) {
              term.write('\b');
            }
          }
          currentInputRef.current = currentInput;
        }
      } else if (domEvent.keyCode === 38) {
        // Arrow Up - History
        if (historyIndexRef.current > 0) {
          // Clear current input
          for (let i = 0; i < currentInput.length; i++) {
            term.write('\b \b');
          }
          historyIndexRef.current--;
          currentInput = commandHistoryRef.current[historyIndexRef.current];
          cursorPosition = currentInput.length;
          term.write(currentInput);
          currentInputRef.current = currentInput;
        }
      } else if (domEvent.keyCode === 40) {
        // Arrow Down - History
        if (historyIndexRef.current < commandHistoryRef.current.length) {
          // Clear current input
          for (let i = 0; i < currentInput.length; i++) {
            term.write('\b \b');
          }
          historyIndexRef.current++;
          if (historyIndexRef.current === commandHistoryRef.current.length) {
            currentInput = '';
          } else {
            currentInput = commandHistoryRef.current[historyIndexRef.current];
          }
          cursorPosition = currentInput.length;
          term.write(currentInput);
          currentInputRef.current = currentInput;
        }
      } else if (domEvent.keyCode === 37) {
        // Arrow Left
        if (cursorPosition > 0) {
          cursorPosition--;
          term.write('\x1b[D');
        }
      } else if (domEvent.keyCode === 39) {
        // Arrow Right
        if (cursorPosition < currentInput.length) {
          cursorPosition++;
          term.write('\x1b[C');
        }
      } else if (domEvent.keyCode === 9) {
        // Tab - Auto-complete (basic)
        domEvent.preventDefault();
        // TODO: Implement auto-complete
      } else if (domEvent.keyCode === 67 && domEvent.ctrlKey) {
        // Ctrl+C
        term.write('^C\r\n');
        writePrompt(term, currentDirectory);
        currentInput = '';
        cursorPosition = 0;
        currentInputRef.current = '';
      } else if (domEvent.keyCode === 76 && domEvent.ctrlKey) {
        // Ctrl+L - Clear screen
        term.clear();
        writePrompt(term, currentDirectory);
        currentInput = '';
        cursorPosition = 0;
        currentInputRef.current = '';
      } else if (printable) {
        currentInput = currentInput.slice(0, cursorPosition) + key + currentInput.slice(cursorPosition);
        cursorPosition++;
        term.write(key);
        // Redraw rest of line
        if (cursorPosition < currentInput.length) {
          term.write(currentInput.slice(cursorPosition));
          for (let i = 0; i < currentInput.length - cursorPosition; i++) {
            term.write('\b');
          }
        }
        currentInputRef.current = currentInput;
      }
    });

    // Handle resize
    const handleResize = () => {
      fitAddon.fit();
    };

    window.addEventListener('resize', handleResize);
    setTerminalReady(true);

    return () => {
      window.removeEventListener('resize', handleResize);
      term.dispose();
    };
  }, [currentDirectory]);

  return (
    <div className="terminal-container">
      <div className="terminal-header">
        <div className="terminal-title">
          <span className="terminal-icon">▶</span>
          <span>Terminal</span>
          <span className="terminal-directory">{currentDirectory}</span>
        </div>
        <div className="terminal-actions">
          <button className="terminal-action-btn" title="New Terminal">
            <span>＋</span>
          </button>
          <button className="terminal-action-btn" title="Split Terminal">
            <span>⬌</span>
          </button>
          <button className="terminal-action-btn" title="Clear Terminal">
            <span>🗑️</span>
          </button>
          {onClose && (
            <button className="terminal-action-btn" onClick={onClose} title="Close Terminal">
              <span>×</span>
            </button>
          )}
        </div>
      </div>
      <div ref={terminalRef} className="terminal-content" />
    </div>
  );
};

function writePrompt(term: XTerm, directory: string) {
  term.write(`\x1b[32m⋋\x1b[0m \x1b[36m${directory}\x1b[0m \x1b[33m$\x1b[0m `);
}

async function executeCommand(
  term: XTerm,
  command: string,
  currentDirectory: string,
  setCurrentDirectory: (dir: string) => void
) {
  const parts = command.trim().split(' ');
  const cmd = parts[0];
  const args = parts.slice(1);

  switch (cmd) {
    case 'help':
      term.writeln('\x1b[36mGeneral Commands:\x1b[0m');
      term.writeln('');
      term.writeln('  \x1b[33mhelp\x1b[0m             - Show this help message');
      term.writeln('  \x1b[33mnav-help\x1b[0m         - Show NAVΛ navigation commands');
      term.writeln('  \x1b[33mclear\x1b[0m            - Clear the terminal');
      term.writeln('  \x1b[33mecho\x1b[0m <text>      - Print text');
      term.writeln('  \x1b[33mpwd\x1b[0m              - Print working directory');
      term.writeln('  \x1b[33mcd\x1b[0m <dir>         - Change directory');
      term.writeln('  \x1b[33mls\x1b[0m               - List directory contents');
      term.writeln('  \x1b[33mnpm\x1b[0m <command>    - Run npm commands');
      term.writeln('  \x1b[33mnode\x1b[0m <file>       - Run Node.js scripts');
      term.writeln('  \x1b[33mpython\x1b[0m <file>     - Run Python scripts');
      term.writeln('  \x1b[33mgit\x1b[0m <command>    - Git version control');
      term.writeln('  \x1b[33msystem-info\x1b[0m      - Show system information');
      term.writeln('');
      term.writeln('\x1b[36mNAVΛ Quick Commands:\x1b[0m');
      term.writeln('  \x1b[33mnavλ\x1b[0m             - NAVΛ language commands');
      term.writeln('  \x1b[33mcompile⋋\x1b[0m          - Compile NAVΛ code');
      term.writeln('  \x1b[33mrun⋋\x1b[0m              - Run NAVΛ program');
      term.writeln('  \x1b[33mnavigate⋋\x1b[0m         - Navigation calculator');
      term.writeln('  \x1b[33msymbols\x1b[0m          - Show ⋋ navigation symbols');
      term.writeln('  \x1b[33mexamples\x1b[0m         - Show NAVΛ code examples');
      term.writeln('');
      term.writeln('\x1b[90mPress Ctrl+C to cancel command\x1b[0m');
      term.writeln('\x1b[90mPress Ctrl+L to clear screen\x1b[0m');
      term.writeln('\x1b[90mUse ↑/↓ arrows for command history\x1b[0m');
      term.writeln('');
      break;

    case 'nav-help':
      term.writeln('\x1b[36m⋋ NAVΛ Navigation Commands:\x1b[0m');
      term.writeln('');
      term.writeln('\x1b[33m🚀 Language Commands:\x1b[0m');
      term.writeln('  \x1b[32mnavλ version\x1b[0m         - Show NAVΛ version');
      term.writeln('  \x1b[32mnavλ init [name]\x1b[0m     - Initialize project');
      term.writeln('  \x1b[32mnavλ build\x1b[0m           - Build project');
      term.writeln('  \x1b[32mnavλ test\x1b[0m            - Run tests');
      term.writeln('  \x1b[32mnavλ repl\x1b[0m            - Start REPL');
      term.writeln('');
      term.writeln('\x1b[33m🔨 Compilation:\x1b[0m');
      term.writeln('  \x1b[32mcompile⋋ [file]\x1b[0m      - Compile NAVΛ code');
      term.writeln('  \x1b[32mrun⋋ [file]\x1b[0m          - Run NAVΛ program');
      term.writeln('');
      term.writeln('\x1b[33m🧭 Navigation:\x1b[0m');
      term.writeln('  \x1b[32mnavigate⋋ x1 y1 z1 x2 y2 z2\x1b[0m  - Calculate path');
      term.writeln('  \x1b[32moptimal-path⋋\x1b[0m        - Find optimal path');
      term.writeln('  \x1b[32menergy⋋\x1b[0m              - Calculate energy');
      term.writeln('  \x1b[32mposition⋋\x1b[0m            - Get position');
      term.writeln('');
      term.writeln('\x1b[33m📐 Symbols & Math:\x1b[0m');
      term.writeln('  \x1b[32msymbols\x1b[0m              - Show all ⋋ symbols');
      term.writeln('  \x1b[32mvnc\x1b[0m                  - Van Laarhoven Calculus');
      term.writeln('  \x1b[32m3d\x1b[0m                   - 3D visualization');
      term.writeln('  \x1b[32m7d\x1b[0m                   - 7D spacetime');
      term.writeln('');
      term.writeln('\x1b[33m📚 Learning:\x1b[0m');
      term.writeln('  \x1b[32mexamples\x1b[0m             - NAVΛ code examples');
      term.writeln('');
      term.writeln('\x1b[90mType NAVΛ code directly to execute!\x1b[0m');
      term.writeln('');
      break;

    case 'clear':
      term.clear();
      return;

    case 'echo':
      term.writeln(args.join(' '));
      break;

    case 'pwd':
      term.writeln(currentDirectory);
      break;

    case 'cd':
      if (args.length === 0) {
        setCurrentDirectory('~');
      } else {
        // Simulate directory change
        const newDir = args[0] === '..' ? currentDirectory.split('/').slice(0, -1).join('/') || '/' : args[0];
        setCurrentDirectory(newDir);
      }
      break;

    case 'ls':
      term.writeln('\x1b[36msrc/\x1b[0m          \x1b[36mpublic/\x1b[0m       package.json');
      term.writeln('\x1b[36mnode_modules/\x1b[0m tsconfig.json  README.md');
      term.writeln('\x1b[36mdist/\x1b[0m         vite.config.ts index.html');
      break;

    case 'npm':
      term.writeln(`\x1b[90m$ npm ${args.join(' ')}\x1b[0m`);
      term.writeln('\x1b[33mExecuting npm command...\x1b[0m');
      // Simulate npm execution
      await simulateDelay(1000);
      if (args[0] === 'install') {
        term.writeln('\x1b[32m✓\x1b[0m Package installed successfully');
      } else if (args[0] === 'run') {
        term.writeln('\x1b[32m✓\x1b[0m Script executed successfully');
      } else {
        term.writeln('\x1b[32m✓\x1b[0m Command completed');
      }
      break;

    case 'git':
      term.writeln(`\x1b[90m$ git ${args.join(' ')}\x1b[0m`);
      if (args[0] === 'status') {
        term.writeln('\x1b[32mOn branch main\x1b[0m');
        term.writeln('Your branch is up to date with \'origin/main\'.');
        term.writeln('');
        term.writeln('nothing to commit, working tree clean');
      } else if (args[0] === 'log') {
        term.writeln('\x1b[33mcommit abc123def456\x1b[0m');
        term.writeln('Author: NAVΛ Studio <dev@navlambda.studio>');
        term.writeln('Date:   ' + new Date().toDateString());
        term.writeln('');
        term.writeln('    Latest updates to VNC compiler');
      } else {
        term.writeln('\x1b[32m✓\x1b[0m Git command executed');
      }
      break;

    case 'compile':
      term.writeln('\x1b[36m🔨 Compiling VNC code...\x1b[0m');
      await simulateDelay(1500);
      term.writeln('\x1b[32m✓ Compilation successful\x1b[0m');
      term.writeln('\x1b[90m  Output: dist/main.wasm\x1b[0m');
      break;

    case 'run':
      term.writeln('\x1b[36m▶ Running project...\x1b[0m');
      await simulateDelay(1000);
      term.writeln('\x1b[32m✓ Project started successfully\x1b[0m');
      term.writeln('\x1b[90m  Server running at http://localhost:5173\x1b[0m');
      break;

    case 'test':
      term.writeln('\x1b[36m🧪 Running tests...\x1b[0m');
      await simulateDelay(2000);
      term.writeln('\x1b[32m✓ All tests passed (12/12)\x1b[0m');
      break;

    case 'build':
      term.writeln('\x1b[36m🏗️  Building project...\x1b[0m');
      await simulateDelay(3000);
      term.writeln('\x1b[32m✓ Build completed successfully\x1b[0m');
      term.writeln('\x1b[90m  Output directory: dist/\x1b[0m');
      break;

    case 'system-info':
      term.writeln('\x1b[36mSystem Information:\x1b[0m');
      term.writeln('');
      term.writeln(`  OS: ${navigator.platform}`);
      term.writeln(`  Browser: ${navigator.userAgent.split(' ').pop()}`);
      term.writeln(`  Cores: ${navigator.hardwareConcurrency}`);
      term.writeln(`  Memory: ${(performance as any).memory?.jsHeapSizeLimit ? Math.round((performance as any).memory.jsHeapSizeLimit / 1024 / 1024) + ' MB' : 'Unknown'}`);
      term.writeln(`  Screen: ${window.screen.width}x${window.screen.height}`);
      term.writeln('');
      break;

    case 'node':
    case 'python':
    case 'cargo':
    case 'rustc':
      term.writeln(`\x1b[90m$ ${command}\x1b[0m`);
      term.writeln(`\x1b[33mExecuting ${cmd} command...\x1b[0m`);
      await simulateDelay(1000);
      term.writeln('\x1b[32m✓ Command completed\x1b[0m');
      break;

    // NAVΛ Navigation Commands
    case 'navλ':
    case 'navlambda':
      if (args.length === 0 || args[0] === 'help') {
        term.writeln('\x1b[36m⋋ NAVΛ Language Commands:\x1b[0m');
        term.writeln('  navλ version    - Show version');
        term.writeln('  navλ init       - Initialize project');
        term.writeln('  navλ build      - Build project');
        term.writeln('  navλ test       - Run tests');
        term.writeln('  navλ repl       - Start REPL');
      } else if (args[0] === 'version') {
        term.writeln('\x1b[32m⋋-language NAVΛ version 1.0\x1b[0m');
        term.writeln('Van Laarhoven Navigation Calculus');
        term.writeln('Frank Van Laarhoven © 2025');
      } else if (args[0] === 'init') {
        const name = args[1] || 'nav-project';
        term.writeln(`\x1b[32m🚀 Initializing NAVΛ Project: ${name}\x1b[0m`);
        await simulateDelay(500);
        term.writeln('  ✓ Created project structure');
        term.writeln('  ✓ Generated nav.config.json');
        term.writeln('\x1b[32m  Project ready!\x1b[0m');
      } else if (args[0] === 'build') {
        term.writeln('\x1b[36m🏗️  Building NAVΛ Project...\x1b[0m');
        await simulateDelay(1500);
        term.writeln('  ⋋ Compiling navigation modules');
        term.writeln('  ✓ main.nav → main.wasm');
        term.writeln('\x1b[32m  Build successful!\x1b[0m');
      } else if (args[0] === 'test') {
        term.writeln('\x1b[36m🧪 Running NAVΛ Tests...\x1b[0m');
        await simulateDelay(1000);
        term.writeln('  test_navigation ... \x1b[32m✓\x1b[0m');
        term.writeln('  test_optimal_path ... \x1b[32m✓\x1b[0m');
        term.writeln('  test_energy ... \x1b[32m✓\x1b[0m');
        term.writeln('\x1b[32m  All tests passed!\x1b[0m');
      }
      break;

    case 'compile⋋':
      term.writeln('\x1b[36m🔨 Compiling NAVΛ code...\x1b[0m');
      await simulateDelay(1500);
      term.writeln('  ⋋ Parsing navigation symbols');
      term.writeln('  ✓ Van Laarhoven Calculus verified');
      term.writeln('  ✓ 3D space calculations optimized');
      term.writeln('\x1b[32m✓ Compilation successful\x1b[0m');
      term.writeln('\x1b[90m  Output: dist/main.wasm\x1b[0m');
      break;

    case 'run⋋':
      term.writeln('\x1b[36m▶ Running NAVΛ program...\x1b[0m');
      await simulateDelay(1000);
      term.writeln('  ⋋ Navigation system initialized');
      term.writeln('  ✓ Position: (0.0, 0.0, 0.0)');
      term.writeln('\x1b[32m  Program complete!\x1b[0m');
      break;

    case 'navigate⋋':
      if (args.length < 6) {
        term.writeln('\x1b[31mUsage: navigate⋋ x1 y1 z1 x2 y2 z2\x1b[0m');
      } else {
        term.writeln('\x1b[36m🧭 Calculating navigation path...\x1b[0m');
        await simulateDelay(800);
        const x1 = args[0], y1 = args[1], z1 = args[2];
        const x2 = args[3], y2 = args[4], z2 = args[5];
        const distance = Math.sqrt(
          Math.pow(parseFloat(x2) - parseFloat(x1), 2) +
          Math.pow(parseFloat(y2) - parseFloat(y1), 2) +
          Math.pow(parseFloat(z2) - parseFloat(z1), 2)
        ).toFixed(3);
        term.writeln(`  Start: (${x1}, ${y1}, ${z1})`);
        term.writeln(`  Goal:  (${x2}, ${y2}, ${z2})`);
        term.writeln(`  Distance: ${distance}⋋`);
        term.writeln(`  Energy: ${(parseFloat(distance) * 1.234).toFixed(3)}⋋`);
        term.writeln('\x1b[32m  ✓ Optimal path calculated!\x1b[0m');
      }
      break;

    case 'symbols':
      term.writeln('\x1b[36m📐 NAVΛ Navigation Symbols:\x1b[0m');
      term.writeln('');
      term.writeln('  \x1b[32m⋋\x1b[0m   - Lambda-Nav (primary operator)');
      term.writeln('  \x1b[32m⊗⋋\x1b[0m  - Navigation tensor product');
      term.writeln('  \x1b[32m⊕⋋\x1b[0m  - Navigation addition');
      term.writeln('  \x1b[32m⊘⋋\x1b[0m  - Navigation division');
      term.writeln('  \x1b[32m⊖⋋\x1b[0m  - Navigation subtraction');
      term.writeln('  \x1b[32m∪⋋\x1b[0m  - Navigation union');
      term.writeln('  \x1b[32m∩⋋\x1b[0m  - Navigation intersection');
      term.writeln('  \x1b[32m→⋋\x1b[0m  - East navigation');
      term.writeln('  \x1b[32m←⋋\x1b[0m  - West navigation');
      term.writeln('  \x1b[32m↑⋋\x1b[0m  - North navigation');
      term.writeln('  \x1b[32m↓⋋\x1b[0m  - South navigation');
      term.writeln('  \x1b[32mλ\x1b[0m   - Lambda function');
      term.writeln('  \x1b[32mℕ⋋\x1b[0m  - Navigation natural numbers');
      term.writeln('');
      term.writeln('\x1b[90mKeyboard: Alt+L → ⋋, Alt+T → ⊗, Alt+S → ⊕\x1b[0m');
      break;

    case 'vnc':
      if (args[0] === 'gradient') {
        term.writeln('\x1b[36m🔬 VNC Energy Gradient:\x1b[0m');
        term.writeln(`  ∇E = (${Math.random().toFixed(3)}, ${Math.random().toFixed(3)}, ${Math.random().toFixed(3)})`);
      } else if (args[0] === 'energy') {
        term.writeln('\x1b[36m⚡ Navigation Energy:\x1b[0m');
        term.writeln(`  Total: ${(Math.random() * 100).toFixed(2)}⋋`);
        term.writeln(`  Kinetic: ${(Math.random() * 50).toFixed(2)}⋋`);
        term.writeln(`  Potential: ${(Math.random() * 50).toFixed(2)}⋋`);
      } else {
        term.writeln('\x1b[36mVan Laarhoven Navigation Calculus:\x1b[0m');
        term.writeln('  vnc gradient  - Energy gradient');
        term.writeln('  vnc energy    - Energy calculation');
        term.writeln('  vnc geodesic  - Geodesic path');
      }
      break;

    case '3d':
    case 'visualize⋋':
      term.writeln('\x1b[36m🎨 3D Visualization:\x1b[0m');
      await simulateDelay(500);
      term.writeln('  ✓ WebGL 2.0 initialized');
      term.writeln('  ✓ Navigation mesh loaded');
      term.writeln('  ✓ Path rendered');
      term.writeln('\x1b[32m  3D view ready!\x1b[0m');
      break;

    case '7d':
    case 'spacetime⋋':
      term.writeln('\x1b[36m🌌 7D Spacetime Navigation:\x1b[0m');
      term.writeln('  Dimensions: X, Y, Z, T, G, I, C');
      term.writeln(`  Point7D⋋: (${Math.random().toFixed(2)}, ${Math.random().toFixed(2)}, ${Math.random().toFixed(2)}, ${Math.random().toFixed(2)}, ${Math.random().toFixed(2)}, ${Math.random().toFixed(2)}, ${Math.random().toFixed(2)})`);
      term.writeln('  ✓ Van Laarhoven 7D metric verified');
      break;

    case 'optimal-path⋋':
    case 'find-path⋋':
      term.writeln('\x1b[36m🎯 Finding optimal path...\x1b[0m');
      await simulateDelay(1000);
      term.writeln('  Algorithm: A* with VNC');
      term.writeln(`  Path length: ${(Math.random() * 100 + 50).toFixed(2)}⋋`);
      term.writeln(`  Energy cost: ${(Math.random() * 50 + 10).toFixed(2)}⋋`);
      term.writeln('\x1b[32m  ✓ Optimal path found!\x1b[0m');
      break;

    case 'energy⋋':
      term.writeln('\x1b[36m⚡ Energy Landscape Analysis:\x1b[0m');
      term.writeln(`  Total Energy: ${(Math.random() * 100).toFixed(3)}⋋`);
      term.writeln(`  Kinetic: ${(Math.random() * 50).toFixed(3)}⋋`);
      term.writeln(`  Potential: ${(Math.random() * 50).toFixed(3)}⋋`);
      term.writeln('  ✓ Energy conservation verified');
      break;

    case 'position⋋':
      term.writeln('\x1b[36m📍 Current Position:\x1b[0m');
      term.writeln('  Position⋋: Vector3D⋋(0.0, 0.0, 0.0)');
      term.writeln('  Dimension: 3D');
      break;

    case 'examples':
      term.writeln('\x1b[36m📚 NAVΛ Code Examples:\x1b[0m');
      term.writeln('');
      term.writeln('\x1b[33m1. Basic Navigation:\x1b[0m');
      term.writeln('   position⋋ ← Vector3D⋋(0.0, 0.0, 0.0)');
      term.writeln('   destination⋋ ← Vector3D⋋(10.0, 5.0, 2.0)');
      term.writeln('   path⋋ ← navigate_to⋋(position⋋, destination⋋)');
      term.writeln('');
      term.writeln('\x1b[33m2. Optimal Path:\x1b[0m');
      term.writeln('   optimal⋋ ← find_optimal_path⋋(start⋋, goal⋋)');
      term.writeln('');
      term.writeln('\x1b[33m3. Energy Calculation:\x1b[0m');
      term.writeln('   energy⋋ ← calculate_energy⋋(path⋋)');
      term.writeln('');
      term.writeln('\x1b[33m4. Testing Framework:\x1b[0m');
      term.writeln('   test_suite NavigationTests:');
      term.writeln('     test "Calculate Distance":');
      term.writeln('       result⋋ ← distance⋋(p1⋋, p2⋋)');
      term.writeln('       assert⋋(result⋋ >⋋ 0⋋)');
      term.writeln('');
      term.writeln('\x1b[90mTry: navλ test or compile⋋\x1b[0m');
      break;

    case 'test-examples':
    case 'testing':
      term.writeln('\x1b[36m🧪 NAVΛ Testing Framework:\x1b[0m');
      term.writeln('');
      term.writeln('\x1b[33mTest Suite Example:\x1b[0m');
      term.writeln('test_suite NavigationTests:');
      term.writeln('  test "Calculate Distance":');
      term.writeln('    p1⋋ ← Vector3D⋋(0.0, 0.0, 0.0)');
      term.writeln('    p2⋋ ← Vector3D⋋(10.0, 10.0, 10.0)');
      term.writeln('    result⋋ ← distance⋋(p1⋋, p2⋋)');
      term.writeln('    assert⋋(result⋋ =⋋ 17.321⋋)');
      term.writeln('');
      term.writeln('  test "Optimal Path Finding":');
      term.writeln('    path⋋ ← find_optimal_path⋋(start⋋, goal⋋)');
      term.writeln('    expect⋋(path⋋.waypoints.length >⋋ 0⋋)');
      term.writeln('');
      term.writeln('\x1b[33mWeb Scraper Test:\x1b[0m');
      term.writeln('test_suite ScraperTests:');
      term.writeln('  test "Fetches Page Title":');
      term.writeln('    result⋋ ← webscrape_title⋋("https://navλ-lang.org")');
      term.writeln('    assert⋋(result⋋ =⋋ "NAVΛ Language Home")');
      term.writeln('');
      term.writeln('\x1b[90mRun: navλ test\x1b[0m');
      break;

    default:
      if (cmd) {
        term.writeln(`\x1b[31mCommand not found: ${cmd}\x1b[0m`);
        term.writeln(`\x1b[90mType 'help' or 'nav-help' for commands\x1b[0m`);
      }
      break;
  }

  term.writeln('');
  writePrompt(term, currentDirectory);
}

function simulateDelay(ms: number): Promise<void> {
  return new Promise(resolve => setTimeout(resolve, ms));
}

