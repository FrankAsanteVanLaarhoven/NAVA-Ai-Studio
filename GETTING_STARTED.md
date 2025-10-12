# Getting Started with NAVΛ Studio

Welcome to **NAVΛ Studio** - the world's first IDE for Van Laarhoven Navigation Calculus programming!

## 🚀 Quick Start (5 Minutes)

### 1. Prerequisites

Before you begin, ensure you have:

- **Rust** 1.75 or higher ([Install Rust](https://rustup.rs/))
- **Node.js** 20 or higher ([Install Node.js](https://nodejs.org/))
- **System Dependencies** (platform-specific, see below)

### 2. Clone and Setup

```bash
# Clone the repository
git clone https://github.com/frankvanlaarhoven/navlambda-studio.git
cd navlambda-studio

# Run the automated setup script
./scripts/setup-dev-environment.sh

# This script will:
# ✓ Install Rust toolchain
# ✓ Install wasm-pack
# ✓ Install Tauri CLI
# ✓ Install npm dependencies
# ✓ Build WebAssembly preview engine
```

### 3. Start Development Server

```bash
npm run tauri:dev
```

That's it! NAVΛ Studio will open with a sample VNC program ready to edit.

## 📚 Your First NAVΛ Program

Let's write a simple navigation program:

```navλ
// Define start and goal positions
let start = [0.0, 0.0, 0.0]
let goal = [10.0, 10.0, 0.0]

// Navigate using VNC optimization
let path = navigate_to⋋(start, goal)

// Visualize the result
visualize⋋(path)
```

### Using VNC Symbols

NAVΛ Studio has built-in keyboard shortcuts for VNC symbols:

| Symbol | Keyboard Shortcut | Name |
|--------|------------------|------|
| ⋋ | `Alt+L` | Lambda Navigation |
| ⊗⋋ | `Alt+Shift+T` | Tensor Product |
| ⊕⋋ | `Alt+Shift+S` | Navigation Sum |
| 𝒩ℐ | `Alt+Shift+M` | Master Operator |

Or click the symbol buttons in the editor toolbar!

### Running Your Program

1. **Click the "Run" button** in the toolbar (or press `F5`)
2. **View the 3D visualization** in the right panel
3. **Compile to your target** using the "Compile" menu

## 🎯 Key Features

### 1. Intelligent Code Completion

As you type, NAVΛ Studio provides:
- VNC symbol suggestions
- Function parameter hints
- Documentation on hover
- Real-time error detection

### 2. 3D Navigation Visualization

Click **"Visualize"** to see:
- Real-time 3D navigation paths
- Energy landscape rendering
- Interactive path exploration
- VNC equations in 3D space

### 3. Multi-Target Compilation

Compile your NAVΛ code to:
- **C++** - High performance applications
- **Python** - Data science and prototyping
- **WebAssembly** - Browser applications
- **GLSL** - GPU-accelerated compute

```bash
# Compile to C++
navλ compile --target cpp your-program.vnc -o output.cpp

# Compile to Python
navλ compile --target python your-program.vnc -o output.py

# Compile to WebAssembly
navλ compile --target wasm your-program.vnc
```

### 4. Live Preview

Changes appear instantly in the visualization as you type!

### 5. Cloud Deployment

One-click deployment to:
- Docker containers
- Kubernetes clusters
- Cloud platforms (AWS, GCP, Azure)

## 📖 Learning Path

### Beginner (Start Here)

1. **Basic Navigation** - `assets/examples/basic-navigation.vnc`
   - Learn fundamental VNC concepts
   - Understand the ⋋ operator
   - Simple path visualization

2. **Energy Landscapes** - `assets/examples/energy-landscape.vnc`
   - Navigate through complex terrains
   - Use the master operator 𝒩ℐ
   - Compare with classical methods

### Intermediate

3. **Multi-Goal Navigation** - `assets/examples/multi-goal-navigation.vnc`
   - Use navigation operators (⊕⋋, ∪⋋, ∩⋋)
   - Parallel path optimization
   - Advanced visualizations

4. **Real-World Robotics** - `assets/examples/real-world-robotics.vnc`
   - Practical robot navigation
   - Obstacle avoidance
   - Real-time re-planning

### Advanced

5. **Quantum Navigation** - `assets/examples/quantum-navigation.vnc`
   - Quantum-inspired algorithms
   - Evolution operator ℰ
   - Probability visualization

6. **Consciousness Integration** - `assets/examples/consciousness-integration.vnc`
   - Consciousness-aware navigation
   - Intent alignment
   - Ethical AI navigation

## 🛠️ Development Workflow

### Typical Session

```bash
# 1. Start development server
npm run tauri:dev

# 2. Make changes to:
#    - Rust backend: src-tauri/src/
#    - Frontend: src/
#    - WASM: wasm-preview/src/

# 3. Changes hot-reload automatically!

# 4. Run tests
./scripts/test.sh

# 5. Build for production
./scripts/build.sh
```

### Project Structure

```
navλ-studio/
├── src/                    # React frontend
│   ├── components/         # UI components
│   ├── services/          # API services
│   └── hooks/             # React hooks
├── src-tauri/             # Rust backend
│   └── src/
│       ├── lsp/           # Language server
│       ├── compiler/      # Multi-target compiler
│       └── debugger/      # Navigation debugger
├── wasm-preview/          # WebAssembly preview
├── docs/                  # Documentation
├── scripts/               # Build scripts
└── assets/                # Examples and icons
```

## 🔧 Configuration

### Editor Settings

Customize NAVΛ Studio in `Settings` → `Editor`:
- Theme (VNC Dark/Light)
- Font size and family
- Symbol shortcuts
- Auto-save preferences

### Compiler Options

Configure compilation in `Settings` → `Compiler`:
- Default target language
- Optimization level
- VNC-specific optimizations
- Output formatting

## 🐛 Troubleshooting

### Issue: Tauri dev server won't start

**Solution**:
```bash
# Clean and rebuild
rm -rf node_modules dist
npm install
npm run tauri:dev
```

### Issue: WASM preview not working

**Solution**:
```bash
# Rebuild WASM module
cd wasm-preview
wasm-pack build --target web
cd ..
npm run tauri:dev
```

### Issue: Compilation errors

**Solution**:
```bash
# Check Rust toolchain
rustc --version
cargo --version

# Update if needed
rustup update
```

### Issue: Missing ⋋ symbols

**Solution**: Ensure you have a font that supports mathematical symbols. NAVΛ Studio includes fallback fonts, but installing "Latin Modern Math" or "STIX Two Math" provides the best experience.

## 📚 Next Steps

- **Read the docs**: Explore `docs/` for in-depth guides
  - [Architecture](docs/architecture.md)
  - [Language Reference](docs/vnc-language-reference.md)
  - [Plugin Development](docs/plugin-development.md)
  - [Multi-Target Compilation](docs/compilation-targets.md)
  - [Deployment Guide](docs/deployment-guide.md)

- **Try examples**: Run all examples in `assets/examples/`

- **Join the community**:
  - Discord: https://discord.gg/navlambda
  - Twitter: @navlambda_studio
  - GitHub Discussions

- **Contribute**: See [CONTRIBUTING.md](CONTRIBUTING.md)

## 🌟 Pro Tips

1. **Use keyboard shortcuts** - Much faster than clicking!
   - `F5` - Run program
   - `Ctrl/Cmd + S` - Save
   - `Alt + L` - Insert ⋋ symbol
   - `Ctrl/Cmd + B` - Compile

2. **Split view** - Toggle visualization panel for side-by-side editing

3. **Live preview** - Leave preview on for instant feedback

4. **Examples as templates** - Copy examples and modify for your needs

5. **Documentation on hover** - Hover over any VNC symbol or function for docs

## 🎓 Learning Resources

- **Video Tutorials**: Coming soon on YouTube
- **Interactive Tutorial**: Built into NAVΛ Studio (Help → Tutorial)
- **VNC Theory**: Read Van Laarhoven's original papers (linked in docs)
- **Community Examples**: Browse shared projects in Discord

## 💬 Getting Help

Stuck? We're here to help:

1. **Check FAQ**: See common questions in docs
2. **Search Issues**: Someone may have asked already
3. **Ask Discord**: Real-time community help
4. **Create Issue**: Detailed bug reports on GitHub
5. **Email Support**: support@navlambda.studio

## 🚀 Build Something Amazing

You now have everything you need to start programming with Van Laarhoven Navigation Calculus!

**Ideas to get started**:
- Robot path planning
- Drone navigation
- Game AI pathfinding
- Logistics optimization
- Neural network training paths
- Quantum algorithm simulation
- Consciousness modeling

The only limit is your imagination! 🌟⋋

---

**Welcome to the future of navigation programming!**

Made with 💚 by the NAVΛ Studio team

