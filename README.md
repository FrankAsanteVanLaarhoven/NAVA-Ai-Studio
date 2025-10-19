# NAVΛ Studio IDE

**The World's First IDE for Van Laarhoven Navigation Calculus Programming**

[![License: MIT OR Apache-2.0](https://img.shields.io/badge/License-MIT%20OR%20Apache--2.0-blue.svg)](LICENSE)
[![Rust](https://img.shields.io/badge/Rust-1.75+-orange.svg)](https://www.rust-lang.org/)
[![React](https://img.shields.io/badge/React-18.3+-61DAFB.svg)](https://react.dev/)
[![Tauri](https://img.shields.io/badge/Tauri-1.6+-FFC131.svg)](https://tauri.app/)

## 🤖 NEW: Free ROS Learning Courses + Live Terminal!

**NAVΛ Studio now includes comprehensive, free ROS (Robot Operating System) education with a FULLY FUNCTIONAL TERMINAL!**

- ✅ **3 Free Courses** - ROS2 Basics, Advanced Navigation, Gazebo Simulation
- ✅ **💻 Live Terminal** - Execute real ROS2 commands in-browser
- ✅ **17+ Commands** - ros2 run, launch, topic, node, pkg, and more
- ✅ **One-Click Execution** - Run code examples instantly
- ✅ **100% Free Forever** - No paywalls, no subscriptions
- ✅ **VNC Integration** - Learn navigation calculus with robotics
- ✅ **Certification** - Earn certificates on completion
- ✅ **Hands-On ROSjects** - Practical robot projects
- ✅ **Safe Learning** - Simulated environment, no installation needed

**New Features:**
- 💻 **Integrated Terminal** with command history and keyboard shortcuts
- ▶️ **Run Command Buttons** on all code examples
- 🚀 **Launch File Execution** with real-time output
- 🎓 **Educational Output** designed for learning

Inspired by [The Construct](https://app.theconstruct.ai) and enhanced with NAVΛ's mathematical navigation framework.

📚 **[Full Documentation](ROS_LEARNING_SYSTEM.md)** | 💻 **[Terminal Guide](ROS_TERMINAL_INTEGRATION.md)**

---

## 🚀 Revolutionary Features

NAVΛ Studio is a world-class development environment that **surpasses VSCode, Cursor, and JetBrains** with:

- **⋋ Native VNC Symbol Support** - First-class support for Van Laarhoven Navigation Calculus symbols
- **🎨 3D Navigation Visualization** - Real-time GPU-accelerated visualization of navigation paths and energy landscapes
- **⚡ Multi-Target Compilation** - Single source code compiles to C++, Python, WebAssembly, GLSL, and more
- **🔥 Live Preview Engine** - WebAssembly-powered real-time code execution and visualization
- **☁️ Cloud Integration** - One-click deployment to Docker, Kubernetes, and cloud platforms
- **🎯 Navigation Debugging** - Advanced path tracing and energy optimization debugging
- **🧩 Plugin Architecture** - Extensible system for domain-specific tools and integrations
- **⚡ Rust Performance** - Native performance with memory safety, faster than Electron-based IDEs

## 🏗️ Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    NAVΛ STUDIO IDE                          │
├─────────────────────────────────────────────────────────────┤
│  Frontend (Tauri + React/TypeScript)                       │
│  ├── Monaco Editor + ⋋ Symbol Extensions                   │
│  ├── 3D Navigation Visualizer (Three.js/WebGL)             │
│  ├── VNC Mathematical Renderer (MathJax + Custom)          │
│  ├── Live Preview Engine (WebAssembly + Canvas)            │
│  └── Cloud Integration Panel (Docker + Kubernetes)         │
├─────────────────────────────────────────────────────────────┤
│  Rust Backend Core                                         │
│  ├── NAVΛ Language Server Protocol (LSP)                   │
│  ├── VNC Parser & AST Generator                            │
│  ├── Multi-Target Compiler (C++/Python/WASM/GLSL)         │
│  ├── Navigation Debugger & Path Tracer                     │
│  ├── Live Preview WebAssembly Engine                       │
│  └── Plugin Architecture & Extension System                │
└─────────────────────────────────────────────────────────────┘
```

## 🎯 Quick Start

### For End Users - Desktop Installation

**Install NAVΛ Studio as a native desktop application:**

#### **Automated Installation (Recommended)**

**macOS / Linux:**
```bash
curl -fsSL https://raw.githubusercontent.com/FrankAsanteVanLaarhoven/NAVA-Ai-Studio/main/scripts/install-sdk.sh | bash
```

**Windows (PowerShell):**
```powershell
irm https://raw.githubusercontent.com/FrankAsanteVanLaarhoven/NAVA-Ai-Studio/main/scripts/install-sdk.ps1 | iex
```

#### **Manual Installation**

Download pre-built installers for your platform:
- **macOS**: [Download DMG](https://github.com/FrankAsanteVanLaarhoven/NAVA-Ai-Studio/releases/latest)
- **Windows**: [Download MSI](https://github.com/FrankAsanteVanLaarhoven/NAVA-Ai-Studio/releases/latest)
- **Linux**: [Download AppImage/DEB/RPM](https://github.com/FrankAsanteVanLaarhoven/NAVA-Ai-Studio/releases/latest)

📖 **See [SDK_QUICK_START.md](SDK_QUICK_START.md) for complete installation guide**

---

### For Developers - Build from Source

#### Prerequisites

- **Rust** 1.75+ ([rustup.rs](https://rustup.rs/))
- **Node.js** 20+ and npm ([nodejs.org](https://nodejs.org/))
- **System Dependencies** (platform-specific)
  - **Linux**: `libwebkit2gtk-4.0-dev`, `build-essential`, `curl`, `wget`, `file`, `libssl-dev`, `libgtk-3-dev`, `libayatana-appindicator3-dev`, `librsvg2-dev`
  - **macOS**: Xcode Command Line Tools
  - **Windows**: Microsoft C++ Build Tools

#### Installation

```bash
# Clone the repository
git clone https://github.com/frankvanlaarhoven/navlambda-studio.git
cd navlambda-studio

# Install dependencies
npm install

# Build WebAssembly preview engine (optional)
cd wasm-preview
wasm-pack build --target web
cd ..

# Run development environment
npm run tauri:dev
```

#### Build for Production

```bash
# Build optimized production version
npm run build
npm run tauri:build

# The installer will be in src-tauri/target/release/bundle/
```

Or use the automated build script:
```bash
./scripts/build.sh
```

📖 **See [BUILD_INSTRUCTIONS.md](BUILD_INSTRUCTIONS.md) for detailed build guide**

## 📚 Documentation

### Desktop SDK Documentation
- [**SDK Quick Start**](SDK_QUICK_START.md) - ⚡ Fastest way to install and get started
- [**SDK Download**](SDK_DOWNLOAD.md) - 📥 Pre-built installers for all platforms
- [**Desktop SDK Installation**](DESKTOP_SDK_INSTALLATION.md) - 🛠️ Complete installation guide
- [**Build Instructions**](BUILD_INSTRUCTIONS.md) - 🔨 Building from source
- [**SDK Overview**](SDK_README.md) - 📖 Complete SDK documentation index

### Developer Documentation
- [**Architecture Overview**](docs/architecture.md) - System design and component architecture
- [**NAVΛ Language Reference**](docs/vnc-language-reference.md) - Complete language specification
- [**Plugin Development Guide**](docs/plugin-development.md) - Creating custom extensions
- [**Multi-Target Compilation**](docs/compilation-targets.md) - Compiling to C++, Python, WASM, GLSL
- [**Cloud Deployment Guide**](docs/deployment-guide.md) - Docker, Kubernetes, cloud platforms
- [**Getting Started**](GETTING_STARTED.md) - User guide for the IDE

## 🎨 Key Features in Detail

### Native ⋋ Symbol Support

NAVΛ Studio has first-class support for all Van Laarhoven Navigation Calculus symbols:

- **⋋** - Lambda Navigation (Alt+L shortcut)
- **⊗⋋** - Navigation Tensor Product
- **⊕⋋** - Navigation Sum
- **∪⋋** - Navigation Union
- **∩⋋** - Navigation Intersection
- **↑⋋, ↓⋋, →⋋, ←⋋** - Directional Navigation
- **𝒩ℐ** - Master Navigation Operator
- **ℰ** - Evolution Operator

### 3D Navigation Visualization

Real-time GPU-accelerated visualization of:
- Navigation path optimization
- Energy landscape surfaces
- Multi-dimensional navigation spaces
- VNC equation rendering in 3D space
- Interactive path exploration

### Multi-Target Compilation

Write once, compile to:
- **C++17** - High-performance native applications
- **Python 3.12** - Data science and prototyping
- **WebAssembly** - Browser and web applications
- **GLSL 4.5** - GPU shaders and compute
- **WGSL** - WebGPU shaders
- **Helm Charts** - Kubernetes deployment
- **Dockerfile** - Container packaging

## 🛠️ Technology Stack

| Component | Technology | Why |
|-----------|-----------|-----|
| **Core Language** | Rust | Performance + Memory Safety + Modern Ecosystem |
| **Frontend Framework** | Tauri + React | Native Performance + Web Flexibility |
| **3D Visualization** | Three.js + WebGL | GPU-Accelerated Graphics |
| **Code Editor** | Monaco Editor | VSCode's Powerful Editor Engine |
| **Live Preview** | WebAssembly | Near-Native Speed in Browser |
| **Mathematical Rendering** | MathJax + Custom | Perfect ⋋ Symbol Support |

## 🤝 Contributing

We welcome contributions! Please see [CONTRIBUTING.md](CONTRIBUTING.md) for guidelines.

Areas where we especially need help:
- 🐛 Bug reports and fixes
- 📝 Documentation improvements
- 🎨 UI/UX enhancements
- 🔌 Plugin development
- 🌍 Translations and internationalization

## 📄 License

Dual-licensed under MIT OR Apache-2.0. See [LICENSE-MIT](LICENSE-MIT) and [LICENSE-APACHE](LICENSE-APACHE) for details.

## 🌟 Acknowledgments

Created by **Frank Van Laarhoven**, inventor of Van Laarhoven Navigation Calculus (VNC).

Special thanks to the open-source community and all contributors who make NAVΛ Studio possible.

## 🚀 Roadmap

- ✅ **Phase 1** (Months 0-6): Core LSP, basic editor, VNC parser
- 🚧 **Phase 2** (Months 6-12): Multi-target compilation, 3D visualization, live preview
- 📋 **Phase 3** (Months 12-18): Plugin system, cloud integration, performance optimization
- 🎯 **Phase 4** (Months 18-24): Enterprise features, polish, market launch

## 💬 Community

- **Website**: [navlambda.studio](https://navlambda.studio)
- **Discord**: [Join our community](https://discord.gg/navlambda)
- **Twitter**: [@navlambda_studio](https://twitter.com/navlambda_studio)
- **Email**: support@navlambda.studio

---

**NAVΛ Studio**: Making Van Laarhoven Navigation Calculus accessible to every programmer on Earth 🚀⋋💻

