# NAVΛ Studio Desktop SDK

Welcome to the NAVΛ Studio Desktop SDK! This comprehensive guide helps you install, build, and distribute the world's first IDE for Van Laarhoven Navigation Calculus.

---

## 📋 What is NAVΛ Studio?

**NAVΛ Studio IDE** is a revolutionary development environment designed specifically for Van Laarhoven Navigation Calculus (VNC) programming. It features:

✨ **Native ⋋ Symbol Support** - First-class mathematical notation  
🎨 **3D Navigation Visualization** - See your navigation paths in real-time  
⚡ **Multi-Target Compilation** - Compile to C++, Python, GLSL, WASM, and more  
🧠 **Intelligent Language Server** - Advanced IntelliSense and code completion  
🌍 **Cross-Platform** - Native apps for macOS, Windows, and Linux  
☁️ **Cloud Integration** - Optional cloud deployment and sync  

---

## 🚀 Quick Installation

### For End Users (Install Pre-built App)

Choose your platform and run:

#### **macOS / Linux:**
```bash
curl -fsSL https://raw.githubusercontent.com/FrankAsanteVanLaarhoven/NAVA-Ai-Studio/main/scripts/install-sdk.sh | bash
```

#### **Windows (PowerShell):**
```powershell
irm https://raw.githubusercontent.com/FrankAsanteVanLaarhoven/NAVA-Ai-Studio/main/scripts/install-sdk.ps1 | iex
```

### For Developers (Build from Source)

```bash
git clone https://github.com/FrankAsanteVanLaarhoven/NAVA-Ai-Studio.git
cd NAVA-Ai-Studio
npm install
./scripts/build.sh
```

---

## 📚 Documentation Structure

This SDK includes comprehensive documentation:

| Document | Purpose |
|----------|---------|
| **SDK_QUICK_START.md** | ⚡ Fastest way to get started |
| **SDK_DOWNLOAD.md** | 📥 Download links and installers |
| **DESKTOP_SDK_INSTALLATION.md** | 🛠️ Detailed installation guide |
| **BUILD_INSTRUCTIONS.md** | 🔨 Complete build documentation |
| **GETTING_STARTED.md** | 🎓 User guide for the IDE |
| **docs/** | 📖 Full technical documentation |

### Documentation Quick Links

- **New User?** Start with [`SDK_QUICK_START.md`](SDK_QUICK_START.md)
- **Want to Install?** See [`SDK_DOWNLOAD.md`](SDK_DOWNLOAD.md)
- **Building from Source?** Read [`BUILD_INSTRUCTIONS.md`](BUILD_INSTRUCTIONS.md)
- **Learning VNC?** Check [`docs/vnc-language-reference.md`](docs/vnc-language-reference.md)
- **Need Help?** Visit [`GETTING_STARTED.md`](GETTING_STARTED.md)

---

## 🎯 Use Cases

### 1️⃣ **End User Installation**
You just want to use NAVΛ Studio:
- ✅ Run the installation script
- ✅ Download pre-built installer
- ✅ See [`SDK_DOWNLOAD.md`](SDK_DOWNLOAD.md)

### 2️⃣ **Developer Building from Source**
You want to customize or contribute:
- ✅ Clone the repository
- ✅ Follow [`BUILD_INSTRUCTIONS.md`](BUILD_INSTRUCTIONS.md)
- ✅ Use development mode with `npm run tauri:dev`

### 3️⃣ **Distribution / Deployment**
You want to deploy to your team/organization:
- ✅ Build installers for all platforms
- ✅ Use `./scripts/create-sdk-package.sh`
- ✅ See [`DESKTOP_SDK_INSTALLATION.md`](DESKTOP_SDK_INSTALLATION.md)

### 4️⃣ **Plugin Development**
You want to extend NAVΛ Studio:
- ✅ Read [`docs/plugin-development.md`](docs/plugin-development.md)
- ✅ Explore the plugin system
- ✅ Create custom visualizations and compilers

---

## 🛠️ System Requirements

### Minimum
- **OS**: macOS 10.15+ / Windows 10+ / Ubuntu 20.04+
- **RAM**: 4 GB
- **Storage**: 500 MB
- **CPU**: Dual-core processor

### Recommended
- **OS**: macOS 13+ / Windows 11 / Ubuntu 22.04+
- **RAM**: 8 GB+
- **Storage**: 2 GB
- **CPU**: Quad-core processor
- **GPU**: Dedicated GPU for 3D visualization

### Build Requirements (Developers Only)
- **Node.js**: 18+
- **npm**: 9+
- **Rust**: Latest stable (1.75+)
- **Platform Tools**:
  - macOS: Xcode Command Line Tools
  - Windows: Visual Studio Build Tools
  - Linux: webkit2gtk, build-essential

---

## 🎨 Features Overview

### Core IDE Features
- 🖥️ **Native Desktop Performance** - No browser overhead
- 📝 **Monaco Editor Integration** - VSCode-quality editing
- 🎯 **Command Palette** - Quick access to all features
- 🔍 **Symbol Search** - Find functions, types, and variables
- 📁 **File Explorer** - Full file system integration
- ⚙️ **Settings Panel** - Customizable preferences

### VNC Language Features
- ⋋ **Native Lambda Symbol** - Type `\lambda` or use palette
- 📊 **Mathematical Notation** - LaTeX-style math rendering
- 🧮 **Equation Editor** - Visual equation building
- ✓ **Theorem Checker** - Verify mathematical proofs
- 📈 **Type System** - Advanced navigation type inference

### Visualization
- 🎮 **3D Navigation Paths** - Interactive WebGL rendering
- 🗺️ **Energy Landscapes** - Visualize optimization surfaces
- 🔄 **Live Preview** - See changes in real-time
- 📹 **Path Animation** - Animate navigation trajectories
- 📊 **Performance Profiler** - Analyze execution

### Compilation Targets
- 🔷 **C++** - High-performance native code
- 🐍 **Python** - NumPy/JAX integration
- 🎨 **GLSL** - GPU shader generation
- 🌐 **WebAssembly** - Browser-ready modules
- ☁️ **Cloud Functions** - Deploy to AWS/Azure/GCP

### Collaboration
- 👥 **Live Collaboration** - Real-time code sharing
- 💬 **Chat Integration** - In-IDE communication
- 🔗 **WebRTC** - Peer-to-peer connections
- 📤 **Cloud Sync** - Sync projects across devices

---

## 📦 Installation Methods Comparison

| Method | Time | Difficulty | Use Case |
|--------|------|------------|----------|
| **Script Install** | 30 sec | ⭐ Easy | End users |
| **Download Installer** | 2 min | ⭐ Easy | Manual install |
| **Build from Source** | 20 min | ⭐⭐⭐ Medium | Developers |
| **Docker** | 5 min | ⭐⭐ Easy | Testing |

---

## 🌟 Platform Support

### macOS ✅
- Intel (x86_64)
- Apple Silicon (ARM64)
- Universal Binary
- DMG installer
- System integration

### Windows ✅
- 64-bit (x86_64)
- MSI installer
- Portable EXE
- Auto-updates
- Start Menu integration

### Linux ✅
- Ubuntu/Debian (.deb)
- Fedora/RHEL (.rpm)
- Universal AppImage
- System tray icon
- Desktop file integration

---

## 🔧 Build Scripts

The SDK includes several build scripts:

| Script | Purpose |
|--------|---------|
| `scripts/build.sh` | Build desktop application |
| `scripts/package.sh` | Create distribution package |
| `scripts/install-sdk.sh` | Automated installation (Linux/Mac) |
| `scripts/install-sdk.ps1` | Automated installation (Windows) |
| `scripts/create-sdk-package.sh` | Create complete SDK bundle |
| `scripts/deploy.sh` | Deploy to cloud services |
| `scripts/test.sh` | Run test suite |

---

## 🚢 Release Process

### 1. Build for All Platforms
```bash
# Run on each platform or use CI/CD
./scripts/build.sh
```

### 2. Create SDK Package
```bash
./scripts/create-sdk-package.sh
```

### 3. Test Installers
Test on each platform before release.

### 4. Tag Release
```bash
git tag -a v1.0.0 -m "Release v1.0.0"
git push origin v1.0.0
```

### 5. Upload to GitHub
```bash
gh release create v1.0.0 <installer-files> \
  --title "NAVΛ Studio v1.0.0" \
  --notes "Release notes"
```

---

## 📖 Learning Path

### New to NAVΛ Studio?
1. ✅ Install using [`SDK_QUICK_START.md`](SDK_QUICK_START.md)
2. ✅ Follow [`GETTING_STARTED.md`](GETTING_STARTED.md)
3. ✅ Try example projects in `assets/examples/`
4. ✅ Read [`docs/vnc-language-reference.md`](docs/vnc-language-reference.md)

### Want to Contribute?
1. ✅ Build from source using [`BUILD_INSTRUCTIONS.md`](BUILD_INSTRUCTIONS.md)
2. ✅ Read [`CONTRIBUTING.md`](CONTRIBUTING.md)
3. ✅ Check [`docs/architecture.md`](docs/architecture.md)
4. ✅ Join discussions on GitHub

### Want to Extend?
1. ✅ Read [`docs/plugin-development.md`](docs/plugin-development.md)
2. ✅ Study [`docs/compilation-targets.md`](docs/compilation-targets.md)
3. ✅ Explore the plugin system in `src/services/plugin-service.ts`

---

## 🐛 Troubleshooting

### Installation Issues
See [`DESKTOP_SDK_INSTALLATION.md`](DESKTOP_SDK_INSTALLATION.md) - Section: Troubleshooting

### Build Issues
See [`BUILD_INSTRUCTIONS.md`](BUILD_INSTRUCTIONS.md) - Section: Troubleshooting

### Runtime Issues
Check [`docs/architecture.md`](docs/architecture.md) for system details

### Get Help
- 📧 Email: support@navlambda.studio
- 💬 GitHub Discussions
- 🐛 GitHub Issues
- 📖 Full docs in `docs/`

---

## 📊 Project Stats

- **Languages**: TypeScript, Rust, VNC
- **Framework**: Tauri (Rust + React)
- **Lines of Code**: ~50,000+
- **Platforms**: macOS, Windows, Linux
- **License**: MIT / Apache-2.0 (dual)

---

## 🎓 Example Projects

Check `assets/examples/` for sample VNC projects:

1. **basic-navigation.vnc** - Introduction to VNC
2. **quantum-navigation.vnc** - Quantum state navigation
3. **energy-landscape.vnc** - Optimization visualization
4. **consciousness-integration.vnc** - Consciousness models
5. **multi-goal-navigation.vnc** - Multiple objectives
6. **real-world-robotics.vnc** - Robotics applications

---

## 🔗 Important Links

- **GitHub Repository**: https://github.com/FrankAsanteVanLaarhoven/NAVA-Ai-Studio
- **Latest Releases**: https://github.com/FrankAsanteVanLaarhoven/NAVA-Ai-Studio/releases
- **Issue Tracker**: https://github.com/FrankAsanteVanLaarhoven/NAVA-Ai-Studio/issues
- **Discussions**: https://github.com/FrankAsanteVanLaarhoven/NAVA-Ai-Studio/discussions
- **Documentation**: Full docs in `docs/` directory

---

## 📝 License

NAVΛ Studio is dual-licensed under:
- **MIT License** - [`LICENSE-MIT`](LICENSE-MIT)
- **Apache License 2.0** - [`LICENSE-APACHE`](LICENSE-APACHE)

You may choose either license for your use case.

---

## 💖 Credits

**Created by Frank Van Laarhoven**

NAVΛ Studio IDE - The world's first IDE for Van Laarhoven Navigation Calculus.

*Making mathematical navigation programming accessible to everyone.* 🚀⋋

---

## 🎯 Quick Command Reference

```bash
# Install (end user)
curl -fsSL https://raw.githubusercontent.com/.../install-sdk.sh | bash

# Build from source
npm install && ./scripts/build.sh

# Development mode
npm run tauri:dev

# Create SDK package
./scripts/create-sdk-package.sh

# Run tests
./scripts/test.sh

# Deploy to cloud
./scripts/deploy.sh
```

---

**Ready to start?** Choose your path:
- 🏃 **Quick Install**: See [`SDK_QUICK_START.md`](SDK_QUICK_START.md)
- 📥 **Download**: See [`SDK_DOWNLOAD.md`](SDK_DOWNLOAD.md)
- 🔨 **Build**: See [`BUILD_INSTRUCTIONS.md`](BUILD_INSTRUCTIONS.md)
- 🎓 **Learn**: See [`GETTING_STARTED.md`](GETTING_STARTED.md)

---

**NAVΛ Studio** - Revolutionizing Navigation Calculus Programming 🚀⋋

