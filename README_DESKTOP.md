# 🚀 NAVΛ Studio IDE - Desktop Installation

**Welcome to your complete, working clone of NAVΛ Studio IDE!**

This is a **production-ready** robotics development platform with all integrations included.

---

## ⚡ Quick Start (30 Seconds)

### Option 1: Double-Click to Start

```bash
./START.sh
```

Then choose:
1. **IDE only** - Basic development environment
2. **Univarm Advanced** - With production Rust backend
3. **ROBOTIS-SYSTEMIC** - Complete enterprise platform
4. **ROS 2 Kobuki** - Robot simulation with RViz

### Option 2: Command Line

```bash
# Just the IDE
npm run dev:vite

# With Univarm backend
./start-univarm-backend.sh
npm run dev:vite

# Complete platform
./start-robotis-system.sh
```

**That's it!** Your browser opens automatically.

---

## 🌟 What's Included

### **3 Univarm Systems**
- 🔷 **ROBOTIS-SYSTEMIC** - Full WebXR robot control
- ⚡ **Univarm Starter** - Code generation & path optimization
- 🦀 **Univarm Advanced** - Production Rust backend with SSE

### **ROS 2 Integration**
- 🤖 **6 ROS 2 Packages** - Kobuki, TB2, Helix, msgs, emitter, bridge
- 📡 **gRPC Path Planning** - C++ emitter with custom services
- 🐍 **Python Drive Bridge** - Backend ↔ ROS 2

### **Complete Infrastructure**
- 🦀 **3 Backend Services** - Rust servers with real solvers
- 📊 **SSE Streaming** - Real-time telemetry
- 🎯 **Multi-Language Codegen** - Rust, C++, Python, TypeScript
- 🛡️ **Trust & Security** - Ed25519, SBOM, attestations

---

## 📂 Your Desktop Installation

**Location**: `/Users/frankvanlaarhoven/Desktop/NAVA Studio IDE`

```
NAVA Studio IDE/
├── START.sh                    # ⭐ Run this to start!
├── README_DESKTOP.md           # This file
├── src/                        # Source code
├── ros2_ws/                    # ROS 2 packages
├── backend-univarm/            # Rust backend
├── start-*.sh                  # Various startup scripts
└── docs/                       # Documentation
```

---

## 🎯 Available Commands

```bash
# Start IDE
npm run dev:vite

# Start Univarm backend
./start-univarm-backend.sh

# Start ROBOTIS platform
./start-robotis-system.sh

# Build ROS 2 packages
./build-ros2.sh

# Launch Kobuki
./start-kobuki-demo.sh

# Verification
./verify-integration.sh

# Stop everything
./stop-robotis-system.sh
```

---

## 🌐 Access Points

When running:

| Service | URL | How |
|---------|-----|-----|
| **Main Workspace** | http://localhost:5175/workspace.html | Opens automatically |
| **Univarm Starter** | Click ⚡ in dock | Path optimization |
| **Univarm Advanced** | Click 🦀 in dock | Production planning |
| **ROBOTIS Full** | http://localhost:3000 | WebXR control |

---

## 📚 Documentation

**Quick Start Guides**:
- `⭐_START_HERE_ROBOTIS.md` - Main quick reference
- `START_ROBOTIS.md` - ROBOTIS-SYSTEMIC quick start
- `README_DESKTOP.md` - This file

**Complete Guides**:
- `ROBOTIS_INTEGRATION_COMPLETE.md` - Full ROBOTIS integration
- `UNIVARM_STARTER_INTEGRATION.md` - Univarm Starter
- `UNIVARM_ADVANCED_INTEGRATION.md` - Univarm Advanced
- `UNIVARM_COMPLETE_ARCHITECTURE.md` - Complete architecture
- `ROS2_INTEGRATION_COMPLETE.md` - ROS 2 integration

---

## 🔧 First Time Setup

### Install Dependencies

The dependencies are already installed, but if you need to reinstall:

```bash
npm install
```

### Install ROS 2 (for robot features)

If you want to use ROS 2 features:

```bash
# Ubuntu/Debian
# See: https://docs.ros.org/en/humble/Installation.html

# macOS (experimental)
# Use Docker or VM
```

### Build Rust Backends

```bash
# Univarm Advanced backend
cd backend-univarm/rust
cargo build --release

# ROBOTIS robotd (if using)
# Already at: /Users/frankvanlaarhoven/Downloads/ROBOTIS-SYSTEMIC/services/robotd
```

---

## ⚙️ Configuration

All configuration is pre-set for local development:

- **Ports**: 5175 (IDE), 3000 (Univarm Web), 8080 (backends)
- **SSE Token**: `demo-token-123`
- **CORS**: Enabled for all local origins

No configuration needed to start!

---

## 🐛 Troubleshooting

### Port Already in Use

```bash
# Kill processes on ports
lsof -ti:5175 | xargs kill -9  # IDE
lsof -ti:3000 | xargs kill -9  # Univarm Web
lsof -ti:8080 | xargs kill -9  # Backends

# Then restart
./START.sh
```

### Services Won't Start

```bash
# Run verification
./verify-integration.sh

# Check logs
tail -f logs/*.log
```

### Need to Update

```bash
# Pull latest from GitHub
git pull origin main

# Reinstall dependencies
npm install

# Rebuild if needed
npm run build
```

---

## 🌟 Features Available

### Web-Based Development
- ✅ Monaco code editor
- ✅ File explorer
- ✅ Terminal integration
- ✅ ROS Learning Center
- ✅ Simulation platform

### Robot Control
- ✅ WebXR interface (ROBOTIS)
- ✅ Path optimization (Univarm Starter)
- ✅ Production planning (Univarm Advanced)
- ✅ Real-time telemetry

### ROS 2 Integration
- ✅ Kobuki/TurtleBot2 bringup
- ✅ Helix humanoid skeleton
- ✅ gRPC path planning
- ✅ Python drive bridge

### Code Generation
- ✅ Rust
- ✅ C++
- ✅ Python
- ✅ TypeScript

---

## 🎉 You're Ready!

**Everything is set up and ready to use!**

Just run:
```bash
cd ~/Desktop/"NAVA Studio IDE"
./START.sh
```

**Your browser will open to the workspace automatically!** 🚀

---

## 📞 Support

- **Documentation**: See the guides in this directory
- **Issues**: https://github.com/FrankAsanteVanLaarhoven/NAVA-Ai-Studio/issues
- **Email**: support@navlambda.studio

---

**🎊 Welcome to NAVΛ Studio IDE - The future of robotics development!** 🤖💻✨

