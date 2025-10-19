# NAVΛ Studio Desktop SDK - Complete Installation Guide

## 🚀 Quick Start - Install Pre-built Desktop App

### For End Users (Recommended)

Download and install the pre-built desktop application for your platform:

#### **macOS** 🍎
```bash
# Download the universal installer (Intel + Apple Silicon)
curl -LO https://github.com/FrankAsanteVanLaarhoven/NAVA-Ai-Studio/releases/latest/download/NAVA-Studio_universal.dmg

# Open the DMG
open NAVA-Studio_universal.dmg

# Drag to Applications folder, then launch
```

#### **Windows** 🪟
```powershell
# Download the MSI installer
Invoke-WebRequest -Uri "https://github.com/FrankAsanteVanLaarhoven/NAVA-Ai-Studio/releases/latest/download/NAVA-Studio_x64_en-US.msi" -OutFile "NAVA-Studio-Setup.msi"

# Run the installer
Start-Process "NAVA-Studio-Setup.msi"
```

#### **Linux** 🐧
```bash
# Ubuntu/Debian - Download and install .deb
wget https://github.com/FrankAsanteVanLaarhoven/NAVA-Ai-Studio/releases/latest/download/nava-studio_amd64.deb
sudo dpkg -i nava-studio_amd64.deb
sudo apt-get install -f  # Fix any dependencies

# Launch
nava-studio
```

Or use the universal **AppImage**:
```bash
# Download AppImage
wget https://github.com/FrankAsanteVanLaarhoven/NAVA-Ai-Studio/releases/latest/download/NAVA-Studio_amd64.AppImage

# Make executable and run
chmod +x NAVA-Studio_amd64.AppImage
./NAVA-Studio_amd64.AppImage
```

---

## 🛠️ Build from Source - For Developers

### Prerequisites

#### **1. Install Node.js (v18+)**
```bash
# macOS
brew install node

# Windows (using winget)
winget install OpenJS.NodeJS

# Linux
curl -fsSL https://deb.nodesource.com/setup_20.x | sudo -E bash -
sudo apt-get install -y nodejs
```

#### **2. Install Rust (Latest Stable)**
```bash
# All platforms
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
source $HOME/.cargo/env
```

#### **3. Platform-Specific Dependencies**

**macOS:**
```bash
# Install Xcode Command Line Tools
xcode-select --install
```

**Windows:**
- Install [Microsoft Visual Studio Build Tools](https://visualstudio.microsoft.com/downloads/)
- Or via command line:
```powershell
winget install Microsoft.VisualStudio.2022.BuildTools
```

**Linux (Ubuntu/Debian):**
```bash
sudo apt-get update
sudo apt-get install -y \
    libwebkit2gtk-4.0-dev \
    build-essential \
    curl \
    wget \
    file \
    libssl-dev \
    libgtk-3-dev \
    libayatana-appindicator3-dev \
    librsvg2-dev
```

**Linux (Fedora/RHEL):**
```bash
sudo dnf install \
    webkit2gtk4.0-devel \
    openssl-devel \
    curl wget file \
    libappindicator-gtk3-devel \
    librsvg2-devel
sudo dnf groupinstall "C Development Tools and Libraries"
```

---

### Build Instructions

#### **1. Clone the Repository**
```bash
git clone https://github.com/FrankAsanteVanLaarhoven/NAVA-Ai-Studio.git
cd NAVA-Ai-Studio
```

#### **2. Install Dependencies**
```bash
npm install
```

#### **3. Build Desktop Application**
```bash
# Use the automated build script
./scripts/build.sh

# Or build manually
npm run build
npm run tauri:build
```

#### **Build Output Locations:**
- **macOS**: `src-tauri/target/release/bundle/dmg/NAVΛ Studio.dmg`
- **Windows**: `src-tauri/target/release/bundle/msi/NAVΛ Studio_1.0.0_x64.msi`
- **Linux**: 
  - AppImage: `src-tauri/target/release/bundle/appimage/NAVΛ Studio_1.0.0_amd64.AppImage`
  - Debian: `src-tauri/target/release/bundle/deb/nava-studio_1.0.0_amd64.deb`
  - RPM: `src-tauri/target/release/bundle/rpm/nava-studio-1.0.0-1.x86_64.rpm`

---

## 🎯 Platform-Specific Build Commands

### **macOS Universal Binary** (Intel + Apple Silicon)
```bash
# Install universal target
rustup target add aarch64-apple-darwin
rustup target add x86_64-apple-darwin

# Build universal binary
npm run tauri:build -- --target universal-apple-darwin
```

### **Windows 64-bit**
```bash
npm run tauri:build -- --target x86_64-pc-windows-msvc
```

### **Linux 64-bit** (All package formats)
```bash
npm run tauri:build -- --bundles deb,appimage,rpm
```

---

## 📦 Package and Distribute

### **Create Distribution Package**
```bash
# Creates a complete distribution archive
./scripts/package.sh
```

This will create:
- `dist/navlambda-studio-<date>.tar.gz` - Complete distribution package
- Contains all installers for all platforms
- Includes documentation and examples

---

## 🚢 Release to GitHub

### **1. Tag a Release**
```bash
# Create a new version tag
git tag -a v1.0.0 -m "Release v1.0.0"
git push origin v1.0.0
```

### **2. Upload to GitHub Releases**
```bash
# Using GitHub CLI
gh release create v1.0.0 \
  src-tauri/target/release/bundle/dmg/*.dmg \
  src-tauri/target/release/bundle/msi/*.msi \
  src-tauri/target/release/bundle/appimage/*.AppImage \
  src-tauri/target/release/bundle/deb/*.deb \
  src-tauri/target/release/bundle/rpm/*.rpm \
  --title "NAVΛ Studio v1.0.0" \
  --notes "First stable release"
```

---

## 🔧 Development Mode

### **Run in Development Mode** (with hot reload)
```bash
npm run tauri:dev
```

This launches the app with:
- ✅ Hot module reloading
- ✅ DevTools enabled
- ✅ Fast iteration cycle
- ✅ Live code updates

---

## 🎨 Customization

### **Change App Icon**
```bash
# Generate icons from a single source image
npm run tauri icon path/to/your-icon.png
```

This generates all required icon formats:
- macOS: `.icns`
- Windows: `.ico`
- Linux: `.png` (multiple sizes)

### **Update App Information**
Edit `tauri.conf.json`:
```json
{
  "package": {
    "productName": "Your App Name",
    "version": "1.0.0"
  },
  "tauri": {
    "bundle": {
      "identifier": "com.yourcompany.appname",
      "copyright": "Copyright (c) 2025 Your Name"
    }
  }
}
```

---

## 🔐 Code Signing (Optional but Recommended)

### **macOS Code Signing**
```bash
# 1. Get your Apple Developer Certificate
# 2. Add to tauri.conf.json
{
  "tauri": {
    "bundle": {
      "macOS": {
        "signingIdentity": "Developer ID Application: Your Name (TEAM_ID)"
      }
    }
  }
}
```

### **Windows Code Signing**
```bash
# Sign the MSI with your certificate
npm run tauri:build -- \
  --certificate /path/to/cert.pfx \
  --certificate-password YOUR_PASSWORD
```

---

## 🧪 Testing

### **Run Tests**
```bash
# Unit tests
npm test

# E2E tests
npm run test:e2e

# Run all tests
npm run test:all
```

### **Test the Built Application**
```bash
# Build and manually test
npm run tauri:build

# Then install and test the generated installer
```

---

## 📊 Build Performance

**Typical Build Times:**
- **First build**: 10-20 minutes (downloads dependencies)
- **Incremental builds**: 2-5 minutes
- **Development mode startup**: 5-10 seconds

**Ways to Speed Up Builds:**
```bash
# 1. Install sccache for caching
cargo install sccache
export RUSTC_WRAPPER=sccache

# 2. Use parallel compilation
export CARGO_BUILD_JOBS=8
```

---

## 🐛 Troubleshooting

### **Problem: "Command not found: cargo"**
```bash
# Solution: Install Rust
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
source $HOME/.cargo/env
```

### **Problem: "webkit2gtk not found" (Linux)**
```bash
# Solution: Install webkit2gtk
sudo apt-get install libwebkit2gtk-4.0-dev
```

### **Problem: Build fails on macOS with Xcode error**
```bash
# Solution: Install/update Xcode Command Line Tools
xcode-select --install
sudo xcode-select --reset
```

### **Problem: Windows build fails with linker error**
```powershell
# Solution: Install Visual Studio Build Tools
winget install Microsoft.VisualStudio.2022.BuildTools
# Restart your terminal after installation
```

### **Clean Build (if all else fails)**
```bash
# Remove all build artifacts and start fresh
rm -rf node_modules src-tauri/target dist
npm install
npm run tauri:build
```

---

## 📈 Build Size Optimization

Add to `src-tauri/Cargo.toml`:
```toml
[profile.release]
opt-level = "z"        # Optimize for size
lto = true             # Link Time Optimization
codegen-units = 1      # Better optimization
strip = true           # Strip debug symbols
panic = "abort"        # Smaller panic handler
```

**Typical Application Sizes:**
- **macOS**: 80-120 MB (DMG)
- **Windows**: 70-100 MB (MSI)
- **Linux**: 75-110 MB (AppImage)

---

## 🌍 Continuous Integration

### **GitHub Actions Workflow**

Create `.github/workflows/build-desktop.yml`:
```yaml
name: Build Desktop Apps

on:
  push:
    tags:
      - 'v*'
  workflow_dispatch:

jobs:
  build:
    strategy:
      fail-fast: false
      matrix:
        platform: [macos-latest, ubuntu-22.04, windows-latest]

    runs-on: ${{ matrix.platform }}

    steps:
      - uses: actions/checkout@v4

      - name: Setup Node.js
        uses: actions/setup-node@v4
        with:
          node-version: '20'

      - name: Install Rust
        uses: dtolnay/rust-toolchain@stable

      - name: Install dependencies (Ubuntu)
        if: matrix.platform == 'ubuntu-22.04'
        run: |
          sudo apt-get update
          sudo apt-get install -y libwebkit2gtk-4.0-dev \
            build-essential curl wget file libssl-dev \
            libgtk-3-dev libayatana-appindicator3-dev librsvg2-dev

      - name: Install dependencies
        run: npm ci

      - name: Build
        run: npm run tauri:build

      - name: Upload artifacts
        uses: actions/upload-artifact@v4
        with:
          name: ${{ matrix.platform }}-build
          path: src-tauri/target/release/bundle/
```

---

## 📚 Additional Resources

- **Tauri Documentation**: https://tauri.app/
- **NAVΛ Studio Docs**: [docs/](./docs/)
- **GitHub Repository**: https://github.com/FrankAsanteVanLaarhoven/NAVA-Ai-Studio
- **Issues & Support**: https://github.com/FrankAsanteVanLaarhoven/NAVA-Ai-Studio/issues

---

## 💡 Tips for End Users

### **First Launch**
1. Launch NAVΛ Studio from Applications (macOS) or Start Menu (Windows)
2. Open Command Palette: `Ctrl+Shift+P` (Windows/Linux) or `⌘⇧P` (Mac)
3. Try "Help: Getting Started"
4. Explore example projects: **File → Open Examples**

### **Keyboard Shortcuts**
- **Command Palette**: `Ctrl+Shift+P` / `⌘⇧P`
- **Quick Open**: `Ctrl+P` / `⌘P`
- **Toggle Terminal**: `Ctrl+`** / `⌘`**
- **Save File**: `Ctrl+S` / `⌘S`

### **Getting Help**
- Press `F1` for help
- Visit [docs/GETTING_STARTED.md](./GETTING_STARTED.md)
- Join discussions on GitHub

---

## 🎓 Next Steps

After installation:

1. ✅ **Open Example Projects**
   - Navigate to `File → Examples`
   - Try `basic-navigation.vnc`

2. ✅ **Learn the Language**
   - Read [vnc-language-reference.md](./docs/vnc-language-reference.md)
   - Explore mathematical notation with native ⋋ symbol

3. ✅ **Watch Visualizations**
   - Click on the **Visualizer** tab
   - See your navigation paths in 3D

4. ✅ **Compile to Multiple Targets**
   - Try **Build → Compile to C++**
   - Or Python, GLSL, WASM, and more!

---

## 🌟 Features

✅ **Native Desktop Performance** - No browser, pure native speed  
✅ **Offline First** - Work without internet connection  
✅ **System Integration** - File associations, native dialogs  
✅ **Auto-Updates** - Get new features automatically  
✅ **Hardware Acceleration** - GPU-powered 3D rendering  
✅ **Cross-Platform** - macOS, Windows, Linux  
✅ **Open Source** - MIT/Apache-2.0 dual license  

---

## 📝 License

NAVΛ Studio is dual-licensed under:
- **MIT License** - [LICENSE-MIT](./LICENSE-MIT)
- **Apache License 2.0** - [LICENSE-APACHE](./LICENSE-APACHE)

Choose the license that best fits your needs.

---

## 💖 Built by Frank Van Laarhoven

**NAVΛ Studio IDE** - The world's first IDE for Van Laarhoven Navigation Calculus

*Making mathematical navigation programming accessible to everyone* 🚀⋋

---

**Ready to build?** Run `./scripts/build.sh` and get started! 🎉

