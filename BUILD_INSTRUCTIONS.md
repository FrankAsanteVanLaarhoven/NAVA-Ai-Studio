# Building NAVΛ Studio IDE from Source

## 🔨 Build Desktop Installers

This guide shows you how to build native desktop applications for macOS, Windows, and Linux.

---

## 📋 Prerequisites

### **All Platforms**
- **Node.js**: v18.x or later
- **npm**: v9.x or later
- **Rust**: Latest stable (install from https://rustup.rs/)
- **Git**: For cloning the repository

### **Platform-Specific Requirements**

#### **macOS**
```bash
# Install Xcode Command Line Tools
xcode-select --install

# Install Homebrew (if not installed)
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"

# Install dependencies
brew install rust
```

#### **Windows**
```powershell
# Install Rust from https://rustup.rs/
# Install Microsoft Visual Studio Build Tools
# https://visualstudio.microsoft.com/downloads/

# Or use winget
winget install Microsoft.VisualStudio.2022.BuildTools
winget install Rustlang.Rustup
```

#### **Linux (Ubuntu/Debian)**
```bash
# Install Rust
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh

# Install dependencies
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

#### **Linux (Fedora)**
```bash
# Install Rust
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh

# Install dependencies
sudo dnf install \
    webkit2gtk4.0-devel \
    openssl-devel \
    curl \
    wget \
    file \
    libappindicator-gtk3-devel \
    librsvg2-devel
sudo dnf groupinstall "C Development Tools and Libraries"
```

---

## 🚀 Quick Start

```bash
# Clone the repository
git clone https://github.com/FrankAsanteVanLaarhoven/NAVA-Ai-Studio.git
cd NAVA-Ai-Studio

# Install dependencies
npm install

# Build desktop app for current platform
npm run tauri build
```

**Build output location:**
- **macOS**: `src-tauri/target/release/bundle/dmg/`
- **Windows**: `src-tauri/target/release/bundle/msi/` and `nsis/`
- **Linux**: `src-tauri/target/release/bundle/deb/`, `appimage/`, `rpm/`

---

## 🎯 Build Commands

### **Development Build (with hot reload)**
```bash
npm run tauri dev
```

### **Production Build (optimized)**
```bash
npm run tauri build
```

### **Build with specific target**
```bash
# macOS Universal (Intel + Apple Silicon)
npm run tauri build -- --target universal-apple-darwin

# macOS Apple Silicon only
npm run tauri build -- --target aarch64-apple-darwin

# macOS Intel only
npm run tauri build -- --target x86_64-apple-darwin

# Windows 64-bit
npm run tauri build -- --target x86_64-pc-windows-msvc

# Linux 64-bit
npm run tauri build -- --target x86_64-unknown-linux-gnu
```

### **Build with debug info**
```bash
npm run tauri build -- --debug
```

### **Build specific bundles**
```bash
# macOS DMG only
npm run tauri build -- --bundles dmg

# Windows MSI only
npm run tauri build -- --bundles msi

# Linux AppImage only
npm run tauri build -- --bundles appimage

# Multiple formats
npm run tauri build -- --bundles deb,appimage,rpm
```

---

## 📦 Available Bundle Formats

### **macOS**
- **DMG** (`.dmg`) - Disk image installer
- **App Bundle** (`.app`) - Application package

### **Windows**
- **MSI** (`.msi`) - Windows Installer package
- **NSIS** (`.exe`) - Nullsoft Installer

### **Linux**
- **AppImage** (`.AppImage`) - Portable executable
- **Debian** (`.deb`) - Debian/Ubuntu package
- **RPM** (`.rpm`) - Fedora/RHEL package

---

## 🔧 Configuration

### **Custom App Icon**
Place your icons in:
- `src-tauri/icons/icon.icns` (macOS)
- `src-tauri/icons/icon.ico` (Windows)
- `src-tauri/icons/icon.png` (Linux)

Then rebuild:
```bash
npm run tauri icon path/to/your-icon.png
```

### **App Signing (macOS)**
Add to `tauri.conf.json`:
```json
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

### **App Signing (Windows)**
```bash
# Sign with certificate
tauri build --config "src-tauri/tauri.conf.json" \
  --certificate /path/to/cert.pfx \
  --certificate-password YOUR_PASSWORD
```

---

## 🌍 Cross-Platform Building

### **Build for All Platforms (GitHub Actions)**

Create `.github/workflows/build.yml`:
```yaml
name: Build Desktop Apps

on:
  push:
    tags:
      - 'v*'

jobs:
  build:
    strategy:
      matrix:
        platform: [macos-latest, ubuntu-latest, windows-latest]
    
    runs-on: ${{ matrix.platform }}
    
    steps:
      - uses: actions/checkout@v3
      
      - name: Setup Node
        uses: actions/setup-node@v3
        with:
          node-version: 18
      
      - name: Setup Rust
        uses: actions-rs/toolchain@v1
        with:
          toolchain: stable
      
      - name: Install dependencies (Ubuntu)
        if: matrix.platform == 'ubuntu-latest'
        run: |
          sudo apt-get update
          sudo apt-get install -y libwebkit2gtk-4.0-dev \
            build-essential curl wget file libssl-dev \
            libgtk-3-dev libayatana-appindicator3-dev \
            librsvg2-dev
      
      - name: Install dependencies
        run: npm install
      
      - name: Build
        run: npm run tauri build
      
      - name: Upload artifacts
        uses: actions/upload-artifact@v3
        with:
          name: ${{ matrix.platform }}-build
          path: src-tauri/target/release/bundle/
```

---

## 🚢 Release Process

### **1. Update Version**
```bash
# Update version in package.json and tauri.conf.json
npm version 1.0.1
```

### **2. Build for All Platforms**
```bash
# macOS
npm run tauri build -- --target universal-apple-darwin

# Windows (on Windows machine)
npm run tauri build

# Linux (on Linux machine)
npm run tauri build -- --bundles deb,appimage,rpm
```

### **3. Create GitHub Release**
```bash
# Tag the release
git tag -a v1.0.1 -m "Release v1.0.1"
git push origin v1.0.1

# Create release on GitHub
gh release create v1.0.1 \
  ./src-tauri/target/release/bundle/dmg/*.dmg \
  ./src-tauri/target/release/bundle/msi/*.msi \
  ./src-tauri/target/release/bundle/appimage/*.AppImage \
  ./src-tauri/target/release/bundle/deb/*.deb \
  ./src-tauri/target/release/bundle/rpm/*.rpm \
  --title "NAVΛ Studio v1.0.1" \
  --notes "Release notes here"
```

---

## 🧪 Testing Builds

### **Test before release**
```bash
# Run in dev mode
npm run tauri dev

# Build and test
npm run tauri build
# Then manually test the generated installer
```

### **Automated tests**
```bash
# Run unit tests
npm test

# Run integration tests
npm run test:integration

# Run E2E tests
npm run test:e2e
```

---

## 📊 Build Size Optimization

### **Reduce bundle size**
```bash
# Enable size optimization in Cargo.toml
[profile.release]
opt-level = "z"     # Optimize for size
lto = true          # Enable Link Time Optimization
codegen-units = 1   # Better optimization
strip = true        # Strip symbols
```

### **Analyze build size**
```bash
# Install cargo-bloat
cargo install cargo-bloat

# Analyze
cd src-tauri
cargo bloat --release --crates
```

---

## 🐛 Troubleshooting

### **Common Build Errors**

#### **"No such file or directory: 'webkit2gtk'"**
```bash
# Linux: Install webkit2gtk
sudo apt-get install libwebkit2gtk-4.0-dev
```

#### **"error: linker 'cc' not found"**
```bash
# Linux: Install build tools
sudo apt-get install build-essential
```

#### **Windows: "LINK : fatal error LNK1181"**
```powershell
# Install Visual Studio Build Tools
# https://visualstudio.microsoft.com/downloads/
```

#### **macOS: "xcrun: error: invalid active developer path"**
```bash
xcode-select --install
```

### **Clean build**
```bash
# Remove all build artifacts
rm -rf src-tauri/target
rm -rf node_modules
npm install
npm run tauri build
```

---

## 📈 Performance Tips

1. **Parallel Builds**: Use `--jobs` flag
   ```bash
   cargo build --release --jobs 8
   ```

2. **Incremental Compilation**: Enable in development
   ```bash
   export CARGO_INCREMENTAL=1
   ```

3. **Build Cache**: Use `sccache` for faster rebuilds
   ```bash
   cargo install sccache
   export RUSTC_WRAPPER=sccache
   ```

---

## 🎓 Additional Resources

- **Tauri Documentation**: https://tauri.app/
- **Rust Documentation**: https://doc.rust-lang.org/
- **GitHub Actions**: https://docs.github.com/actions

---

## 📝 Notes

- First build takes 10-20 minutes
- Subsequent builds are much faster (2-5 minutes)
- Universal macOS builds require both Intel and ARM toolchains
- Windows builds require Visual Studio Build Tools
- Linux builds produce multiple package formats

---

**Happy Building! 🚀**


