# NAVΛ Studio SDK - Quick Start Guide

## 🚀 For End Users: Install in 30 Seconds

### Automated Installation (Easiest)

**Run this one command:**

#### macOS / Linux:
```bash
curl -fsSL https://raw.githubusercontent.com/FrankAsanteVanLaarhoven/NAVA-Ai-Studio/main/scripts/install-sdk.sh | bash
```

Or download and run:
```bash
wget https://raw.githubusercontent.com/FrankAsanteVanLaarhoven/NAVA-Ai-Studio/main/scripts/install-sdk.sh
chmod +x install-sdk.sh
./install-sdk.sh
```

#### Windows:
```powershell
# PowerShell (Run as Administrator)
Invoke-WebRequest -Uri "https://github.com/FrankAsanteVanLaarhoven/NAVA-Ai-Studio/releases/latest/download/NAVA-Studio_x64_en-US.msi" -OutFile "NAVA-Studio.msi"
Start-Process msiexec.exe -Wait -ArgumentList '/I NAVA-Studio.msi /quiet'
```

---

## 📦 For Developers: Build and Package

### Prerequisites Check

```bash
# Check if you have everything
node --version  # Should be v18+
npm --version   # Should be v9+
cargo --version # Rust toolchain
```

If missing, install:
```bash
# Node.js (macOS)
brew install node

# Rust (all platforms)
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
```

### Build Desktop App (3 steps)

```bash
# 1. Clone and enter directory
git clone https://github.com/FrankAsanteVanLaarhoven/NAVA-Ai-Studio.git
cd NAVA-Ai-Studio

# 2. Install dependencies
npm install

# 3. Build
./scripts/build.sh
```

**Output:** Installers in `src-tauri/target/release/bundle/`

---

## 📤 Create SDK Package for Distribution

```bash
# Create complete SDK package with all installers
./scripts/create-sdk-package.sh
```

**Output:** `dist/navlambda-studio-sdk-v1.0.0-<timestamp>.tar.gz`

This package includes:
- ✅ Installers for macOS, Windows, Linux
- ✅ All documentation
- ✅ Example projects
- ✅ Installation scripts
- ✅ Checksums

---

## 🌐 Release to GitHub

### Method 1: GitHub CLI (Recommended)

```bash
# Tag the release
git tag -a v1.0.0 -m "Release v1.0.0"
git push origin v1.0.0

# Create release and upload installers
gh release create v1.0.0 \
  src-tauri/target/release/bundle/dmg/*.dmg \
  src-tauri/target/release/bundle/msi/*.msi \
  src-tauri/target/release/bundle/appimage/*.AppImage \
  src-tauri/target/release/bundle/deb/*.deb \
  src-tauri/target/release/bundle/rpm/*.rpm \
  --title "NAVΛ Studio v1.0.0" \
  --notes "First stable release of NAVΛ Studio IDE"
```

### Method 2: GitHub Web UI

1. Go to your GitHub repository
2. Click **"Releases"** → **"Draft a new release"**
3. Create tag: `v1.0.0`
4. Upload installers from `src-tauri/target/release/bundle/`
5. Write release notes
6. Click **"Publish release"**

---

## 🎯 Platform-Specific Builds

### macOS Universal (Intel + Apple Silicon)
```bash
rustup target add aarch64-apple-darwin
rustup target add x86_64-apple-darwin
npm run tauri:build -- --target universal-apple-darwin
```

### Windows 64-bit
```bash
npm run tauri:build -- --target x86_64-pc-windows-msvc
```

### Linux (All formats)
```bash
npm run tauri:build -- --bundles deb,appimage,rpm
```

---

## 🧪 Testing Before Release

### Test in Development Mode
```bash
npm run tauri:dev
```

### Test Production Build
```bash
# Build
npm run tauri:build

# Install and test the generated installer
# macOS: Open the .dmg from src-tauri/target/release/bundle/dmg/
# Windows: Run the .msi from src-tauri/target/release/bundle/msi/
# Linux: Install the .deb/.rpm or run the .AppImage
```

---

## 📊 Build Time Estimates

| Build Type | First Build | Incremental |
|------------|-------------|-------------|
| Development | 5-10 min | 30 sec |
| Production | 15-25 min | 2-5 min |
| Universal macOS | 20-30 min | 3-7 min |

**Tip:** Use `sccache` to speed up Rust compilation:
```bash
cargo install sccache
export RUSTC_WRAPPER=sccache
```

---

## 🔍 Verify Installation

After users install, they should verify:

```bash
# Check if installed (Linux/macOS)
which nava-studio

# Check version
nava-studio --version

# Launch
nava-studio
```

---

## 🐛 Common Issues & Solutions

### Issue: "Command not found: cargo"
**Solution:**
```bash
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
source $HOME/.cargo/env
```

### Issue: "webkit2gtk not found" (Linux)
**Solution:**
```bash
sudo apt-get install libwebkit2gtk-4.0-dev
```

### Issue: Build fails on macOS
**Solution:**
```bash
xcode-select --install
sudo xcode-select --reset
```

### Issue: Build takes too long
**Solution:**
```bash
# Use release profile optimization
export CARGO_BUILD_JOBS=8

# Or install build cache
cargo install sccache
export RUSTC_WRAPPER=sccache
```

---

## 📱 Platform-Specific Installers

| Platform | Installer Type | Location | Size |
|----------|---------------|----------|------|
| macOS | .dmg | bundle/dmg/ | ~100 MB |
| Windows | .msi | bundle/msi/ | ~85 MB |
| Windows | .exe | bundle/nsis/ | ~90 MB |
| Linux | .AppImage | bundle/appimage/ | ~95 MB |
| Debian/Ubuntu | .deb | bundle/deb/ | ~80 MB |
| Fedora/RHEL | .rpm | bundle/rpm/ | ~82 MB |

---

## 🎓 Next Steps After Installation

### For Users:
1. Launch NAVΛ Studio
2. Press `Ctrl+Shift+P` (or `⌘⇧P` on Mac) for Command Palette
3. Type "Help: Getting Started"
4. Open example: `File → Examples → basic-navigation.vnc`

### For Developers:
1. Read `docs/vnc-language-reference.md`
2. Check `docs/plugin-development.md` for plugins
3. Review `docs/architecture.md` for internals
4. Join discussions on GitHub

---

## 📚 Documentation Files

- **SDK_DOWNLOAD.md** - Download links and instructions
- **DESKTOP_SDK_INSTALLATION.md** - Detailed installation guide
- **BUILD_INSTRUCTIONS.md** - Complete build documentation
- **GETTING_STARTED.md** - User getting started guide
- **docs/** - Complete technical documentation

---

## 🔗 Useful Links

- **GitHub**: https://github.com/FrankAsanteVanLaarhoven/NAVA-Ai-Studio
- **Releases**: https://github.com/FrankAsanteVanLaarhoven/NAVA-Ai-Studio/releases
- **Issues**: https://github.com/FrankAsanteVanLaarhoven/NAVA-Ai-Studio/issues
- **Discussions**: https://github.com/FrankAsanteVanLaarhoven/NAVA-Ai-Studio/discussions

---

## ✨ One-Line Install Command

**macOS/Linux:**
```bash
curl -fsSL https://raw.githubusercontent.com/FrankAsanteVanLaarhoven/NAVA-Ai-Studio/main/scripts/install-sdk.sh | bash
```

**Windows:**
```powershell
Invoke-WebRequest -Uri "https://github.com/FrankAsanteVanLaarhoven/NAVA-Ai-Studio/releases/latest/download/NAVA-Studio_x64_en-US.msi" -OutFile "Setup.msi"; Start-Process Setup.msi
```

---

## 💖 Support the Project

⭐ Star the repository on GitHub  
🐛 Report bugs and issues  
💡 Suggest features  
📖 Improve documentation  
🤝 Contribute code  

---

**NAVΛ Studio IDE** - Making Navigation Calculus Programming Accessible to Everyone 🚀⋋

Built with ❤️ by Frank Van Laarhoven

