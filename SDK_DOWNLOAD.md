# NAVΛ Studio IDE - Desktop SDK Download

## 📦 Desktop Application Installers

Download and install NAVΛ Studio IDE as a native desktop application for your operating system.

---

## 🖥️ Supported Platforms

### **macOS** (Intel & Apple Silicon)
- ✅ macOS 10.15 (Catalina) or later
- ✅ Intel x86_64 processors
- ✅ Apple Silicon (M1/M2/M3) ARM64

### **Windows**
- ✅ Windows 10 or later
- ✅ 64-bit systems
- ✅ .msi installer included

### **Linux**
- ✅ Ubuntu 20.04+ / Debian 11+
- ✅ Fedora 35+ / RHEL 8+
- ✅ AppImage (portable)
- ✅ .deb packages
- ✅ .rpm packages

---

## 📥 Download Links

### **Latest Release: v1.0.0**

#### **macOS**
- **Universal Binary (Intel + Apple Silicon)**
  - [`NAVA-Studio_1.0.0_universal.dmg`](https://github.com/FrankAsanteVanLaarhoven/NAVA-Ai-Studio/releases) (120 MB)
  - Install: Open DMG → Drag to Applications folder

- **Apple Silicon Only (M1/M2/M3)**
  - [`NAVA-Studio_1.0.0_aarch64.dmg`](https://github.com/FrankAsanteVanLaarhoven/NAVA-Ai-Studio/releases) (95 MB)
  
- **Intel Only (x86_64)**
  - [`NAVA-Studio_1.0.0_x64.dmg`](https://github.com/FrankAsanteVanLaarhoven/NAVA-Ai-Studio/releases) (98 MB)

#### **Windows**
- **MSI Installer**
  - [`NAVA-Studio_1.0.0_x64_en-US.msi`](https://github.com/FrankAsanteVanLaarhoven/NAVA-Ai-Studio/releases) (85 MB)
  - Install: Double-click and follow wizard

- **Portable EXE**
  - [`NAVA-Studio_1.0.0_x64.exe`](https://github.com/FrankAsanteVanLaarhoven/NAVA-Ai-Studio/releases) (90 MB)
  - No installation required

#### **Linux**
- **AppImage (Universal)**
  - [`NAVA-Studio_1.0.0_amd64.AppImage`](https://github.com/FrankAsanteVanLaarhoven/NAVA-Ai-Studio/releases) (95 MB)
  - Make executable: `chmod +x NAVA-Studio_1.0.0_amd64.AppImage`
  - Run: `./NAVA-Studio_1.0.0_amd64.AppImage`

- **Debian/Ubuntu (.deb)**
  - [`nava-studio_1.0.0_amd64.deb`](https://github.com/FrankAsanteVanLaarhoven/NAVA-Ai-Studio/releases) (80 MB)
  - Install: `sudo dpkg -i nava-studio_1.0.0_amd64.deb`

- **Fedora/RHEL (.rpm)**
  - [`nava-studio-1.0.0-1.x86_64.rpm`](https://github.com/FrankAsanteVanLaarhoven/NAVA-Ai-Studio/releases) (82 MB)
  - Install: `sudo rpm -i nava-studio-1.0.0-1.x86_64.rpm`

---

## ⚡ Quick Install (Automated)

### **One-Line Installation (Recommended)**

#### **macOS / Linux:**
```bash
curl -fsSL https://raw.githubusercontent.com/FrankAsanteVanLaarhoven/NAVA-Ai-Studio/main/scripts/install-sdk.sh | bash
```

Or download and run:
```bash
wget https://raw.githubusercontent.com/FrankAsanteVanLaarhoven/NAVA-Ai-Studio/main/scripts/install-sdk.sh
chmod +x install-sdk.sh
./install-sdk.sh
```

#### **Windows (PowerShell):**
```powershell
Invoke-WebRequest -Uri "https://github.com/FrankAsanteVanLaarhoven/NAVA-Ai-Studio/releases/latest/download/NAVA-Studio_x64_en-US.msi" -OutFile "NAVA-Studio.msi"; Start-Process NAVA-Studio.msi
```

---

## 🚀 Manual Installation Instructions

### **macOS Installation**

1. **Download** the `.dmg` file for your processor
2. **Open** the downloaded DMG file
3. **Drag** "NAVΛ Studio" to your Applications folder
4. **Launch** from Applications or Spotlight
5. If you see "unverified developer" warning:
   - Go to **System Preferences → Security & Privacy**
   - Click **"Open Anyway"**

**Command Line Installation:**
```bash
# Download and mount DMG
curl -LO https://github.com/FrankAsanteVanLaarhoven/NAVA-Ai-Studio/releases/download/v1.0.0/NAVA-Studio_1.0.0_universal.dmg
hdiutil attach NAVA-Studio_1.0.0_universal.dmg
cp -R "/Volumes/NAVΛ Studio/NAVΛ Studio.app" /Applications/
hdiutil detach "/Volumes/NAVΛ Studio"
```

---

### **Windows Installation**

#### **Option 1: MSI Installer (Recommended)**
1. **Download** the `.msi` installer
2. **Double-click** to run
3. Follow the installation wizard
4. Launch from Start Menu or Desktop shortcut

#### **Option 2: Portable EXE**
1. **Download** the `.exe` file
2. **Place** in desired folder (e.g., `C:\Program Files\NAVA Studio\`)
3. **Double-click** to run
4. No installation needed!

**Command Line Installation (PowerShell):**
```powershell
# Download installer
Invoke-WebRequest -Uri "https://github.com/FrankAsanteVanLaarhoven/NAVA-Ai-Studio/releases/download/v1.0.0/NAVA-Studio_1.0.0_x64_en-US.msi" -OutFile "NAVA-Studio-Setup.msi"

# Install
Start-Process msiexec.exe -Wait -ArgumentList '/I NAVA-Studio-Setup.msi /quiet'
```

---

### **Linux Installation**

#### **Ubuntu/Debian (.deb)**
```bash
# Download
wget https://github.com/FrankAsanteVanLaarhoven/NAVA-Ai-Studio/releases/download/v1.0.0/nava-studio_1.0.0_amd64.deb

# Install
sudo dpkg -i nava-studio_1.0.0_amd64.deb

# Fix dependencies if needed
sudo apt-get install -f

# Launch
nava-studio
```

#### **Fedora/RHEL (.rpm)**
```bash
# Download
wget https://github.com/FrankAsanteVanLaarhoven/NAVA-Ai-Studio/releases/download/v1.0.0/nava-studio-1.0.0-1.x86_64.rpm

# Install
sudo rpm -i nava-studio-1.0.0-1.x86_64.rpm

# Launch
nava-studio
```

#### **AppImage (Universal)**
```bash
# Download
wget https://github.com/FrankAsanteVanLaarhoven/NAVA-Ai-Studio/releases/download/v1.0.0/NAVA-Studio_1.0.0_amd64.AppImage

# Make executable
chmod +x NAVA-Studio_1.0.0_amd64.AppImage

# Run
./NAVA-Studio_1.0.0_amd64.AppImage
```

---

## 🔧 System Requirements

### **Minimum Requirements**
- **OS**: macOS 10.15+ / Windows 10+ / Ubuntu 20.04+
- **RAM**: 4 GB
- **Storage**: 500 MB free space
- **CPU**: Dual-core processor

### **Recommended Requirements**
- **OS**: macOS 13+ / Windows 11 / Ubuntu 22.04+
- **RAM**: 8 GB or more
- **Storage**: 2 GB free space
- **CPU**: Quad-core processor
- **GPU**: Dedicated GPU for 3D visualization

---

## 🎯 Features in Desktop App

✅ **Native Performance** - Runs natively, no browser required  
✅ **Offline Mode** - Full functionality without internet  
✅ **System Integration** - File associations, menu bar integration  
✅ **Auto Updates** - Automatic update notifications  
✅ **Hardware Acceleration** - GPU-accelerated 3D rendering  
✅ **Native Dialogs** - System-native file pickers and dialogs  
✅ **Tray Icon** - Quick access from system tray  
✅ **Keyboard Shortcuts** - Global hotkeys support  

---

## 🆕 Auto-Update

The desktop app checks for updates automatically:
- **Notification** when new version available
- **One-click update** installation
- **Background downloads** while you work
- **Rollback support** if issues occur

Manual update check:
- Menu: **Help → Check for Updates**
- Keyboard: `Ctrl+U` (Windows/Linux) or `⌘U` (Mac)

---

## 🐛 Troubleshooting

### **macOS: "App is damaged" error**
```bash
# Remove quarantine attribute
xattr -d com.apple.quarantine /Applications/NAVΛ\ Studio.app
```

### **Windows: SmartScreen warning**
1. Click **"More info"**
2. Click **"Run anyway"**

### **Linux: Missing dependencies**
```bash
# Ubuntu/Debian
sudo apt-get install -y webkit2gtk-4.0 libayatana-appindicator3-1

# Fedora
sudo dnf install webkit2gtk3 libappindicator-gtk3
```

### **Permission Errors**
```bash
# Linux/macOS: Give execute permission
chmod +x /path/to/nava-studio
```

---

## 📊 Build Information

- **Version**: 1.0.0
- **Build Date**: 2025-01-12
- **Framework**: Tauri 1.5 + React 18
- **Rust Version**: 1.75+
- **Node Version**: 18.x+

---

## 🔐 Security & Privacy

✅ **Code Signed** (macOS & Windows)  
✅ **Sandboxed** for security  
✅ **No telemetry** by default  
✅ **Local-first** - your code stays on your machine  
✅ **Open Source** - audit the code yourself  

---

## 📚 Additional Resources

- **Documentation**: https://github.com/FrankAsanteVanLaarhoven/NAVA-Ai-Studio/docs
- **GitHub**: https://github.com/FrankAsanteVanLaarhoven/NAVA-Ai-Studio
- **Issues**: https://github.com/FrankAsanteVanLaarhoven/NAVA-Ai-Studio/issues
- **Discussions**: https://github.com/FrankAsanteVanLaarhoven/NAVA-Ai-Studio/discussions

---

## 🎓 Getting Started

After installation:
1. **Launch** NAVΛ Studio
2. **Open** Command Palette (`Ctrl+Shift+P` or `⌘⇧P`)
3. Try **"Help: Getting Started"**
4. Open example projects from **File → Examples**

---

## 🌟 Support

- **Documentation**: [docs/](./docs/)
- **Community**: GitHub Discussions
- **Bug Reports**: GitHub Issues
- **Email**: support@navlambda.studio

---

## 📝 License

NAVΛ Studio IDE is dual-licensed under:
- MIT License
- Apache License 2.0

Choose the license that best suits your needs.

---

**Built with ❤️ by Frank Van Laarhoven**

*The world's first IDE for Van Laarhoven Navigation Calculus*


