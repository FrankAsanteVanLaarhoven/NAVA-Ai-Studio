# NAVΛ Studio - Complete Installation Index

This guide helps you find the right installation documentation for your needs.

---

## 🎯 Choose Your Path

### 👤 **I'm an End User**
**You just want to use NAVΛ Studio**

➡️ Start here: **[SDK_QUICK_START.md](SDK_QUICK_START.md)**

**Quick Install:**
- **macOS/Linux**: Run `curl -fsSL https://raw.githubusercontent.com/.../install-sdk.sh | bash`
- **Windows**: Run PowerShell script (see [SDK_QUICK_START.md](SDK_QUICK_START.md))

**Alternative**: Download installers from [SDK_DOWNLOAD.md](SDK_DOWNLOAD.md)

---

### 👨‍💻 **I'm a Developer**
**You want to build from source or contribute**

➡️ Start here: **[BUILD_INSTRUCTIONS.md](BUILD_INSTRUCTIONS.md)**

**Quick Build:**
```bash
git clone https://github.com/FrankAsanteVanLaarhoven/NAVA-Ai-Studio.git
cd NAVA-Ai-Studio
npm install
./scripts/build.sh
```

**Also check:**
- [CONTRIBUTING.md](CONTRIBUTING.md) - Contribution guidelines
- [docs/architecture.md](docs/architecture.md) - System architecture

---

### 🚀 **I'm Distributing to My Team**
**You need to package and deploy**

➡️ Start here: **[DESKTOP_SDK_INSTALLATION.md](DESKTOP_SDK_INSTALLATION.md)**

**Create SDK Package:**
```bash
./scripts/create-sdk-package.sh
```

**Also check:**
- [scripts/README.md](scripts/README.md) - All available scripts
- [docs/deployment-guide.md](docs/deployment-guide.md) - Deployment options

---

### ☁️ **I Want Cloud Deployment**
**You're deploying to cloud infrastructure**

➡️ Start here: **[docs/deployment-guide.md](docs/deployment-guide.md)**

**Quick Deploy:**
```bash
./scripts/deploy.sh
```

---

## 📚 All Installation Documentation

### Quick Reference Guides

| Document | For | Time | Difficulty |
|----------|-----|------|-----------|
| [SDK_QUICK_START.md](SDK_QUICK_START.md) | End users | 30 sec | ⭐ Easy |
| [SDK_DOWNLOAD.md](SDK_DOWNLOAD.md) | Manual install | 2 min | ⭐ Easy |
| [BUILD_INSTRUCTIONS.md](BUILD_INSTRUCTIONS.md) | Developers | 20 min | ⭐⭐⭐ Medium |
| [DESKTOP_SDK_INSTALLATION.md](DESKTOP_SDK_INSTALLATION.md) | Detailed guide | 5 min read | ⭐⭐ Easy |

### Comprehensive Guides

1. **[SDK_README.md](SDK_README.md)**
   - Complete SDK overview
   - All features and capabilities
   - Links to all other docs

2. **[GETTING_STARTED.md](GETTING_STARTED.md)**
   - User guide after installation
   - First steps with NAVΛ Studio
   - Example projects

3. **[scripts/README.md](scripts/README.md)**
   - All build and installation scripts
   - Script usage and options
   - Common workflows

### Platform-Specific Guides

#### macOS
- ✅ Universal binary (Intel + Apple Silicon)
- ✅ DMG installer
- ✅ Homebrew support (coming soon)
- 📖 See: [SDK_DOWNLOAD.md](SDK_DOWNLOAD.md) → macOS section

#### Windows
- ✅ MSI installer
- ✅ Portable EXE
- ✅ PowerShell installation script
- 📖 See: [SDK_DOWNLOAD.md](SDK_DOWNLOAD.md) → Windows section

#### Linux
- ✅ AppImage (universal)
- ✅ DEB packages (Debian/Ubuntu)
- ✅ RPM packages (Fedora/RHEL)
- 📖 See: [SDK_DOWNLOAD.md](SDK_DOWNLOAD.md) → Linux section

---

## 🔍 Find by Topic

### Installation Methods

**Automated Installation**:
- [SDK_QUICK_START.md](SDK_QUICK_START.md) - One-line install commands
- [scripts/install-sdk.sh](scripts/install-sdk.sh) - Bash installation script
- [scripts/install-sdk.ps1](scripts/install-sdk.ps1) - PowerShell installation script

**Manual Installation**:
- [SDK_DOWNLOAD.md](SDK_DOWNLOAD.md) - Download links
- [DESKTOP_SDK_INSTALLATION.md](DESKTOP_SDK_INSTALLATION.md) - Manual install steps

**Build from Source**:
- [BUILD_INSTRUCTIONS.md](BUILD_INSTRUCTIONS.md) - Complete build guide
- [scripts/build.sh](scripts/build.sh) - Automated build script

### Build and Package

**Build Desktop App**:
- [BUILD_INSTRUCTIONS.md](BUILD_INSTRUCTIONS.md) - Building installers
- [scripts/build.sh](scripts/build.sh) - Build script

**Create Distribution**:
- [scripts/create-sdk-package.sh](scripts/create-sdk-package.sh) - Create SDK bundle
- [scripts/package.sh](scripts/package.sh) - Package for distribution

**Release to Users**:
- [SDK_DOWNLOAD.md](SDK_DOWNLOAD.md) - Distribution guide
- [docs/deployment-guide.md](docs/deployment-guide.md) - Deployment strategies

### Troubleshooting

**Installation Issues**:
- [DESKTOP_SDK_INSTALLATION.md](DESKTOP_SDK_INSTALLATION.md) → Troubleshooting section
- [SDK_QUICK_START.md](SDK_QUICK_START.md) → Common Issues section

**Build Issues**:
- [BUILD_INSTRUCTIONS.md](BUILD_INSTRUCTIONS.md) → Troubleshooting section
- [scripts/README.md](scripts/README.md) → Troubleshooting section

### Learning and Usage

**Getting Started**:
- [GETTING_STARTED.md](GETTING_STARTED.md) - User guide
- [docs/vnc-language-reference.md](docs/vnc-language-reference.md) - Language guide

**Advanced Usage**:
- [docs/architecture.md](docs/architecture.md) - System internals
- [docs/plugin-development.md](docs/plugin-development.md) - Creating plugins

---

## 📊 Documentation by User Type

### New User
1. [SDK_QUICK_START.md](SDK_QUICK_START.md) - Install
2. [GETTING_STARTED.md](GETTING_STARTED.md) - Learn basics
3. [docs/vnc-language-reference.md](docs/vnc-language-reference.md) - Learn language

### Power User
1. [SDK_README.md](SDK_README.md) - All features
2. [docs/compilation-targets.md](docs/compilation-targets.md) - Advanced compilation
3. [docs/plugin-development.md](docs/plugin-development.md) - Extend functionality

### Developer/Contributor
1. [BUILD_INSTRUCTIONS.md](BUILD_INSTRUCTIONS.md) - Build setup
2. [CONTRIBUTING.md](CONTRIBUTING.md) - Contribution guide
3. [docs/architecture.md](docs/architecture.md) - System design

### DevOps/IT
1. [DESKTOP_SDK_INSTALLATION.md](DESKTOP_SDK_INSTALLATION.md) - Deployment
2. [docs/deployment-guide.md](docs/deployment-guide.md) - Cloud deployment
3. [scripts/README.md](scripts/README.md) - Automation scripts

---

## 🎯 Common Scenarios

### Scenario 1: "I just want to try NAVΛ Studio"
➡️ [SDK_QUICK_START.md](SDK_QUICK_START.md) → Run one-line install command

### Scenario 2: "I need to install on 50 employee computers"
➡️ [DESKTOP_SDK_INSTALLATION.md](DESKTOP_SDK_INSTALLATION.md) → Create deployment package

### Scenario 3: "I want to contribute code"
➡️ [BUILD_INSTRUCTIONS.md](BUILD_INSTRUCTIONS.md) → Build from source  
➡️ [CONTRIBUTING.md](CONTRIBUTING.md) → Contribution guidelines

### Scenario 4: "I need to customize the IDE"
➡️ [docs/plugin-development.md](docs/plugin-development.md) → Plugin development  
➡️ [docs/architecture.md](docs/architecture.md) → System architecture

### Scenario 5: "Deploy to Kubernetes cluster"
➡️ [docs/deployment-guide.md](docs/deployment-guide.md) → Cloud deployment  
➡️ [scripts/deploy.sh](scripts/deploy.sh) → Deployment script

---

## 🔗 Quick Links

| Need | Link |
|------|------|
| **Install now** | [SDK_QUICK_START.md](SDK_QUICK_START.md) |
| **Download** | [SDK_DOWNLOAD.md](SDK_DOWNLOAD.md) |
| **Build guide** | [BUILD_INSTRUCTIONS.md](BUILD_INSTRUCTIONS.md) |
| **User guide** | [GETTING_STARTED.md](GETTING_STARTED.md) |
| **All scripts** | [scripts/README.md](scripts/README.md) |
| **Language reference** | [docs/vnc-language-reference.md](docs/vnc-language-reference.md) |
| **Support** | [GitHub Issues](https://github.com/FrankAsanteVanLaarhoven/NAVA-Ai-Studio/issues) |

---

## 🌟 One-Line Install

**Don't want to read docs? Just run this:**

**macOS/Linux:**
```bash
curl -fsSL https://raw.githubusercontent.com/FrankAsanteVanLaarhoven/NAVA-Ai-Studio/main/scripts/install-sdk.sh | bash
```

**Windows:**
```powershell
irm https://raw.githubusercontent.com/FrankAsanteVanLaarhoven/NAVA-Ai-Studio/main/scripts/install-sdk.ps1 | iex
```

---

## 💡 Tips

- **Bookmark this page** for easy reference
- **Check [SDK_README.md](SDK_README.md)** for complete SDK overview
- **Join GitHub Discussions** for community help
- **Read [GETTING_STARTED.md](GETTING_STARTED.md)** after installation

---

## 🆘 Need Help?

**Can't find what you need?**

1. Check the [SDK_README.md](SDK_README.md) - comprehensive index
2. Search GitHub Issues - someone might have asked already
3. Open a new Issue - we're here to help!
4. Join Discord community - get real-time help

**Support Channels:**
- 📧 Email: support@navlambda.studio
- 💬 GitHub Discussions
- 🐛 GitHub Issues
- 🌐 Website: navlambda.studio

---

**NAVΛ Studio** - Making Navigation Calculus Programming Accessible 🚀⋋

*Choose your documentation path above and start your journey!*

