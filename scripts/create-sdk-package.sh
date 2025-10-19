#!/bin/bash
# NAVΛ Studio - Create Complete SDK Distribution Package
# This creates a downloadable SDK package with all installers and documentation

set -e

GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo -e "${BLUE}"
echo "╔═══════════════════════════════════════════════╗"
echo "║   NAVΛ STUDIO - SDK Package Creator         ║"
echo "╚═══════════════════════════════════════════════╝"
echo -e "${NC}\n"

# Get version from package.json
VERSION=$(node -p "require('./package.json').version")
TIMESTAMP=$(date +%Y%m%d-%H%M%S)
PACKAGE_NAME="navlambda-studio-sdk-v${VERSION}-${TIMESTAMP}"
DIST_DIR="dist/${PACKAGE_NAME}"

echo -e "${BLUE}Creating SDK package: ${GREEN}${PACKAGE_NAME}${NC}\n"

# Create directory structure
echo -e "${BLUE}Setting up directory structure...${NC}"
mkdir -p "${DIST_DIR}"/{installers/{macos,windows,linux},docs,examples,scripts}

# Build the application
echo -e "\n${BLUE}Building application...${NC}"
echo "This will take several minutes..."

# Check if we should build or use existing
if [ "$1" == "--no-build" ]; then
    echo -e "${YELLOW}Skipping build (using existing artifacts)${NC}"
else
    npm install
    npm run build
    npm run tauri:build
fi

# Copy installers (if they exist)
echo -e "\n${BLUE}Copying installers...${NC}"

# macOS
if [ -d "src-tauri/target/release/bundle/dmg" ]; then
    cp src-tauri/target/release/bundle/dmg/*.dmg "${DIST_DIR}/installers/macos/" 2>/dev/null || true
    echo -e "${GREEN}✓ macOS DMG${NC}"
fi

# Windows
if [ -d "src-tauri/target/release/bundle/msi" ]; then
    cp src-tauri/target/release/bundle/msi/*.msi "${DIST_DIR}/installers/windows/" 2>/dev/null || true
    echo -e "${GREEN}✓ Windows MSI${NC}"
fi
if [ -d "src-tauri/target/release/bundle/nsis" ]; then
    cp src-tauri/target/release/bundle/nsis/*.exe "${DIST_DIR}/installers/windows/" 2>/dev/null || true
    echo -e "${GREEN}✓ Windows EXE${NC}"
fi

# Linux
if [ -d "src-tauri/target/release/bundle/appimage" ]; then
    cp src-tauri/target/release/bundle/appimage/*.AppImage "${DIST_DIR}/installers/linux/" 2>/dev/null || true
    echo -e "${GREEN}✓ Linux AppImage${NC}"
fi
if [ -d "src-tauri/target/release/bundle/deb" ]; then
    cp src-tauri/target/release/bundle/deb/*.deb "${DIST_DIR}/installers/linux/" 2>/dev/null || true
    echo -e "${GREEN}✓ Linux DEB${NC}"
fi
if [ -d "src-tauri/target/release/bundle/rpm" ]; then
    cp src-tauri/target/release/bundle/rpm/*.rpm "${DIST_DIR}/installers/linux/" 2>/dev/null || true
    echo -e "${GREEN}✓ Linux RPM${NC}"
fi

# Copy documentation
echo -e "\n${BLUE}Copying documentation...${NC}"
cp README.md "${DIST_DIR}/"
cp GETTING_STARTED.md "${DIST_DIR}/" 2>/dev/null || true
cp DESKTOP_SDK_INSTALLATION.md "${DIST_DIR}/"
cp SDK_DOWNLOAD.md "${DIST_DIR}/" 2>/dev/null || true
cp LICENSE-MIT "${DIST_DIR}/"
cp LICENSE-APACHE "${DIST_DIR}/"
cp -r docs/* "${DIST_DIR}/docs/" 2>/dev/null || true
echo -e "${GREEN}✓ Documentation copied${NC}"

# Copy examples
echo -e "\n${BLUE}Copying examples...${NC}"
cp -r assets/examples/* "${DIST_DIR}/examples/" 2>/dev/null || true
echo -e "${GREEN}✓ Examples copied${NC}"

# Copy installation scripts
echo -e "\n${BLUE}Copying installation scripts...${NC}"
cp scripts/install-sdk.sh "${DIST_DIR}/scripts/"
chmod +x "${DIST_DIR}/scripts/install-sdk.sh"
echo -e "${GREEN}✓ Scripts copied${NC}"

# Create SDK README
echo -e "\n${BLUE}Creating SDK README...${NC}"
cat > "${DIST_DIR}/README-SDK.md" << 'EOF'
# NAVΛ Studio Desktop SDK

## 📦 What's Included

This SDK package contains everything you need to install and use NAVΛ Studio on your desktop.

### Directory Structure

```
navlambda-studio-sdk/
├── installers/
│   ├── macos/          # macOS DMG installers
│   ├── windows/        # Windows MSI/EXE installers
│   └── linux/          # Linux AppImage/DEB/RPM packages
├── docs/               # Complete documentation
├── examples/           # Example VNC projects
├── scripts/            # Installation scripts
├── README.md           # Main README
├── DESKTOP_SDK_INSTALLATION.md  # Installation guide
└── SDK_DOWNLOAD.md     # Download instructions
```

## 🚀 Quick Installation

### Easy Install (Recommended)

Run the installation script:

```bash
cd scripts
./install-sdk.sh
```

This will guide you through the installation process for your platform.

### Manual Installation

#### macOS
1. Go to `installers/macos/`
2. Open the `.dmg` file
3. Drag NAVΛ Studio to Applications
4. Launch from Applications or Spotlight

#### Windows
1. Go to `installers/windows/`
2. Run the `.msi` installer
3. Follow the installation wizard
4. Launch from Start Menu

#### Linux (Ubuntu/Debian)
```bash
cd installers/linux
sudo dpkg -i nava-studio_*.deb
sudo apt-get install -f
nava-studio
```

#### Linux (Fedora/RHEL)
```bash
cd installers/linux
sudo rpm -i nava-studio-*.rpm
nava-studio
```

#### Linux (Universal - AppImage)
```bash
cd installers/linux
chmod +x NAVA-Studio_*.AppImage
./NAVA-Studio_*.AppImage
```

## 📚 Documentation

- **DESKTOP_SDK_INSTALLATION.md** - Complete installation guide
- **GETTING_STARTED.md** - Getting started tutorial
- **docs/** - Full documentation
  - `vnc-language-reference.md` - VNC language guide
  - `architecture.md` - System architecture
  - `plugin-development.md` - Creating plugins
  - And more...

## 🎯 Example Projects

Check out the `examples/` directory for sample VNC projects:

- `basic-navigation.vnc` - Basic navigation concepts
- `quantum-navigation.vnc` - Quantum navigation
- `energy-landscape.vnc` - Energy landscape visualization
- `consciousness-integration.vnc` - Consciousness models
- And more...

## 🔧 System Requirements

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

## 🌟 Features

✅ **Native Desktop Performance** - No browser required
✅ **Offline Mode** - Full functionality without internet
✅ **3D Visualization** - GPU-accelerated navigation rendering
✅ **Multi-Target Compilation** - C++, Python, GLSL, WASM
✅ **Language Server** - IntelliSense & code completion
✅ **Real-time Preview** - See results as you code
✅ **Plugin System** - Extend functionality
✅ **Cloud Sync** - Optional cloud integration

## 🆘 Support

- **Documentation**: See `docs/` directory
- **GitHub**: https://github.com/FrankAsanteVanLaarhoven/NAVA-Ai-Studio
- **Issues**: https://github.com/FrankAsanteVanLaarhoven/NAVA-Ai-Studio/issues
- **Discussions**: https://github.com/FrankAsanteVanLaarhoven/NAVA-Ai-Studio/discussions

## 📝 License

Dual-licensed under MIT and Apache 2.0.
See LICENSE-MIT and LICENSE-APACHE for details.

## 🎓 Getting Started

1. Install NAVΛ Studio using one of the methods above
2. Launch the application
3. Open Command Palette (Ctrl+Shift+P or ⌘⇧P)
4. Type "Help: Getting Started"
5. Open an example project from the examples/ directory

## 💖 Credits

**Built by Frank Van Laarhoven**

NAVΛ Studio IDE - The world's first IDE for Van Laarhoven Navigation Calculus

*Making mathematical navigation programming accessible to everyone* 🚀⋋

---

**Version**: $(VERSION)
**Build Date**: $(TIMESTAMP)
**Package**: $(PACKAGE_NAME)

For the latest version, visit: https://github.com/FrankAsanteVanLaarhoven/NAVA-Ai-Studio/releases
EOF

# Replace template variables
sed -i.bak "s/\$(VERSION)/$VERSION/g" "${DIST_DIR}/README-SDK.md"
sed -i.bak "s/\$(TIMESTAMP)/$TIMESTAMP/g" "${DIST_DIR}/README-SDK.md"
sed -i.bak "s/\$(PACKAGE_NAME)/$PACKAGE_NAME/g" "${DIST_DIR}/README-SDK.md"
rm "${DIST_DIR}/README-SDK.md.bak"

echo -e "${GREEN}✓ SDK README created${NC}"

# Create checksums
echo -e "\n${BLUE}Generating checksums...${NC}"
cd "${DIST_DIR}/installers"
find . -type f \( -name "*.dmg" -o -name "*.msi" -o -name "*.exe" -o -name "*.AppImage" -o -name "*.deb" -o -name "*.rpm" \) -exec shasum -a 256 {} \; > ../CHECKSUMS.txt 2>/dev/null || true
cd ../../..
echo -e "${GREEN}✓ Checksums generated${NC}"

# Create archive
echo -e "\n${BLUE}Creating distribution archive...${NC}"
cd dist
tar -czf "${PACKAGE_NAME}.tar.gz" "${PACKAGE_NAME}/"
ZIP_SIZE=$(du -h "${PACKAGE_NAME}.tar.gz" | cut -f1)
cd ..

echo -e "\n${GREEN}╔════════════════════════════════════════════╗${NC}"
echo -e "${GREEN}║   ✓ SDK Package Created Successfully!      ║${NC}"
echo -e "${GREEN}╚════════════════════════════════════════════╝${NC}"
echo ""
echo -e "${BLUE}Package Details:${NC}"
echo -e "  Name: ${GREEN}${PACKAGE_NAME}${NC}"
echo -e "  Version: ${GREEN}${VERSION}${NC}"
echo -e "  Size: ${GREEN}${ZIP_SIZE}${NC}"
echo -e "  Location: ${GREEN}dist/${PACKAGE_NAME}.tar.gz${NC}"
echo ""
echo -e "${BLUE}Package Contents:${NC}"
echo -e "  ✓ Installers for macOS, Windows, Linux"
echo -e "  ✓ Complete documentation"
echo -e "  ✓ Example projects"
echo -e "  ✓ Installation scripts"
echo -e "  ✓ Checksums for verification"
echo ""
echo -e "${YELLOW}Next Steps:${NC}"
echo "  1. Test the installers on each platform"
echo "  2. Upload to GitHub Releases"
echo "  3. Update download links in documentation"
echo ""
echo -e "${BLUE}To upload to GitHub:${NC}"
echo "  gh release create v${VERSION} dist/${PACKAGE_NAME}.tar.gz"
echo ""
echo -e "${GREEN}🎉 SDK package is ready for distribution!${NC}"
echo ""

