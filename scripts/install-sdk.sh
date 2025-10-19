#!/bin/bash
# NAVΛ Studio SDK - Quick Installation Script
# This script automates the installation of NAVΛ Studio on your desktop

set -e

# Colors for pretty output
RED='\033[0;31m'
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${BLUE}"
echo "╔═══════════════════════════════════════════════╗"
echo "║   NAVΛ STUDIO IDE - Desktop SDK Installer   ║"
echo "║   Van Laarhoven Navigation Calculus          ║"
echo "╚═══════════════════════════════════════════════╝"
echo -e "${NC}"

# Detect OS
OS="unknown"
if [[ "$OSTYPE" == "linux-gnu"* ]]; then
    OS="linux"
elif [[ "$OSTYPE" == "darwin"* ]]; then
    OS="macos"
elif [[ "$OSTYPE" == "msys" || "$OSTYPE" == "cygwin" ]]; then
    OS="windows"
fi

echo -e "${BLUE}Detected OS: ${GREEN}$OS${NC}\n"

# Check if we should build from source or download pre-built
echo "Installation Options:"
echo "  1) Download pre-built installer (Recommended)"
echo "  2) Build from source (for developers)"
echo ""
read -p "Choose an option (1 or 2): " INSTALL_CHOICE

if [ "$INSTALL_CHOICE" == "1" ]; then
    # Download pre-built
    echo -e "\n${BLUE}Downloading pre-built installer...${NC}"
    
    GITHUB_REPO="FrankAsanteVanLaarhoven/NAVA-Ai-Studio"
    BASE_URL="https://github.com/$GITHUB_REPO/releases/latest/download"
    
    case $OS in
        macos)
            echo -e "${BLUE}Downloading for macOS...${NC}"
            
            # Detect architecture
            ARCH=$(uname -m)
            if [ "$ARCH" == "arm64" ]; then
                echo -e "${GREEN}Detected Apple Silicon (M1/M2/M3)${NC}"
                INSTALLER="NAVA-Studio_1.0.0_aarch64.dmg"
            else
                echo -e "${GREEN}Detected Intel processor${NC}"
                INSTALLER="NAVA-Studio_1.0.0_universal.dmg"
            fi
            
            echo "Downloading $INSTALLER..."
            curl -LO "$BASE_URL/$INSTALLER"
            
            echo -e "\n${GREEN}✓ Downloaded successfully!${NC}"
            echo -e "${YELLOW}To install:${NC}"
            echo "  1. Open the downloaded DMG file"
            echo "  2. Drag 'NAVΛ Studio' to your Applications folder"
            echo "  3. Launch from Applications or Spotlight"
            echo ""
            echo "Opening DMG now..."
            open "$INSTALLER"
            ;;
            
        linux)
            echo -e "${BLUE}Downloading for Linux...${NC}"
            
            # Detect distro
            if [ -f /etc/debian_version ]; then
                echo -e "${GREEN}Detected Debian/Ubuntu${NC}"
                INSTALLER="nava-studio_1.0.0_amd64.deb"
                echo "Downloading $INSTALLER..."
                wget "$BASE_URL/$INSTALLER"
                
                echo -e "\n${YELLOW}Installing...${NC}"
                sudo dpkg -i "$INSTALLER"
                sudo apt-get install -f -y
                
                echo -e "\n${GREEN}✓ Installation complete!${NC}"
                echo "Launch with: nava-studio"
                
            elif [ -f /etc/redhat-release ]; then
                echo -e "${GREEN}Detected Fedora/RHEL${NC}"
                INSTALLER="nava-studio-1.0.0-1.x86_64.rpm"
                echo "Downloading $INSTALLER..."
                wget "$BASE_URL/$INSTALLER"
                
                echo -e "\n${YELLOW}Installing...${NC}"
                sudo rpm -i "$INSTALLER"
                
                echo -e "\n${GREEN}✓ Installation complete!${NC}"
                echo "Launch with: nava-studio"
            else
                # Generic Linux - use AppImage
                echo -e "${GREEN}Using universal AppImage${NC}"
                INSTALLER="NAVA-Studio_1.0.0_amd64.AppImage"
                echo "Downloading $INSTALLER..."
                wget "$BASE_URL/$INSTALLER"
                
                chmod +x "$INSTALLER"
                
                echo -e "\n${GREEN}✓ Downloaded successfully!${NC}"
                echo "Launch with: ./$INSTALLER"
            fi
            ;;
            
        windows)
            echo -e "${YELLOW}Windows detected. Please use PowerShell to install:${NC}"
            echo ""
            echo "Run the following in PowerShell:"
            echo "  Invoke-WebRequest -Uri \"$BASE_URL/NAVA-Studio_1.0.0_x64_en-US.msi\" -OutFile \"NAVA-Studio-Setup.msi\""
            echo "  Start-Process \"NAVA-Studio-Setup.msi\""
            ;;
            
        *)
            echo -e "${RED}Unsupported OS. Please visit:${NC}"
            echo "https://github.com/$GITHUB_REPO/releases"
            exit 1
            ;;
    esac
    
else
    # Build from source
    echo -e "\n${BLUE}Building from source...${NC}\n"
    
    # Check prerequisites
    echo -e "${BLUE}Checking prerequisites...${NC}"
    
    if ! command -v node &> /dev/null; then
        echo -e "${RED}✗ Node.js not found${NC}"
        echo "Please install Node.js 18+ from: https://nodejs.org/"
        exit 1
    fi
    echo -e "${GREEN}✓ Node.js $(node --version)${NC}"
    
    if ! command -v cargo &> /dev/null; then
        echo -e "${RED}✗ Rust not found${NC}"
        echo "Installing Rust..."
        curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
        source $HOME/.cargo/env
    fi
    echo -e "${GREEN}✓ Rust $(cargo --version)${NC}"
    
    if ! command -v npm &> /dev/null; then
        echo -e "${RED}✗ npm not found${NC}"
        echo "npm should come with Node.js"
        exit 1
    fi
    echo -e "${GREEN}✓ npm $(npm --version)${NC}"
    
    # Install platform-specific dependencies
    case $OS in
        macos)
            echo -e "\n${BLUE}Installing macOS dependencies...${NC}"
            if ! command -v xcode-select &> /dev/null; then
                echo "Installing Xcode Command Line Tools..."
                xcode-select --install
            fi
            ;;
            
        linux)
            echo -e "\n${BLUE}Installing Linux dependencies...${NC}"
            if [ -f /etc/debian_version ]; then
                sudo apt-get update
                sudo apt-get install -y \
                    libwebkit2gtk-4.0-dev \
                    build-essential \
                    curl wget file \
                    libssl-dev \
                    libgtk-3-dev \
                    libayatana-appindicator3-dev \
                    librsvg2-dev
            elif [ -f /etc/redhat-release ]; then
                sudo dnf install -y \
                    webkit2gtk4.0-devel \
                    openssl-devel \
                    curl wget file \
                    libappindicator-gtk3-devel \
                    librsvg2-devel
                sudo dnf groupinstall -y "C Development Tools and Libraries"
            fi
            ;;
    esac
    
    # Install dependencies
    echo -e "\n${BLUE}Installing project dependencies...${NC}"
    npm install
    
    # Build
    echo -e "\n${BLUE}Building NAVΛ Studio...${NC}"
    echo "This may take 10-20 minutes on first build..."
    npm run build
    npm run tauri:build
    
    echo -e "\n${GREEN}╔════════════════════════════════════╗${NC}"
    echo -e "${GREEN}║   ✓ Build Complete!                ║${NC}"
    echo -e "${GREEN}╚════════════════════════════════════╝${NC}"
    echo ""
    echo -e "${BLUE}Installers available at:${NC}"
    echo "  src-tauri/target/release/bundle/"
    echo ""
    
    # Show specific installer path
    case $OS in
        macos)
            echo -e "${GREEN}macOS DMG:${NC} src-tauri/target/release/bundle/dmg/"
            ;;
        linux)
            echo -e "${GREEN}Linux packages:${NC}"
            echo "  - AppImage: src-tauri/target/release/bundle/appimage/"
            echo "  - Debian: src-tauri/target/release/bundle/deb/"
            echo "  - RPM: src-tauri/target/release/bundle/rpm/"
            ;;
    esac
fi

echo ""
echo -e "${BLUE}═══════════════════════════════════════════════${NC}"
echo -e "${GREEN}🎉 NAVΛ Studio Installation Complete!${NC}"
echo -e "${BLUE}═══════════════════════════════════════════════${NC}"
echo ""
echo -e "${YELLOW}Next Steps:${NC}"
echo "  1. Launch NAVΛ Studio"
echo "  2. Open Command Palette (Ctrl+Shift+P / ⌘⇧P)"
echo "  3. Try 'Help: Getting Started'"
echo "  4. Explore example projects"
echo ""
echo -e "${BLUE}Documentation:${NC} docs/GETTING_STARTED.md"
echo -e "${BLUE}Support:${NC} https://github.com/FrankAsanteVanLaarhoven/NAVA-Ai-Studio/issues"
echo ""
echo -e "${GREEN}Happy Navigation Calculus Programming! 🚀⋋${NC}"
echo ""

