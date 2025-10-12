#!/bin/bash
# NAVΛ Studio - Development Environment Setup

set -e

echo "🔧 Setting up NAVΛ Studio development environment..."
echo "====================================================="

# Colors
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m'

# Check and install Rust
echo -e "${BLUE}Checking Rust installation...${NC}"
if ! command -v rustc >/dev/null 2>&1; then
    echo -e "${YELLOW}Installing Rust...${NC}"
    curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
    source "$HOME/.cargo/env"
fi
echo -e "${GREEN}✓ Rust installed${NC}"

# Check and install Node.js
echo -e "\n${BLUE}Checking Node.js installation...${NC}"
if ! command -v node >/dev/null 2>&1; then
    echo -e "${YELLOW}Please install Node.js 20+ from https://nodejs.org/${NC}"
    exit 1
fi
echo -e "${GREEN}✓ Node.js installed${NC}"

# Install wasm-pack for WebAssembly compilation
echo -e "\n${BLUE}Installing wasm-pack...${NC}"
if ! command -v wasm-pack >/dev/null 2>&1; then
    curl https://rustwasm.github.io/wasm-pack/installer/init.sh -sSf | sh
fi
echo -e "${GREEN}✓ wasm-pack installed${NC}"

# Install Tauri CLI
echo -e "\n${BLUE}Installing Tauri CLI...${NC}"
cargo install tauri-cli --version "^1.0" || true

# Install frontend dependencies
echo -e "\n${BLUE}Installing frontend dependencies...${NC}"
npm install

# Build WebAssembly preview
echo -e "\n${BLUE}Building WebAssembly preview engine...${NC}"
if [ -d "wasm-preview" ]; then
    cd wasm-preview
    wasm-pack build --target web || echo "⚠️  WASM build failed, continuing..."
    cd ..
fi

# Platform-specific setup
case "$(uname -s)" in
    Darwin*)
        echo -e "\n${BLUE}macOS detected - checking Xcode tools...${NC}"
        xcode-select --install 2>/dev/null || true
        ;;
    Linux*)
        echo -e "\n${BLUE}Linux detected - checking system dependencies...${NC}"
        echo "Please ensure you have: libwebkit2gtk-4.0-dev build-essential curl wget file libssl-dev libgtk-3-dev libayatana-appindicator3-dev librsvg2-dev"
        ;;
    MINGW*|MSYS*|CYGWIN*)
        echo -e "\n${BLUE}Windows detected${NC}"
        echo "Please ensure you have Microsoft C++ Build Tools installed"
        ;;
esac

echo -e "\n${GREEN}✓ Development environment setup complete!${NC}"
echo -e "\n📚 Next steps:"
echo "  1. Run 'npm run tauri:dev' to start the development server"
echo "  2. Edit code in src/ (React) or src-tauri/ (Rust)"
echo "  3. Changes will hot-reload automatically"
echo -e "\n🚀 Happy coding with NAVΛ Studio!"

