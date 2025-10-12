#!/bin/bash
# NAVΛ Studio - Complete Build Script

set -e

echo "🚀 Building NAVΛ Studio IDE..."
echo "================================"

# Colors
GREEN='\033[0;32m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Check prerequisites
echo -e "${BLUE}Checking prerequisites...${NC}"
command -v node >/dev/null 2>&1 || { echo "❌ Node.js is required but not installed."; exit 1; }
command -v cargo >/dev/null 2>&1 || { echo "❌ Rust is required but not installed."; exit 1; }
command -v npm >/dev/null 2>&1 || { echo "❌ npm is required but not installed."; exit 1; }

echo -e "${GREEN}✓ Prerequisites satisfied${NC}"

# Install frontend dependencies
echo -e "\n${BLUE}Installing frontend dependencies...${NC}"
npm install

# Build WebAssembly preview engine
echo -e "\n${BLUE}Building WebAssembly preview engine...${NC}"
if [ -d "wasm-preview" ]; then
    cd wasm-preview
    if command -v wasm-pack >/dev/null 2>&1; then
        wasm-pack build --target web
    else
        echo "⚠️  wasm-pack not found, skipping WASM build"
    fi
    cd ..
fi

# Build frontend
echo -e "\n${BLUE}Building frontend...${NC}"
npm run build

# Build Tauri application
echo -e "\n${BLUE}Building Tauri application...${NC}"
npm run tauri:build

echo -e "\n${GREEN}✓ Build complete!${NC}"
echo -e "\n📦 Installers available in: src-tauri/target/release/bundle/"
echo -e "🎉 NAVΛ Studio is ready to launch!"

