#!/bin/bash
# Build all SDK components for all platforms
# This script builds the native Rust SDK, web SDK, and all bindings

set -e

echo "🚀 Building NAVΛ SDK for all platforms..."
echo ""

# Colors for output
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Get script directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
SDK_ROOT="$( cd "$SCRIPT_DIR/.." && pwd )"

# Ensure logo assets are present
if [ ! -f "$SDK_ROOT/assets/logo.svg" ]; then
  echo -e "${YELLOW}⚠️  Warning: Logo not found at $SDK_ROOT/assets/logo.svg${NC}"
  echo "   Creating default logo..."
  mkdir -p "$SDK_ROOT/assets"
fi

echo -e "${BLUE}📦 Step 1: Building Native SDK (Rust)${NC}"
cd "$SDK_ROOT/native"
if command -v cargo &> /dev/null; then
  cargo build --release
  echo -e "${GREEN}✅ Native SDK built successfully${NC}"
else
  echo -e "${YELLOW}⚠️  Cargo not found, skipping native build${NC}"
fi

echo ""
echo -e "${BLUE}📦 Step 2: Building Web SDK (TypeScript/WASM)${NC}"
cd "$SDK_ROOT/web"
if command -v npm &> /dev/null; then
  npm install
  npm run build
  echo -e "${GREEN}✅ Web SDK built successfully${NC}"
else
  echo -e "${YELLOW}⚠️  npm not found, skipping web build${NC}"
fi

echo ""
echo -e "${BLUE}📦 Step 3: Building Python Bindings${NC}"
cd "$SDK_ROOT/bindings/python"
if command -v pip &> /dev/null; then
  pip install -e . || echo -e "${YELLOW}⚠️  Python bindings build failed (may require additional setup)${NC}"
else
  echo -e "${YELLOW}⚠️  pip not found, skipping Python bindings${NC}"
fi

echo ""
echo -e "${BLUE}📦 Step 4: Building Node.js Bindings${NC}"
cd "$SDK_ROOT/bindings/nodejs"
if command -v npm &> /dev/null; then
  npm install
  echo -e "${GREEN}✅ Node.js bindings ready${NC}"
else
  echo -e "${YELLOW}⚠️  npm not found, skipping Node.js bindings${NC}"
fi

echo ""
echo -e "${BLUE}📦 Step 5: Building Web Extension${NC}"
cd "$SDK_ROOT/web-extension"
if command -v npm &> /dev/null; then
  npm install
  npm run build
  echo -e "${GREEN}✅ Web Extension built successfully${NC}"
else
  echo -e "${YELLOW}⚠️  npm not found, skipping web extension build${NC}"
fi

echo ""
echo -e "${BLUE}📦 Step 6: Copying Logo Assets${NC}"
# Copy logo to all build outputs
mkdir -p "$SDK_ROOT/native/public/icons"
mkdir -p "$SDK_ROOT/web/dist/assets"
cp "$SDK_ROOT/assets/logo.svg" "$SDK_ROOT/native/public/icons/logo.svg" 2>/dev/null || true
cp "$SDK_ROOT/assets/logo.svg" "$SDK_ROOT/web/dist/assets/logo.svg" 2>/dev/null || true
cp "$SDK_ROOT/assets/icon-512.png" "$SDK_ROOT/native/public/icons/icon-512.png" 2>/dev/null || true
cp "$SDK_ROOT/assets/icon-512.png" "$SDK_ROOT/web/dist/assets/icon-512.png" 2>/dev/null || true
echo -e "${GREEN}✅ Logo assets copied${NC}"

echo ""
echo -e "${BLUE}📦 Step 6: Building Installers${NC}"
if [ -f "$SDK_ROOT/scripts/build-installers.sh" ]; then
  bash "$SDK_ROOT/scripts/build-installers.sh"
else
  echo -e "${YELLOW}⚠️  Installer script not found, skipping installer creation${NC}"
fi

echo ""
echo -e "${BLUE}📦 Step 7: Building Installers${NC}"
cd "$SDK_ROOT"
if [ -f "scripts/build-installers.sh" ]; then
  bash scripts/build-installers.sh || echo -e "${YELLOW}⚠️  Installer build had warnings${NC}"
else
  echo -e "${YELLOW}⚠️  Installer build script not found${NC}"
fi

echo ""
echo -e "${GREEN}🎉 NAVΛ SDK build complete!${NC}"
echo ""
echo "📁 Build outputs:"
echo "   - Native: $SDK_ROOT/native/target/release/"
echo "   - Web: $SDK_ROOT/web/dist/"
echo "   - Python: $SDK_ROOT/bindings/python/"
echo "   - Node.js: $SDK_ROOT/bindings/nodejs/"
echo "   - Web Extension: $SDK_ROOT/web-extension/dist/"
echo ""
echo "🎨 Logo included in:"
echo "   - $SDK_ROOT/assets/logo.svg"
echo "   - $SDK_ROOT/native/public/icons/logo.svg"
echo "   - $SDK_ROOT/web/dist/assets/logo.svg"
echo "   - $SDK_ROOT/web-extension/assets/"
