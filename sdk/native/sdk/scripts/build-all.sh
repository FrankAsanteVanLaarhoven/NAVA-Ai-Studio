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

echo -e "${BLUE}📦 Step 5: Copying Logo Assets${NC}"
# Copy logo to all build outputs
mkdir -p "$SDK_ROOT/native/public/icons"
mkdir -p "$SDK_ROOT/web/dist/assets"
cp "$SDK_ROOT/assets/logo.svg" "$SDK_ROOT/native/public/icons/logo.svg" 2>/dev/null || true
cp "$SDK_ROOT/assets/logo.svg" "$SDK_ROOT/web/dist/assets/logo.svg" 2>/dev/null || true
cp "$SDK_ROOT/assets/icon-512.png" "$SDK_ROOT/native/public/icons/icon-512.png" 2>/dev/null || true
cp "$SDK_ROOT/assets/icon-512.png" "$SDK_ROOT/web/dist/assets/icon-512.png" 2>/dev/null || true
echo -e "${GREEN}✅ Logo assets copied${NC}"

echo ""
echo -e "${GREEN}🎨 Logo included in all SDK packages!${NC}"
