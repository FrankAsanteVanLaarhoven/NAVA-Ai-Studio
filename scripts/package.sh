#!/bin/bash
# NAVΛ Studio - Packaging Script

set -e

echo "📦 Packaging NAVΛ Studio for distribution..."
echo "============================================="

GREEN='\033[0;32m'
BLUE='\033[0;34m'
NC='\033[0m'

# Build the application
echo -e "${BLUE}Building application...${NC}"
./scripts/build.sh

# Create distribution directory
echo -e "\n${BLUE}Creating distribution package...${NC}"
mkdir -p dist/navlambda-studio

# Copy build artifacts
cp -r src-tauri/target/release/bundle/* dist/navlambda-studio/ 2>/dev/null || true

# Copy documentation
cp README.md dist/navlambda-studio/
cp -r docs dist/navlambda-studio/ 2>/dev/null || true

# Create archive
cd dist
tar -czf navlambda-studio-$(date +%Y%m%d).tar.gz navlambda-studio/
cd ..

echo -e "\n${GREEN}✓ Package created: dist/navlambda-studio-$(date +%Y%m%d).tar.gz${NC}"

