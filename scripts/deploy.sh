#!/bin/bash
# NAVΛ Studio - Deployment Script

set -e

echo "☁️  Deploying NAVΛ Studio..."
echo "============================"

GREEN='\033[0;32m'
BLUE='\033[0;34m'
NC='\033[0m'

# Build the application
echo -e "${BLUE}Building for production...${NC}"
./scripts/build.sh

echo -e "\n${GREEN}✓ Build complete${NC}"
echo -e "\n📦 Ready for deployment"
echo "Choose your deployment target:"
echo "  - Docker: docker build -t navlambda-studio ."
echo "  - GitHub Releases: Create a new release and upload artifacts"
echo "  - Website: Upload to your hosting provider"

