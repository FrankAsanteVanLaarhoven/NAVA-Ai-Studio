#!/bin/bash

# NAVΛ Documentation Site Launcher
# Starts the Docusaurus documentation site

set -e

echo "================================================"
echo "📚 Starting NAVΛ Documentation Site"
echo "================================================"
echo ""

# Color codes
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m'

DOCS_DIR="docs-site"

# Navigate to docs directory
cd "$DOCS_DIR"

# Check if node_modules exists
if [ ! -d "node_modules" ]; then
    echo -e "${YELLOW}Installing Docusaurus dependencies...${NC}"
    npm install
    echo ""
fi

echo -e "${BLUE}Starting Docusaurus dev server...${NC}"
echo ""
echo "Documentation will be available at:"
echo "  http://localhost:3000"
echo ""
echo "Press Ctrl+C to stop"
echo ""

npm run start

