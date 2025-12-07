#!/bin/bash

# NAVΛ Studio IDE - Simple Startup Script
# Run this anytime to start your robotics development platform!

set -e

echo "╔════════════════════════════════════════════════╗"
echo "║     🚀 NAVΛ Studio IDE - Quick Start 🚀        ║"
echo "╚════════════════════════════════════════════════╝"
echo ""

# Color codes
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo -e "${BLUE}Starting NAVΛ Studio IDE...${NC}"
echo ""

# Check if node_modules exists
if [ ! -d "node_modules" ]; then
    echo -e "${YELLOW}⚠ Installing dependencies (first time)...${NC}"
    npm install
    echo ""
fi

# Create logs directory
mkdir -p logs

echo -e "${GREEN}✓ Ready to start!${NC}"
echo ""
echo "Choose your mode:"
echo ""
echo "  1) Start IDE only (Vite dev server)"
echo "  2) Start with Univarm Advanced backend"
echo "  3) Start complete ROBOTIS-SYSTEMIC platform"
echo "  4) Start ROS 2 Kobuki demo"
echo ""
read -p "Enter choice (1-4): " choice
echo ""

case $choice in
  1)
    echo -e "${BLUE}Starting NAVΛ Studio IDE...${NC}"
    npm run dev:vite
    ;;
  2)
    echo -e "${BLUE}Starting Univarm Advanced backend + IDE...${NC}"
    ./start-univarm-backend.sh &
    sleep 3
    npm run dev:vite
    ;;
  3)
    echo -e "${BLUE}Starting complete ROBOTIS-SYSTEMIC platform...${NC}"
    ./start-robotis-system.sh
    ;;
  4)
    echo -e "${BLUE}Starting ROS 2 Kobuki demo...${NC}"
    ./start-kobuki-demo.sh
    ;;
  *)
    echo -e "${YELLOW}Invalid choice. Starting IDE only...${NC}"
    npm run dev:vite
    ;;
esac

