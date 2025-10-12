#!/bin/bash
# NAVΛ Studio - Test Execution Script

set -e

echo "🧪 Running NAVΛ Studio Tests..."
echo "================================"

# Colors
GREEN='\033[0;32m'
BLUE='\033[0;34m'
NC='\033[0m'

# Run Rust tests
echo -e "${BLUE}Running Rust backend tests...${NC}"
cd src-tauri
cargo test --all-features
cd ..

echo -e "${GREEN}✓ Rust tests passed${NC}"

# Run frontend tests
echo -e "\n${BLUE}Running frontend tests...${NC}"
npm run test

echo -e "\n${GREEN}✓ All tests passed!${NC}"

