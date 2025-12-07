#!/bin/bash

# Univarm Advanced Backend Startup Script
# Starts the Rust backend for path planning

set -e

echo "================================================"
echo "🦀 Starting Univarm Advanced Backend (Rust)"
echo "================================================"
echo ""

# Color codes
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

BACKEND_DIR="/Users/frankvanlaarhoven/Downloads/NAVΛ STUDIO IDE/backend-univarm/rust"

# Check if backend directory exists
if [ ! -d "$BACKEND_DIR" ]; then
    echo -e "${RED}❌ Backend directory not found: $BACKEND_DIR${NC}"
    exit 1
fi

echo -e "${GREEN}✓ Found backend directory${NC}"
echo ""

# Check if Rust is installed
if ! command -v cargo &> /dev/null; then
    echo -e "${RED}❌ Rust/Cargo not found. Please install Rust:${NC}"
    echo "   curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh"
    exit 1
fi

echo -e "${GREEN}✓ Rust/Cargo found: $(cargo --version)${NC}"
echo ""

# Navigate to backend directory
cd "$BACKEND_DIR"

# Set environment variables
export NAVL_SOLVER="${NAVL_SOLVER:-wasm}"  # Default to wasm mode
export RUST_LOG="${RUST_LOG:-info}"

echo -e "${YELLOW}Configuration:${NC}"
echo "  Solver mode: $NAVL_SOLVER"
echo "  Rust log level: $RUST_LOG"
echo ""

# Check if running
if lsof -Pi :8080 -sTCP:LISTEN -t >/dev/null 2>&1; then
    echo -e "${YELLOW}⚠ Port 8080 already in use${NC}"
    echo "  Kill existing process? (y/n)"
    read -r response
    if [[ "$response" =~ ^[Yy]$ ]]; then
        lsof -ti:8080 | xargs kill -9 2>/dev/null || true
        sleep 1
    else
        exit 1
    fi
fi

echo -e "${YELLOW}Starting Rust backend...${NC}"
echo ""

# Run the backend
cargo run --release 2>&1 | tee ../../logs/univarm-backend.log &
BACKEND_PID=$!

echo -e "${GREEN}✓ Backend started with PID: $BACKEND_PID${NC}"
echo "$BACKEND_PID" > ../../.univarm-backend.pid

# Wait for backend to be ready
echo -e "${YELLOW}⏳ Waiting for backend on port 8080...${NC}"
for i in {1..30}; do
    if lsof -Pi :8080 -sTCP:LISTEN -t >/dev/null 2>&1; then
        echo -e "${GREEN}✓ Backend is ready!${NC}"
        echo ""
        echo "API Endpoints:"
        echo "  POST http://localhost:8080/api/solve"
        echo "  POST http://localhost:8080/api/drive/path"
        echo "  GET  http://localhost:8080/api/drive/sse"
        echo "  GET  http://localhost:8080/api/ui/presets"
        echo ""
        echo "Press Ctrl+C to stop"
        wait $BACKEND_PID
        exit 0
    fi
    sleep 1
done

echo -e "${RED}❌ Backend failed to start${NC}"
exit 1

