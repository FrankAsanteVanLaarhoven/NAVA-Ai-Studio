#!/bin/bash

# ROBOTIS-SYSTEMIC Full Integration Startup Script
# This script starts all required services for the fully integrated platform

# Don't exit on error - continue even if one service fails
# set -e

echo "================================================"
echo "🚀 Starting ROBOTIS-SYSTEMIC Integration"
echo "================================================"
echo ""

# Color codes for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Configuration
ROBOTIS_DIR="/Users/frankvanlaarhoven/Downloads/ROBOTIS-SYSTEMIC"
NAVA_DIR="/Users/frankvanlaarhoven/Desktop/NAVA Studio IDE"

# Check if directories exist
if [ ! -d "$ROBOTIS_DIR" ]; then
    echo -e "${RED}❌ ROBOTIS-SYSTEMIC directory not found: $ROBOTIS_DIR${NC}"
    exit 1
fi

if [ ! -d "$NAVA_DIR" ]; then
    echo -e "${RED}❌ NAVΛ STUDIO IDE directory not found: $NAVA_DIR${NC}"
    exit 1
fi

echo -e "${GREEN}✓ Found ROBOTIS-SYSTEMIC at: $ROBOTIS_DIR${NC}"
echo -e "${GREEN}✓ Found NAVΛ STUDIO IDE at: $NAVA_DIR${NC}"
echo ""

# Function to check if a port is in use
check_port() {
    local port=$1
    if lsof -Pi :$port -sTCP:LISTEN -t >/dev/null 2>&1; then
        return 0
    else
        return 1
    fi
}

# Function to wait for a service to be ready
wait_for_service() {
    local port=$1
    local name=$2
    local max_attempts=30
    local attempt=0
    
    echo -e "${YELLOW}⏳ Waiting for $name on port $port...${NC}"
    
    while [ $attempt -lt $max_attempts ]; do
        if check_port $port; then
            echo -e "${GREEN}✓ $name is ready on port $port${NC}"
            return 0
        fi
        attempt=$((attempt + 1))
        sleep 1
    done
    
    echo -e "${RED}❌ Timeout waiting for $name on port $port${NC}"
    return 1
}

# Kill existing processes on required ports
echo "🧹 Cleaning up existing processes..."
for port in 8080 3000 5175; do
    if check_port $port; then
        echo -e "${YELLOW}⚠ Port $port is in use, attempting to kill process...${NC}"
        lsof -ti:$port | xargs kill -9 2>/dev/null || true
        sleep 1
    fi
done
echo ""

# Start robotd (Rust backend) on port 8080
echo "================================================"
echo -e "${BLUE}📦 Starting robotd (Rust backend)${NC}"
echo "================================================"
cd "$ROBOTIS_DIR/services/robotd"

# Check if Cargo.toml exists
if [ ! -f "Cargo.toml" ]; then
    echo -e "${RED}❌ Cargo.toml not found in robotd directory${NC}"
    exit 1
fi

# Set environment variables for robotd
export SSE_TOKEN="demo-token-123"
export API_BASE="http://localhost:8080"
export RUST_LOG="robotd=info,tower_http=debug"

# Start robotd in background
echo -e "${YELLOW}Starting robotd with SSE_TOKEN=$SSE_TOKEN...${NC}"
cargo run --release > "$NAVA_DIR/logs/robotd.log" 2>&1 &
ROBOTD_PID=$!
echo -e "${GREEN}✓ robotd started with PID: $ROBOTD_PID${NC}"
echo $ROBOTD_PID > "$NAVA_DIR/.robotd.pid"

# Wait for robotd to be ready
if ! wait_for_service 8080 "robotd"; then
    echo -e "${RED}❌ Failed to start robotd${NC}"
    kill $ROBOTD_PID 2>/dev/null || true
    exit 1
fi
echo ""

# Start Univarm Web (Next.js) on port 3000
echo "================================================"
echo -e "${BLUE}🌐 Starting Univarm Web (Next.js)${NC}"
echo "================================================"
cd "$ROBOTIS_DIR/apps/web"

# Create .env.local if it doesn't exist
if [ ! -f ".env.local" ]; then
    echo -e "${YELLOW}Creating .env.local...${NC}"
    cat > .env.local << EOF
NEXT_PUBLIC_API_BASE=http://localhost:8080
API_BASE=http://localhost:8080
NEXT_PUBLIC_SSE_TOKEN=demo-token-123
EOF
    echo -e "${GREEN}✓ Created .env.local${NC}"
fi

# Check if node_modules exists, if not install dependencies
if [ ! -d "node_modules" ]; then
    echo -e "${YELLOW}Installing dependencies...${NC}"
    npm install
fi

# Start Next.js in background
echo -e "${YELLOW}Starting Univarm Web on port 3000...${NC}"
npm run dev > "$NAVA_DIR/logs/univarm-web.log" 2>&1 &
UNIVARM_PID=$!
echo -e "${GREEN}✓ Univarm Web started with PID: $UNIVARM_PID${NC}"
echo $UNIVARM_PID > "$NAVA_DIR/.univarm.pid"

# Wait for Univarm to be ready
if ! wait_for_service 3000 "Univarm Web"; then
    echo -e "${RED}❌ Failed to start Univarm Web${NC}"
    kill $UNIVARM_PID 2>/dev/null || true
    kill $ROBOTD_PID 2>/dev/null || true
    exit 1
fi
echo ""

# Start NAVΛ Studio IDE (Vite) on port 5175
echo "================================================"
echo -e "${BLUE}💻 Starting NAVΛ Studio IDE (Vite)${NC}"
echo "================================================"
cd "$NAVA_DIR"

# Create logs directory if it doesn't exist
mkdir -p logs

# Check if node_modules exists
if [ ! -d "node_modules" ]; then
    echo -e "${YELLOW}Installing dependencies...${NC}"
    npm install
fi

# Start Vite in background
echo -e "${YELLOW}Starting NAVΛ Studio IDE on port 5175...${NC}"
npm run dev:vite > "$NAVA_DIR/logs/nava-studio.log" 2>&1 &
NAVA_PID=$!
echo -e "${GREEN}✓ NAVΛ Studio IDE started with PID: $NAVA_PID${NC}"
echo $NAVA_PID > "$NAVA_DIR/.nava.pid"

# Wait for NAVΛ to be ready
if ! wait_for_service 5175 "NAVΛ Studio IDE"; then
    echo -e "${RED}❌ Failed to start NAVΛ Studio IDE${NC}"
    kill $NAVA_PID 2>/dev/null || true
    kill $UNIVARM_PID 2>/dev/null || true
    kill $ROBOTD_PID 2>/dev/null || true
    exit 1
fi
echo ""

# All services started successfully
echo "================================================"
echo -e "${GREEN}✅ ALL SERVICES STARTED SUCCESSFULLY!${NC}"
echo "================================================"
echo ""
echo "Service Status:"
echo "  • robotd (Backend):       http://localhost:8080"
echo "  • Univarm Web (Frontend): http://localhost:3000"
echo "  • NAVΛ Studio IDE:        http://localhost:5175"
echo ""
echo "Access Points:"
echo "  🌐 Main IDE:       http://localhost:5175/workspace.html"
echo "  🔷 Univarm Direct: http://localhost:3000"
echo "  🔷 Univarm in IDE: Click 'Univarm' in the dock"
echo "  📊 API Endpoint:   http://localhost:8080/api"
echo "  📡 SSE Stream:     http://localhost:8080/api/rt/subscribe?token=demo-token-123"
echo ""
echo "Process IDs:"
echo "  • robotd:      $ROBOTD_PID"
echo "  • Univarm Web: $UNIVARM_PID"
echo "  • NAVΛ IDE:    $NAVA_PID"
echo ""
echo "Logs:"
echo "  • robotd:      $NAVA_DIR/logs/robotd.log"
echo "  • Univarm Web: $NAVA_DIR/logs/univarm-web.log"
echo "  • NAVΛ IDE:    $NAVA_DIR/logs/nava-studio.log"
echo ""
echo "To stop all services:"
echo "  ./stop-robotis-system.sh"
echo ""
echo "Opening NAVΛ Studio IDE in browser..."
sleep 2

# Open browser
if command -v open &> /dev/null; then
    # macOS
    open "http://localhost:5175/workspace.html"
elif command -v xdg-open &> /dev/null; then
    # Linux
    xdg-open "http://localhost:5175/workspace.html"
elif command -v start &> /dev/null; then
    # Windows
    start "http://localhost:5175/workspace.html"
else
    echo -e "${YELLOW}⚠ Could not open browser automatically. Please open: http://localhost:5175/workspace.html${NC}"
fi

echo ""
echo -e "${GREEN}🎉 ROBOTIS-SYSTEMIC is fully integrated and running!${NC}"
echo ""

# Keep script running and monitor processes
trap 'echo ""; echo "Stopping all services..."; kill $NAVA_PID $UNIVARM_PID $ROBOTD_PID 2>/dev/null; exit 0' SIGINT SIGTERM

echo "Press Ctrl+C to stop all services..."
wait

