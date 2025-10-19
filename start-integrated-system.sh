#!/bin/bash

# NAVΛ Integrated System Startup
# ================================
#
# Starts the complete world-class system:
# 1. Rust Simulation Backend (NAVΛ SIM)
# 2. Dataset Server (Waymo, KITTI, RT-X)
# 3. NIF Integration Bridge
# 4. Frontend (React/Vite)
#
# Patent-worthy integrated architecture!

echo "🌟 Starting NAVΛ Integrated System"
echo "=================================="
echo ""

# Colors for output
GREEN='\033[0.32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Check if Python is installed
if ! command -v python3 &> /dev/null; then
    echo "❌ Python 3 is required but not installed"
    exit 1
fi

# Check if Rust is installed
if ! command -v cargo &> /dev/null; then
    echo "❌ Rust is required but not installed"
    echo "📦 Install from: https://rustup.rs/"
    exit 1
fi

# Check if Node.js is installed
if ! command -v npm &> /dev/null; then
    echo "❌ Node.js is required but not installed"
    exit 1
fi

echo "${BLUE}[1/4] Installing Python dependencies...${NC}"
cd simulation-platform
python3 -m pip install -q -r requirements-dataset.txt
if [ $? -ne 0 ]; then
    echo "${YELLOW}⚠️  Some dependencies failed to install, continuing anyway...${NC}"
fi

echo ""
echo "${BLUE}[2/4] Starting Rust Simulation Backend (port 3030)...${NC}"
cargo run --release --bin navlambda-simulation-platform -- --api-only --port 3030 > /tmp/navlambda-sim.log 2>&1 &
SIM_PID=$!
echo "✅ Simulation backend started (PID: $SIM_PID)"

echo ""
echo "${BLUE}[3/4] Starting Dataset Server (port 5000)...${NC}"
python3 dataset_server.py > /tmp/navlambda-dataset.log 2>&1 &
DATASET_PID=$!
echo "✅ Dataset server started (PID: $DATASET_PID)"

echo ""
echo "${BLUE}[4/4] Starting NIF Integration Bridge (port 5001)...${NC}"
python3 nif_bridge.py > /tmp/navlambda-nif.log 2>&1 &
NIF_PID=$!
echo "✅ NIF bridge started (PID: $NIF_PID)"

# Wait for services to start
echo ""
echo "⏳ Waiting for services to initialize..."
sleep 5

# Check if services are running
echo ""
echo "🔍 Checking service health..."

# Check simulation backend
if curl -s http://localhost:3030/api/state > /dev/null 2>&1; then
    echo "${GREEN}✅ Simulation Backend: RUNNING${NC}"
else
    echo "❌ Simulation Backend: NOT RESPONDING"
fi

# Check dataset server
if curl -s http://localhost:5000/health > /dev/null 2>&1; then
    echo "${GREEN}✅ Dataset Server: RUNNING${NC}"
else
    echo "❌ Dataset Server: NOT RESPONDING"
fi

# Check NIF bridge
if curl -s http://localhost:5001/health > /dev/null 2>&1; then
    echo "${GREEN}✅ NIF Bridge: RUNNING${NC}"
else
    echo "❌ NIF Bridge: NOT RESPONDING"
fi

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo "${GREEN}🎉 Backend services are running!${NC}"
echo ""
echo "📊 Service URLs:"
echo "  • Simulation API:  http://localhost:3030"
echo "  • Dataset Server:  http://localhost:5000"
echo "  • NIF Bridge:      http://localhost:5001"
echo ""
echo "📝 Logs:"
echo "  • Simulation: /tmp/navlambda-sim.log"
echo "  • Dataset:    /tmp/navlambda-dataset.log"
echo "  • NIF Bridge: /tmp/navlambda-nif.log"
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""

# Start frontend
cd ..
echo "${BLUE}Starting Frontend (port 5173)...${NC}"
echo "🌐 Opening browser at http://localhost:5173"
echo ""
echo "💡 Click the 🎮 icon to access NAVΛ SIM"
echo ""

# Save PIDs for cleanup
echo "$SIM_PID $DATASET_PID $NIF_PID" > /tmp/navlambda-pids.txt

# Start frontend (this will block)
npm run dev

# Cleanup on exit
echo ""
echo "🛑 Shutting down services..."
kill $SIM_PID $DATASET_PID $NIF_PID 2>/dev/null
rm /tmp/navlambda-pids.txt 2>/dev/null
echo "✅ All services stopped"

