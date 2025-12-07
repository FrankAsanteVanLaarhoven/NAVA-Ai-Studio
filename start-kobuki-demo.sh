#!/bin/bash

# Kobuki Demo Launcher
# Starts Univarm backend + ROS 2 Kobuki simulation

set -e

echo "================================================"
echo "🤖 Starting Kobuki Demo with Univarm Integration"
echo "================================================"
echo ""

# Color codes
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BLUE='\033[0;34m'
NC='\033[0m'

# Check ROS 2
if [ -z "$ROS_DISTRO" ]; then
    for distro in humble iron jazzy rolling; do
        if [ -f "/opt/ros/$distro/setup.bash" ]; then
            export ROS_DISTRO=$distro
            break
        fi
    done
fi

if [ -z "$ROS_DISTRO" ]; then
    echo -e "${RED}❌ ROS 2 not found${NC}"
    exit 1
fi

echo -e "${GREEN}✓ Using ROS 2 $ROS_DISTRO${NC}"

# Source ROS 2
source /opt/ros/$ROS_DISTRO/setup.bash

# Check if workspace is built
if [ ! -f "ros2_ws/install/setup.bash" ]; then
    echo -e "${YELLOW}⚠ Workspace not built. Running build...${NC}"
    ./build-ros2.sh
fi

# Source workspace
echo -e "${BLUE}Sourcing workspace...${NC}"
source ros2_ws/install/setup.bash

# Start Univarm backend in background
echo ""
echo -e "${BLUE}Starting Univarm backend...${NC}"
cd backend-univarm/rust
cargo run --release > ../../logs/univarm-backend.log 2>&1 &
BACKEND_PID=$!
echo -e "${GREEN}✓ Backend started (PID: $BACKEND_PID)${NC}"
cd ../..

# Wait for backend
echo -e "${YELLOW}Waiting for backend on port 8080...${NC}"
for i in {1..30}; do
    if lsof -Pi :8080 -sTCP:LISTEN -t >/dev/null 2>&1; then
        echo -e "${GREEN}✓ Backend ready${NC}"
        break
    fi
    sleep 1
done

echo ""
echo -e "${BLUE}Launching Kobuki with RViz...${NC}"
echo ""

# Launch Kobuki
ros2 launch univarm_kobuki_bringup bringup.launch.py use_rviz:=true

# Cleanup on exit
trap "kill $BACKEND_PID 2>/dev/null" EXIT

